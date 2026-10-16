//! Projectile-related ECS systems.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{
    Builder, Entities, Entity, Join, LazyUpdate, Read, ReadExpect, ReadStorage, System, Write,
    WriteStorage,
};

use super::components::{Grenade, Lifetime, Projectile};
use super::config::GrenadeConfig;
use super::held::HeldGrenade;
use super::spawn::spawn_grenade;
use super::throw::grenade_launch;
use crate::camera::FollowTarget;
use crate::character::{CharacterIntent, Throws};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Rotation,
};
use crate::explosion::Explosion;
use crate::model::Model;
use crate::systems::PhysicsResource;
use crate::time::Time;

/// System that updates entity lifetimes and removes expired entities.
pub struct LifetimeSystem;

impl<'a> System<'a> for LifetimeSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, Time>,
        WriteStorage<'a, Lifetime>,
    );

    fn run(&mut self, (entities, time, mut lifetimes): Self::SystemData) {
        let delta = time.delta_seconds();

        // Collect entities to delete (can't delete while iterating)
        let mut to_delete = Vec::new();

        for (entity, lifetime) in (&entities, &mut lifetimes).join() {
            lifetime.remaining -= delta;
            if lifetime.remaining <= 0.0 {
                to_delete.push(entity);
            }
        }

        // Delete expired entities
        for entity in to_delete {
            let _ = entities.delete(entity);
        }
    }
}

/// Resource to store the grenade model for spawning.
#[derive(Default)]
pub struct GrenadeModelResource {
    pub model: Option<Arc<Model>>,
}

/// Resource to track grenade throw cooldown.
#[derive(Default)]
pub struct GrenadeCooldown {
    pub remaining: f32,
}

/// System that throws grenades for any character with `Throws` asking to.
///
/// The click commits to a throw and puts a grenade in the hand; the grenade
/// flies once the arm has wound up (`GrenadeConfig::release_delay`), from
/// where the thrower is then, along where it is then aiming: the intent's aim
/// if it has one, where the camera looks if the camera follows it, its own
/// facing if not.
pub struct GrenadeSpawnSystem;

impl<'a> System<'a> for GrenadeSpawnSystem {
    #[allow(clippy::type_complexity)]
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, GrenadeConfig>,
        ReadExpect<'a, Time>,
        Write<'a, PhysicsResource>,
        Write<'a, GrenadeCooldown>,
        Read<'a, GrenadeModelResource>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, FollowTarget>,
        WriteStorage<'a, Throws>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Orientation>,
        WriteStorage<'a, ModelInstance>,
        WriteStorage<'a, Renderable>,
        WriteStorage<'a, HeldGrenade>,
        Read<'a, LazyUpdate>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            config,
            time,
            mut physics,
            mut cooldown,
            grenade_model,
            intents,
            rotations,
            follow_targets,
            mut throws,
            mut positions,
            mut orientations,
            mut models,
            mut renderables,
            mut held,
            lazy,
        ) = data;

        cooldown.remaining = (cooldown.remaining - time.delta_seconds()).max(0.0);

        let Some(model) = grenade_model.model.clone() else {
            return;
        };
        let world_gravity = physics.world.config().gravity;
        // Yaw and pitch a thrower aims along. orbit_angle is where the camera
        // sits, so the look direction is opposite it.
        let aim_of = |thrower: Entity, intent: &CharacterIntent, yaw: f32| {
            if let Some(aim) = intent.aim {
                return (aim.x.atan2(aim.z), 0.0);
            }
            follow_targets
                .join()
                .find(|follow| follow.target == thrower)
                .map_or((yaw, 0.0), |follow| {
                    (follow.orbit_angle + std::f32::consts::PI, follow.pitch)
                })
        };

        let mut new_props = Vec::new();
        let mut releases = Vec::new();
        for (thrower, intent, position, rotation, throws) in
            (&entities, &intents, &positions, &rotations, &mut throws).join()
        {
            let (look_angle, camera_pitch) = aim_of(thrower, intent, rotation.0);
            // throw_grenade is resolved by PlayerControlSystem — only true
            // when the throw action was not consumed by a grab-throw.
            let commit = cooldown.remaining <= 0.0 && intent.throw_grenade;
            if commit {
                cooldown.remaining = config.cooldown;
            }
            let aim = commit.then(|| Vector3::new(look_angle.sin(), 0.0, look_angle.cos()));
            let released = throws.advance(time.delta_seconds(), aim, config.release_delay);

            if commit {
                if let Some(stroke) = throws.strokes().last().filter(|s| s.elapsed == 0.0) {
                    new_props.push((thrower, stroke.id, stroke.hand, position.0));
                }
            }
            for stroke in released {
                releases.push((thrower, stroke.id, position.0, look_angle, camera_pitch));
            }
        }

        for (thrower, stroke, hand, at) in new_props {
            let prop = entities.create();
            let _ = positions.insert(prop, Position(at));
            let _ = orientations.insert(prop, Orientation::default());
            let _ = models.insert(prop, ModelInstance::new(model.clone()));
            let _ = renderables.insert(prop, Renderable);
            let _ = held.insert(
                prop,
                HeldGrenade {
                    thrower,
                    stroke,
                    hand,
                },
            );
        }

        for (thrower, stroke, at, look_angle, camera_pitch) in releases {
            for (prop, held) in (&entities, &held).join() {
                if held.thrower == thrower && held.stroke == stroke {
                    let _ = entities.delete(prop);
                }
            }
            // The one statement of what a grenade throw is, shared with the
            // aiming cursor so the two cannot drift apart.
            let launch = grenade_launch(
                &config,
                Point3::from(at),
                look_angle,
                camera_pitch,
                world_gravity,
            );
            spawn_grenade(
                lazy.create_entity(&entities),
                &mut physics.world,
                &config,
                model.clone(),
                launch.origin,
                launch.velocity,
            );
        }
    }
}

/// System that burns grenade fuses and detonates grenades on hard impacts.
///
/// Detonation has two triggers. The fuse always fires, so a grenade that comes
/// to rest still goes off. Impact detonation fires early when the frame's
/// normal impulse exceeds what it would take to arrest the grenade from
/// `detonation_impact_speed`, which is what makes a direct throw explode on
/// contact while a bounce off a menhir merely bounces.
pub struct ProjectileDetonationSystem;

impl<'a> System<'a> for ProjectileDetonationSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        ReadExpect<'a, Time>,
        Read<'a, GrenadeConfig>,
        ReadStorage<'a, Projectile>,
        WriteStorage<'a, Grenade>,
        ReadStorage<'a, RigidBodyComponent>,
        Read<'a, LazyUpdate>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, mut physics, time, config, projectiles, mut grenades, bodies, lazy) = data;
        let delta = time.delta_seconds();

        let mut detonations: Vec<(Entity, Point3<f32>)> = Vec::new();

        for (entity, _, grenade, body) in (&entities, &projectiles, &mut grenades, &bodies).join() {
            grenade.tick(delta);

            let Some(rigid_body) = physics.world.body(body.0) else {
                continue;
            };
            let centre = rigid_body.position();

            if grenade.fuse_expired() {
                detonations.push((entity, centre));
                continue;
            }

            if !grenade.is_armed() {
                continue;
            }

            // The impulse that would arrest this grenade from the configured
            // speed. Deriving it from mass keeps the tunable speed-like even if
            // the collider's size or density changes.
            let threshold = rigid_body.mass() * config.detonation_impact_speed;
            if let Some(impact) = physics.world.impacts().get(body.0) {
                if impact.total_impulse >= threshold {
                    detonations.push((entity, impact.point));
                }
            }
        }

        for (entity, position) in detonations {
            lazy.create_entity(&entities)
                .with(Explosion::new(position))
                .build();

            if let Some(body) = bodies.get(entity) {
                let _ = physics.world.remove_body(body.0);
            }

            let _ = entities.delete(entity);
        }
    }
}
