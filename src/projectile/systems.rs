//! Projectile-related ECS systems.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{
    Builder, Entities, Entity, Join, LazyUpdate, Read, ReadExpect, ReadStorage, System, Write,
    WriteStorage,
};

use super::components::{Grenade, Lifetime, Projectile};
use super::config::GrenadeConfig;
use super::throw::grenade_launch;
use crate::aim::launch::gravity_scale;
use crate::camera::FollowTarget;
use crate::character::CharacterIntent;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::explosion::Explosion;
use crate::model::Model;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::player::Player;
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

/// System that spawns grenades on left mouse click.
pub struct GrenadeSpawnSystem;

impl<'a> System<'a> for GrenadeSpawnSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, GrenadeConfig>,
        ReadExpect<'a, Time>,
        Write<'a, PhysicsResource>,
        Write<'a, GrenadeCooldown>,
        Read<'a, GrenadeModelResource>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, FollowTarget>,
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
            players,
            player_targets,
            positions,
            follow_targets,
            lazy,
        ) = data;

        // Update cooldown
        cooldown.remaining = (cooldown.remaining - time.delta_seconds()).max(0.0);

        // throw_grenade is resolved by PlayerControlSystem — only true when the
        // throw action was not consumed by a grab-throw.
        let should_throw = (&players, &player_targets)
            .join()
            .any(|(_, target)| target.throw_grenade);
        if !should_throw {
            return;
        }

        // Check cooldown
        if cooldown.remaining > 0.0 {
            return;
        }

        // Get the grenade model
        let model = match &grenade_model.model {
            Some(m) => m.clone(),
            None => {
                log::warn!("No grenade model available");
                return;
            }
        };

        // Get player position
        let player_pos = match (&players, &positions).join().next() {
            Some((_, pos)) => pos.0,
            None => return,
        };

        // Get camera orientation from FollowTarget
        // orbit_angle is where the camera is positioned, so we add π to get the look direction
        let (look_angle, camera_pitch) = match (&follow_targets,).join().next() {
            Some((follow,)) => (follow.orbit_angle + std::f32::consts::PI, follow.pitch),
            None => {
                log::warn!("No camera FollowTarget found");
                return;
            }
        };

        // The one statement of what a grenade throw is, shared with the
        // aiming cursor so the two cannot drift apart.
        let world_gravity = physics.world.config().gravity;
        let launch = grenade_launch(
            &config,
            Point3::new(player_pos.x, player_pos.y, player_pos.z),
            look_angle,
            camera_pitch,
            world_gravity,
        );
        let spawn_pos = Vector3::new(launch.origin.x, launch.origin.y, launch.origin.z);
        let throw_velocity = launch.velocity;
        let grenade_gravity_scale = gravity_scale(world_gravity, config.gravity);

        let body_handle = {
            let body_desc = RigidBodyDesc::dynamic()
                .position(launch.origin)
                .linear_velocity(throw_velocity)
                .gravity_scale(grenade_gravity_scale);

            let body_handle = physics.world.create_body(body_desc);

            let collider_desc = ColliderDesc::sphere(config.radius)
                .density(2000.0)
                .restitution(0.0)
                .friction(0.3);
            physics.world.attach_collider(body_handle, collider_desc);

            body_handle
        };

        lazy.create_entity(&entities)
            .with(Position(spawn_pos))
            .with(Velocity(throw_velocity))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(Projectile)
            .with(Grenade::new(config.fuse_time, config.arm_delay))
            .with(Lifetime::new(config.max_lifetime))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build();

        // Set cooldown
        cooldown.remaining = config.cooldown;
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
