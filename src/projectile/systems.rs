//! Projectile-related ECS systems.

use std::collections::HashMap;
use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{
    Builder, Entities, Entity, Join, LazyUpdate, Read, ReadExpect, ReadStorage, System, Write,
    WriteStorage,
};

use super::components::{Grenade, Lifetime, Projectile};
use super::config::GrenadeConfig;
use crate::camera::FollowTarget;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::explosion::Explosion;
use crate::input::GameplayActions;
use crate::model::Model;
use crate::physics::{ColliderDesc, ContactEvent, RigidBodyDesc, RigidBodyHandle};
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
        ReadExpect<'a, GameplayActions>,
        ReadExpect<'a, GrenadeConfig>,
        ReadExpect<'a, Time>,
        Write<'a, PhysicsResource>,
        Write<'a, GrenadeCooldown>,
        Read<'a, GrenadeModelResource>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, FollowTarget>,
        Read<'a, LazyUpdate>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            actions,
            config,
            time,
            mut physics,
            mut cooldown,
            grenade_model,
            players,
            positions,
            follow_targets,
            lazy,
        ) = data;

        // Update cooldown
        cooldown.remaining = (cooldown.remaining - time.delta_seconds()).max(0.0);

        if !actions.throw_grenade {
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

        // Calculate throw pitch based on camera orientation.
        // Camera pitch is positive when looking down, so we subtract it.
        // upward_offset provides a base upward angle, pitch_influence controls how much
        // the camera pitch affects the throw direction.
        let throw_pitch = config.upward_offset - camera_pitch * config.pitch_influence;
        let throw_pitch = throw_pitch.clamp(-0.4, 0.8); // Clamp to reasonable range

        // Calculate 3D throw direction using spherical coordinates
        let cos_pitch = throw_pitch.cos();
        let sin_pitch = throw_pitch.sin();
        let throw_dir = Vector3::new(
            look_angle.sin() * cos_pitch,
            sin_pitch,
            look_angle.cos() * cos_pitch,
        );

        // Spawn position: slightly in front of and above the player
        let horizontal_dir = Vector3::new(look_angle.sin(), 0.0, look_angle.cos());
        let spawn_offset = horizontal_dir * 0.1 + Vector3::new(0.0, 0.5, 0.0);
        let spawn_pos = player_pos + spawn_offset;

        // Calculate throw velocity: directional throw + additional arc factor
        let throw_velocity =
            throw_dir * config.throw_speed + Vector3::new(0.0, config.arc_factor, 0.0);

        let gravity_mag = physics.world.config().gravity.magnitude();
        let gravity_scale = if gravity_mag > 1e-6 {
            config.gravity / gravity_mag
        } else {
            1.0
        };

        let body_handle = {
            let body_desc = RigidBodyDesc::dynamic()
                .position(Point3::new(spawn_pos.x, spawn_pos.y, spawn_pos.z))
                .linear_velocity(throw_velocity)
                .gravity_scale(gravity_scale);

            let body_handle = physics.world.create_body(body_desc);

            let collider_desc = ColliderDesc::sphere(config.radius);
            physics.world.attach_collider(body_handle, collider_desc);

            body_handle
        };

        lazy.create_entity(&entities)
            .with(Position(spawn_pos))
            .with(Velocity(throw_velocity))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(Projectile)
            .with(Grenade::new())
            .with(Lifetime::new(config.max_lifetime))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build();

        // Set cooldown
        cooldown.remaining = config.cooldown;
    }
}

/// System that detects projectile impacts with terrain and triggers explosions.
pub struct ProjectileImpactDetectionSystem;

impl<'a> System<'a> for ProjectileImpactDetectionSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        ReadStorage<'a, Projectile>,
        ReadStorage<'a, Grenade>,
        ReadStorage<'a, RigidBodyComponent>,
        Read<'a, LazyUpdate>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, mut physics, projectiles, grenades, bodies, lazy) = data;

        // Collect grenades that should explode
        let mut explosions: Vec<(Entity, Point3<f32>)> = Vec::new();
        let mut best_static_contacts: HashMap<RigidBodyHandle, (f32, Point3<f32>)> = HashMap::new();

        for contact in physics.world.contact_events() {
            if !contact_is_static(contact) {
                continue;
            }
            if contact.normal.magnitude_squared() <= 1e-8 {
                continue;
            }
            let entry = best_static_contacts
                .entry(contact.body_b)
                .or_insert((contact.depth, contact.point));
            if contact.depth > entry.0 {
                *entry = (contact.depth, contact.point);
            }
        }

        for (entity, _, grenade, body) in (&entities, &projectiles, &grenades, &bodies).join() {
            if !grenade.armed {
                continue;
            }

            if let Some((_, point)) = best_static_contacts.get(&body.0) {
                explosions.push((entity, point.clone()));
            }
        }

        // Create explosion entities and delete grenades
        for (entity, pos) in explosions {
            // Create explosion entity
            lazy.create_entity(&entities)
                .with(Explosion::new(pos))
                .build();

            if let Some(body) = bodies.get(entity) {
                let _ = physics.world.remove_body(body.0);
            }

            // Delete the grenade
            let _ = entities.delete(entity);
        }
    }
}

fn contact_is_static(contact: &ContactEvent) -> bool {
    contact.body_a.is_none()
}
