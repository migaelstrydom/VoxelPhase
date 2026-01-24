//! Projectile-related ECS systems.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{
    Builder, Entities, Entity, Join, LazyUpdate, Read, ReadExpect, ReadStorage, System, Write,
    WriteStorage,
};

use super::components::{Grenade, Lifetime, Projectile};
use super::config::GrenadeConfig;
use crate::components::{
    Acceleration, Collider, Gravity, ModelInstance, MotionState, Position, Renderable, Rotation,
    Velocity,
};
use crate::explosion::Explosion;
use crate::input::GameplayActions;
use crate::model::Model;
use crate::player::Player;
use crate::terrain::TerrainManager;
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
        Write<'a, GrenadeCooldown>,
        Read<'a, GrenadeModelResource>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        Read<'a, LazyUpdate>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            actions,
            config,
            time,
            mut cooldown,
            grenade_model,
            players,
            positions,
            rotations,
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

        // Get player position and rotation (facing direction)
        let (player_pos, throw_dir) = match (&players, &positions, &rotations).join().next() {
            Some((_, pos, rot)) => {
                // Calculate forward direction from player's rotation
                // Rotation.0 is the Y-axis rotation angle
                let forward = Vector3::new(rot.0.sin(), 0.0, rot.0.cos());
                (pos.0, forward.normalize())
            }
            None => return,
        };

        // Spawn position: slightly in front of and above the player
        let spawn_offset = throw_dir * 0.8 + Vector3::new(0.0, 0.5, 0.0);
        let spawn_pos = player_pos + spawn_offset;

        // Calculate throw velocity: forward + upward arc
        let throw_velocity =
            throw_dir * config.throw_speed + Vector3::new(0.0, config.arc_factor, 0.0);

        // Spawn the grenade entity
        let spawn_point = Point3::new(spawn_pos.x, spawn_pos.y, spawn_pos.z);
        lazy.create_entity(&entities)
            .with(Position(spawn_pos))
            .with(Velocity(throw_velocity))
            .with(Acceleration(Vector3::zeros()))
            .with(Rotation(0.0))
            .with(Gravity(config.gravity))
            .with(Collider::sphere(config.radius))
            .with(MotionState::new(spawn_point))
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

/// System that detects projectile collisions with terrain and triggers explosions.
pub struct ProjectileCollisionSystem;

impl<'a> System<'a> for ProjectileCollisionSystem {
    type SystemData = (
        Entities<'a>,
        Option<Read<'a, TerrainManager>>,
        ReadExpect<'a, Time>,
        ReadStorage<'a, Projectile>,
        ReadStorage<'a, Grenade>,
        ReadStorage<'a, Collider>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        Read<'a, LazyUpdate>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            terrain_opt,
            time,
            projectiles,
            grenades,
            colliders,
            mut positions,
            mut velocities,
            lazy,
        ) = data;

        let terrain = match terrain_opt {
            Some(ref t) => t,
            None => return,
        };

        let dt = time.delta_seconds();

        // Collect grenades that should explode
        let mut explosions: Vec<(Entity, Point3<f32>)> = Vec::new();

        for (entity, _, grenade, collider, pos, vel) in (
            &entities,
            &projectiles,
            &grenades,
            &colliders,
            &mut positions,
            &mut velocities,
        )
            .join()
        {
            if !grenade.armed {
                continue;
            }

            let radius = collider.shape.radius;

            // Calculate next position
            let current = Point3::from(pos.0);
            let next = Point3::from(pos.0 + vel.0 * dt);

            // Swept sphere collision detection
            if let Some(contact) = terrain.query_swept_sphere(current, next, radius) {
                // Hit terrain - explode at contact point
                let explosion_pos = current + (next - current) * contact.t;
                explosions.push((entity, explosion_pos));
            }
        }

        // Create explosion entities and delete grenades
        for (entity, pos) in explosions {
            // Create explosion entity
            lazy.create_entity(&entities)
                .with(Explosion::new(pos))
                .build();

            // Delete the grenade
            let _ = entities.delete(entity);
        }
    }
}
