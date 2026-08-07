//! Explosion-related ECS systems.

use nalgebra::Vector3;
use specs::{Builder, Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use super::components::Explosion;
use crate::components::{Position, Velocity};
use crate::particles::ParticleEmitter;
use crate::physics::{PhysicsImpulse, PhysicsImpulseQueue};
use crate::terrain::TerrainWorld;

/// System that processes explosion events.
///
/// For each unprocessed explosion:
/// 1. Carves a crater in the terrain using modify_sphere
/// 2. Applies knockback force to nearby entities with Velocity
/// 3. Marks the explosion as processed for cleanup
pub struct ExplosionSystem;

impl<'a> System<'a> for ExplosionSystem {
    type SystemData = (
        Entities<'a>,
        Option<Write<'a, TerrainWorld>>,
        WriteStorage<'a, Explosion>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        Read<'a, specs::LazyUpdate>,
        Write<'a, PhysicsImpulseQueue>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            mut terrain_manager_opt,
            mut explosions,
            positions,
            mut velocities,
            lazy,
            mut impulse_queue,
        ) = data;

        // Collect explosion data first to avoid borrow issues
        let explosion_data: Vec<_> = (&entities, &explosions)
            .join()
            .filter(|(_, e)| !e.processed)
            .map(|(entity, e)| {
                (
                    entity,
                    e.center,
                    e.crater_radius,
                    e.blast_radius,
                    e.force,
                    e.terrain_damage,
                )
            })
            .collect();

        if explosion_data.is_empty() {
            return;
        }

        // Process terrain destruction
        if let Some(ref mut terrain_manager) = terrain_manager_opt {
            for &(_, center, crater_radius, _, _, terrain_damage) in &explosion_data {
                terrain_manager.damage_sphere(center, crater_radius, terrain_damage);
            }
        }

        // Apply knockback to entities with velocity
        for &(_, center, _, blast_radius, force, _) in &explosion_data {
            let center_vec = center.coords;

            for (pos, vel) in (&positions, &mut velocities).join() {
                let to_entity = pos.0 - center_vec;
                let distance = to_entity.magnitude();

                // Skip if outside blast radius or at explosion center
                if distance >= blast_radius || distance < 0.01 {
                    continue;
                }

                // Calculate knockback with linear falloff
                let falloff = 1.0 - (distance / blast_radius);
                let direction = to_entity.normalize();
                let knockback = direction * force * falloff;

                // Add upward component for more dramatic effect
                let upward_boost = Vector3::new(0.0, force * falloff * 0.5, 0.0);

                vel.0 += knockback + upward_boost;
            }
        }

        // Queue physics impulses for rigid bodies (applied by physics system)
        for &(_, center, _, blast_radius, force, _) in &explosion_data {
            impulse_queue.push(PhysicsImpulse::radial(center, blast_radius, force, 0.5));
        }

        // Spawn particle emitters at explosion locations
        for &(_, center, _, _, _, _) in &explosion_data {
            let pos = Position(center.coords);

            // Flash - bright, short-lived burst
            lazy.create_entity(&entities)
                .with(pos.clone())
                .with(ParticleEmitter::explosion_flash())
                .build();

            // Smoke - rising, long-lived
            lazy.create_entity(&entities)
                .with(pos.clone())
                .with(ParticleEmitter::smoke())
                .build();

            // Debris - physics chunks
            lazy.create_entity(&entities)
                .with(pos.clone())
                .with(ParticleEmitter::debris())
                .build();

            // Sparks - fast, bright trails
            lazy.create_entity(&entities)
                .with(pos)
                .with(ParticleEmitter::sparks())
                .build();
        }

        // Mark explosions as processed
        for (entity, _, _, _, _, _) in explosion_data {
            if let Some(explosion) = explosions.get_mut(entity) {
                explosion.processed = true;
            }
        }

        // Delete processed explosion entities
        for (entity, explosion) in (&entities, &explosions).join() {
            if explosion.processed {
                let _ = entities.delete(entity);
            }
        }
    }
}
