//! Explosion-related ECS systems.

use nalgebra::Vector3;
use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use super::components::{Explosion, UPWARD_BOOST};
use super::visuals::ExplosionVisuals;
use crate::components::{Position, Velocity};
use crate::physics::PhysicsImpulseQueue;
use crate::rubble::RubbleQueue;
use crate::terrain::TerrainWorld;

/// System that processes explosion events.
///
/// For each unprocessed explosion:
/// 1. Carves a crater in the terrain, and queues what it cut loose as rubble
/// 2. Applies knockback force to nearby entities with Velocity
/// 3. Hands the blast to [`ExplosionVisuals`], which owns its own timing
/// 4. Marks the explosion as processed for cleanup
#[derive(Default)]
pub struct ExplosionSystem {
    visuals: ExplosionVisuals,
}

impl ExplosionSystem {
    /// Use a non-default look.
    #[allow(dead_code)]
    pub fn new(visuals: ExplosionVisuals) -> Self {
        Self { visuals }
    }
}

impl<'a> System<'a> for ExplosionSystem {
    type SystemData = (
        Entities<'a>,
        Option<Write<'a, TerrainWorld>>,
        WriteStorage<'a, Explosion>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        Read<'a, specs::LazyUpdate>,
        Write<'a, PhysicsImpulseQueue>,
        Write<'a, RubbleQueue>,
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
            mut rubble,
        ) = data;

        // Collect explosion data first to avoid borrow issues
        let explosion_data: Vec<_> = (&entities, &explosions)
            .join()
            .filter(|(_, e)| !e.processed)
            .map(|(entity, e)| {
                (
                    entity,
                    e.center,
                    e.blast,
                    e.blast_radius,
                    e.force,
                    e.physics_impulse(),
                )
            })
            .collect();

        if explosion_data.is_empty() {
            return;
        }

        // Process terrain destruction. What a blast cuts loose is lifted out of
        // the field here and handed on as rubble.
        if let Some(ref mut terrain_manager) = terrain_manager_opt {
            for &(_, center, blast, _, _, _) in &explosion_data {
                rubble.extend(terrain_manager.detonate(center, &blast));
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
                let upward_boost = Vector3::new(0.0, force * falloff * UPWARD_BOOST, 0.0);

                vel.0 += knockback + upward_boost;
            }
        }

        // Queue physics impulses for rigid bodies (applied by physics system)
        for &(_, _, _, _, _, impulse) in &explosion_data {
            impulse_queue.push(impulse);
        }

        // Hand each blast to the visuals, which stages it out over the next
        // few seconds. The blast radius drives the scale, since it is the
        // extent the player is actually being told about.
        for &(_, center, _, blast_radius, _, _) in &explosion_data {
            self.visuals.spawn(&entities, &lazy, center, blast_radius);
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
