//! Explosion-related ECS systems.

use specs::{Entities, Join, Read, System, Write, WriteStorage};

use super::components::Explosion;
use super::visuals::ExplosionVisuals;
use crate::physics::PhysicsImpulseQueue;
use crate::rubble::RubbleQueue;
use crate::terrain::TerrainWorld;

/// System that processes explosion events.
///
/// For each unprocessed explosion:
/// 1. Carves a crater in the terrain, and queues what it cut loose as rubble
/// 2. Queues its shove for the physics step
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
        Read<'a, specs::LazyUpdate>,
        Write<'a, PhysicsImpulseQueue>,
        Write<'a, RubbleQueue>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            mut terrain_manager_opt,
            mut explosions,
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
            for &(_, center, blast, _, shove) in &explosion_data {
                rubble.push_blast(shove, terrain_manager.detonate(center, &blast));
            }
        }

        // The shove is a physics impulse, delivered by the physics step to
        // every body in reach; ECS `Velocity` is the physics' own report and
        // is not written here.
        for &(_, _, _, _, impulse) in &explosion_data {
            impulse_queue.push(impulse);
        }

        // Hand each blast to the visuals, which stages it out over the next
        // few seconds. The blast radius drives the scale, since it is the
        // extent the player is actually being told about.
        for &(_, center, _, blast_radius, _) in &explosion_data {
            self.visuals.spawn(&entities, &lazy, center, blast_radius);
        }

        // Mark explosions as processed
        for (entity, _, _, _, _) in explosion_data {
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

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;
    use crate::components::{Position, Velocity};

    /// A blast's shove reaches bodies through the physics step alone. The
    /// ECS velocity is the physics' report of a body's motion, and writing a
    /// knockback into it gave everything that read it before the next step
    /// (the water's wakes and ripples, impact damage) a speed no body had.
    #[test]
    fn a_blast_queues_its_shove_and_leaves_velocity_to_physics() {
        let mut world = World::new();
        world.register::<Explosion>();
        world.register::<Position>();
        world.register::<Velocity>();
        world.insert(PhysicsImpulseQueue::default());
        world.insert(RubbleQueue::default());
        let crate_near = world
            .create_entity()
            .with(Position(Vector3::new(1.0, 0.0, 0.0)))
            .with(Velocity(Vector3::zeros()))
            .build();
        world
            .create_entity()
            .with(Explosion::new(Point3::origin()))
            .build();

        ExplosionSystem::default().run_now(&world);

        assert_eq!(
            world.read_storage::<Velocity>().get(crate_near).unwrap().0,
            Vector3::zeros()
        );
        assert_eq!(
            world
                .write_resource::<PhysicsImpulseQueue>()
                .drain()
                .count(),
            1
        );
    }
}
