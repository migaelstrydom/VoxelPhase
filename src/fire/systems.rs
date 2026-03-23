//! Fire-related ECS systems.
//!
//! `FireIgnitionSystem` detects explosions near flammable entities and sets
//! them on fire. `FireCleanupSystem` removes the `OnFire` component when fuel
//! is exhausted. GPU simulation and rendering are driven by `RenderSystem`.

use specs::{Entities, Join, ReadStorage, System, WriteStorage};

use crate::components::Position;
use crate::explosion::Explosion;

use super::components::{Flammable, OnFire};

/// Ignites flammable entities caught in an explosion's blast radius.
///
/// Runs after `ExplosionSystem` (which marks explosions as processed).
/// Only unprocessed explosions trigger ignition — but we read the explosion
/// before it's deleted, in the same frame.
pub struct FireIgnitionSystem;

impl<'a> System<'a> for FireIgnitionSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Explosion>,
        ReadStorage<'a, Flammable>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, OnFire>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, explosions, flammables, positions, mut on_fires) = data;

        // Collect active explosions (not yet processed)
        let active_explosions: Vec<_> = (&explosions)
            .join()
            .filter(|e| !e.processed)
            .map(|e| (e.center, e.blast_radius))
            .collect();

        if active_explosions.is_empty() {
            return;
        }

        // Check each flammable entity against each explosion
        let ignitions: Vec<_> = (&entities, &flammables, &positions, !&on_fires)
            .join()
            .filter_map(|(entity, flammable, pos, _)| {
                for &(center, blast_radius) in &active_explosions {
                    let distance = (pos.0 - center.coords).magnitude();
                    if distance < blast_radius {
                        let falloff = 1.0 - (distance / blast_radius);
                        let fuel = flammable.fuel * falloff;
                        return Some((entity, fuel));
                    }
                }
                None
            })
            .collect();

        for (entity, fuel) in ignitions {
            let _ = on_fires.insert(
                entity,
                OnFire {
                    fuel_remaining: fuel,
                    burn_time: 0.0,
                },
            );
        }
    }
}

/// Removes the `OnFire` component from entities whose fuel is exhausted.
pub struct FireCleanupSystem;

impl<'a> System<'a> for FireCleanupSystem {
    type SystemData = (
        Entities<'a>,
        WriteStorage<'a, OnFire>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, mut on_fires) = data;

        let extinguished: Vec<_> = (&entities, &on_fires)
            .join()
            .filter(|(_, fire)| fire.fuel_remaining <= 0.0 && fire.burn_time > 2.0)
            .map(|(entity, _)| entity)
            .collect();

        for entity in extinguished {
            on_fires.remove(entity);
        }
    }
}
