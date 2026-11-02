//! Fire-related ECS systems.
//!
//! `FireIgnitionSystem` detects explosions near flammable entities and sets
//! them on fire. `FireCleanupSystem` removes the `OnFire` component when fuel
//! is exhausted. GPU simulation and rendering are driven by `RenderSystem`.

use specs::{Entities, Join, Read, ReadStorage, System, WriteStorage};

use crate::components::Position;
use crate::explosion::Explosion;
use crate::time::Time;

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

        // Collect explosions that fired this frame (processed by ExplosionSystem
        // but not yet deleted — entity deletion is deferred until world.maintain()).
        let active_explosions: Vec<_> = (&explosions)
            .join()
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
                    burnt_out_for: 0.0,
                },
            );
        }
    }
}

/// Ticks down fuel and burn time on burning entities, then removes `OnFire`
/// when fuel is exhausted and a grace period has elapsed.
pub struct FireCleanupSystem;

/// Rate at which ECS fuel_remaining is consumed per second.
/// This controls how long the fire *exists* (source injection fades with fuel).
/// Separate from FireSimParams::burn_rate which controls per-voxel GPU fuel.
/// With fuel=20, fire lasts about 67 seconds.
const FUEL_BURN_RATE: f32 = 0.3;

/// Seconds a fire is kept after its fuel runs out, while the volume, no
/// longer fed, burns off and its smoke rises and thins.
const SMOKE_GRACE: f32 = 2.0;

impl<'a> System<'a> for FireCleanupSystem {
    type SystemData = (Entities<'a>, WriteStorage<'a, OnFire>, Read<'a, Time>);

    fn run(&mut self, data: Self::SystemData) {
        let (entities, mut on_fires, time) = data;
        let dt = time.delta_seconds();

        // Tick fuel and burn time, and once the fuel is gone the grace period.
        for fire in (&mut on_fires).join() {
            fire.burn_time += dt;
            if fire.fuel_remaining <= 0.0 {
                fire.burnt_out_for += dt;
            }
            fire.fuel_remaining = (fire.fuel_remaining - FUEL_BURN_RATE * dt).max(0.0);
        }

        // Remove fires whose smoke has had its grace period to clear.
        let extinguished: Vec<_> = (&entities, &on_fires)
            .join()
            .filter(|(_, fire)| fire.burnt_out_for > SMOKE_GRACE)
            .map(|(entity, _)| entity)
            .collect();

        for entity in extinguished {
            on_fires.remove(entity);
        }
    }
}

#[cfg(test)]
mod tests {
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;

    /// A fire is kept for the grace period after its fuel runs out, however
    /// long it had been burning, so its smoke clears rather than vanishing.
    #[test]
    fn a_burnt_out_fire_is_kept_while_its_smoke_clears() {
        let dt = 0.1;
        let mut world = World::new();
        world.register::<OnFire>();
        world.insert(Time::fixed(dt));
        let entity = world
            .create_entity()
            .with(OnFire {
                fuel_remaining: FUEL_BURN_RATE * dt,
                burn_time: 30.0,
                burnt_out_for: 0.0,
            })
            .build();

        let mut cleanup = FireCleanupSystem;
        cleanup.run_now(&world);
        let burning = |world: &World| world.read_storage::<OnFire>().contains(entity);
        assert!(burning(&world), "removed the frame its fuel ran out");

        let frames = (SMOKE_GRACE / dt).ceil() as usize + 1;
        for _ in 0..frames {
            cleanup.run_now(&world);
        }
        assert!(!burning(&world), "kept after the grace period");
    }
}
