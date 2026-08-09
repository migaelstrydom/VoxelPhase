use specs::{Entities, Read, System, Write, WriteStorage};

use super::config::DamageConfig;
use super::event::DamageQueue;
use super::health::{Dead, Health};

/// Drains the damage queue into `Health` and marks the newly dead.
///
/// The single writer of `Health`, so a frame's blast, burn and impact all land
/// in one place and exactly one of them can be the killing blow. Death effects
/// are `DeathSystem`'s job — this system only decides *that* something died.
pub struct DamageApplySystem;

impl<'a> System<'a> for DamageApplySystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, DamageConfig>,
        Write<'a, DamageQueue>,
        WriteStorage<'a, Health>,
        WriteStorage<'a, Dead>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, config, mut queue, mut healths, mut deads) = data;

        if queue.is_empty() {
            return;
        }

        for event in queue.drain() {
            // The target may have been deleted between the source system
            // queueing this and now — a crate destroyed by the same blast.
            if !entities.is_alive(event.target) {
                continue;
            }
            let Some(health) = healths.get_mut(event.target) else {
                continue;
            };
            if health.apply(event.amount) {
                if config.deaths_enabled {
                    let _ = deads.insert(event.target, Dead::default());
                } else {
                    // Leaving it parked at zero would make every later hit look
                    // like a fresh killing blow. Floor it instead, so the entity
                    // is merely very hurt.
                    health.current = health.current.max(1.0);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::damage::event::DamageKind;
    use specs::{Builder, DispatcherBuilder, World, WorldExt};

    /// Build a world holding one entity at `hp`, push `damage` at it, run the
    /// system with the given config, and report whether it ended up `Dead`.
    fn kill_attempt(hp: f32, damage: f32, deaths_enabled: bool) -> (bool, f32) {
        let mut world = World::new();
        world.register::<Health>();
        world.register::<Dead>();
        world.insert(DamageQueue::default());
        world.insert(DamageConfig { deaths_enabled });

        let entity = world.create_entity().with(Health::persistent(hp)).build();
        world
            .write_resource::<DamageQueue>()
            .push(entity, damage, DamageKind::Blast);

        let mut dispatcher = DispatcherBuilder::new()
            .with(DamageApplySystem, "damage_apply", &[])
            .build();
        dispatcher.dispatch(&world);
        world.maintain();

        let is_dead = world.read_storage::<Dead>().get(entity).is_some();
        let remaining = world.read_storage::<Health>().get(entity).unwrap().current;
        (is_dead, remaining)
    }

    /// Death is switched off until it has been designed. Nothing downstream of
    /// it exists — no respawn, no feedback — and a dead player silently loses
    /// `VelocityDriven`, which drops it out of `CharacterControlSystem` and
    /// reads as the controls locking up.
    #[test]
    fn a_fatal_hit_does_not_kill_while_deaths_are_disabled() {
        let (is_dead, remaining) = kill_attempt(100.0, 500.0, false);

        assert!(!is_dead);
        assert!(
            remaining > 0.0,
            "health floored above zero, got {remaining}"
        );
    }

    /// Parking health at exactly zero would make every later hit look like a
    /// fresh killing blow.
    #[test]
    fn an_overkilled_entity_can_still_take_another_hit() {
        let (is_dead, _) = kill_attempt(1.0, 500.0, false);
        assert!(!is_dead);

        let (is_dead_again, remaining) = kill_attempt(1.0, 500.0, false);
        assert!(!is_dead_again);
        assert!(remaining > 0.0);
    }

    /// The mechanic itself still works — this is a switch, not a removal.
    #[test]
    fn the_same_hit_kills_once_deaths_are_enabled() {
        let (is_dead, _) = kill_attempt(100.0, 500.0, true);
        assert!(is_dead);
    }

    #[test]
    fn a_survivable_hit_is_unaffected_by_the_switch() {
        for enabled in [false, true] {
            let (is_dead, remaining) = kill_attempt(100.0, 30.0, enabled);
            assert!(!is_dead);
            assert_eq!(remaining, 70.0);
        }
    }
}
