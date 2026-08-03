use specs::{Entities, System, Write, WriteStorage};

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
        Write<'a, DamageQueue>,
        WriteStorage<'a, Health>,
        WriteStorage<'a, Dead>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, mut queue, mut healths, mut deads) = data;

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
                let _ = deads.insert(event.target, Dead::default());
            }
        }
    }
}
