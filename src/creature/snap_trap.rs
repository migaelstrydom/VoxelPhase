//! A trap that lies open and snaps shut on whoever steps into it.
//!
//! ```text
//!   Perception ──target within `radius`──▶ bite: damage, shut for `hold`
//!   shut ──`hold` seconds──▶ open again
//! ```
//!
//! Its two looks are whole models, swapped on its entity as it opens and
//! shuts: a trap has two states and nothing in between worth drawing.

use std::sync::Arc;

use specs::{
    Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, Write, WriteStorage,
};

use super::perception::Perception;
use crate::components::{ModelInstance, Position};
use crate::damage::{DamageKind, DamageQueue, Dead};
use crate::model::Model;
use crate::time::Time;

/// How far above or below the trap its victim may be and still be in it, m.
const DEPTH: f32 = 1.0;

/// A trap and where it stands in its cycle.
#[derive(Component, Clone)]
#[storage(DenseVecStorage)]
pub struct SnapTrap {
    /// Distance from its middle within which it bites, m.
    pub radius: f32,
    /// Hit points a bite takes off.
    pub damage: f32,
    /// Seconds it stays shut after a bite.
    pub hold: f32,
    /// Seconds until it opens again; zero while it lies open.
    pub shut_for: f32,
    /// How it looks lying open.
    pub open: Arc<Model>,
    /// How it looks shut.
    pub shut: Arc<Model>,
}

impl SnapTrap {
    /// Whether it lies open, ready to bite.
    pub fn is_open(&self) -> bool {
        self.shut_for <= 0.0
    }

    /// Advance by `dt` with something in it or not. Returns whether it bit,
    /// and whether it changed between open and shut.
    fn tick(&mut self, dt: f32, occupied: bool) -> (bool, bool) {
        if self.is_open() {
            if occupied {
                self.shut_for = self.hold;
                return (true, true);
            }
            return (false, false);
        }
        self.shut_for = (self.shut_for - dt).max(0.0);
        (false, self.is_open())
    }
}

/// Springs and resets every trap.
pub struct SnapTrapSystem;

impl<'a> System<'a> for SnapTrapSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Perception>,
        WriteStorage<'a, SnapTrap>,
        WriteStorage<'a, ModelInstance>,
        Write<'a, DamageQueue>,
        ReadStorage<'a, Dead>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, positions, perceptions, mut traps, mut models, mut queue, deads) =
            data;
        for (entity, position, perception, trap, _) in
            (&entities, &positions, &perceptions, &mut traps, !&deads).join()
        {
            let victim = perception.target.filter(|target| {
                target.visible
                    && target.distance <= trap.radius
                    && (target.position.y - position.0.y).abs() <= DEPTH
            });
            let (bit, changed) = trap.tick(time.delta_seconds(), victim.is_some());
            if let Some(victim) = victim.filter(|_| bit) {
                queue.push(victim.entity, trap.damage, DamageKind::Melee);
            }
            if changed {
                let look = if trap.is_open() {
                    trap.open.clone()
                } else {
                    trap.shut.clone()
                };
                // The trap's own entity, alive in this join.
                let _ = models.insert(entity, ModelInstance::new(look));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn trap() -> SnapTrap {
        let empty = Arc::new(Model::flat(Vec::new()));
        SnapTrap {
            radius: 0.6,
            damage: 30.0,
            hold: 2.0,
            shut_for: 0.0,
            open: empty.clone(),
            shut: empty,
        }
    }

    #[test]
    fn it_bites_once_holds_shut_and_then_opens() {
        let mut trap = trap();
        assert_eq!(trap.tick(0.1, false), (false, false), "sprung by nothing");
        assert_eq!(trap.tick(0.1, true), (true, true));
        assert!(!trap.is_open());
        // Shut, it does not bite again whoever is still in it.
        assert_eq!(trap.tick(1.0, true), (false, false));
        assert_eq!(trap.tick(1.5, true), (false, true), "it should have opened");
        assert!(trap.is_open());
        // And is ready again.
        assert_eq!(trap.tick(0.1, true), (true, true));
    }
}
