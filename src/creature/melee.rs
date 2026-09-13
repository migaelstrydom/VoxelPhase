//! Hitting something at close range.
//!
//! ```text
//!   Brain (Attacking) ─► MeleeAttackSystem ─► DamageQueue
//!                              │
//!                              └─► StrikePhase ─► whatever draws the creature
//! ```
//!
//! The brain decides *that* a creature is fighting; this decides when a
//! blow actually lands. Keeping them apart is what lets the same swing
//! timing sit under a peck, a headbutt or a claw: only the rig that reads
//! [`MeleeAttack::thrust`] knows which it is.
//!
//! Like every other damage source, this one is a pure producer — it pushes
//! events and never touches `Health`.

use nalgebra::Vector3;
use specs::{
    Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, Write, WriteStorage,
};

use super::brain::{Behaviour, Brain};
use super::perception::Perception;
use crate::character::CharacterIntent;
use crate::damage::{DamageKind, DamageQueue};
use crate::debug::DebugLog;
use crate::time::Time;

/// Where a strike is in its swing.
///
/// The wind-up is the whole point: a blow that lands the instant a
/// creature is in range is unreadable, and a player who cannot see it
/// coming cannot do anything but take it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StrikePhase {
    /// Free to commit the moment something comes into reach.
    Ready,
    /// Committed, and cannot be called off. `elapsed` counts up.
    Winding { elapsed: f32 },
    /// The blow has been thrown, hit or miss. `remaining` counts down.
    Recovering { remaining: f32 },
}

/// A creature's close-range attack.
#[derive(Component, Debug, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct MeleeAttack {
    /// Hit points a connecting blow takes off.
    pub damage: f32,
    /// Centre-to-centre distance a blow connects within. Not the length of
    /// the limb that throws it — it has to cover the target's own radius
    /// too, or a creature standing against the player misses.
    pub reach: f32,
    /// Seconds from committing to the blow landing. This is the warning.
    pub windup: f32,
    /// Seconds after a blow before another can start.
    pub recovery: f32,
    pub phase: StrikePhase,
}

impl MeleeAttack {
    /// Whether the creature is mid-swing, and so has given up moving and
    /// turning until it is over.
    pub fn is_committed(&self) -> bool {
        !matches!(self.phase, StrikePhase::Ready)
    }
}

impl MeleeAttack {
    pub fn new(damage: f32, reach: f32) -> Self {
        Self {
            damage,
            reach,
            windup: 0.45,
            recovery: 0.8,
            phase: StrikePhase::Ready,
        }
    }

    /// Override the swing timing. A long wind-up and a short recovery is a
    /// heavy, dodgeable blow; the reverse is a jab that punishes standing
    /// still.
    pub fn with_timing(mut self, windup: f32, recovery: f32) -> Self {
        self.windup = windup.max(1e-3);
        self.recovery = recovery.max(1e-3);
        self
    }

    /// How far through the swing the striking part is, in `[-1, 1]`:
    /// negative while it is drawn back, positive while it is thrown out,
    /// zero at rest.
    ///
    /// One number rather than an exposed phase, because a rig should be
    /// able to animate a strike without knowing what a strike *is*.
    pub fn thrust(&self) -> f32 {
        match self.phase {
            StrikePhase::Ready => 0.0,
            StrikePhase::Winding { elapsed } => -(elapsed / self.windup).clamp(0.0, 1.0),
            StrikePhase::Recovering { remaining } => (remaining / self.recovery).clamp(0.0, 1.0),
        }
    }
}

/// Runs every creature's swing timer and lands the blows.
///
/// Runs after `BrainSystem`, so a creature that decided to attack this
/// frame starts winding up on it — and so it can overrule the intent the
/// brain just wrote, which is how a committed swing plants the creature.
pub struct MeleeAttackSystem;

impl<'a> System<'a> for MeleeAttackSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        ReadStorage<'a, Brain>,
        ReadStorage<'a, Perception>,
        WriteStorage<'a, MeleeAttack>,
        WriteStorage<'a, CharacterIntent>,
        Write<'a, DamageQueue>,
        Write<'a, DebugLog>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            time,
            brains,
            perceptions,
            mut attacks,
            mut intents,
            mut queue,
            mut debug_log,
        ) = data;
        let dt = time.delta_seconds();
        let mut swinging = 0;

        // Intent is looked up rather than joined on: a creature that swings
        // without walking — something anchored, or a roller that headbutts —
        // has no `CharacterIntent`, and joining on one would silently cost it
        // its attack rather than just its footwork.
        for (entity, brain, perception, attack) in
            (&entities, &brains, &perceptions, &mut attacks).join()
        {
            let target = perception.target.filter(|target| target.visible);
            let in_reach = target.is_some_and(|target| target.distance <= attack.reach);
            let committed = matches!(brain.behaviour, Behaviour::Attacking { .. });

            let (phase, connected) = advance(*attack, committed, in_reach, dt);
            attack.phase = phase;

            if connected {
                // Reach is re-tested at the moment of the blow, not at the
                // moment of committing: stepping out of a wind-up is the
                // player's half of the exchange.
                if let Some(target) = target.filter(|_| in_reach) {
                    queue.push(target.entity, attack.damage, DamageKind::Melee);
                }
            }
            if attack.is_committed() {
                // A swing costs the creature its feet. It neither closes nor
                // circles until the blow is over, and since the shared
                // control chain faces a character where it steers, it does
                // not turn either: the strike lands where it was aimed when
                // it was thrown, and stepping aside is what beats it.
                if let Some(intent) = intents.get_mut(entity) {
                    intent.direction = Vector3::zeros();
                }
                swinging += 1;
            }
        }

        if swinging > 0 {
            debug_log.add("Creature/Striking", swinging.to_string());
        }
    }
}

/// Advance one swing. Returns the next phase and whether a blow was thrown
/// on this step.
///
/// Pure, so the timing is testable without a world.
fn advance(attack: MeleeAttack, committed: bool, in_reach: bool, dt: f32) -> (StrikePhase, bool) {
    match attack.phase {
        StrikePhase::Ready => {
            if committed && in_reach {
                (StrikePhase::Winding { elapsed: 0.0 }, false)
            } else {
                (StrikePhase::Ready, false)
            }
        }
        StrikePhase::Winding { elapsed } => {
            let elapsed = elapsed + dt;
            if elapsed >= attack.windup {
                (
                    StrikePhase::Recovering {
                        remaining: attack.recovery,
                    },
                    true,
                )
            } else {
                (StrikePhase::Winding { elapsed }, false)
            }
        }
        StrikePhase::Recovering { remaining } => {
            let remaining = remaining - dt;
            if remaining <= 0.0 {
                (StrikePhase::Ready, false)
            } else {
                (StrikePhase::Recovering { remaining }, false)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn an_attack() -> MeleeAttack {
        MeleeAttack::new(10.0, 2.0).with_timing(0.4, 0.5)
    }

    #[test]
    fn a_strike_needs_both_the_intent_and_the_range() {
        let attack = an_attack();
        assert_eq!(advance(attack, true, false, 0.1).0, StrikePhase::Ready);
        assert_eq!(advance(attack, false, true, 0.1).0, StrikePhase::Ready);
        assert!(matches!(
            advance(attack, true, true, 0.1).0,
            StrikePhase::Winding { .. }
        ));
    }

    #[test]
    fn a_committed_swing_lands_exactly_once() {
        let mut attack = an_attack();
        attack.phase = StrikePhase::Winding { elapsed: 0.39 };

        let (phase, connected) = advance(attack, true, true, 0.05);
        assert!(connected, "the wind-up must expire into a blow");
        attack.phase = phase;

        let (_, connected_again) = advance(attack, true, true, 0.05);
        assert!(!connected_again, "recovery must not land a second blow");
    }

    #[test]
    fn a_swing_cannot_be_called_off_once_committed() {
        let mut attack = an_attack();
        attack.phase = StrikePhase::Winding { elapsed: 0.1 };
        attack.phase = advance(attack, false, false, 0.05).0;
        assert!(
            matches!(attack.phase, StrikePhase::Winding { .. }),
            "a target that steps away must not rewind the swing"
        );
    }

    #[test]
    fn thrust_reads_back_then_forward() {
        let mut attack = an_attack();
        attack.phase = StrikePhase::Winding { elapsed: 0.4 };
        assert!(attack.thrust() < -0.9, "a full wind-up is fully drawn back");

        attack.phase = StrikePhase::Recovering { remaining: 0.5 };
        assert!(attack.thrust() > 0.9, "the blow starts fully thrown out");

        attack.phase = StrikePhase::Ready;
        assert_eq!(attack.thrust(), 0.0);
    }
}
