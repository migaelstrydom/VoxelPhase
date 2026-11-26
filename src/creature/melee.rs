//! Hitting something at close range.
//!
//! ```text
//!   Brain (Attacking) ─► MeleeAttackSystem ─► DamageQueue: hit points off
//!                              │                its target's health
//!                              ├─► PhysicsImpulseQueue: a strike on the
//!                              │     first body its swing touches, which
//!                              │     the step gives it and records as a
//!                              │     shove (harm reads it)
//!                              └─► StrikePhase ─► whatever draws the creature
//! ```
//!
//! The brain decides *that* a creature is fighting; this decides when a
//! blow actually lands. Keeping them apart is what lets the same swing
//! timing sit under a peck, a headbutt or a claw: only the rig that reads
//! [`MeleeAttack::thrust`] knows which it is.
//!
//! Like every other damage source, this one is a pure producer — it pushes
//! events and never touches `Health` or a body.

use nalgebra::Vector3;
use specs::{
    Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, Write, WriteStorage,
};

use super::brain::{Behaviour, Brain};
use super::perception::{PerceivedTarget, Perception};
use super::swing::Swing;
use crate::character::{facing_from_rotation, CharacterIntent, CharacterState};
use crate::components::{RigidBodyComponent, Rotation};
use crate::damage::{DamageKind, DamageQueue};
use crate::debug::DebugLog;
use crate::physics::bench_harness::geometry::EmptyGeometry;
use crate::physics::{PhysicsImpulse, PhysicsImpulseQueue, StaticGeometry};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
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
    /// What its blow sweeps and how hard it lands on the body it touches:
    /// what knocks a body off its feet. `None` for a blow that only wounds
    /// its target, whatever is in the way.
    pub swing: Option<Swing>,
    /// Centre-to-centre distance to its target within which it swings, and
    /// a blow without a swing connects. Not the length of the limb that
    /// throws it — it has to cover the target's own radius too, or a
    /// creature standing against the player never swings.
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
            swing: None,
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

    /// Land each blow on the first body `swing` touches.
    pub fn with_swing(mut self, swing: Swing) -> Self {
        self.swing = Some(swing);
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
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, CharacterState>,
        ReadStorage<'a, Rotation>,
        Read<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
        Write<'a, DamageQueue>,
        Write<'a, PhysicsImpulseQueue>,
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
            bodies,
            states,
            rotations,
            physics,
            terrain,
            mut queue,
            mut impulses,
            mut debug_log,
        ) = data;
        let scenery: &dyn StaticGeometry = match terrain.as_deref() {
            Some(terrain) => terrain,
            None => &EmptyGeometry,
        };
        let dt = time.delta_seconds();
        let mut swinging = 0;

        // Intent is looked up rather than joined on: a creature that swings
        // without walking — something anchored, or a roller that headbutts —
        // has no `CharacterIntent`, and joining on one would silently cost it
        // its attack rather than just its footwork.
        for (entity, brain, perception, attack) in
            (&entities, &brains, &perceptions, &mut attacks).join()
        {
            // A creature knocked down drops whatever blow it was throwing.
            if states
                .get(entity)
                .is_some_and(|state| !state.locomotion.in_control())
            {
                attack.phase = StrikePhase::Ready;
                continue;
            }
            // A body already knocked down is not struck: the creature waits
            // for it to get up, which it can then step away from.
            let target = perception.target.filter(|target| {
                target.visible
                    && !states
                        .get(target.entity)
                        .is_some_and(|state| state.locomotion.is_down())
            });
            let committed = matches!(brain.behaviour, Behaviour::Attacking { .. });
            let striker = bodies.get(entity).map(|body| body.0);
            // The blow it would land if it threw it now: on whatever its
            // swing touches first, thrown the way its body faces. A creature
            // with no front strikes at what it is after.
            let strike = |swing: &Swing| {
                let striker = striker?;
                let body = physics.world.body(striker)?;
                let centre = body.position();
                let facing = rotations
                    .get(entity)
                    .map(|rotation| facing_from_rotation(rotation.0))
                    .or_else(|| level(body.rotation() * Vector3::z()))
                    .or_else(|| level(target?.position - centre))?;
                swing.land(&physics.world, scenery, striker, centre, facing)
            };
            let on = |strike: &PhysicsImpulse, target: PerceivedTarget| {
                let PhysicsImpulse::Strike { body: struck, .. } = *strike else {
                    return false;
                };
                bodies.get(target.entity).map(|body| body.0) == Some(struck)
            };
            let in_reach = target.is_some_and(|target| {
                target.distance <= attack.reach
                    && match attack.swing {
                        // A swing is begun only where it would land on its
                        // target: beside it or under its reach, the creature
                        // squares up to it (its brain faces it at its target)
                        // rather than striking the air.
                        Some(swing) if committed && !attack.is_committed() => {
                            strike(&swing).is_some_and(|strike| on(&strike, target))
                        }
                        _ => true,
                    }
            });

            let (phase, connected) = advance(*attack, committed, in_reach, dt);
            attack.phase = phase;

            // Whether it lands is decided at the moment of the blow, not at
            // the moment of committing: stepping out of a wind-up is the
            // player's half of the exchange.
            if connected {
                match attack.swing {
                    Some(swing) => {
                        if let Some(strike) = strike(&swing) {
                            if let Some(target) = target.filter(|&target| on(&strike, target)) {
                                queue.push(target.entity, attack.damage, DamageKind::Melee);
                            }
                            impulses.push(strike);
                        }
                    }
                    None => {
                        if let Some(target) = target.filter(|_| in_reach) {
                            queue.push(target.entity, attack.damage, DamageKind::Melee);
                        }
                    }
                }
            }
            if attack.is_committed() {
                // A swing costs the creature its feet. It neither closes nor
                // circles until the blow is over, and it does not turn
                // either: the strike lands where it was aimed when it was
                // thrown, and stepping aside is what beats it.
                if let Some(intent) = intents.get_mut(entity) {
                    intent.direction = Vector3::zeros();
                    intent.face = None;
                }
                swinging += 1;
            } else if committed && attack.swing.is_some() && !in_reach {
                // Ready, but its blow would not land from where it stands:
                // circling just out of its reach, or with its target beside
                // it. It steps in at it, squared up, until it would.
                let toward = target.zip(striker.and_then(|striker| physics.world.body(striker)));
                let toward =
                    toward.and_then(|(target, body)| level(target.position - body.position()));
                if let Some((intent, toward)) = intents.get_mut(entity).zip(toward) {
                    intent.direction = toward;
                }
            }
        }

        if swinging > 0 {
            debug_log.add("Creature/Striking", swinging.to_string());
        }
    }
}

/// `v` laid level, unit length; `None` if it is all but vertical.
fn level(v: Vector3<f32>) -> Option<Vector3<f32>> {
    Vector3::new(v.x, 0.0, v.z).try_normalize(1e-4)
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
    use nalgebra::{Point3, Vector2};
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;
    use crate::character::locomotion::Down;
    use crate::character::LocomotionState;
    use crate::physics::{ColliderDesc, RigidBodyDesc, RigidBodyHandle};

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

    /// A peck: from just above the striker's centre to 0.8 m ahead of it.
    const PECK: Swing = Swing {
        from: Vector2::new(0.0, 0.3),
        to: Vector2::new(0.8, 0.3),
        radius: 0.2,
        impulse: 300.0,
    };

    fn ball(physics: &mut PhysicsResource, at: Point3<f32>) -> RigidBodyHandle {
        let body = physics
            .world
            .create_body(RigidBodyDesc::dynamic().position(at));
        physics
            .world
            .attach_collider(body, ColliderDesc::sphere(0.3).density(1000.0));
        body
    }

    /// A striker at the origin facing +z, ready and attacking a ball at
    /// `at`, after the system has run once: where it is in its swing, and
    /// the way it steps.
    fn ready_at(at: Point3<f32>) -> (StrikePhase, Vector3<f32>) {
        ready_in(at, LocomotionState::default())
    }

    /// [`ready_at`], the striker's body `locomotion`.
    fn ready_in(at: Point3<f32>, locomotion: LocomotionState) -> (StrikePhase, Vector3<f32>) {
        let mut world = World::new();
        System::setup(&mut MeleeAttackSystem, &mut world);
        let mut physics = PhysicsResource::default();
        let (mine, theirs) = (ball(&mut physics, Point3::origin()), ball(&mut physics, at));
        world.insert(physics);

        let target = world
            .create_entity()
            .with(RigidBodyComponent(theirs))
            .build();
        let striker = world
            .create_entity()
            .with(Brain {
                behaviour: Behaviour::Attacking {
                    strafe_clockwise: true,
                    remaining: 1.0,
                },
                ..Brain::hunter(1.0)
            })
            .with(Perception {
                target: Some(PerceivedTarget {
                    entity: target,
                    position: at,
                    distance: at.coords.norm(),
                    visible: true,
                    since_seen: 0.0,
                }),
                ..Perception::ground_creature(20.0)
            })
            .with(MeleeAttack::new(10.0, 2.0).with_swing(PECK))
            .with(CharacterIntent {
                direction: Vector3::x(),
                ..CharacterIntent::default()
            })
            .with(RigidBodyComponent(mine))
            .with(Rotation(0.0))
            .with(CharacterState {
                locomotion,
                ..CharacterState::default()
            })
            .build();

        MeleeAttackSystem.run_now(&world);
        let phase = world
            .read_storage::<MeleeAttack>()
            .get(striker)
            .unwrap()
            .phase;
        let direction = world
            .read_storage::<CharacterIntent>()
            .get(striker)
            .unwrap()
            .direction;
        (phase, direction)
    }

    /// With its target in front, where its blow would land, a ready striker
    /// begins its swing and stands to throw it.
    #[test]
    fn a_striker_swings_at_a_target_its_blow_would_land_on() {
        let (phase, direction) = ready_at(Point3::new(0.0, 0.0, 0.9));
        assert!(matches!(phase, StrikePhase::Winding { .. }), "{phase:?}");
        assert_eq!(direction, Vector3::zeros());
    }

    /// With its target beside it, within its reach but not where its blow
    /// would land, a ready striker does not swing at the air: it steps in
    /// at its target.
    #[test]
    fn beside_its_target_a_striker_steps_in_at_it_rather_than_striking_the_air() {
        let (phase, direction) = ready_at(Point3::new(-0.9, 0.0, 0.0));
        assert_eq!(phase, StrikePhase::Ready);
        assert!((direction + Vector3::x()).norm() < 1e-4, "{direction:?}");
    }

    /// Knocked down, a striker throws nothing, though its blow would land.
    #[test]
    fn a_striker_knocked_down_strikes_at_nothing() {
        let down = LocomotionState::Down(Down::knocked(&LocomotionState::default(), 2.0));
        let (phase, _) = ready_in(Point3::new(0.0, 0.0, 0.9), down);
        assert_eq!(phase, StrikePhase::Ready);
    }
}
