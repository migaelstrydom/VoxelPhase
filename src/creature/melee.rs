//! Hitting something at close range, with the whole body.
//!
//! ```text
//!   Brain (Attacking) ─► MeleeAttackSystem ─► StrikePhase ─► whatever draws the creature
//!                              │
//!                              ├─► CharacterIntent: planted through the wind-up
//!                              │     and the recovery, and between them a rush
//!                              │     along its facing as fast as it can drive
//!                              │     (`sprint`); left to the brain otherwise
//!                              └─► DamageQueue: hit points off a target with
//!                                    health that its rush would meet
//! ```
//!
//! The brain decides *that* a creature is fighting; this decides when it
//! throws its blow. The blow itself is not this system's: it is the
//! striker's body meeting whatever is in front of it ([`Rush`]), a hit
//! like any other, which harm judges. So it lands as hard as the striker's
//! weight and speed make it, on whatever is in the way, and the striker is
//! pushed back by what it gives. Keeping the timing apart is what lets the
//! same swing sit under a peck, a headbutt or a bite: only the rig that
//! reads [`MeleeAttack::thrust`] knows which it is.

use nalgebra::Vector3;
use specs::{
    Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, Write, WriteStorage,
};

use super::brain::{Behaviour, Brain};
use super::perception::{PerceivedTarget, Perception};
use super::rush::Rush;
use crate::character::{facing_from_rotation, CharacterIntent, CharacterState};
use crate::components::{RigidBodyComponent, Rotation};
use crate::damage::{DamageKind, DamageQueue};
use crate::debug::DebugLog;
use crate::physics::bench_harness::geometry::EmptyGeometry;
use crate::physics::StaticGeometry;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::time::Time;

/// How squarely a striker faces its target before it throws its blow, as
/// the cosine of the angle it is off it: within 10°. Glancing, a rush
/// meets its target with a fraction of its speed.
const SQUARED_UP: f32 = 0.985;

/// How far into its rush, as a share of it, the striking part is thrown
/// fully out: as its body arrives, before it drives on.
const THROWN_OUT_BY: f32 = 0.75;

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
    /// The blow thrown: it drives its body along `along` (level, unit
    /// length), the way it faced as it threw it, for `seconds`. `elapsed`
    /// counts up.
    Rushing {
        elapsed: f32,
        seconds: f32,
        along: Vector3<f32>,
    },
    /// The blow over, hit or miss. `remaining` counts down.
    Recovering { remaining: f32 },
}

/// A creature's close-range attack.
#[derive(Component, Debug, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct MeleeAttack {
    /// Hit points it takes off a target with health that its rush meets.
    pub damage: f32,
    /// What it strikes with: its body, rushed along its facing.
    pub rush: Rush,
    /// Centre-to-centre distance to its target within which it throws its
    /// blow, if its rush would meet it.
    pub reach: f32,
    /// Seconds from committing to the blow. This is the warning.
    pub windup: f32,
    /// Seconds after its rush before another blow can start.
    pub recovery: f32,
    pub phase: StrikePhase,
}

impl MeleeAttack {
    pub fn new(damage: f32, reach: f32, rush: Rush) -> Self {
        Self {
            damage,
            rush,
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

    /// Whether the creature is mid-swing, and so has given up choosing
    /// where it goes and which way it faces until it is over.
    pub fn is_committed(&self) -> bool {
        !matches!(self.phase, StrikePhase::Ready)
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
            StrikePhase::Rushing {
                elapsed, seconds, ..
            } => (2.0 * elapsed / (THROWN_OUT_BY * seconds) - 1.0).clamp(-1.0, 1.0),
            StrikePhase::Recovering { remaining } => (remaining / self.recovery).clamp(0.0, 1.0),
        }
    }
}

/// Runs every creature's swing timer and throws its blows.
///
/// Runs after `BrainSystem`, so a creature that decided to attack this
/// frame starts winding up on it — and so it can overrule the intent the
/// brain just wrote, which is how a committed swing plants the creature
/// and then throws it.
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
            mut debug_log,
        ) = data;
        let scenery: &dyn StaticGeometry = match terrain.as_deref() {
            Some(terrain) => terrain,
            None => &EmptyGeometry,
        };
        let dt = time.delta_seconds();
        let mut swinging = 0;

        // Intent is looked up rather than joined on: a creature that swings
        // without walking has no `CharacterIntent`, and joining on one would
        // silently cost it its attack rather than just its footwork.
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
            let striker = bodies
                .get(entity)
                .and_then(|rb| Some((rb.0, physics.world.body(rb.0)?)));
            // The way it faces, level: the way its rush goes. A creature
            // with no front rushes at what it is after.
            let facing = striker.and_then(|(_, body)| {
                rotations
                    .get(entity)
                    .map(|rotation| facing_from_rotation(rotation.0))
                    .or_else(|| level(body.rotation() * Vector3::z()))
                    .or_else(|| level(target?.position - body.position()))
            });
            let rush = attack.rush;
            // The gap a rush thrown now would cross to its target, if it
            // would meet it before anything else.
            let gap = |target: PerceivedTarget| {
                let ((handle, body), facing) = striker.zip(facing)?;
                let (met, gap) = rush.meets(
                    &physics.world,
                    scenery,
                    handle,
                    body.position(),
                    facing,
                    target.position.y,
                )?;
                (Some(met) == bodies.get(target.entity).map(|rb| rb.0)).then_some(gap)
            };
            // Whether it faces its target squarely.
            let squared = |target: PerceivedTarget| {
                let toward = striker.and_then(|(_, body)| level(target.position - body.position()));
                toward
                    .zip(facing)
                    .is_some_and(|(toward, facing)| toward.dot(&facing) >= SQUARED_UP)
            };
            // A blow is begun only where its rush would meet its target
            // square on, up to speed. Off it, the creature squares up to it
            // first (its brain faces it at its target), and too near it
            // backs off to its standoff (its brain keeps it there) to get
            // its run-up.
            let in_reach = committed
                && !attack.is_committed()
                && target.is_some_and(|target| {
                    target.distance <= attack.reach
                        && squared(target)
                        && gap(target).is_some_and(|gap| gap >= rush.nearest())
                });

            // Thrown, it drives for as long as the gap to its target takes
            // to cross; at the air, as far as it carries.
            let along = facing.unwrap_or_else(Vector3::zeros);
            let seconds = || {
                let across = target.and_then(|target| gap(target)).unwrap_or(rush.carry);
                rush.seconds_across(across)
            };
            let (phase, thrown) = advance(*attack, committed, in_reach, along, seconds, dt);
            attack.phase = phase;
            if thrown {
                if let Some(target) = target.filter(|&target| gap(target).is_some()) {
                    queue.push(target.entity, attack.damage, DamageKind::Melee);
                }
            }

            if let Some(intent) = intents.get_mut(entity) {
                match attack.phase {
                    // It throws its body the way it faced as it threw the blow,
                    // as fast as it can drive it, and does not turn: stepping
                    // aside is what beats it.
                    StrikePhase::Rushing { along, .. } => {
                        intent.direction = along;
                        intent.sprint = true;
                        intent.face = None;
                    }
                    // A swing costs the creature its feet: it neither closes nor
                    // circles while it winds up or recovers.
                    StrikePhase::Winding { .. } | StrikePhase::Recovering { .. } => {
                        intent.direction = Vector3::zeros();
                        intent.face = None;
                    }
                    StrikePhase::Ready => {}
                }
            }
            if attack.is_committed() {
                swinging += 1;
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

/// Advance one swing, for a striker facing `along` (level, unit length)
/// whose rush, thrown now, lasts `seconds()`. Returns the next phase and
/// whether its blow was thrown on this step.
///
/// Pure, so the timing is testable without a world.
fn advance(
    attack: MeleeAttack,
    committed: bool,
    in_reach: bool,
    along: Vector3<f32>,
    seconds: impl FnOnce() -> f32,
    dt: f32,
) -> (StrikePhase, bool) {
    let recovering = StrikePhase::Recovering {
        remaining: attack.recovery,
    };
    match attack.phase {
        StrikePhase::Ready if committed && in_reach => {
            (StrikePhase::Winding { elapsed: 0.0 }, false)
        }
        StrikePhase::Ready => (StrikePhase::Ready, false),
        StrikePhase::Winding { elapsed } if elapsed + dt >= attack.windup => (
            StrikePhase::Rushing {
                elapsed: 0.0,
                seconds: seconds(),
                along,
            },
            true,
        ),
        StrikePhase::Winding { elapsed } => (
            StrikePhase::Winding {
                elapsed: elapsed + dt,
            },
            false,
        ),
        StrikePhase::Rushing {
            elapsed, seconds, ..
        } if elapsed + dt >= seconds => (recovering, false),
        StrikePhase::Rushing {
            elapsed,
            seconds,
            along,
        } => (
            StrikePhase::Rushing {
                elapsed: elapsed + dt,
                seconds,
                along,
            },
            false,
        ),
        StrikePhase::Recovering { remaining } if remaining <= dt => (StrikePhase::Ready, false),
        StrikePhase::Recovering { remaining } => (
            StrikePhase::Recovering {
                remaining: remaining - dt,
            },
            false,
        ),
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;
    use crate::character::locomotion::Down;
    use crate::character::LocomotionState;
    use crate::physics::{ColliderDesc, RigidBodyDesc, RigidBodyHandle};

    /// A body 0.6 m across rushing 0.8 m to 4 m/s, which it is up to after
    /// 0.4 m: it rushes across no less than 0.2 m.
    const RUSH: Rush = Rush {
        speed: 4.0,
        grip: 20.0,
        carry: 0.8,
        radius: 0.3,
        half_height: 0.3,
    };

    /// [`advance`], for a rush that lasts 0.2 s.
    fn advance(
        attack: MeleeAttack,
        committed: bool,
        in_reach: bool,
        along: Vector3<f32>,
        dt: f32,
    ) -> (StrikePhase, bool) {
        super::advance(attack, committed, in_reach, along, || 0.2, dt)
    }

    fn an_attack() -> MeleeAttack {
        MeleeAttack::new(10.0, 2.0, RUSH).with_timing(0.4, 0.5)
    }

    #[test]
    fn a_strike_needs_both_the_intent_and_the_range() {
        let attack = an_attack();
        let along = Vector3::z();
        assert_eq!(
            advance(attack, true, false, along, 0.1).0,
            StrikePhase::Ready
        );
        assert_eq!(
            advance(attack, false, true, along, 0.1).0,
            StrikePhase::Ready
        );
        assert!(matches!(
            advance(attack, true, true, along, 0.1).0,
            StrikePhase::Winding { .. }
        ));
    }

    /// The wind-up over, the blow is thrown once: the striker rushes the
    /// way it faced then for the rush's seconds, and then recovers.
    #[test]
    fn a_blow_is_thrown_once_as_a_rush_the_way_it_faced_then_recovered_from() {
        let mut attack = an_attack();
        attack.phase = StrikePhase::Winding { elapsed: 0.39 };
        let along = Vector3::x();

        let (phase, thrown) = advance(attack, true, true, along, 0.05);
        assert!(thrown, "the wind-up must expire into a blow");
        assert_eq!(
            phase,
            StrikePhase::Rushing {
                elapsed: 0.0,
                seconds: 0.2,
                along
            }
        );
        attack.phase = phase;

        let (phase, thrown) = advance(attack, true, true, Vector3::z(), 0.15);
        assert!(!thrown, "a rush is one blow");
        assert!(
            matches!(phase, StrikePhase::Rushing { along: kept, .. } if kept == along),
            "it rushes the way it faced as it threw: {phase:?}"
        );
        attack.phase = phase;

        let (phase, thrown) = advance(attack, true, true, along, 0.1);
        assert!(!thrown);
        assert_eq!(phase, StrikePhase::Recovering { remaining: 0.5 });
    }

    #[test]
    fn a_swing_cannot_be_called_off_once_committed() {
        let mut attack = an_attack();
        attack.phase = StrikePhase::Winding { elapsed: 0.1 };
        attack.phase = advance(attack, false, false, Vector3::z(), 0.05).0;
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

        let rushing = |elapsed| StrikePhase::Rushing {
            elapsed,
            seconds: 0.4,
            along: Vector3::z(),
        };
        attack.phase = rushing(0.0);
        assert_eq!(attack.thrust(), -1.0, "it rushes from drawn back");
        attack.phase = rushing(0.15);
        assert!(attack.thrust().abs() < 1e-6, "through where it rests");
        attack.phase = rushing(0.3);
        assert_eq!(attack.thrust(), 1.0, "thrown out as its body arrives");
        attack.phase = rushing(0.4);
        assert_eq!(attack.thrust(), 1.0, "and as it drives on");

        attack.phase = StrikePhase::Recovering { remaining: 0.5 };
        assert!(attack.thrust() > 0.9, "and recovers from there");

        attack.phase = StrikePhase::Ready;
        assert_eq!(attack.thrust(), 0.0);
    }

    /// What a striker asks of its body.
    #[derive(Debug)]
    struct Asked {
        direction: Vector3<f32>,
        sprint: bool,
        face: Option<Vector3<f32>>,
    }

    fn ball(physics: &mut PhysicsResource, at: Point3<f32>) -> RigidBodyHandle {
        let body = physics
            .world
            .create_body(RigidBodyDesc::dynamic().position(at));
        physics
            .world
            .attach_collider(body, ColliderDesc::sphere(0.3).density(1000.0));
        body
    }

    /// A striker at the origin facing +z, attacking a ball at `at` from
    /// `phase` with its body `locomotion`, after the system has run once:
    /// where it is in its swing, and what it asks of its body.
    fn struck_from(
        at: Point3<f32>,
        phase: StrikePhase,
        locomotion: LocomotionState,
    ) -> (StrikePhase, Asked) {
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
            .with(MeleeAttack {
                phase,
                ..an_attack()
            })
            .with(CharacterIntent {
                direction: Vector3::x(),
                face: Some(Vector3::x()),
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
        let intents = world.read_storage::<CharacterIntent>();
        let intent = intents.get(striker).unwrap();
        let asked = Asked {
            direction: intent.direction,
            sprint: intent.sprint,
            face: intent.face,
        };
        (phase, asked)
    }

    /// [`struck_from`], ready and on its feet.
    fn ready_at(at: Point3<f32>) -> (StrikePhase, Asked) {
        struck_from(at, StrikePhase::Ready, LocomotionState::default())
    }

    /// With its target in front, where its rush would meet it, a ready
    /// striker begins its swing and stands to throw it.
    #[test]
    fn a_striker_winds_up_at_a_target_its_rush_would_meet() {
        let (phase, intent) = ready_at(Point3::new(0.0, 0.0, 0.9));
        assert!(matches!(phase, StrikePhase::Winding { .. }), "{phase:?}");
        assert_eq!(intent.direction, Vector3::zeros());
    }

    /// With its target beside it, within its reach but not where its rush
    /// would meet it, a ready striker does not rush at the air, and goes
    /// where its brain takes it.
    #[test]
    fn beside_its_target_a_striker_does_not_rush_the_air() {
        let (phase, intent) = ready_at(Point3::new(-0.9, 0.0, 0.0));
        assert_eq!(phase, StrikePhase::Ready);
        assert_eq!(intent.direction, Vector3::x());
    }

    /// With its target within its rush but off square, a ready striker
    /// does not rush it a glancing blow, and goes where its brain takes it
    /// (round to face it).
    #[test]
    fn off_square_a_striker_does_not_rush_its_target() {
        let (phase, intent) = ready_at(Point3::new(0.4, 0.0, 0.9));
        assert_eq!(phase, StrikePhase::Ready);
        assert_eq!(intent.direction, Vector3::x());
    }

    /// With its target in front but too near for its body to get up to
    /// speed, a ready striker does not rush it, and goes where its brain
    /// takes it (back to where it can).
    #[test]
    fn too_near_its_target_a_striker_does_not_rush_it() {
        let (phase, intent) = ready_at(Point3::new(0.0, 0.0, 0.7));
        assert_eq!(phase, StrikePhase::Ready);
        assert_eq!(intent.direction, Vector3::x());
    }

    /// Rushing, a striker drives its body the way it faced as it threw the
    /// blow, as fast as it can, wherever its target has gone.
    #[test]
    fn a_rushing_striker_drives_the_way_it_threw_as_fast_as_it_can() {
        let along = Vector3::new(0.6, 0.0, 0.8);
        let rushing = StrikePhase::Rushing {
            elapsed: 0.0,
            seconds: 0.2,
            along,
        };
        let (phase, intent) = struck_from(
            Point3::new(-0.9, 0.0, 0.0),
            rushing,
            LocomotionState::default(),
        );
        assert!(matches!(phase, StrikePhase::Rushing { .. }), "{phase:?}");
        assert_eq!(intent.direction, along);
        assert!(intent.sprint, "{intent:?}");
        assert_eq!(intent.face, None);
    }

    /// Knocked down, a striker throws nothing, though its rush would meet
    /// its target.
    #[test]
    fn a_striker_knocked_down_strikes_at_nothing() {
        let down = LocomotionState::Down(Down::knocked(&LocomotionState::default(), 2.0));
        let (phase, _) = struck_from(Point3::new(0.0, 0.0, 0.9), StrikePhase::Ready, down);
        assert_eq!(phase, StrikePhase::Ready);
    }
}
