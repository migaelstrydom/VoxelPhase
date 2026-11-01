use nalgebra::{Point3, Vector3};

use super::*;
use crate::character::{Immersion, LocomotionConfig, WaterAtBody};

const DT: f32 = 1.0 / 60.0;

/// One frame's facts and requests.
#[derive(Clone, Copy)]
struct Frame {
    facts: Facts,
    requests: Requests,
}

/// Standing on dry ground at the origin, asking nothing.
fn ground() -> Frame {
    Frame {
        facts: Facts {
            grounded: true,
            immersion: Immersion::dry(),
            position: Point3::new(0.0, 0.5, 0.0),
            velocity: Vector3::zeros(),
            since_contact: 0.0,
        },
        requests: Requests::default(),
    }
}

/// The same, with nothing under it.
fn air() -> Frame {
    let mut frame = ground();
    frame.facts.grounded = false;
    frame
}

/// Sea level at 0, floor `depth` under it.
fn sea(depth: f32) -> Immersion {
    Immersion::in_water(WaterAtBody {
        level: 0.0,
        surface: 0.0,
        floor: -depth,
        current: Vector3::zeros(),
    })
}

/// A landing a metre ahead along +x, `in_seconds` from now.
fn landing_ahead(in_seconds: f32) -> PlannedLanding {
    PlannedLanding {
        foot: Foot::Left,
        at: Point3::new(1.0, 0.0, 0.0),
        in_seconds,
        footfall: None,
        reach: 0.6,
    }
}

const HOP: PlannedTakeoff = PlannedTakeoff {
    planar: Vector3::new(4.0, 0.0, 0.0),
    climb: Some(1.5),
    landing: None,
};

impl Frame {
    fn jump(mut self) -> Self {
        self.requests.jump = true;
        self
    }
    fn long_jump(mut self) -> Self {
        self.requests.move_dir = Vector3::x();
        self.requests.long_jump_armed = true;
        self.jump()
    }
    fn takeoff(mut self, takeoff: PlannedTakeoff) -> Self {
        self.requests.takeoff = Some(takeoff);
        self
    }
    fn moving(mut self, velocity: Vector3<f32>) -> Self {
        self.facts.velocity = velocity;
        self
    }
    /// In the sea `depth` deep, the body's centre at `y`.
    fn in_sea(mut self, depth: f32, y: f32) -> Self {
        self.facts.immersion = sea(depth);
        self.facts.position.y = y;
        self
    }
}

fn advance(from: LocomotionState, frame: Frame) -> (LocomotionState, EntryEffects) {
    let config = LocomotionConfig::player();
    from.advance(&LocomotionInput {
        dt: DT,
        facts: frame.facts,
        requests: frame.requests,
        config: &config,
    })
}

fn on_foot() -> LocomotionState {
    LocomotionState::OnFoot(OnFoot::default())
}
fn grace(left: f32) -> LocomotionState {
    LocomotionState::OnFoot(OnFoot { grace: Some(left) })
}
fn hop(phase: TakeoffPhase, landing: PlannedLanding) -> LocomotionState {
    LocomotionState::Hop(Hop {
        phase,
        landing: Some(landing),
        rise_held: None,
    })
}
fn jump(kind: JumpKind, phase: TakeoffPhase) -> LocomotionState {
    let steering = match kind {
        JumpKind::Jump => AirSteering::Responsive,
        JumpKind::LongJump => AirSteering::Locked {
            velocity: Vector3::new(12.5, 0.0, 0.0),
            remaining: 2.0,
        },
    };
    LocomotionState::Jump(Jump {
        kind,
        phase,
        steering,
    })
}
fn launch() -> TakeoffPhase {
    TakeoffPhase::launch(0.2)
}
const FLIGHT: TakeoffPhase = TakeoffPhase::Flight;

/// Every transition the design's table names, and the ways each state
/// stays put. A body flying toward a landing a metre ahead at 4 m/s comes
/// down a quarter second on.
#[test]
fn the_transitions_are_the_designs() {
    let toward = Vector3::new(4.0, -1.0, 0.0);
    let on_course = hop(FLIGHT, landing_ahead(0.25));
    let rows: Vec<(&str, LocomotionState, Frame, &str)> = vec![
        ("standing", on_foot(), ground(), "on_foot"),
        ("contact lost", on_foot(), air(), "grace"),
        ("contact back in grace", grace(0.05), ground(), "on_foot"),
        ("grace counting down", grace(0.05), air(), "grace"),
        ("grace run out", grace(0.5 * DT), air(), "fall"),
        ("jump pressed", on_foot(), ground().jump(), "jump"),
        ("jump pressed in grace", grace(0.05), air().jump(), "jump"),
        ("long jump in grace", grace(0.05), air().long_jump(), "jump"),
        ("plan's takeoff", on_foot(), ground().takeoff(HOP), "hop"),
        (
            "plan's takeoff in grace",
            grace(0.05),
            air().takeoff(HOP),
            "hop",
        ),
        (
            "a jump wins",
            on_foot(),
            ground().jump().takeoff(HOP),
            "jump",
        ),
        (
            "hop launching",
            hop(launch(), landing_ahead(0.3)),
            ground(),
            "hop",
        ),
        (
            "hop late in launch",
            hop(launch(), landing_ahead(-1.0)),
            ground(),
            "hop",
        ),
        (
            "hop off the ground",
            hop(launch(), landing_ahead(0.3)),
            air(),
            "hop",
        ),
        ("hop on course", on_course, air().moving(toward), "hop"),
        ("hop down", on_course, ground().moving(toward), "on_foot"),
        (
            "hop past its landing",
            hop(FLIGHT, landing_ahead(-0.1)),
            air(),
            "fall",
        ),
        (
            "hop steered away",
            on_course,
            air().moving(Vector3::new(0.0, -1.0, 4.0)),
            "fall",
        ),
        (
            "hop jump pressed",
            on_course,
            air().moving(toward).jump(),
            "hop",
        ),
        (
            "jump launching",
            jump(JumpKind::Jump, launch()),
            ground(),
            "jump",
        ),
        (
            "jump in flight",
            jump(JumpKind::Jump, FLIGHT),
            air(),
            "jump",
        ),
        (
            "jump down",
            jump(JumpKind::Jump, FLIGHT),
            ground(),
            "on_foot",
        ),
        (
            "long jump down",
            jump(JumpKind::LongJump, FLIGHT),
            ground(),
            "on_foot",
        ),
        ("falling", LocomotionState::Fall(Fall), air(), "fall"),
        (
            "fall down",
            LocomotionState::Fall(Fall),
            ground(),
            "on_foot",
        ),
        ("wading in", on_foot(), ground().in_sea(3.0, -0.2), "swim"),
        ("on a pier", on_foot(), ground().in_sea(3.0, 2.5), "on_foot"),
        (
            "falling in",
            LocomotionState::Fall(Fall),
            air().in_sea(3.0, -0.2),
            "swim",
        ),
        (
            "hop into the sea",
            on_course,
            air().in_sea(3.0, -0.2),
            "swim",
        ),
        (
            "jump falling in",
            jump(JumpKind::Jump, FLIGHT),
            air().in_sea(3.0, -0.2).moving(-Vector3::y()),
            "swim",
        ),
        (
            "jump rising out",
            jump(JumpKind::Jump, FLIGHT),
            air().in_sea(3.0, -0.2).moving(Vector3::y()),
            "jump",
        ),
        (
            "swimming",
            LocomotionState::Swim(Swim),
            air().in_sea(3.0, -0.2),
            "swim",
        ),
        (
            "swim jump",
            LocomotionState::Swim(Swim),
            air().in_sea(3.0, -0.2).jump(),
            "swim",
        ),
        (
            "swim at a wall",
            LocomotionState::Swim(Swim),
            ground().in_sea(3.0, -0.2),
            "swim",
        ),
        (
            "feet down wading",
            LocomotionState::Swim(Swim),
            ground().in_sea(0.4, 0.1),
            "on_foot",
        ),
        (
            "afloat in the shallows",
            LocomotionState::Swim(Swim),
            air().in_sea(0.4, 0.1),
            "swim",
        ),
        (
            "swum out over a drop",
            LocomotionState::Swim(Swim),
            air(),
            "fall",
        ),
        (
            "swum ashore",
            LocomotionState::Swim(Swim),
            ground(),
            "on_foot",
        ),
        (
            "risen clear",
            LocomotionState::Swim(Swim),
            air().in_sea(3.0, 0.6),
            "fall",
        ),
    ];
    for (name, from, frame, to) in rows {
        let (next, _) = advance(from, frame);
        assert_eq!(next.tag(), to, "{name}: from {from:?} to {next:?}");
    }
}

/// A jump pressed on the ground launches; in grace it is already flying.
#[test]
fn a_jump_in_grace_is_already_in_flight() {
    let (launched, _) = advance(on_foot(), ground().jump());
    assert!(matches!(
        launched,
        LocomotionState::Jump(Jump {
            phase: TakeoffPhase::Launch { .. },
            ..
        })
    ));
    let (flying, _) = advance(grace(0.05), air().long_jump());
    assert_eq!(flying, jump(JumpKind::LongJump, FLIGHT));
}

/// A jump's impulse and air speed, its press spent and the ground's
/// forgiveness with it; a tap already released is cut up front.
#[test]
fn a_jump_leaves_with_its_impulse() {
    let config = LocomotionConfig::player();
    let (_, effects) = advance(
        on_foot(),
        ground().jump().moving(Vector3::new(7.0, 0.0, 0.0)),
    );
    assert_eq!(
        effects,
        EntryEffects {
            vertical_speed: Some(config.jump_speed),
            air_speed: Some(7.0),
            consume_jump: true,
            cancel_forgiveness: true,
            cut_rise: true,
            arm_crouch_lockout: false,
        }
    );
    let mut held = ground().jump();
    held.requests.jump_held = true;
    let (_, effects) = advance(on_foot(), held);
    assert!(!effects.cut_rise);
    assert_eq!(effects.air_speed, Some(config.walk_speed));
}

/// A long jump is committed: its lock, its low climb, no cut.
#[test]
fn a_long_jump_is_committed() {
    let config = LocomotionConfig::player();
    let (state, effects) = advance(on_foot(), ground().long_jump());
    let speed = config.walk_speed * config.long_jump_speed_mul;
    assert_eq!(
        state,
        LocomotionState::Jump(Jump {
            kind: JumpKind::LongJump,
            phase: TakeoffPhase::launch(config.launch_window),
            steering: AirSteering::Locked {
                velocity: Vector3::new(speed, 0.0, 0.0),
                remaining: config.long_jump_air_lock_duration,
            },
        })
    );
    assert_eq!(
        effects.vertical_speed,
        Some(config.jump_speed * config.long_jump_vertical_mul)
    );
    assert_eq!(effects.air_speed, Some(speed));
    assert!(!effects.cut_rise && !state.allows_jump_cutoff());
}

/// A hop leaves with the plan's climb and planar speed; a roll-off with the
/// climb the body has. Either cancels forgiveness, and spends no jump.
#[test]
fn a_planned_takeoff_leaves_as_planned() {
    let (state, effects) = advance(on_foot(), ground().takeoff(HOP));
    assert!(matches!(
        state,
        LocomotionState::Hop(Hop {
            phase: TakeoffPhase::Launch { .. },
            ..
        })
    ));
    assert_eq!(
        effects,
        EntryEffects {
            vertical_speed: Some(1.5),
            air_speed: Some(4.0),
            cancel_forgiveness: true,
            ..EntryEffects::default()
        }
    );
    let roll_off = PlannedTakeoff { climb: None, ..HOP };
    let (_, effects) = advance(on_foot(), ground().takeoff(roll_off));
    assert_eq!(effects.vertical_speed, None);
    assert_eq!(effects.air_speed, Some(4.0));
}

/// The Hop takes its landing from the takeoff, moves it on the plan's
/// request, and ages it on a quiet frame.
#[test]
fn a_hop_keeps_its_landing_until_the_plan_moves_it() {
    let takeoff = PlannedTakeoff {
        landing: Some(landing_ahead(0.3)),
        ..HOP
    };
    let (state, _) = advance(on_foot(), ground().takeoff(takeoff));
    let landing = |state: LocomotionState| match state {
        LocomotionState::Hop(hop) => hop.landing.expect("a landing"),
        other => panic!("{other:?}"),
    };
    assert_eq!(landing(state), landing_ahead(0.3));

    let flying = air().moving(Vector3::new(4.0, -1.0, 0.0));
    let (quiet, _) = advance(state, flying);
    assert!((landing(quiet).in_seconds - (0.3 - DT)).abs() < 1.0e-6);

    let mut moved = flying;
    moved.requests.landing = Some(landing_ahead(0.2));
    let (moved, _) = advance(quiet, moved);
    assert_eq!(landing(moved), landing_ahead(0.2));
}

/// Landing from a long jump arms the crouch lockout; from a jump it does
/// not. A missed Hop keeps the air speed it took off with; a walk-off
/// latches its own.
#[test]
fn entering_depends_on_the_state_left_only_where_it_must() {
    let (_, effects) = advance(jump(JumpKind::LongJump, FLIGHT), ground());
    assert!(effects.arm_crouch_lockout);
    let (_, effects) = advance(jump(JumpKind::Jump, FLIGHT), ground());
    assert!(!effects.arm_crouch_lockout);

    let (_, effects) = advance(hop(FLIGHT, landing_ahead(-1.0)), air());
    assert_eq!(effects.air_speed, None);
    let (_, effects) = advance(grace(0.5 * DT), air().moving(Vector3::new(7.0, 0.0, 0.0)));
    assert_eq!(effects.air_speed, Some(7.0));
}

/// A state moved on is not entered: grace counting down does nothing.
#[test]
fn a_state_moved_on_has_no_entry_effects() {
    let (state, effects) = advance(on_foot(), air());
    assert_eq!(state.tag(), "grace");
    assert_eq!(effects, EntryEffects::default());
    let (_, effects) = advance(jump(JumpKind::Jump, launch()), air());
    assert_eq!(effects, EntryEffects::default());
}

/// A long jump that never reports leaving the ground must not be held in
/// its launch, where it snaps a committed velocity and handles no input:
/// the window runs out, and the next tick lands it.
#[test]
fn a_launch_cannot_be_held_by_grounding_that_never_goes_false() {
    let (mut state, _) = advance(on_foot(), ground().long_jump());
    let mut landed = false;
    for _ in 0..120 {
        state = advance(state, ground()).0;
        landed |= state == on_foot();
    }
    assert!(landed, "{state:?}");
}

/// The steering lock is a duration: time in the launch counts against it.
#[test]
fn the_committed_steering_lock_decays_during_launch() {
    let config = LocomotionConfig::player();
    let (state, _) = advance(on_foot(), ground().long_jump());
    match advance(state, ground()).0 {
        LocomotionState::Jump(Jump {
            phase: TakeoffPhase::Launch { .. },
            steering: AirSteering::Locked { remaining, .. },
            ..
        }) => assert!(remaining < config.long_jump_air_lock_duration),
        other => panic!("expected a still-locked launch, got {other:?}"),
    }
}

const SPEEDS: RuleSpeeds = RuleSpeeds {
    ground: 5.0,
    air: 6.0,
    air_accel: 8.0,
    swim: 2.0,
    swim_accel: 14.0,
};

/// While locked, the rule asks for no steering authority and keeps the
/// committed velocity.
#[test]
fn a_locked_jump_ignores_the_movement_input() {
    let rule = jump(JumpKind::LongJump, launch()).movement_rule(-Vector3::x(), &SPEEDS);
    assert_eq!(rule.steer_accel, None);
    assert!(rule.target.x > 0.0, "target follows takeoff, not input");
}

/// On foot and in a takeoff's launch, the ground speed; in the air, the air
/// cap; grace cancels the rise; a swimmer strokes at the swim rate.
#[test]
fn each_state_moves_by_its_own_rule() {
    let x = Vector3::x();
    let rule = |state: LocomotionState| state.movement_rule(x, &SPEEDS);
    assert_eq!(rule(on_foot()).target.x, 5.0);
    assert!(!rule(on_foot()).clamp_up);
    assert!(rule(grace(0.05)).clamp_up);
    assert_eq!(rule(grace(0.05)).target.x, 5.0);
    assert_eq!(rule(hop(launch(), landing_ahead(0.3))).target.x, 5.0);
    assert_eq!(rule(hop(FLIGHT, landing_ahead(0.3))).target.x, 6.0);
    assert_eq!(rule(jump(JumpKind::Jump, launch())).target.x, 5.0);
    assert_eq!(rule(jump(JumpKind::Jump, FLIGHT)).target.x, 6.0);
    assert_eq!(rule(LocomotionState::Fall(Fall)).target.x, 6.0);
    let swim = rule(LocomotionState::Swim(Swim));
    assert_eq!((swim.target.x, swim.steer_accel), (2.0, Some(14.0)));
}

/// A roll-off keeps the climb it had for its held time off the ground, and
/// then has its rise cancelled; a hop's rise is its own.
#[test]
fn a_roll_off_keeps_its_rise_only_so_long() {
    let config = LocomotionConfig::player();
    let roll_off = PlannedTakeoff { climb: None, ..HOP };
    let (mut state, _) = advance(on_foot(), ground().takeoff(roll_off));
    let rising = air().moving(Vector3::new(4.0, 1.0, 0.0));
    let mut frames = 0;
    while !state.movement_rule(Vector3::x(), &SPEEDS).clamp_up {
        state = advance(state, rising).0;
        frames += 1;
        assert!(frames < 60, "never cancelled: {state:?}");
    }
    let held = (frames - 1) as f32 * DT;
    assert!(
        (held - config.roll_off_rise_held).abs() <= DT,
        "held {held} s"
    );

    // Asked for off the brow already, it has spent that much of its rise.
    let mut late = ground().takeoff(roll_off);
    late.facts.since_contact = config.roll_off_rise_held;
    let (late, _) = advance(on_foot(), late);
    let late = advance(advance(late, rising).0, rising).0;
    assert!(late.movement_rule(Vector3::x(), &SPEEDS).clamp_up);

    let (mut hop, _) = advance(on_foot(), ground().takeoff(HOP));
    for _ in 0..30 {
        hop = advance(hop, rising).0;
        assert!(!hop.movement_rule(Vector3::x(), &SPEEDS).clamp_up);
    }
}
