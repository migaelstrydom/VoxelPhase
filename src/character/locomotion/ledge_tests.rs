//! The Ledge state on scripted facts: what catches an edge, the settle, the
//! leans that commit and the presses that do not wait, and how a ledge ends.

use nalgebra::{Point3, Unit, Vector3};
use traversal_planner::section::Footing;

use super::*;
use crate::character::ledge::{Edge, SensedEdge, WorldEdge};
use crate::character::{Immersion, LocomotionConfig, WaterAtBody};

const DT: f32 = 1.0 / 60.0;
/// The body's centre: hanging, falling or standing, as each test says.
const CENTRE: Point3<f32> = Point3::new(0.0, 1.0, 0.0);
/// Into the wall, which faces -x.
const INTO: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
const OUT: Vector3<f32> = Vector3::new(-1.0, 0.0, 0.0);
const ALONG: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

/// A terrain lip `lip` metres above the body's centre, its face at x = 0.3,
/// with a top to stand on if `climbable`.
fn lip(lip: f32, climbable: bool) -> SensedEdge {
    let y = CENTRE.y + lip;
    let at = WorldEdge {
        hands: [Point3::new(0.34, y, -0.15), Point3::new(0.34, y, 0.15)],
        outward: Unit::new_normalize(OUT),
        stand: climbable.then(|| Point3::new(0.65, y + 0.5, 0.0)),
    };
    SensedEdge {
        edge: Edge {
            body: None,
            hands: at.hands,
            outward: at.outward,
            stand: at.stand,
            footing: Footing::Fixed,
        },
        at,
        room: [true; 2],
    }
}

/// A hanging grip: the lip above the shoulders.
fn hang_lip() -> SensedEdge {
    lip(0.5, true)
}

#[derive(Clone, Copy)]
struct Frame {
    facts: Facts,
    requests: Requests,
}

fn frame(grounded: bool) -> Frame {
    Frame {
        facts: Facts {
            grounded,
            immersion: Immersion::dry(),
            position: CENTRE,
            velocity: Vector3::zeros(),
            since_contact: 0.0,
        },
        requests: Requests::default(),
    }
}

fn air() -> Frame {
    frame(false)
}

impl Frame {
    fn edge(mut self, edge: SensedEdge) -> Self {
        self.requests.edge = Some(edge);
        self
    }
    fn stick(mut self, dir: Vector3<f32>) -> Self {
        self.requests.move_dir = dir;
        self
    }
    fn jump(mut self) -> Self {
        self.requests.jump = true;
        self
    }
    fn crouch(mut self) -> Self {
        self.requests.drop = true;
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

/// `from` ticked for `seconds` on the same frame.
fn hold(mut state: LocomotionState, frame: Frame, seconds: f32) -> LocomotionState {
    for _ in 0..(seconds / DT).round() as usize {
        state = advance(state, frame).0;
    }
    state
}

/// `from` ticked on the same frame until it leaves the ledge: the state it
/// leaves for, and after how long.
fn until_let_go(mut state: LocomotionState, frame: Frame) -> (LocomotionState, f32) {
    for tick in 1..=120 {
        state = advance(state, frame).0;
        if !matches!(state, LocomotionState::Ledge(_)) {
            return (state, tick as f32 * DT);
        }
    }
    (state, f32::INFINITY)
}

fn ledge(edge: SensedEdge, phase: LedgePhase) -> LocomotionState {
    LocomotionState::Ledge(Ledge {
        edge: edge.edge,
        phase,
    })
}

fn hanging() -> LocomotionState {
    ledge(hang_lip(), LedgePhase::Hang { lean: None })
}

fn phase(state: LocomotionState) -> Option<LedgePhase> {
    match state {
        LocomotionState::Ledge(ledge) => Some(ledge.phase),
        _ => None,
    }
}

fn config() -> crate::character::ledge::LedgeConfig {
    LocomotionConfig::player().ledge
}

#[test]
fn a_fall_catches_the_edge_at_its_hands_and_takes_its_velocity() {
    let (state, effects) = advance(LocomotionState::Fall(Fall), air().edge(hang_lip()));
    assert_eq!(phase(state), Some(LedgePhase::Grab { elapsed: 0.0 }));
    assert_eq!(effects.match_edge, Some(hang_lip()));
    assert!(
        effects.consume_jump,
        "a jump pressed on the way up is spent"
    );
}

#[test]
fn a_hop_and_a_jump_still_launching_keep_their_hands() {
    let hop = LocomotionState::Hop(Hop {
        phase: TakeoffPhase::Flight,
        landing: None,
        rise_held: None,
    });
    assert!(!matches!(
        advance(hop, air().edge(hang_lip())).0,
        LocomotionState::Ledge(_)
    ));
    let launching = LocomotionState::Jump(Jump {
        kind: JumpKind::Jump,
        phase: TakeoffPhase::launch(0.2),
        steering: AirSteering::Responsive,
    });
    assert!(!matches!(
        advance(launching, frame(true).edge(hang_lip())).0,
        LocomotionState::Ledge(_)
    ));
}

#[test]
fn the_water_gives_up_a_swimmer_to_an_edge_and_never_takes_a_hanger() {
    let deep = Immersion::in_water(WaterAtBody {
        level: 2.0,
        surface: 2.0,
        floor: -3.0,
        current: Vector3::zeros(),
    });
    let mut wet = air().edge(hang_lip());
    wet.facts.immersion = deep;
    let (caught, _) = advance(LocomotionState::Swim(Swim), wet);
    assert!(matches!(caught, LocomotionState::Ledge(_)), "{caught:?}");
    assert!(matches!(
        advance(hanging(), wet).0,
        LocomotionState::Ledge(_)
    ));
}

#[test]
fn on_foot_only_a_push_into_a_climbable_edge_at_the_chest_mantles() {
    let chest = lip(0.22, true);
    let mantle = advance(
        LocomotionState::OnFoot(OnFoot::default()),
        frame(true).edge(chest).stick(INTO),
    )
    .0;
    assert_eq!(
        phase(mantle),
        Some(LedgePhase::PullUp {
            elapsed: 0.0,
            duration: config().mantle_time
        })
    );
    let on_foot = LocomotionState::OnFoot(OnFoot::default());
    for (what, frame) in [
        ("stick neutral", frame(true).edge(chest)),
        ("stick along", frame(true).edge(chest).stick(ALONG)),
        (
            "above the shoulders",
            frame(true).edge(hang_lip()).stick(INTO),
        ),
        (
            "nowhere to stand",
            frame(true).edge(lip(0.22, false)).stick(INTO),
        ),
    ] {
        assert_eq!(advance(on_foot, frame).0, on_foot, "{what}");
    }
}

#[test]
fn a_fall_pushing_into_an_edge_at_the_chest_mantles_without_hanging() {
    let state = advance(
        LocomotionState::Fall(Fall),
        air().edge(lip(0.22, true)).stick(INTO),
    )
    .0;
    assert!(matches!(phase(state), Some(LedgePhase::PullUp { .. })));
}

#[test]
fn the_stick_is_ignored_until_the_grab_settles() {
    let grab = ledge(hang_lip(), LedgePhase::Grab { elapsed: 0.0 });
    let away = air().edge(hang_lip()).stick(OUT);
    let settling = hold(grab, away, config().settle - 2.0 * DT);
    assert!(matches!(phase(settling), Some(LedgePhase::Grab { .. })));
    let settled = hold(settling, air().edge(hang_lip()), 2.0 * DT);
    assert_eq!(phase(settled), Some(LedgePhase::Hang { lean: None }));
}

#[test]
fn a_drop_held_away_commits_and_one_let_go_early_does_not() {
    let away = air().edge(hang_lip()).stick(OUT);
    let leaning = hold(hanging(), away, config().commit_hold - 2.0 * DT);
    assert!(
        matches!(
            phase(leaning),
            Some(LedgePhase::Hang {
                lean: Some(Lean {
                    towards: LeanTowards::Drop,
                    ..
                })
            })
        ),
        "{leaning:?}"
    );
    let released = advance(leaning, air().edge(hang_lip())).0;
    assert_eq!(phase(released), Some(LedgePhase::Hang { lean: None }));
    let (dropped, after) = until_let_go(hanging(), away);
    assert_eq!(dropped, LocomotionState::Fall(Fall));
    assert!(
        (after - config().commit_hold).abs() <= DT,
        "dropped after {after} s"
    );
}

#[test]
fn a_pull_up_held_in_commits_only_where_the_body_can_stand() {
    let into = air().edge(hang_lip()).stick(INTO);
    let climbing = hold(hanging(), into, config().commit_hold + DT);
    assert!(matches!(phase(climbing), Some(LedgePhase::PullUp { .. })));

    let narrow = lip(0.5, false);
    let straining = hold(
        ledge(narrow, LedgePhase::Hang { lean: None }),
        air().edge(narrow).stick(INTO),
        1.0,
    );
    assert!(
        matches!(phase(straining), Some(LedgePhase::Hang { lean: Some(_) })),
        "{straining:?}"
    );
}

#[test]
fn a_shimmy_leans_nowhere() {
    let along = hold(hanging(), air().edge(hang_lip()).stick(ALONG), 1.0);
    assert_eq!(phase(along), Some(LedgePhase::Hang { lean: None }));
}

#[test]
fn crouch_drops_at_once_even_while_settling() {
    let grab = ledge(hang_lip(), LedgePhase::Grab { elapsed: 0.0 });
    assert_eq!(
        advance(grab, air().edge(hang_lip()).crouch()).0,
        LocomotionState::Fall(Fall)
    );
}

#[test]
fn a_back_jump_goes_straight_out_unless_the_stick_steers_it() {
    let push = config().back_jump_speed;
    for stick in [Vector3::zeros(), INTO] {
        let (state, effects) = advance(hanging(), air().edge(hang_lip()).stick(stick).jump());
        let LocomotionState::Jump(jump) = state else {
            panic!("{state:?}");
        };
        assert_eq!(jump.kind, JumpKind::BackJump);
        assert_eq!(
            jump.steering,
            AirSteering::Locked {
                velocity: OUT * push,
                remaining: config().back_jump_lock
            },
            "stick {stick}"
        );
        assert_eq!(effects.planar_velocity, Some(OUT * push));
        assert_eq!(
            effects.vertical_speed,
            Some(LocomotionConfig::player().jump_speed)
        );
    }
    for stick in [OUT, ALONG] {
        let (state, _) = advance(hanging(), air().edge(hang_lip()).stick(stick).jump());
        assert!(
            matches!(
                state,
                LocomotionState::Jump(Jump {
                    steering: AirSteering::Responsive,
                    ..
                })
            ),
            "stick {stick}: {state:?}"
        );
    }
}

#[test]
fn an_edge_lost_or_found_on_another_body_is_a_fall() {
    assert_eq!(advance(hanging(), air()).0, LocomotionState::Fall(Fall));
    let mut world = crate::physics::PhysicsWorld::new(crate::physics::PhysicsConfig::default());
    let other = world.create_body(crate::physics::RigidBodyDesc::dynamic());
    let mut elsewhere = hang_lip();
    elsewhere.edge.body = Some(other);
    assert_eq!(
        advance(hanging(), air().edge(elsewhere)).0,
        LocomotionState::Fall(Fall)
    );
}

#[test]
fn a_pull_up_runs_its_course_onto_its_feet_at_the_edge_velocity() {
    let duration = config().pull_up_time;
    let climbing = ledge(
        hang_lip(),
        LedgePhase::PullUp {
            elapsed: 0.0,
            duration,
        },
    );
    let held = air().edge(hang_lip()).jump().crouch();
    let almost = hold(climbing, held, duration - 2.0 * DT);
    assert!(
        matches!(phase(almost), Some(LedgePhase::PullUp { .. })),
        "a pull-up is committed: {almost:?}"
    );
    let (done, effects) = advance(
        hold(almost, air().edge(hang_lip()), DT),
        air().edge(hang_lip()),
    );
    assert_eq!(done, LocomotionState::OnFoot(OnFoot::default()));
    assert_eq!(effects.match_edge, Some(hang_lip()));
}

#[test]
fn the_hands_are_free_on_foot_in_flight_and_in_the_water() {
    let jump = |phase| {
        LocomotionState::Jump(Jump {
            kind: JumpKind::Jump,
            phase,
            steering: AirSteering::Responsive,
        })
    };
    assert!(LocomotionState::OnFoot(OnFoot::default()).hands_free());
    assert!(LocomotionState::Fall(Fall).hands_free());
    assert!(LocomotionState::Swim(Swim).hands_free());
    assert!(jump(TakeoffPhase::Flight).hands_free());
    assert!(!jump(TakeoffPhase::launch(0.2)).hands_free());
    assert!(!hanging().hands_free());
}
