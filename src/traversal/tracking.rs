use traversal_planner::plan::Limb;
use traversal_planner::section::Profile;
use traversal_planner::{GaitParams, Motion, Phase, Plan};

use super::path_frame::PathFrame;
use crate::character::{Foot, PlannedLanding, PlannedTakeoff};

/// What a plan asks of the body over the next frame: how fast to drive, the
/// takeoff to leave with if one falls in it, and where a flight under way
/// comes down.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tracking {
    pub pace: Pace,
    pub takeoff: Option<PlannedTakeoff>,
    pub landing: Option<PlannedLanding>,
}

/// How fast a tracked body is driven along the path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pace {
    /// As the stick asks: the plan is speeding up, and the drive speeds up
    /// as fast as it does untracked (the plan's acceleration is the drive's
    /// mean from standing, which its first frames on firm ground outrun);
    /// or the body is in flight, where it steers as it does untracked.
    Asked,
    /// At the plan's speed, in m/s: it holds the body below the asked speed
    /// or slows it, to meet a hop or for a slope.
    Planned(f32),
}

/// How much faster the plan must be at the frame's end than now for it to
/// count as speeding up, in m/s.
const SPEEDING_UP: f32 = 1.0e-3;
/// How much more upward speed than the body has a hop must ask for to be a
/// takeoff, in m/s: asking less, the body is already on its arc.
const PUSHED_CLIMB: f32 = 0.05;

/// Follows `plan`, made from where the body is now, rising at `climb` m/s,
/// through the frame of `dt` seconds ahead, along `frame`'s path; `reach` is
/// how far a landing may lie from where the body comes down
/// ([`landing_reach`]).
///
/// Where the plan is speeding up the stick sets the pace ([`Pace::Asked`]);
/// elsewhere it is the plan's speed at the frame's end. In flight the stick
/// sets it too: a plan's flight holds its takeoff speed, so pacing a flight
/// at the plan's speed would hold the body at whatever it took off with
/// (0.3 m/s, jumping from standing) and air steering could never speed it
/// up. A hop's air speed is capped at its launch instead, and a body
/// steered off the plan's arc is replanned from where it goes. A `refused` plan
/// knows no way through, so it leaves the body to the drive: the stick sets
/// the pace and nothing takes off, which is never worse than no plan at all.
///
/// A flight is taken off on the frame whose start is nearest its own, half a
/// frame early at most. A hop asks for the climb it needs over the body's
/// own; a roll-off, which keeps its stance foot, asks for none, and keeps
/// its rise for `BodyDims::rise_held`. A hop asking no more climb than the
/// body has asks for nothing: a plan made in the air, or in a bounce off a
/// landing, starts with the flight the body is already on. A flight under
/// way asks for its landing every frame.
pub fn track(
    plan: &Plan,
    frame: &PathFrame,
    climb: f32,
    dt: f32,
    refused: bool,
    reach: f32,
) -> Tracking {
    let (now, next) = (speed_at(plan, 0.0), speed_at(plan, dt));
    let pace = if refused || flying_at(plan, 0.0) || next > now + SPEEDING_UP {
        Pace::Asked
    } else {
        Pace::Planned(next)
    };
    let landing_of = |flight: &Phase| landing(plan, flight, frame, reach);
    let takeoff = plan
        .phases
        .iter()
        .take_while(|phase| !refused && phase.start < 0.5 * dt)
        .find_map(|phase| match phase.motion {
            Motion::Flight { takeoff, .. } => {
                // A hop's phase holds no contact; a roll-off's keeps the stance.
                let rolls_off = !phase.contacts.is_empty();
                (rolls_off || takeoff.climb > climb + PUSHED_CLIMB).then(|| PlannedTakeoff {
                    planar: frame.forward() * takeoff.speed,
                    climb: (!rolls_off).then_some(takeoff.climb),
                    landing: landing_of(phase),
                })
            }
            Motion::Grounded { .. } => None,
        });
    let landing = phase_at(plan, 0.0)
        .filter(|phase| !refused && matches!(phase.motion, Motion::Flight { .. }))
        .and_then(landing_of);
    Tracking {
        pace,
        takeoff,
        landing,
    }
}

/// How far a landing may lie from where the body's centre comes down and
/// still be stood on, in metres: a leg's reach below the hip, and the hip's
/// offset from the centre.
pub fn landing_reach(params: &GaitParams) -> f32 {
    params.foot.leg_reach + (0.5 * params.body.height - params.foot.hip_height).abs()
}

/// Where `flight`, a phase of `plan`, comes down: the first footfall after it
/// starts that lands a flight.
fn landing(plan: &Plan, flight: &Phase, frame: &PathFrame, reach: f32) -> Option<PlannedLanding> {
    plan.footfalls()
        .find(|(time, _)| *time > flight.start && plan.lands_a_flight(*time))
        .map(|(time, contact)| PlannedLanding {
            foot: match contact.limb {
                Limb::LeftFoot => Foot::Left,
                Limb::RightFoot => Foot::Right,
            },
            at: frame.world_point(contact.at),
            in_seconds: time,
            footfall: contact.footfall.map(|id| id.0),
            reach,
        })
}

/// How far ahead the floor's grade under the body is read, in metres: a
/// stride's part, over which the drive feels the slope.
const GRADE_RUN: f32 = 0.2;

/// The grade of `floor` at `s`: its rise over the next [`GRADE_RUN`], level
/// where it has no sample there or then.
pub fn floor_grade(floor: &Profile, s: f32) -> f32 {
    let height = |at: f32| {
        floor
            .surfaces
            .iter()
            .flat_map(|surface| surface.points.windows(2))
            .find(|pair| pair[0].s <= at && at <= pair[1].s)
            .map(|pair| {
                let fraction = (at - pair[0].s) / (pair[1].s - pair[0].s).max(f32::EPSILON);
                pair[0].z + fraction * (pair[1].z - pair[0].z)
            })
    };
    match (height(s), height(s + GRADE_RUN)) {
        (Some(here), Some(ahead)) => (ahead - here) / GRADE_RUN,
        _ => 0.0,
    }
}

/// The plan's speed along the path `t` seconds in; nothing past its end.
fn speed_at(plan: &Plan, t: f32) -> f32 {
    phase_at(plan, t).map_or(0.0, |phase| phase.progress((t - phase.start).max(0.0)).1)
}

/// Whether the plan is in flight `t` seconds in.
fn flying_at(plan: &Plan, t: f32) -> bool {
    phase_at(plan, t).is_some_and(|phase| matches!(phase.motion, Motion::Flight { .. }))
}

/// The plan's phase `t` seconds in; nothing past its end.
fn phase_at(plan: &Plan, t: f32) -> Option<&Phase> {
    plan.phases
        .iter()
        .find(|phase| t < phase.start + phase.duration)
}

#[cfg(test)]
mod tests {
    use smallvec::smallvec;
    use traversal_planner::plan::{Contact, Limb, Phase};
    use traversal_planner::section::{Footing, Surface, SurfacePoint};
    use traversal_planner::{BodyState, PathVec};

    use nalgebra::{Point3, Vector3};

    use super::*;

    const DT: f32 = 1.0 / 60.0;
    const REACH: f32 = 0.8;

    /// A path along +Z from the origin.
    fn along_z() -> PathFrame {
        PathFrame::new(Point3::origin(), 0.0)
    }

    fn track_z(plan: &Plan, climb: f32, refused: bool) -> Tracking {
        track(plan, &along_z(), climb, DT, refused, REACH)
    }

    fn stance() -> Contact {
        Contact {
            footfall: None,
            limb: Limb::LeftFoot,
            at: PathVec::new(0.0, 0.1, 0.0),
            normal: PathVec::UP,
            footing: Footing::Fixed,
        }
    }

    fn ride(start: f32, duration: f32, from: f32, speeds: (f32, f32)) -> Phase {
        Phase {
            start,
            duration,
            contacts: smallvec![stance()],
            motion: Motion::Grounded {
                from,
                to: from + 0.5 * (speeds.0 + speeds.1) * duration,
                start_speed: speeds.0,
                end_speed: speeds.1,
            },
        }
    }

    fn flight(start: f32, s: f32, contacts: bool) -> Phase {
        Phase {
            start,
            duration: 0.3,
            contacts: if contacts {
                smallvec![stance()]
            } else {
                smallvec![]
            },
            motion: Motion::Flight {
                takeoff: BodyState {
                    s,
                    z: 0.25,
                    speed: 4.0,
                    climb: 1.5,
                },
                gravity: 9.81,
                rise_held: None,
            },
        }
    }

    fn plan(phases: Vec<Phase>) -> Plan {
        Plan {
            phases,
            ..Plan::default()
        }
    }

    /// A floor rising 0.5 m in a metre and then level reads at its grade
    /// on the rise, level on the flat, and level where it has no samples.
    #[test]
    fn the_floor_grade_is_its_rise_ahead() {
        let points = (0..=40)
            .map(|i| {
                let s = i as f32 * 0.05;
                SurfacePoint {
                    s,
                    z: 0.5 * s.min(1.0),
                    normal: PathVec::UP,
                }
            })
            .collect();
        let floor = Profile {
            surfaces: vec![Surface {
                footing: Footing::Fixed,
                points,
            }],
        };
        assert!((floor_grade(&floor, 0.3) - 0.5).abs() < 1.0e-4);
        assert!(floor_grade(&floor, 1.2).abs() < 1.0e-4);
        assert_eq!(floor_grade(&floor, 5.0), 0.0);
    }

    /// A plan slowing or holding sets the pace at its speed at the frame's
    /// end; one speeding up leaves it to the stick.
    #[test]
    fn the_speed_is_the_plans_at_the_end_of_the_frame() {
        let slowing_down = plan(vec![ride(0.0, 1.0, 0.0, (5.0, 3.0))]);
        let tracked = track_z(&slowing_down, 0.0, false);
        let Pace::Planned(speed) = tracked.pace else {
            panic!("{tracked:?}");
        };
        assert!((speed - (5.0 - 2.0 * DT)).abs() < 1.0e-5, "{tracked:?}");
        assert_eq!(tracked.takeoff, None);
        let holding = plan(vec![ride(0.0, 1.0, 0.0, (3.0, 3.0))]);
        assert_eq!(track_z(&holding, 0.0, false).pace, Pace::Planned(3.0));
        let speeding_up = plan(vec![ride(0.0, 1.0, 0.0, (3.0, 5.0))]);
        assert_eq!(track_z(&speeding_up, 0.0, false).pace, Pace::Asked);
    }

    /// In flight the stick sets the pace, though the plan's flight holds
    /// its speed: holding it would keep the body at its takeoff speed, and
    /// air steering could never speed it up. Into a flight that starts
    /// later in the frame, the plan still sets it.
    #[test]
    fn in_flight_the_stick_sets_the_pace() {
        let arc = plan(vec![
            flight(0.0, 0.0, false),
            ride(0.3, 1.0, 1.2, (4.0, 2.0)),
        ]);
        assert_eq!(track_z(&arc, -1.0, false).pace, Pace::Asked);
        let landing = plan(vec![
            flight(-0.29, 0.0, false),
            ride(0.01, 1.0, 1.2, (4.0, 2.0)),
        ]);
        assert_eq!(
            track_z(&landing, -1.0, false).pace,
            Pace::Asked,
            "the frame it lands in"
        );
        let to_a_hop = plan(vec![
            ride(0.0, 0.01, 0.0, (4.0, 4.0)),
            flight(0.01, 0.04, false),
        ]);
        assert_eq!(track_z(&to_a_hop, 0.0, false).pace, Pace::Planned(4.0));
    }

    #[test]
    fn a_flight_is_taken_off_on_the_nearest_frame_and_an_arc_under_way_is_not() {
        let hop = plan(vec![flight(0.0, 0.0, false)]);
        assert_eq!(
            track_z(&hop, 0.0, false).takeoff,
            Some(PlannedTakeoff {
                planar: Vector3::new(0.0, 0.0, 4.0),
                climb: Some(1.5),
                landing: None,
            })
        );

        // A roll-off leaves on the climb the body has.
        let roll_off = plan(vec![
            ride(0.0, 0.005, 0.0, (4.0, 4.0)),
            flight(0.005, 0.02, true),
        ]);
        let rolled = track_z(&roll_off, 0.0, false).takeoff.expect("a roll-off");
        assert_eq!(rolled.climb, None);

        // Nearer the next frame's start than this one's: taken off then.
        let later = plan(vec![
            ride(0.0, 0.012, 0.0, (4.0, 4.0)),
            flight(0.012, 0.05, false),
        ]);
        assert_eq!(track_z(&later, 0.0, false).takeoff, None);

        // Already rising as fast as the hop leaves: in the air, or bouncing.
        assert_eq!(track_z(&hop, 1.5, false).takeoff, None);
    }

    /// A takeoff carries where its flight comes down, and a flight under way
    /// asks for it every frame; a plan on the ground asks for none.
    #[test]
    fn a_flight_names_its_landing() {
        let hop = plan(vec![
            flight(0.0, 0.0, false),
            ride(0.3, 1.0, 1.2, (4.0, 2.0)),
        ]);
        let landing = track_z(&hop, 0.0, false)
            .takeoff
            .and_then(|takeoff| takeoff.landing)
            .expect("a landing");
        assert_eq!(landing.foot, Foot::Left);
        assert!((landing.in_seconds - 0.3).abs() < 1.0e-6);
        assert_eq!(landing.at, along_z().world_point(stance().at));
        assert_eq!(landing.reach, REACH);
        assert_eq!(track_z(&hop, 1.5, false).landing, Some(landing));

        let walking = plan(vec![ride(0.0, 1.0, 0.0, (4.0, 4.0))]);
        assert_eq!(track_z(&walking, 0.0, false).landing, None);
    }

    /// A refused plan leaves the body to the drive, even at its very end:
    /// the stick sets the pace and nothing takes off.
    #[test]
    fn a_refused_plan_leaves_the_body_to_the_drive() {
        let at_the_edge = plan(vec![ride(0.0, 0.01, 0.0, (5.0, 5.0))]);
        assert_eq!(track_z(&at_the_edge, 0.0, true).pace, Pace::Asked);
        let hop = plan(vec![
            ride(0.0, 0.005, 0.0, (4.0, 4.0)),
            flight(0.005, 0.02, false),
        ]);
        assert!(track_z(&hop, 0.0, false).takeoff.is_some());
        assert_eq!(track_z(&hop, 0.0, true).takeoff, None);
    }
}
