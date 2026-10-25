use nalgebra::Vector3;
use traversal_planner::section::Profile;
use traversal_planner::{Motion, Plan};

use crate::character::PlannedTakeoff;

/// What a plan asks of the body over the next frame: how fast to drive, and
/// the takeoff to leave with if one falls in it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tracking {
    pub pace: Pace,
    pub takeoff: Option<PlannedTakeoff>,
}

/// How fast a tracked body is driven along the path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pace {
    /// As the stick asks: the plan is speeding up, and the drive speeds up
    /// as fast as it does untracked. The plan's acceleration is the drive's
    /// mean from standing, which its first frames on firm ground outrun.
    Asked,
    /// At the plan's speed, in m/s: it holds the body below the asked speed
    /// or slows it, to meet a hop or for a slope.
    Planned(f32),
    /// Braking to stop where a refused plan ran out, at this speed in m/s.
    Braking(f32),
}

/// How much faster the plan must be at the frame's end than now for it to
/// count as speeding up, in m/s.
const SPEEDING_UP: f32 = 1.0e-3;

/// How little of a refused plan may be left ahead of the body for the plan
/// to let it go, in metres: it has no way on from here, and the stick drives
/// as it would untracked, into whatever the plan could not see past.
const RELEASE_ROOM: f32 = 0.05;
/// How slow a body under a refused plan may be for the plan to let it go, in
/// m/s: stopped, by the plan's braking or by what it is pressing against.
const RELEASE_SPEED: f32 = 0.5;

/// Follows `plan`, made from where the body is now, through the frame of
/// `dt` seconds ahead, along the path's horizontal unit `forward`; `None`
/// when a refused plan has no room left ([`RELEASE_ROOM`]) or the body has
/// stopped under it ([`RELEASE_SPEED`]).
///
/// Where the plan is speeding up the stick sets the pace ([`Pace::Asked`]);
/// elsewhere it is the plan's speed at the frame's end. A refused plan stops
/// where it ran out: no faster than braking at `braking` m/s² stops the body
/// there.
/// A hop is taken off on the frame whose start is nearest its own, half a
/// frame early at most; a roll-off, which keeps its stance foot, is the body
/// running off the edge and needs nothing asked.
pub fn track(
    plan: &Plan,
    forward: Vector3<f32>,
    dt: f32,
    refused: bool,
    braking: f32,
) -> Option<Tracking> {
    let (now, next) = (speed_at(plan, 0.0), speed_at(plan, dt));
    let mut pace = if next > now + SPEEDING_UP {
        Pace::Asked
    } else {
        Pace::Planned(next)
    };
    if refused {
        let room = match (plan.phases.first(), plan.phases.last()) {
            (Some(first), Some(last)) => last.end_s() - first.progress(0.0).0,
            _ => 0.0,
        };
        if room < RELEASE_ROOM || now <= RELEASE_SPEED {
            return None;
        }
        let stopping = (2.0 * braking * room).sqrt();
        if stopping < next {
            pace = Pace::Braking(stopping);
        }
    }
    let takeoff = plan
        .phases
        .iter()
        .take_while(|phase| phase.start < 0.5 * dt)
        .find_map(|phase| match phase.motion {
            // A hop's phase holds no contact; a roll-off's keeps the stance.
            Motion::Flight { takeoff, .. } if phase.contacts.is_empty() => Some(PlannedTakeoff {
                velocity: forward * takeoff.speed + Vector3::y() * takeoff.climb,
                flight: phase.duration,
            }),
            _ => None,
        });
    Some(Tracking { pace, takeoff })
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
    plan.phases
        .iter()
        .find(|phase| t < phase.start + phase.duration)
        .map_or(0.0, |phase| phase.progress((t - phase.start).max(0.0)).1)
}

#[cfg(test)]
mod tests {
    use smallvec::smallvec;
    use traversal_planner::plan::{Contact, Limb, Phase};
    use traversal_planner::section::{Footing, Surface, SurfacePoint};
    use traversal_planner::{BodyState, PathVec};

    use super::*;

    const DT: f32 = 1.0 / 60.0;

    fn stance() -> Contact {
        Contact {
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
        let slowing = plan(vec![ride(0.0, 1.0, 0.0, (5.0, 3.0))]);
        let tracked = track(&slowing, Vector3::x(), DT, false, 8.0).unwrap();
        let Pace::Planned(speed) = tracked.pace else {
            panic!("{tracked:?}");
        };
        assert!((speed - (5.0 - 2.0 * DT)).abs() < 1.0e-5, "{tracked:?}");
        assert_eq!(tracked.takeoff, None);
        let holding = plan(vec![ride(0.0, 1.0, 0.0, (3.0, 3.0))]);
        assert_eq!(
            track(&holding, Vector3::x(), DT, false, 8.0).unwrap().pace,
            Pace::Planned(3.0)
        );
        let speeding_up = plan(vec![ride(0.0, 1.0, 0.0, (3.0, 5.0))]);
        assert_eq!(
            track(&speeding_up, Vector3::x(), DT, false, 8.0)
                .unwrap()
                .pace,
            Pace::Asked
        );
    }

    #[test]
    fn a_hop_is_taken_off_on_the_nearest_frame_and_a_roll_off_is_not() {
        let hop = plan(vec![
            ride(0.0, 0.005, 0.0, (4.0, 4.0)),
            flight(0.005, 0.02, false),
        ]);
        let tracked = track(&hop, Vector3::z(), DT, false, 8.0).unwrap();
        assert_eq!(
            tracked.takeoff,
            Some(PlannedTakeoff {
                velocity: Vector3::new(0.0, 1.5, 4.0),
                flight: 0.3,
            })
        );

        let roll_off = plan(vec![
            ride(0.0, 0.005, 0.0, (4.0, 4.0)),
            flight(0.005, 0.02, true),
        ]);
        assert_eq!(
            track(&roll_off, Vector3::z(), DT, false, 8.0)
                .unwrap()
                .takeoff,
            None
        );

        // Nearer the next frame's start than this one's: taken off then.
        let later = plan(vec![
            ride(0.0, 0.012, 0.0, (4.0, 4.0)),
            flight(0.012, 0.05, false),
        ]);
        assert_eq!(
            track(&later, Vector3::z(), DT, false, 8.0).unwrap().takeoff,
            None
        );
    }

    /// A refused plan at full speed half a metre from where it ran out is
    /// braked to stop there; one with no room left, or with the body
    /// stopped, lets it go.
    #[test]
    fn a_refused_plan_is_stopped_where_it_ran_out() {
        let refused = plan(vec![ride(0.0, 0.1, 0.0, (5.0, 5.0))]);
        let tracked = track(&refused, Vector3::x(), DT, true, 8.0).unwrap();
        assert!(
            tracked.pace == Pace::Braking((2.0f32 * 8.0 * 0.5).sqrt()),
            "{tracked:?}"
        );
        assert_eq!(track(&plan(vec![]), Vector3::x(), DT, true, 8.0), None);
        let at_the_edge = plan(vec![ride(0.0, 0.01, 0.0, (0.3, 0.3))]);
        assert_eq!(track(&at_the_edge, Vector3::x(), DT, true, 8.0), None);
        let stalled = plan(vec![ride(0.0, 1.0, 0.0, (0.3, 0.3))]);
        assert_eq!(track(&stalled, Vector3::x(), DT, true, 8.0), None);
    }
}
