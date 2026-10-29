use nalgebra::Point3;
use traversal_planner::plan::{Limb, Plan};

use crate::animation::{FootPhase, FootPlacer, FootSide, PlacerFoot};

use super::path_frame::PathFrame;

/// The feet the legs have down, which the next plan starts from: the plan
/// is the legs' to follow, so it must stand where they stand.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Legs {
    /// The foot holding the body and where it is planted, in the world;
    /// `None` with no foot down, as in flight.
    pub planted: Option<(Limb, Point3<f32>)>,
    /// The foot that lands next, if the legs say: `None` with neither foot
    /// stepping and neither down.
    pub lead: Option<Limb>,
}

impl Legs {
    /// What `placer` has down, planning a frame `dt` long. Off the ground it
    /// holds no foot planted, whatever it keeps for landing. With both feet
    /// down, the foot its clock has released steps next and the other holds
    /// the body; with neither released, the foot planted last holds it. With
    /// neither foot down, the foot with the least of its swing left lands
    /// next, and if that swing ends within the frame, it stands where it
    /// lands: the plan, a frame on, has taken that footfall as made, and
    /// planned from no foot it plans that foot's next one a step on.
    pub fn of(placer: &FootPlacer, dt: f32) -> Self {
        let feet = [&placer.left, &placer.right];
        let released = placer.released();
        let planted = feet
            .iter()
            .filter(|foot| foot.is_planted() && !placer.is_suspended())
            .filter(|foot| Some(foot.side) != released)
            .min_by(|a, b| a.since_plant.total_cmp(&b.since_plant));
        if let Some(foot) = planted {
            let limb = limb(foot.side);
            return Self {
                planted: Some((limb, foot.planted_position)),
                lead: Some(limb.other()),
            };
        }
        let landing = feet
            .iter()
            .filter_map(|foot| swing_left(foot).map(|left| (foot, left)))
            .min_by(|a, b| a.1.total_cmp(&b.1));
        match landing {
            Some((foot, left)) if left <= dt && !placer.is_suspended() => {
                let limb = limb(foot.side);
                Self {
                    planted: swing_target(foot).map(|at| (limb, at)),
                    lead: Some(limb.other()),
                }
            }
            landing => Self {
                planted: None,
                lead: landing.map(|(foot, _)| limb(foot.side)),
            },
        }
    }

    /// The legs of a body that follows `plan`, laid along `frame`, exactly,
    /// `dt` seconds on.
    pub fn following(plan: &Plan, frame: &PathFrame, dt: f32) -> Self {
        Self {
            planted: plan
                .stance_at(dt)
                .map(|stance| (stance.limb, frame.world_point(stance.at))),
            lead: plan.after(dt).footfalls().next().map(|(_, next)| next.limb),
        }
    }
}

/// Where `foot`'s swing lands it; `None` planted.
fn swing_target(foot: &PlacerFoot) -> Option<Point3<f32>> {
    match foot.phase {
        FootPhase::Stepping { to, .. } => Some(to),
        FootPhase::Planted => None,
    }
}

/// Seconds of its swing `foot` has left; `None` planted.
fn swing_left(foot: &PlacerFoot) -> Option<f32> {
    match foot.phase {
        FootPhase::Stepping { t, duration, .. } => Some(duration - t),
        FootPhase::Planted => None,
    }
}

fn limb(side: FootSide) -> Limb {
    match side {
        FootSide::Left => Limb::LeftFoot,
        FootSide::Right => Limb::RightFoot,
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;

    const FRAME: f32 = 1.0 / 60.0;

    /// A placer with both feet in the air, the right `left` seconds from
    /// landing at `to` and the left half a swing from landing.
    fn running(left: f32, to: Point3<f32>) -> FootPlacer {
        let mut placer = FootPlacer::new(Point3::new(0.0, 0.5, 0.0), Vector3::z(), 0.12, 0.0);
        let swing = |from: Point3<f32>, to: Point3<f32>, t: f32| FootPhase::Stepping {
            from,
            to,
            from_forward: Vector3::z(),
            to_forward: Vector3::z(),
            t,
            duration: 0.2,
            peak_lift: 0.05,
        };
        placer.right.phase = swing(Point3::new(-0.12, 0.0, -0.5), to, 0.2 - left);
        placer.left.phase = swing(
            Point3::new(0.12, 0.0, -0.3),
            Point3::new(0.12, 0.0, 0.9),
            0.1,
        );
        placer
    }

    #[test]
    fn a_foot_landing_within_the_frame_stands_where_it_lands() {
        let to = Point3::new(-0.12, 0.0, 0.3);
        let landing = Legs::of(&running(0.5 * FRAME, to), FRAME);
        assert_eq!(landing.planted, Some((Limb::RightFoot, to)));
        assert_eq!(landing.lead, Some(Limb::LeftFoot));

        let swinging = Legs::of(&running(3.0 * FRAME, to), FRAME);
        assert_eq!(swinging.planted, None);
        assert_eq!(swinging.lead, Some(Limb::RightFoot));
    }
}
