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
    /// Where that foot is swinging to, in the world, and the seconds until
    /// it lands there, if it is swinging.
    pub aim: Option<(Point3<f32>, f32)>,
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
            let other = feet.iter().find(|other| other.side != foot.side);
            return Self {
                planted: Some((limb, foot.planted_position)),
                lead: Some(limb.other()),
                aim: other.and_then(aim),
            };
        }
        let landing = feet
            .iter()
            .filter_map(|foot| swing_left(foot).map(|left| (foot, left)))
            .min_by(|a, b| a.1.total_cmp(&b.1));
        match landing {
            Some((foot, left)) if left <= dt && !placer.is_suspended() => {
                let limb = limb(foot.side);
                let other = feet.iter().find(|other| other.side != foot.side);
                Self {
                    planted: swing_target(foot).map(|at| (limb, at)),
                    lead: Some(limb.other()),
                    aim: other.and_then(aim),
                }
            }
            landing => Self {
                planted: None,
                lead: landing.map(|(foot, _)| limb(foot.side)),
                aim: landing.and_then(|(foot, _)| aim(foot)),
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
            aim: None,
        }
    }

    /// The legs as a plan standing on `stance` (a foot, and where it stands
    /// in the world) has them: if the foot the legs land next is swinging to
    /// within `tolerance` of that footfall and lands within `within` seconds,
    /// it stands there. The plan lands a footfall on its clock, the legs on
    /// theirs, often a few frames later; a foot about to land on the plan's
    /// own footfall is not a stance the plan must start over from.
    pub fn caught_up(
        self,
        stance: Option<(Limb, Point3<f32>)>,
        tolerance: f32,
        within: f32,
    ) -> Self {
        match (stance, self.lead, self.aim) {
            (Some((limb, at)), Some(lead), Some((aim, left)))
                if limb == lead
                    && left <= within
                    && (aim.x - at.x).hypot(aim.z - at.z) <= tolerance =>
            {
                Self {
                    planted: Some((limb, aim)),
                    lead: Some(limb.other()),
                    aim: None,
                }
            }
            _ => self,
        }
    }
}

/// Where `foot`'s swing lands it; `None` planted.
fn swing_target(foot: &PlacerFoot) -> Option<Point3<f32>> {
    match foot.phase {
        FootPhase::Stepping(swing) => Some(swing.to),
        FootPhase::Planted => None,
    }
}

/// Where `foot`'s swing lands it and the seconds until it does; `None`
/// planted.
fn aim(foot: &&PlacerFoot) -> Option<(Point3<f32>, f32)> {
    swing_target(foot).zip(swing_left(foot))
}

/// Seconds of its swing `foot` has left; `None` planted.
fn swing_left(foot: &PlacerFoot) -> Option<f32> {
    match foot.phase {
        FootPhase::Stepping(swing) => Some(swing.remaining()),
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
    use crate::animation::Swing;

    const FRAME: f32 = 1.0 / 60.0;

    /// A placer with both feet in the air, the right `left` seconds from
    /// landing at `to` and the left half a swing from landing.
    fn running(left: f32, to: Point3<f32>) -> FootPlacer {
        let mut placer = FootPlacer::new(Point3::new(0.0, 0.5, 0.0), Vector3::z(), 0.12, 0.0);
        let swing = |from: Point3<f32>, to: Point3<f32>, t: f32| {
            let mut swing = Swing::new(
                from,
                to,
                Vector3::z(),
                Vector3::z(),
                0.2,
                0.05,
                Vector3::zeros(),
            );
            swing.t = t;
            FootPhase::Stepping(swing)
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
    fn a_foot_about_to_land_on_the_plans_stance_is_caught_up_to_it() {
        let to = Point3::new(-0.12, 0.0, 0.3);
        let legs = Legs::of(&running(2.0 * FRAME, to), FRAME);
        let stance = Some((Limb::RightFoot, to));
        let caught = legs.caught_up(stance, 0.05, 0.05);
        assert_eq!(caught.planted, Some((Limb::RightFoot, to)));
        assert_eq!(caught.lead, Some(Limb::LeftFoot));

        // Aimed elsewhere, landing later, or the plan standing on the other
        // foot: the legs are as they are.
        let elsewhere = Some((Limb::RightFoot, Point3::new(-0.12, 0.0, 0.5)));
        assert_eq!(legs.caught_up(elsewhere, 0.05, 0.05), legs);
        assert_eq!(legs.caught_up(stance, 0.05, FRAME), legs);
        let other = Some((Limb::LeftFoot, to));
        assert_eq!(legs.caught_up(other, 0.05, 0.05), legs);
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
