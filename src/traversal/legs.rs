use nalgebra::Point3;
use traversal_planner::plan::{FootfallId, Limb, Plan};
use traversal_planner::planner::Aim;

use crate::animation::{FootPhase, FootPlacer, FootSide, PlacerFoot, StepId};

use super::path_frame::PathFrame;

/// The legs' side of the footshake: the feet they have down, which the next
/// plan starts from, and the footfall the foot they land next swings onto.
/// The plan is the legs' to follow, so it must stand where they stand; the
/// two agree on which footfall a foot is on by the plan's name for it, not
/// by where the foot is.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Legs {
    /// The foot holding the body; `None` with no foot down, as in flight.
    pub planted: Option<Planted>,
    /// The foot that lands next, if the legs say: `None` with neither foot
    /// stepping and neither down.
    pub lead: Option<Limb>,
    /// The lead foot's swing onto one of the plan's footfalls: its name, and
    /// the seconds until it lands, `0` if within the frame.
    pub aim: Option<(StepId, f32)>,
}

/// A foot holding the body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Planted {
    pub limb: Limb,
    /// Where it is planted, in the world.
    pub at: Point3<f32>,
    /// The plan's footfall it stands on; `None` where no plan named it.
    pub footfall: Option<StepId>,
}

impl Legs {
    /// What `placer` has down, planning a frame `dt` long. Off the ground it
    /// holds no foot planted, whatever it keeps for landing. With both feet
    /// down, the foot its clock has released steps next and the other holds
    /// the body; with neither released, the foot planted last holds it. With
    /// neither foot down, the foot with the least of its swing left lands
    /// next.
    pub fn of(placer: &FootPlacer, dt: f32) -> Self {
        let feet = [&placer.left, &placer.right];
        let released = placer.released();
        let planted = feet
            .into_iter()
            .filter(|foot| foot.is_planted() && !placer.is_suspended())
            .filter(|foot| Some(foot.side) != released)
            .min_by(|a, b| a.since_plant.total_cmp(&b.since_plant));
        let lead = match planted {
            Some(planted) => feet.into_iter().find(|foot| foot.side != planted.side),
            None => feet
                .into_iter()
                .filter(|foot| swing_left(foot).is_finite())
                .min_by(|a, b| swing_left(a).total_cmp(&swing_left(b))),
        };
        Self {
            planted: planted.map(|foot| Planted {
                limb: limb(foot.side),
                at: foot.planted_position,
                footfall: foot.stands_on,
            }),
            lead: lead.map(|foot| limb(foot.side)),
            aim: lead.and_then(|foot| aim(foot, dt)),
        }
    }

    /// The legs of a body that follows `plan`, laid along `frame`, exactly,
    /// `dt` seconds on.
    pub fn following(plan: &Plan, frame: &PathFrame, dt: f32) -> Self {
        Self {
            planted: plan.stance_at(dt).map(|stance| Planted {
                limb: stance.limb,
                at: frame.world_point(stance.at),
                footfall: stance.footfall.map(step_id),
            }),
            lead: plan.after(dt).footfalls().next().map(|(_, next)| next.limb),
            aim: None,
        }
    }

    /// The lead foot's swing as the planner reads it.
    pub fn planner_aim(&self) -> Option<Aim> {
        self.aim.map(|(footfall, lands_in)| Aim {
            footfall: footfall_id(footfall),
            lands_in,
        })
    }
}

/// The legs' name for the plan's footfall `id`.
pub fn step_id(id: FootfallId) -> StepId {
    StepId(id.0)
}

/// The plan's name for the step the legs know as `id`.
pub fn footfall_id(id: StepId) -> FootfallId {
    FootfallId(id.0)
}

/// The planned step `foot` swings onto and the seconds until it lands
/// there, `0` within the frame `dt`: `None` planted, or swinging onto no
/// planned step.
fn aim(foot: &PlacerFoot, dt: f32) -> Option<(StepId, f32)> {
    match foot.phase {
        FootPhase::Stepping(swing) => {
            let left = swing.remaining();
            swing
                .footfall
                .map(|id| (id, if left <= dt { 0.0 } else { left }))
        }
        FootPhase::Planted => None,
    }
}

/// Seconds of its swing `foot` has left, infinite planted.
fn swing_left(foot: &PlacerFoot) -> f32 {
    match foot.phase {
        FootPhase::Stepping(swing) => swing.remaining(),
        FootPhase::Planted => f32::INFINITY,
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

    /// A swing onto planned step `id`, `t` seconds into its 0.2.
    fn swing(id: u32, t: f32) -> FootPhase {
        let mut swing = Swing::new(
            Point3::origin(),
            Point3::new(0.0, 0.0, 0.5),
            Vector3::z(),
            Vector3::z(),
            0.2,
            0.05,
            Vector3::zeros(),
        );
        swing.t = t;
        swing.footfall = Some(StepId(id));
        FootPhase::Stepping(swing)
    }

    fn placer() -> FootPlacer {
        FootPlacer::new(Point3::new(0.0, 0.5, 0.0), Vector3::z(), 0.12, 0.0)
    }

    #[test]
    fn a_swing_ending_within_the_frame_lands_now() {
        let mut running = placer();
        running.right.phase = swing(7, 0.2 - 0.5 * FRAME);
        running.left.phase = swing(8, 0.1);
        let legs = Legs::of(&running, FRAME);
        assert_eq!(legs.planted, None);
        assert_eq!(legs.lead, Some(Limb::RightFoot));
        assert_eq!(legs.aim, Some((StepId(7), 0.0)));

        running.right.phase = swing(7, 0.2 - 3.0 * FRAME);
        let (id, left) = Legs::of(&running, FRAME).aim.expect("aims");
        assert_eq!(id, StepId(7));
        assert!((left - 3.0 * FRAME).abs() < 1e-5);
    }

    #[test]
    fn a_planted_foot_stands_on_the_step_it_landed_on() {
        let mut walking = placer();
        walking.left.stands_on = Some(StepId(3));
        walking.left.since_plant = 0.0;
        walking.right.phase = swing(4, 0.1);
        let legs = Legs::of(&walking, FRAME);
        let planted = legs.planted.expect("a foot down");
        assert_eq!(
            (planted.limb, planted.footfall),
            (Limb::LeftFoot, Some(StepId(3)))
        );
        assert_eq!(legs.lead, Some(Limb::RightFoot));
        assert_eq!(legs.aim.map(|(id, _)| id), Some(StepId(4)));
    }
}
