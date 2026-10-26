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
    /// What `placer` has down. Off the ground it holds no foot planted,
    /// whatever it keeps for landing. With both feet down, the foot its
    /// clock has released steps next and the other holds the body; with
    /// neither released, the foot planted last holds it. With neither foot
    /// down, the foot with the least of its swing left lands next.
    pub fn of(placer: &FootPlacer) -> Self {
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
        let lead = feet
            .iter()
            .filter_map(|foot| swing_left(foot).map(|left| (foot.side, left)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(side, _)| limb(side));
        Self {
            planted: None,
            lead,
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
