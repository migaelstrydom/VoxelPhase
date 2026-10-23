use std::ops::Range;

use crate::path::PathVec;
use crate::plan::Motion;

use super::surface::Support;

/// The body floor under a point of the path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FloorHit {
    /// Height of the capsule's bottom-sphere centre resting there.
    pub z: f32,
    /// Unit contact normal: how steep the floor is for the body there.
    pub normal: PathVec,
    /// What holds the floor up there.
    pub support: Support,
}

/// A place a foot could land.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Foothold {
    /// The contact point on the ground, the sole's lowest point.
    pub at: PathVec,
    /// Unit ground normal there.
    pub normal: PathVec,
    /// What holds it up.
    pub support: Support,
    /// Distance along `s` to the nearer end of its surface: how far the foot
    /// is from falling off an edge ahead or behind.
    pub edge_margin: f32,
}

/// Where to look for footholds.
#[derive(Clone, Debug, PartialEq)]
pub struct FootholdWindow {
    /// Along the path.
    pub s: Range<f32>,
    /// Across the path.
    pub d: Range<f32>,
}

/// How the planner reads the ground. The strip of lanes is the first
/// implementation; a 3D height patch around the body can replace it without
/// the planner changing.
pub trait SectionQuery {
    /// The body floor under the path at `s`; `None` over a drop with no floor
    /// in reach.
    fn floor(&self, s: f32) -> Option<FloorHit>;

    /// Every standable foothold in `window`, appended to `out` at the
    /// section's resolution. The planner judges which is best.
    fn footholds(&self, window: &FootholdWindow, out: &mut Vec<Foothold>);

    /// Whether the capsule, its bottom-sphere centre following `motion`, stays
    /// on or above the body floor and keeps its top sphere under the ceiling.
    fn clears(&self, motion: &Motion) -> bool;
}
