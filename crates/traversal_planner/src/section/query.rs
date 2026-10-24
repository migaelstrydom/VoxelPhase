use std::ops::Range;

use crate::path::PathVec;
use crate::plan::Phase;

use super::surface::Footing;

/// The body floor under a point of the path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FloorHit {
    /// Height of the capsule's bottom-sphere centre resting there.
    pub z: f32,
    /// Unit contact normal: how steep the floor is for the body there.
    pub normal: PathVec,
    /// What holds the floor up there.
    pub footing: Footing,
}

/// How far the body can ride the floor on its contacts from one point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ride {
    /// The furthest `s` reached: the end asked for, or where the floor stops,
    /// turns too steep for the drive, or leaves too little headroom.
    pub end: f32,
    /// Height gained and lost on the way, in metres: the dips and bumps the
    /// capsule rides through, which it feels as snags and bounces.
    pub vertical_travel: f32,
}

/// A place a foot could land.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Foothold {
    /// The contact point on the ground, the sole's lowest point.
    pub at: PathVec,
    /// Unit ground normal there.
    pub normal: PathVec,
    /// What holds it up.
    pub footing: Footing,
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
    /// Least distance along `s` between two footholds returned on one line
    /// across the path: the resolution the planner searches at.
    pub spacing: f32,
}

/// How the planner reads the ground, for one gait: what the body can ride
/// and fly over depends on its limits, which the query is built with. The
/// strip of lanes is the first implementation; a 3D height patch around the
/// body can replace it without the planner changing.
pub trait SectionQuery {
    /// The body floor under the path at `s`; `None` over a drop with no floor
    /// in reach.
    fn floor(&self, s: f32) -> Option<FloorHit>;
    /// The body riding its floor from `from` toward `to` (`from <= to`).
    fn ride(&self, from: f32, to: f32) -> Ride;
    /// Every standable foothold in `window`, appended to `out`. The planner
    /// judges which is best.
    fn footholds(&self, window: &FootholdWindow, out: &mut Vec<Foothold>);
    /// Whether the body can move as `phase` says: riding a floor it can be
    /// driven over, or flying with clearance over the floor and under the
    /// ceiling.
    fn clears(&self, phase: &Phase) -> bool;
}
