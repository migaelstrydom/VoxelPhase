use serde::{Deserialize, Serialize};

use crate::path::PathVec;

/// A body as the game knows it, opaque to the planner: only used to tell
/// footholds on the same body apart and to hand them back to the game.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BodyId(pub u64);

/// What holds a surface up.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Support {
    /// Terrain or anything else that cannot move.
    Static,
    /// A dynamic body: rubble, a spawnable. It may shift or give way under a
    /// foot.
    Body {
        /// Which body.
        id: BodyId,
        /// Its mass, in kg: a heavy body shifts less under a footfall.
        mass: f32,
        /// Whether it was at rest when sampled.
        asleep: bool,
    },
}

/// One cast's sample of a surface.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SurfacePoint {
    /// Distance along the path.
    pub s: f32,
    /// Height of the cast sphere's centre at rest: the layer is the space that
    /// centre can occupy, so clearance is a comparison of centre heights.
    pub z: f32,
    /// Unit normal at the contact, in path coordinates. Keeps its `d`
    /// component, so ground tilted across the path reads as tilted.
    pub normal: PathVec,
}

/// A run of consecutive samples along one lane with one support: a plank, the
/// ground between two planks, a stone.
///
/// The sampler starts a new surface where the support changes, where a cast
/// finds nothing, and (for footholds) where the ground is too steep to stand
/// on, so a surface's ends are its edges.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Surface {
    /// What holds every sample of this surface up.
    pub support: Support,
    /// Samples in increasing `s`, at the lane's spacing.
    pub points: Vec<SurfacePoint>,
}
