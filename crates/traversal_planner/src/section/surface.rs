use serde::{Deserialize, Serialize};

use crate::path::PathVec;

/// What a surface is held up by, as far as a foot cares.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Footing {
    /// Ground that cannot move: terrain, anything fixed.
    Fixed,
    /// A loose body: rubble, a plank that may rock. It may shift or give way
    /// under a foot.
    Loose {
        /// Its mass, in kg: a heavy body shifts less under a footfall.
        mass: f32,
        /// Whether it was at rest when sampled.
        resting: bool,
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

/// A run of consecutive samples along one lane on one body: a plank, the
/// ground between two planks, a stone.
///
/// The sampler starts a new surface where the body under it changes (so the
/// seam between two flush stones is an edge), where a cast finds nothing, and
/// (for footholds) where the ground is too steep to stand on, so a surface's
/// ends are its edges.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Surface {
    /// What holds every sample of this surface up.
    pub footing: Footing,
    /// Samples in increasing `s`, at the lane's spacing.
    pub points: Vec<SurfacePoint>,
}
