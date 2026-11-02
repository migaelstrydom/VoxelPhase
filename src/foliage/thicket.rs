//! How deep a character stands in leaves, and what that costs its stride.
//!
//! The leaves' drag (`drag`) is honest physics, and a character's drive is
//! not: it pushes several times harder through its feet than friction would
//! let it (`Actuator::drive_gain`), enough to walk through a shrub's drag
//! as if it were not there. So a character also measures the leaves it is
//! in, as it measures the water (`Immersion`), and shortens its stride by
//! that: a wade through a hedge is slower than a walk past one.

use nalgebra::Point3;

use super::components::Foliage;
use super::drag::{immersion_in, LeafDrag};
use crate::physics::PhysicsWorld;

/// How deep a ball of `radius` at `centre` is in any clump's leaves, 0 clear
/// of them to 1 wholly inside: the deepest lobe it is in.
pub fn leaf_depth<'a>(
    clumps: impl IntoIterator<Item = &'a Foliage>,
    world: &PhysicsWorld,
    centre: Point3<f32>,
    radius: f32,
) -> f32 {
    clumps
        .into_iter()
        .map(|clump| immersion_in(&clump.drag_volume(), world, centre, radius))
        .fold(0.0, f32::max)
}

impl LeafDrag {
    /// The share of a stride left to a character `depth` into leaves.
    pub fn stride(&self, depth: f32) -> f32 {
        1.0 - self.stride_loss * depth.clamp(0.0, 1.0)
    }
}
