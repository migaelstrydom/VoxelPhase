//! A grown plant: its limbs and the leaf masses they hold up.

use nalgebra::Isometry3;

use super::limbs::Limbs;
use crate::foliage::FoliageShape;

/// A leaf mass on a plant: its lobes and where its frame sits.
#[derive(Debug, Clone)]
pub struct LeafMass {
    /// The lobes, in the mass's own frame.
    pub shape: FoliageShape,
    /// Where the mass's frame is, in the plant's frame.
    pub pose: Isometry3<f32>,
}

/// A plant as grown: its woody frame and its leaves, in the plant's frame,
/// whose origin is the ground point it grows from and whose y is up.
#[derive(Debug, Clone)]
pub struct Plant {
    /// Trunk and branches.
    pub limbs: Limbs,
    /// The leaf masses the limbs hold up.
    pub masses: Vec<LeafMass>,
}
