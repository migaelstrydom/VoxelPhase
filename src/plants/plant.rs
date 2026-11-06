//! A grown plant: its limbs and the leaf masses they hold up.

use nalgebra::Isometry3;

use super::limbs::Limbs;
use crate::foliage::{FoliageShape, Lobe};

/// A leaf mass on a plant: its lobes and where its frame sits.
#[derive(Debug, Clone)]
pub struct LeafMass {
    /// The lobes, in the mass's own frame.
    pub shape: FoliageShape,
    /// Where the mass's frame is, in the plant's frame.
    pub pose: Isometry3<f32>,
    /// The limb that holds it up, by index: its tip is inside the mass.
    pub limb: usize,
}

impl LeafMass {
    /// Its lobes moved into the plant's frame.
    pub fn placed(&self) -> Vec<Lobe> {
        self.shape
            .lobes
            .iter()
            .map(|lobe| Lobe {
                centre: self.pose * lobe.centre,
                rotation: self.pose.rotation * lobe.rotation,
                ..*lobe
            })
            .collect()
    }
}

/// A plant as grown: its woody frame and its leaves, in the plant's frame,
/// whose origin is the ground point it grows from and whose y is up.
#[derive(Debug, Clone)]
pub struct Plant {
    /// Trunk, branches and roots.
    pub limbs: Limbs,
    /// The leaf masses the limbs hold up.
    pub masses: Vec<LeafMass>,
}

impl Plant {
    /// The leaf masses held by any of `limbs`, by index.
    pub fn masses_on(&self, limbs: &[usize]) -> Vec<usize> {
        (0..self.masses.len())
            .filter(|&m| limbs.contains(&self.masses[m].limb))
            .collect()
    }

    /// The lobes of `masses` together as one clump, in the plant's frame.
    pub fn leaves_of(&self, masses: &[usize]) -> FoliageShape {
        FoliageShape {
            lobes: masses
                .iter()
                .flat_map(|&m| self.masses[m].placed())
                .collect(),
        }
    }
}
