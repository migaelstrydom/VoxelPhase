//! How a plant is drawn: each limb's bark and each crown's leaves, meshed
//! once in the plant's frame, from which the look of each collider of it is
//! made.
//!
//! ```text
//!   Plant ──PlantDrawing::grow──▶ bark per limb (whole, and snapped for a branch)
//!                                 leaves per crown
//!   PlantDrawing ──bark(limb, base) / leaves(crown)──▶ a Look, in the frame
//!                  of the collider it is drawn on (PartLooks)
//! ```

use std::sync::Arc;

use nalgebra::{Isometry3, Translation3};

use super::bark::{BarkSpec, LimbBase};
use super::limbs::LimbKind;
use super::plant::Plant;
use crate::foliage::{Skin, SkinSpec};
use crate::model::Look;
use crate::rendering::material::MaterialId;

/// The materials a plant is drawn in.
#[derive(Debug, Clone, Copy)]
pub struct PlantLook {
    /// The leaves: greens ride on the skin's vertices.
    pub leaves: MaterialId,
    /// The limbs.
    pub bark: MaterialId,
}

/// A plant's meshes, in the plant's frame.
#[derive(Debug, Clone)]
pub struct PlantDrawing {
    /// What they are drawn in.
    look: PlantLook,
    /// Each limb's bark, its base rounded: buried, or inside what it grows
    /// from.
    whole: Vec<Skin>,
    /// Each branch's bark with its base snapped, for once it has broken
    /// off; `None` for a limb that never breaks off.
    snapped: Vec<Option<Skin>>,
    /// Each crown's leaves.
    leaves: Vec<Skin>,
}

impl PlantDrawing {
    /// `plant`'s meshes, grown from `seed`, to be drawn in `look`.
    pub fn grow(plant: &Plant, seed: u32, look: PlantLook) -> Self {
        let bark = BarkSpec::default();
        let tube = |l: usize, base: LimbBase| {
            let limb = &plant.limbs.limbs[l];
            let (vertices, indices) = bark.mesh(limb, seed.wrapping_add(l as u32), base);
            Skin { vertices, indices }
                .moved(&Isometry3::from(Translation3::from(limb.midpoint().coords)))
        };
        let limbs = 0..plant.limbs.limbs.len();
        Self {
            look,
            whole: limbs.clone().map(|l| tube(l, LimbBase::Rounded)).collect(),
            snapped: limbs
                .map(|l| {
                    (plant.limbs.limbs[l].kind == LimbKind::Branch)
                        .then(|| tube(l, LimbBase::Snapped))
                })
                .collect(),
            leaves: plant
                .masses
                .iter()
                .enumerate()
                .map(|(m, mass)| {
                    let skin_seed = seed.wrapping_add(m as u32 * 7919);
                    SkinSpec::default()
                        .grow(&mass.shape, skin_seed)
                        .moved(&mass.pose)
                })
                .collect(),
        }
    }

    /// Limb `l`'s bark with its base drawn as `base`, in the frame at `at`
    /// in the plant's (its collider's). A limb that never breaks off is
    /// always drawn whole.
    pub fn bark(&self, l: usize, base: LimbBase, at: &Isometry3<f32>) -> Arc<Look> {
        let skin = match (base, &self.snapped[l]) {
            (LimbBase::Snapped, Some(snapped)) => snapped,
            _ => &self.whole[l],
        };
        Self::look(skin, at, self.look.bark)
    }

    /// Crown `m`'s leaves, in the frame at `at` in the plant's.
    pub fn leaves(&self, m: usize, at: &Isometry3<f32>) -> Arc<Look> {
        Self::look(&self.leaves[m], at, self.look.leaves)
    }

    /// `skin` drawn in `material`, moved from the plant's frame into the one
    /// at `at`.
    fn look(skin: &Skin, at: &Isometry3<f32>, material: MaterialId) -> Arc<Look> {
        Arc::new(Look {
            primitives: vec![skin.clone().moved(&at.inverse()).primitive(material)],
        })
    }
}
