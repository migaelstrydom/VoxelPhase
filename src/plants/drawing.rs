//! How a plant is drawn: each limb's bark and each crown's leaves, meshed
//! once in the plant's frame, from which any piece of it is drawn.
//!
//! ```text
//!   Plant ──PlantDrawing::grow──▶ bark per limb (whole, and snapped for a branch)
//!                                 leaves per crown
//!   PlantDrawing + a piece's limbs + its frame ──model──▶ one bark primitive,
//!                                                          one leaf primitive
//! ```

use std::sync::Arc;

use nalgebra::{Isometry3, Translation3};

use super::bark::{BarkSpec, LimbBase};
use super::limbs::LimbKind;
use super::plant::Plant;
use crate::foliage::{Skin, SkinSpec};
use crate::model::{Model, ModelPart};
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

    /// The model of the piece of `plant` holding `limbs`, with the plant's
    /// frame at `frame` in the piece's: a branch whose parent is not in the
    /// piece is drawn snapped off it.
    pub fn model(&self, plant: &Plant, limbs: &[usize], frame: &Isometry3<f32>) -> Arc<Model> {
        let bark = limbs.iter().map(|&l| {
            let broken = plant.limbs.limbs[l]
                .parent
                .is_some_and(|p| !limbs.contains(&p));
            match (&self.snapped[l], broken) {
                (Some(snapped), true) => snapped,
                _ => &self.whole[l],
            }
        });
        let leaves = plant.masses_on(limbs).into_iter().map(|m| &self.leaves[m]);
        let mut primitives = vec![joined(bark).moved(frame).primitive(self.look.bark)];
        let leaves = joined(leaves);
        if !leaves.indices.is_empty() {
            primitives.push(leaves.moved(frame).primitive(self.look.leaves));
        }
        Arc::new(Model::flat(vec![ModelPart::new(primitives)]))
    }
}

/// `skins` as one mesh.
fn joined<'a>(skins: impl Iterator<Item = &'a Skin>) -> Skin {
    let mut all = Skin {
        vertices: Vec::new(),
        indices: Vec::new(),
    };
    for skin in skins {
        let first = all.vertices.len() as u32;
        all.vertices.extend_from_slice(&skin.vertices);
        all.indices
            .extend(skin.indices.iter().map(|index| first + index));
    }
    all
}
