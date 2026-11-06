//! A plant's ECS components.

use std::sync::Arc;

use specs::{Component, VecStorage};

use super::bodies::{Joinery, PlantPieceBody};
use super::drawing::PlantDrawing;
use super::plant::Plant;
use crate::foliage::{Foliage, Rooting};
use crate::model::Model;

/// A plant as grown: how it is built, joined and drawn. Shared by every
/// piece of it.
#[derive(Debug)]
pub struct GrownPlant {
    /// Its limbs and leaves.
    pub plant: Plant,
    /// How its bodies are made and broken.
    pub joinery: Joinery,
    /// Its meshes.
    pub drawing: PlantDrawing,
}

/// A piece of a plant: the whole of it until a branch breaks off, which is
/// then a piece of its own.
///
/// On the piece's entity, beside its `RigidBodyComponent`, the
/// `ModelInstance` drawing it and the `Foliage` of its crowns together, which
/// a break rebuilds.
#[derive(Component)]
#[storage(VecStorage)]
pub struct PlantPiece {
    /// The plant it is a piece of.
    pub plant: Arc<GrownPlant>,
    /// Its body and which limbs it holds.
    pub body: PlantPieceBody,
}

impl PlantPiece {
    /// How the piece is drawn as it is now.
    pub fn model(&self) -> Arc<Model> {
        self.plant
            .drawing
            .model(&self.plant.plant, &self.body.limbs(), &self.body.frame)
    }

    /// The leaves of every crown the piece holds, as one clump riding on its
    /// body. `None` if it holds none.
    pub fn foliage(&self) -> Option<Foliage> {
        let plant = &self.plant.plant;
        let masses = plant.masses_on(&self.body.limbs());
        if masses.is_empty() {
            return None;
        }
        let rooting = Rooting::Loose {
            body: self.body.body,
            offset: self.body.frame,
        };
        let mut clump = Foliage::new(Arc::new(plant.leaves_of(&masses)), rooting);
        clump.core = self.plant.joinery.core;
        Some(clump)
    }
}
