//! A clump of foliage in the world.

use std::sync::Arc;

use nalgebra::{Isometry3, Vector3};
use specs::{Component, VecStorage};

use super::deform::{deform, Give, Press, Squash};
use super::drag::{DragVolume, VolumeFrame};
use super::leaf_core::LeafCore;
use super::shape::FoliageShape;
use super::skin::Skin;
use crate::model::MeshPrimitive;
use crate::physics::RigidBodyHandle;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// Whether a clump stands where it grew or has been torn loose.
#[derive(Debug, Clone, Copy)]
pub enum Rooting {
    /// Growing at `pose`: no body, nothing collides with it, only its leaves'
    /// drag acts.
    Rooted {
        /// Where the clump's frame is.
        pose: Isometry3<f32>,
    },
    /// Torn loose: its core is this dynamic body, whose frame is the clump's.
    Loose {
        /// The core.
        body: RigidBodyHandle,
    },
}

/// A clump of leaves: its shape, its drawn skin, and how that skin is giving
/// this frame. Drawn through the entity's `ProceduralMesh` while it moves,
/// and its `ModelInstance` while it is still (`FoliageSkinSystem`).
#[derive(Component)]
#[storage(VecStorage)]
pub struct Foliage {
    /// The lobes, in the clump's frame.
    pub shape: Arc<FoliageShape>,
    /// The skin at rest, in the clump's frame.
    pub skin: Arc<Skin>,
    /// What the skin is drawn in.
    pub material: MaterialId,
    /// How its core is built when it is torn loose.
    pub core: LeafCore,
    /// How its skin gives.
    pub give: Give,
    /// Rooted or loose.
    pub rooting: Rooting,
    /// The surfaces it is pressed against.
    pub press: Press,
    /// Its squash after a landing.
    pub squash: Squash,
    /// The core's velocity last frame, to tell a landing from resting.
    pub last_velocity: Vector3<f32>,
    /// Whether the skin drawn is still right for a clump that has not moved
    /// since: nothing to rebuild.
    pub still_drawn: bool,
}

impl Foliage {
    /// A rooted clump at `pose`.
    pub fn rooted(
        shape: Arc<FoliageShape>,
        skin: Arc<Skin>,
        material: MaterialId,
        pose: Isometry3<f32>,
    ) -> Self {
        Self {
            shape,
            skin,
            material,
            core: LeafCore::default(),
            give: Give::default(),
            rooting: Rooting::Rooted { pose },
            press: Press::default(),
            squash: Squash::default(),
            last_velocity: Vector3::zeros(),
            still_drawn: true,
        }
    }

    /// The core's body, if the clump is loose.
    pub fn body(&self) -> Option<RigidBodyHandle> {
        match self.rooting {
            Rooting::Loose { body } => Some(body),
            Rooting::Rooted { .. } => None,
        }
    }

    /// Its leaves as drag sees them.
    pub fn drag_volume(&self) -> DragVolume {
        DragVolume {
            shape: self.shape.clone(),
            frame: match self.rooting {
                Rooting::Rooted { pose } => VolumeFrame::Fixed(pose),
                Rooting::Loose { body } => VolumeFrame::Body(body),
            },
        }
    }

    /// The skin as it is giving now, for the clump at `pose`: pressed and
    /// squashed, or at rest if nothing is doing either. `scratch` is reused
    /// for the bent vertices.
    pub fn drawn(&self, pose: &Isometry3<f32>, scratch: &mut Vec<Vertex>) -> MeshPrimitive {
        if self.press.is_empty() && self.squash.is_settled() {
            return self.rest_primitive();
        }
        deform(
            &self.skin.vertices,
            pose,
            &self.press,
            &self.squash,
            &self.give,
            scratch,
        );
        self.primitive(std::mem::take(scratch))
    }

    /// The skin as drawn from `vertices`.
    fn primitive(&self, vertices: Vec<Vertex>) -> MeshPrimitive {
        MeshPrimitive {
            vertices,
            indices: self.skin.indices.clone(),
            material: self.material,
        }
    }

    /// The skin at rest, as drawn.
    fn rest_primitive(&self) -> MeshPrimitive {
        self.primitive(self.skin.vertices.clone())
    }
}
