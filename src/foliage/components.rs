//! A clump of foliage in the world.

use std::sync::Arc;

use nalgebra::{Isometry3, Vector3};
use specs::{Component, VecStorage};

use super::drag::{DragVolume, VolumeFrame};
use super::leaf_core::LeafCore;
use super::shape::FoliageShape;
use super::skin::Skin;
use crate::components::ModelInstance;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::rendering::material::MaterialId;

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

/// A clump of leaves: its shape, its drawn skin and how it is rooted. Drawn
/// as its entity's `ModelInstance` ([`Foliage::model`]), which rides on the
/// clump's frame.
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
    /// Rooted or loose.
    pub rooting: Rooting,
    /// The core's velocity last frame, to tell a landing from resting.
    pub last_velocity: Vector3<f32>,
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
            rooting: Rooting::Rooted { pose },
            last_velocity: Vector3::zeros(),
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

    /// Where the clump's frame is now: its pose if rooted, its core's if
    /// loose. `None` if its core has gone.
    pub fn pose(&self, world: &PhysicsWorld) -> Option<Isometry3<f32>> {
        match self.rooting {
            Rooting::Rooted { pose } => Some(pose),
            Rooting::Loose { body } => world
                .body(body)
                .map(|b| Isometry3::from_parts(b.position().coords.into(), b.rotation())),
        }
    }

    /// The skin as a model, uploaded once and drawn in the clump's frame
    /// wherever the clump goes.
    pub fn model(&self) -> ModelInstance {
        let skin = MeshPrimitive {
            vertices: self.skin.vertices.clone(),
            indices: self.skin.indices.clone(),
            material: self.material,
        };
        ModelInstance::new(Arc::new(Model::flat(vec![ModelPart::new(vec![skin])])))
    }
}
