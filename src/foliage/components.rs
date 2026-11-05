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

/// Whether a clump stands where it grew, is held by a plant, or has been
/// torn loose.
#[derive(Debug, Clone, Copy)]
pub enum Rooting {
    /// Growing from the ground at `pose`: no body, nothing collides with it,
    /// only its leaves' drag acts. A blast near it tears it loose.
    Rooted {
        /// Where the clump's frame is.
        pose: Isometry3<f32>,
        /// How far its leaves reach below the ground, m: a clump torn loose
        /// is lifted by this, so its core does not start in the ground.
        sunk: f32,
    },
    /// Held up by a plant's limb at `pose`: as a rooted clump, but a blast
    /// does not tear it loose, since its limb holds it.
    OnLimb {
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
    /// The skin at rest, in the clump's frame, as the model it is drawn as.
    pub skin: Arc<Model>,
    /// How its core is built when it is torn loose.
    pub core: LeafCore,
    /// Rooted, on a limb or loose.
    pub rooting: Rooting,
    /// The core's velocity last frame, to tell a landing from resting.
    pub last_velocity: Vector3<f32>,
}

impl Foliage {
    /// A clump of `shape` drawn as `skin` in `material`, its skin's mesh
    /// handed to the model it is drawn as.
    pub fn grown(
        shape: Arc<FoliageShape>,
        skin: Skin,
        material: MaterialId,
        rooting: Rooting,
    ) -> Self {
        let primitive = MeshPrimitive {
            vertices: skin.vertices,
            indices: skin.indices,
            material,
        };
        Self {
            shape,
            skin: Arc::new(Model::flat(vec![ModelPart::new(vec![primitive])])),
            core: LeafCore::default(),
            rooting,
            last_velocity: Vector3::zeros(),
        }
    }

    /// The core's body, if the clump is loose.
    pub fn body(&self) -> Option<RigidBodyHandle> {
        match self.rooting {
            Rooting::Loose { body } => Some(body),
            Rooting::Rooted { .. } | Rooting::OnLimb { .. } => None,
        }
    }

    /// Its leaves as drag sees them.
    pub fn drag_volume(&self) -> DragVolume {
        DragVolume {
            shape: self.shape.clone(),
            frame: match self.rooting {
                Rooting::Rooted { pose, .. } | Rooting::OnLimb { pose } => VolumeFrame::Fixed(pose),
                Rooting::Loose { body } => VolumeFrame::Body(body),
            },
        }
    }

    /// Where the clump's frame is now: its pose if rooted or on a limb, its
    /// core's if loose. `None` if its core has gone.
    pub fn pose(&self, world: &PhysicsWorld) -> Option<Isometry3<f32>> {
        match self.rooting {
            Rooting::Rooted { pose, .. } | Rooting::OnLimb { pose } => Some(pose),
            Rooting::Loose { body } => world
                .body(body)
                .map(|b| Isometry3::from_parts(b.position().coords.into(), b.rotation())),
        }
    }

    /// The skin as its entity's model, uploaded once and drawn in the
    /// clump's frame wherever the clump goes.
    pub fn model(&self) -> ModelInstance {
        ModelInstance::new(self.skin.clone())
    }
}
