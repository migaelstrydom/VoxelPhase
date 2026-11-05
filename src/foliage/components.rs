//! A clump of foliage in the world.

use std::sync::Arc;

use nalgebra::{Isometry3, Vector3};
use specs::{Component, VecStorage};

use super::drag::{DragVolume, VolumeFrame};
use super::leaf_core::LeafCore;
use super::shape::FoliageShape;
use crate::physics::{PhysicsWorld, RigidBodyHandle};

/// Whether a clump stands where it grew or is on a core of its own.
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
    /// On a dynamic body: its own core, torn loose, or a plant's limb
    /// holding it up as its crown (`plants`).
    Loose {
        /// The body.
        body: RigidBodyHandle,
        /// Where the clump's frame sits in the body's: none on its own core.
        offset: Isometry3<f32>,
    },
}

/// A clump of leaves: its shape, its core and how it is rooted. Its skin is
/// drawn by its entity's `ModelInstance` ([`Skin::model`](super::Skin::model)), which rides on
/// the body or pose it is rooted by.
#[derive(Component)]
#[storage(VecStorage)]
pub struct Foliage {
    /// The lobes, in the clump's frame.
    pub shape: Arc<FoliageShape>,
    /// How its core is built when it is torn loose.
    pub core: LeafCore,
    /// Rooted or on a core.
    pub rooting: Rooting,
    /// The core's velocity last frame, to tell a landing from resting.
    pub last_velocity: Vector3<f32>,
}

impl Rooting {
    /// On `body` as its own core.
    pub fn on(body: RigidBodyHandle) -> Self {
        Self::Loose {
            body,
            offset: Isometry3::identity(),
        }
    }
}

impl Foliage {
    /// A clump of `shape`, rooted as `rooting`.
    pub fn new(shape: Arc<FoliageShape>, rooting: Rooting) -> Self {
        Self {
            shape,
            core: LeafCore::default(),
            rooting,
            last_velocity: Vector3::zeros(),
        }
    }

    /// The core's body, if the clump is loose.
    pub fn body(&self) -> Option<RigidBodyHandle> {
        match self.rooting {
            Rooting::Loose { body, .. } => Some(body),
            Rooting::Rooted { .. } => None,
        }
    }

    /// Its leaves as drag sees them.
    pub fn drag_volume(&self) -> DragVolume {
        DragVolume {
            shape: self.shape.clone(),
            frame: match self.rooting {
                Rooting::Rooted { pose, .. } => VolumeFrame::Fixed(pose),
                Rooting::Loose { body, offset } => VolumeFrame::Body { body, offset },
            },
        }
    }

    /// Where the clump's frame is now: its pose if rooted, its core's if
    /// loose. `None` if its core has gone.
    pub fn pose(&self, world: &PhysicsWorld) -> Option<Isometry3<f32>> {
        match self.rooting {
            Rooting::Rooted { pose, .. } => Some(pose),
            Rooting::Loose { body, offset } => world
                .body(body)
                .map(|b| Isometry3::from_parts(b.position().coords.into(), b.rotation()) * offset),
        }
    }
}
