//! Casting against the rigid bodies while ignoring a few of them, and any
//! the projectile would pass through.

use nalgebra::{Point3, Vector3};

use crate::physics::{Layers, PhysicsWorld, RigidBodyHandle};
use crate::sensing::{ProbeHit, ProbeTarget};

/// The physics world as a probe target, minus a handful of bodies.
///
/// A predicted flight starts inside the thrower and, for a held object, *is*
/// one of the bodies in the world. Casting against everything would report an
/// impact on the thrower's own capsule a few centimetres from the muzzle, so
/// both are named here and skipped.
pub struct BodiesExcept<'a> {
    world: &'a PhysicsWorld,
    ignored: &'a [RigidBodyHandle],
    /// The layers the projectile meets: a grenade's flight passes through
    /// leaves.
    layers: Layers,
}

impl<'a> BodiesExcept<'a> {
    pub fn new(world: &'a PhysicsWorld, ignored: &'a [RigidBodyHandle], layers: Layers) -> Self {
        Self {
            world,
            ignored,
            layers,
        }
    }
}

impl ProbeTarget for BodiesExcept<'_> {
    fn raycast(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Option<ProbeHit> {
        self.world
            .raycast_looking_for(origin, direction, length, self.ignored, self.layers)
    }
}
