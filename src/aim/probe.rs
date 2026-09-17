//! Casting against the rigid bodies while ignoring a few of them.

use nalgebra::{Point3, Vector3};

use crate::physics::{PhysicsWorld, RigidBodyHandle};
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
}

impl<'a> BodiesExcept<'a> {
    pub fn new(world: &'a PhysicsWorld, ignored: &'a [RigidBodyHandle]) -> Self {
        Self { world, ignored }
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
            .raycast_excluding(origin, direction, length, self.ignored)
    }
}
