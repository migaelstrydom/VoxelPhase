//! Handle types for physics objects.
//!
//! Handles are opaque identifiers that reference objects stored in the PhysicsWorld.
//! They use generational indices to detect use-after-free.

use generational_arena::Index;

/// Handle to a rigid body in the physics world.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct RigidBodyHandle(pub(crate) Index);

/// Handle to a collider attached to a rigid body.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct ColliderHandle(pub(crate) Index);

impl RigidBodyHandle {
    /// Expose the raw index parts for diagnostics and deterministic ordering.
    pub(crate) fn raw_parts(self) -> (usize, u64) {
        self.0.into_raw_parts()
    }
}

impl ColliderHandle {
    /// Expose the raw index parts for deterministic ordering.
    pub(crate) fn raw_parts(self) -> (usize, u64) {
        self.0.into_raw_parts()
    }
}
