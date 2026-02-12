//! Trait for static geometry that physics bodies can collide against.
//!
//! This provides a broad-phase interface: the physics engine queries a
//! region and receives a [`MeshPatch`] containing all triangles in that
//! region, along with adjacency information. Narrowphase collision
//! detection (sphere-triangle, OBB-triangle, swept tests) is handled
//! by the physics engine itself.

use crate::collision::{MeshPatch, AABB};

/// Trait for querying collision geometry from static world structures.
///
/// Implementors perform the broad-phase spatial query and return a
/// [`MeshPatch`] with triangles and adjacency data. All narrowphase
/// collision testing is done by the caller.
pub trait StaticGeometry {
    /// Query all triangles intersecting an AABB region.
    ///
    /// Returns a [`MeshPatch`] containing the triangles and their
    /// local adjacency relationships within the queried region.
    fn query_region(&self, aabb: &AABB) -> MeshPatch;
}
