//! Trait for static geometry that physics bodies can collide against.
//!
//! This provides a broad-phase interface: the physics engine queries a
//! region and receives a [`MeshPatch`] containing all triangles in that
//! region, along with adjacency information. Narrowphase collision
//! detection (sphere-triangle, OBB-triangle, swept tests) is handled
//! by the physics engine itself.

use crate::collision::{MeshPatch, Triangle, AABB};

/// Trait for querying collision geometry from static world structures.
///
/// Implementors perform the broad-phase spatial query and return a
/// [`MeshPatch`] with triangles and adjacency data. All narrowphase
/// collision testing is done by the caller.
///
/// Queries run from several threads at once, one per collider, so an
/// implementor must be `Sync`.
pub trait StaticGeometry: Sync {
    /// Query all triangles intersecting an AABB region.
    ///
    /// Returns a [`MeshPatch`] containing the triangles and their
    /// local adjacency relationships within the queried region.
    fn query_region(&self, aabb: &AABB) -> MeshPatch;

    /// Query triangles intersecting an AABB region, without adjacency.
    ///
    /// Swept tests and raycasts consume triangles individually and never read
    /// neighbour links, but resolving those links costs a hash map built over
    /// the whole patch plus three lookups per triangle. Implementors backed by
    /// a mesh should override this to skip that work; the default derives it
    /// from [`Self::query_region`] so existing implementors stay correct.
    fn query_region_triangles(&self, aabb: &AABB) -> Vec<Triangle> {
        self.query_region(aabb)
            .triangles
            .into_iter()
            .map(|pt| pt.triangle)
            .collect()
    }
}
