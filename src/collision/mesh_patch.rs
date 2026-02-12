//! Mesh patch returned by broad-phase static geometry queries.
//!
//! A `MeshPatch` is a localized fragment of a triangle mesh, containing
//! the triangles that intersect a query region along with adjacency
//! information. This allows narrowphase collision code to reason about
//! triangle neighborhoods (e.g., ignoring interior edges between
//! coplanar triangles).

use super::Triangle;

/// A triangle within a [`MeshPatch`], with local adjacency information.
#[derive(Debug, Clone)]
pub struct PatchTriangle {
    /// The collision triangle (three vertices).
    pub triangle: Triangle,

    /// For each edge (0 = v0→v1, 1 = v1→v2, 2 = v2→v0), the index of
    /// the neighboring triangle within this patch, if it exists.
    ///
    /// `None` means the edge is a boundary edge (the neighbor either
    /// doesn't exist or wasn't included in this patch's query region).
    pub neighbors: [Option<u32>; 3],
}

/// A localized fragment of a triangle mesh returned by broad-phase queries.
///
/// Contains all triangles intersecting the query AABB, plus adjacency
/// links between triangles that share an edge within this patch.
/// Triangles in the same patch may form one or more connected clusters;
/// flood-filling through `neighbors` reveals them.
#[derive(Debug, Clone)]
pub struct MeshPatch {
    /// The triangles in this patch, with per-triangle adjacency.
    pub triangles: Vec<PatchTriangle>,
}
