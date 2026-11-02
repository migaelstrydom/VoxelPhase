//! Mesh patch returned by broad-phase static geometry queries.
//!
//! A `MeshPatch` is a localized fragment of a triangle mesh, containing
//! the triangles that intersect a query region along with adjacency
//! information. This allows narrowphase collision code to reason about
//! triangle neighborhoods (e.g., ignoring interior edges between
//! coplanar triangles).

use super::{SurfaceId, Triangle};

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

    /// What the triangle is made of, as the geometry that issued it names it.
    pub surface: SurfaceId,
}

impl PatchTriangle {
    /// A key naming this triangle whatever query returned it, for the feature
    /// IDs warm-starting matches contacts by.
    ///
    /// Its index in a patch will not do: the index follows the order the
    /// geometry walked its triangles in, and every triangle a moving body's
    /// query gains or loses shifts the ones after it. The triangle's own
    /// vertices are bit for bit the same each time it is returned, until the
    /// mesh around it is rebuilt.
    pub fn key(&self) -> u64 {
        let t = &self.triangle;
        [t.v0, t.v1, t.v2]
            .iter()
            .flat_map(|v| [v.x, v.y, v.z])
            .fold(0xcbf29ce484222325, |hash, c| {
                (hash ^ c.to_bits() as u64).wrapping_mul(0x100000001b3)
            })
    }
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
