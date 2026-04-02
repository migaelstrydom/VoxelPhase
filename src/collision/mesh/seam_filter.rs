//! Seam filter: MeshPatch → FilteredPatch.
//!
//! Classifies each triangle edge as internal (coplanar neighbor — a mesh
//! seam) or boundary (no neighbor, or a crease/non-coplanar neighbor).
//!
//! Coplanar triangle pairs that share an edge are merged into a single
//! convex quad when possible. This eliminates contact reducer flickering
//! caused by clipping against separate triangles that represent the same
//! physical surface. Pairs that would form a concave quad are left as
//! individual triangles.
//!
//! Boundary and crease edges are emitted as ContactEdges. Internal seam
//! edges between coplanar neighbors are suppressed.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::FeatureId;
use crate::collision::mesh_patch::MeshPatch;
use crate::collision::sphere_triangle::Triangle;

/// A MeshPatch after seam filtering: contact faces (triangles or merged
/// quads) and boundary/crease edges only.
#[derive(Debug, Clone)]
pub struct FilteredPatch {
    /// Contact faces: individual triangles or merged convex quads.
    pub faces: SmallVec<[ContactFace; 4]>,

    /// Boundary and crease edges that are valid contact features.
    /// Internal mesh seam edges between coplanar neighbors are excluded.
    pub boundary_edges: SmallVec<[ContactEdge; 8]>,
}

/// A convex contact face (triangle or quad).
#[derive(Debug, Clone)]
pub struct ContactFace {
    /// Vertices of the face (3 for triangle, 4 for merged quad).
    pub vertices: SmallVec<[Point3<f32>; 6]>,

    /// Outward face normal.
    pub normal: Vector3<f32>,

    /// Feature ID for manifold persistence.
    pub feature_id: FeatureId,
}

/// A boundary or crease edge, valid as a contact feature.
#[derive(Debug, Clone)]
pub struct ContactEdge {
    pub a: Point3<f32>,
    pub b: Point3<f32>,
    pub feature_id: FeatureId,
}

/// Filter a mesh patch: merge coplanar triangle pairs into convex quads
/// where possible, emit remaining triangles individually, and suppress
/// internal seam edges.
///
/// An edge is classified as internal (suppressed) when its neighbor exists
/// and the two triangles are coplanar within `coplanar_dot_threshold`.
/// All other edges (no neighbor, or crease angle) are emitted as boundary
/// ContactEdges.
pub fn filter_patch(patch: &MeshPatch, coplanar_dot_threshold: f32) -> FilteredPatch {
    if patch.triangles.is_empty() {
        return FilteredPatch {
            faces: SmallVec::new(),
            boundary_edges: SmallVec::new(),
        };
    }

    let mut merged = vec![false; patch.triangles.len()];
    let mut faces = SmallVec::new();
    let mut boundary_edges = SmallVec::new();

    // Pass 1: try to merge coplanar pairs into convex quads.
    for i in 0..patch.triangles.len() {
        if merged[i] {
            continue;
        }
        let pt_i = &patch.triangles[i];
        let normal_i = pt_i.triangle.normal();

        let mut did_merge = false;
        for edge_idx in 0..3usize {
            let Some(j) = pt_i.neighbors[edge_idx] else {
                continue;
            };
            let j = j as usize;
            if merged[j] {
                continue;
            }

            let normal_j = patch.triangles[j].triangle.normal();
            if normal_i.dot(&normal_j) < coplanar_dot_threshold {
                continue;
            }

            if let Some(quad) = try_merge_coplanar(
                &pt_i.triangle,
                edge_idx,
                &patch.triangles[j].triangle,
                &normal_i,
            ) {
                let lo = i.min(j) as u32;
                let hi = i.max(j) as u32;
                faces.push(ContactFace {
                    vertices: SmallVec::from_buf_and_len(
                        [
                            quad[0],
                            quad[1],
                            quad[2],
                            quad[3],
                            Point3::origin(),
                            Point3::origin(),
                        ],
                        4,
                    ),
                    normal: normal_i,
                    feature_id: FeatureId::from_triangle_set(&[lo, hi]),
                });
                merged[i] = true;
                merged[j] = true;
                did_merge = true;
                break;
            }
        }

        if !did_merge {
            faces.push(ContactFace {
                vertices: SmallVec::from_buf_and_len(
                    [
                        pt_i.triangle.v0,
                        pt_i.triangle.v1,
                        pt_i.triangle.v2,
                        Point3::origin(),
                        Point3::origin(),
                        Point3::origin(),
                    ],
                    3,
                ),
                normal: normal_i,
                feature_id: FeatureId::from_face(i as u32),
            });
        }
    }

    // Pass 2: emit boundary edges for all triangles (merged or not).
    // The shared coplanar edge between merged pairs is still suppressed
    // by the coplanar check; the outer edges of both triangles are emitted.
    for (i, pt) in patch.triangles.iter().enumerate() {
        let normal = pt.triangle.normal();
        let ti = i as u32;

        for edge_idx in 0..3u32 {
            let is_internal = match pt.neighbors[edge_idx as usize] {
                None => false,
                Some(nbr) => {
                    let nbr_normal = patch.triangles[nbr as usize].triangle.normal();
                    normal.dot(&nbr_normal) >= coplanar_dot_threshold
                }
            };

            if !is_internal {
                let (a, b) = pt.triangle.edge(edge_idx as usize);
                boundary_edges.push(ContactEdge {
                    a,
                    b,
                    feature_id: FeatureId::from_edge_pair(ti, edge_idx),
                });
            }
        }
    }

    FilteredPatch {
        faces,
        boundary_edges,
    }
}

/// Try to merge two coplanar triangles sharing an edge into a convex quad.
///
/// Returns the 4 quad vertices in winding order consistent with the face
/// normal, or `None` if the result would be concave.
fn try_merge_coplanar(
    tri_i: &Triangle,
    edge_idx: usize,
    tri_j: &Triangle,
    normal: &Vector3<f32>,
) -> Option<[Point3<f32>; 4]> {
    let (shared_a, shared_b) = tri_i.edge(edge_idx);

    let non_shared_i = match edge_idx {
        0 => tri_i.v2,
        1 => tri_i.v0,
        _ => tri_i.v1,
    };

    // Find the vertex of tri_j not on the shared edge.
    let non_shared_j = [tri_j.v0, tri_j.v1, tri_j.v2].into_iter().find(|v| {
        (v - shared_a).magnitude_squared() > 1e-8 && (v - shared_b).magnitude_squared() > 1e-8
    })?;

    // Quad winding: non_shared_i → shared_a → non_shared_j → shared_b
    // This preserves the original triangle's winding direction.
    let quad = [non_shared_i, shared_a, non_shared_j, shared_b];

    // Convexity check: all consecutive edge cross products must agree
    // with the face normal.
    for k in 0..4 {
        let a = quad[k];
        let b = quad[(k + 1) % 4];
        let c = quad[(k + 2) % 4];
        let cross = (b - a).cross(&(c - b));
        if cross.dot(normal) < 1e-6 {
            return None;
        }
    }

    Some(quad)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::mesh_patch::PatchTriangle;

    fn flat_quad_patch() -> MeshPatch {
        // Two triangles forming a flat quad on the XZ plane at y=0.
        //
        //  (0,0,0) --- (1,0,0)
        //      |  \       |
        //      |    \     |
        //      |      \   |
        //  (0,0,1) --- (1,0,1)
        //
        // Tri 0: (0,0,0), (1,0,0), (1,0,1) — edge 2 (v2→v0) is shared
        // Tri 1: (0,0,0), (1,0,1), (0,0,1) — edge 0 (v0→v1) is shared
        MeshPatch {
            triangles: vec![
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(1)],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [Some(0), None, None],
                },
            ],
        }
    }

    fn l_shaped_patch() -> MeshPatch {
        // Two triangles forming an L-shape (crease at the shared edge).
        // Tri 0: flat on XZ plane (y=0)
        // Tri 1: tilted up at 90° from tri 0
        MeshPatch {
            triangles: vec![
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [Some(1), None, None],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(1.0, 1.0, 0.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
            ],
        }
    }

    #[test]
    fn flat_quad_merges_into_one_face() {
        let patch = flat_quad_patch();
        let filtered = filter_patch(&patch, 0.98);

        assert_eq!(
            filtered.faces.len(),
            1,
            "Coplanar triangle pair should merge into one quad, got {} faces",
            filtered.faces.len()
        );
        assert_eq!(
            filtered.faces[0].vertices.len(),
            4,
            "Merged face should have 4 vertices"
        );
    }

    #[test]
    fn flat_quad_merged_feature_id_is_deterministic() {
        let patch = flat_quad_patch();
        let a = filter_patch(&patch, 0.98);
        let b = filter_patch(&patch, 0.98);
        assert_eq!(a.faces[0].feature_id, b.faces[0].feature_id);
    }

    #[test]
    fn flat_quad_merged_quad_is_convex() {
        let patch = flat_quad_patch();
        let filtered = filter_patch(&patch, 0.98);
        let face = &filtered.faces[0];
        let n = face.normal;

        for k in 0..4 {
            let a = face.vertices[k];
            let b = face.vertices[(k + 1) % 4];
            let c = face.vertices[(k + 2) % 4];
            let cross = (b - a).cross(&(c - b));
            assert!(cross.dot(&n) > 0.0, "Merged quad vertex {k} is not convex");
        }
    }

    #[test]
    fn flat_quad_suppresses_internal_edge() {
        let patch = flat_quad_patch();
        let filtered = filter_patch(&patch, 0.98);

        // 6 total edges - 2 internal (shared coplanar diagonal, one from each side) = 4
        assert_eq!(
            filtered.boundary_edges.len(),
            4,
            "Internal coplanar edges should be suppressed, got {}",
            filtered.boundary_edges.len()
        );
    }

    #[test]
    fn l_shape_keeps_crease_edges() {
        let patch = l_shaped_patch();
        let filtered = filter_patch(&patch, 0.98);

        // Crease pair: no merge, 2 separate faces.
        assert_eq!(filtered.faces.len(), 2);
        for face in &filtered.faces {
            assert_eq!(face.vertices.len(), 3);
        }
        // 6 total edges, shared edge is a crease (not coplanar) so NOT suppressed = 6
        assert_eq!(
            filtered.boundary_edges.len(),
            6,
            "Crease edges should be kept as boundary edges"
        );
    }

    #[test]
    fn single_triangle_passes_through() {
        let patch = MeshPatch {
            triangles: vec![PatchTriangle {
                triangle: Triangle::new(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 0.0, 1.0),
                ),
                neighbors: [None, None, None],
            }],
        };
        let filtered = filter_patch(&patch, 0.98);

        assert_eq!(filtered.faces.len(), 1);
        assert_eq!(filtered.faces[0].vertices.len(), 3);
        assert_eq!(filtered.boundary_edges.len(), 3);
    }

    #[test]
    fn empty_patch() {
        let patch = MeshPatch { triangles: vec![] };
        let filtered = filter_patch(&patch, 0.98);
        assert!(filtered.faces.is_empty());
        assert!(filtered.boundary_edges.is_empty());
    }

    #[test]
    fn face_normal_is_unit() {
        let patch = flat_quad_patch();
        let filtered = filter_patch(&patch, 0.98);
        for face in &filtered.faces {
            let len = face.normal.magnitude();
            assert!(
                (len - 1.0).abs() < 1e-5,
                "Face normal should be unit length, got {}",
                len
            );
        }
    }

    #[test]
    fn four_triangle_strip_merges_two_pairs() {
        // Four coplanar triangles in a strip. Should produce 2 merged quads.
        //
        //  v0 --- v1 --- v2 --- v3
        //   |  t0  |  t1  |  t2  |
        //  v4 --- v5 --- v6 --- v7
        //
        // Actually using triangulated quads:
        // t0: v0, v1, v5  (edge 1: v1→v5 shared with t1)
        // t1: v1, v2, v5  (edge 2: v5→v1 shared with t0, edge 1: v2→v5 shared with t2)
        // t2: v2, v6, v5  (edge 2: v5→v2 shared with t1)
        // t3: v2, v3, v6  (no neighbor to t2 via shared edge here in this layout)
        //
        // Simpler: just 4 triangles where pairs share edges.
        let patch = MeshPatch {
            triangles: vec![
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(1)],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [Some(0), Some(2), None],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 2.0),
                    ),
                    neighbors: [Some(1), None, Some(3)],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 2.0),
                        Point3::new(1.0, 0.0, 2.0),
                    ),
                    neighbors: [None, None, Some(2)],
                },
            ],
        };
        let filtered = filter_patch(&patch, 0.98);

        // t0+t1 merge, t2+t3 merge → 2 quad faces.
        assert_eq!(
            filtered.faces.len(),
            2,
            "4 coplanar triangles should merge into 2 quads, got {} faces",
            filtered.faces.len()
        );
        for face in &filtered.faces {
            assert_eq!(face.vertices.len(), 4, "Each merged face should be a quad");
        }
    }

    #[test]
    fn concave_pair_not_merged() {
        // Two coplanar triangles that would form a concave (bowtie) quad.
        // Shared edge crosses through both non-shared vertices.
        //
        // Create a degenerate case: tri_i and tri_j share an edge but
        // the non-shared vertices are on the same side, making the quad
        // self-intersecting.
        let patch = MeshPatch {
            triangles: vec![
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                    ),
                    neighbors: [Some(1), None, None],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        // Shared edge v0→v1 = (0,0,0)→(2,0,0)
                        // but non-shared vertex at (0.5, 0, 0.3) — between
                        // shared_a and the other non-shared, making concave.
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(0.5, 0.0, -0.1),
                    ),
                    neighbors: [Some(0), None, None],
                },
            ],
        };
        let filtered = filter_patch(&patch, 0.98);

        // Should NOT merge because the quad would be concave.
        assert_eq!(
            filtered.faces.len(),
            2,
            "Concave pair should remain as 2 separate triangles"
        );
        for face in &filtered.faces {
            assert_eq!(face.vertices.len(), 3);
        }
    }
}
