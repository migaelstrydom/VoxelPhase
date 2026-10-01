//! Seam filter: MeshPatch → FilteredPatch.
//!
//! Classifies each triangle edge as internal (coplanar neighbor — a mesh
//! seam) or boundary (no neighbor, or a crease/non-coplanar neighbor).
//!
//! Coplanar triangle pairs that share an edge are merged into a single
//! convex quad when possible. This eliminates contact reducer flickering
//! caused by clipping against separate triangles that represent the same
//! physical surface. Pairs that would form a concave quad, or whose fold
//! puts the quad off one plane, are left as individual triangles.
//!
//! Boundary and crease edges are emitted as ContactEdges. Internal seam
//! edges between coplanar neighbors are suppressed.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::FeatureId;
use crate::collision::mesh_patch::MeshPatch;
use crate::collision::triangle::Triangle;
use crate::collision::SurfaceId;

/// Cosine threshold above which two neighbouring faces count as coplanar and
/// may merge. Shared by every production caller of [`filter_patch`] so the
/// discrete narrowphase and the CCD sweep agree on what a seam is — they
/// generate contacts against the same surfaces and must not disagree about
/// which of them are creases.
pub const COPLANAR_DOT: f32 = 0.98;

/// Furthest, in metres, the second triangle's free vertex may lie off the
/// first triangle's plane for the pair to merge into one quad.
///
/// A merged quad is one plane: the first triangle's normal through its
/// vertices. [`COPLANAR_DOT`] bounds the fold's angle, not how far that plane
/// strays over the second triangle, which grows with the second triangle's
/// size. A 2.5 cm sliver at a crater's lip, folded 11° to the flat cell beside
/// it, carried its slope 0.5 m across that cell and stood 10 cm proud of the
/// ground: a slab resting there was pushed up and sideways every frame it came
/// within reach. Well under the solver's 5 mm slop.
const MERGE_PLANARITY: f32 = 1.0e-3;

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
    /// What the face is made of. A merged quad has one surface: only
    /// triangles of the same surface merge.
    pub surface: SurfaceId,
}

/// A boundary or crease edge, valid as a contact feature.
///
/// Carries the adjacent face normals for Gauss map filtering in edge-edge
/// SAT tests. `normal_a` is always present (the face this edge belongs to).
/// `normal_b` is the neighbor face normal for crease edges, or `None` for
/// true boundary edges (no neighbor in the patch).
#[derive(Debug, Clone)]
pub struct ContactEdge {
    pub a: Point3<f32>,
    pub b: Point3<f32>,
    pub feature_id: FeatureId,
    /// Outward normal of the face this edge belongs to.
    pub normal_a: Vector3<f32>,
    /// Outward normal of the neighboring face (crease edge), or `None`
    /// for true boundary edges where no neighbor exists.
    pub normal_b: Option<Vector3<f32>>,
    /// Surface of the face this edge belongs to.
    pub surface: SurfaceId,
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
            if normal_i.dot(&normal_j) < coplanar_dot_threshold
                || patch.triangles[j].surface != pt_i.surface
            {
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
                    surface: pt_i.surface,
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
                surface: pt_i.surface,
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
            let neighbor_normal = pt.neighbors[edge_idx as usize]
                .map(|nbr| patch.triangles[nbr as usize].triangle.normal());

            let is_internal = match neighbor_normal {
                None => false,
                Some(nbr_normal) => normal.dot(&nbr_normal) >= coplanar_dot_threshold,
            };

            if !is_internal {
                let (a, b) = pt.triangle.edge(edge_idx as usize);
                boundary_edges.push(ContactEdge {
                    a,
                    b,
                    feature_id: FeatureId::from_edge_pair(ti, edge_idx),
                    normal_a: normal,
                    normal_b: neighbor_normal,
                    surface: pt.surface,
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
/// normal, or `None` if the result would be concave or the second triangle
/// lies off the first one's plane by more than [`MERGE_PLANARITY`].
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
    if (non_shared_j - shared_a).dot(normal).abs() > MERGE_PLANARITY {
        return None;
    }

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
                    surface: SurfaceId::UNSPECIFIED,
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [Some(0), None, None],
                    surface: SurfaceId::UNSPECIFIED,
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
                    surface: SurfaceId::UNSPECIFIED,
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(1.0, 1.0, 0.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                    surface: SurfaceId::UNSPECIFIED,
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

    /// A merged face has one surface, so two materials meeting on a flat
    /// stay two faces — but the line between them is still no edge.
    #[test]
    fn coplanar_triangles_of_two_surfaces_stay_apart() {
        let mut patch = flat_quad_patch();
        patch.triangles[0].surface = SurfaceId(1);
        patch.triangles[1].surface = SurfaceId(2);
        let filtered = filter_patch(&patch, 0.98);

        let surfaces: Vec<_> = filtered.faces.iter().map(|f| f.surface).collect();
        assert_eq!(surfaces, vec![SurfaceId(1), SurfaceId(2)]);
        assert_eq!(filtered.boundary_edges.len(), 4);
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
                surface: SurfaceId::UNSPECIFIED,
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
                    surface: SurfaceId::UNSPECIFIED,
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [Some(0), Some(2), None],
                    surface: SurfaceId::UNSPECIFIED,
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 2.0),
                    ),
                    neighbors: [Some(1), None, Some(3)],
                    surface: SurfaceId::UNSPECIFIED,
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 2.0),
                        Point3::new(1.0, 0.0, 2.0),
                    ),
                    neighbors: [None, None, Some(2)],
                    surface: SurfaceId::UNSPECIFIED,
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
                    surface: SurfaceId::UNSPECIFIED,
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
                    surface: SurfaceId::UNSPECIFIED,
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

    /// A sliver at a crater's lip beside a flat cell, as a blast left them
    /// under the temple: within [`COPLANAR_DOT`] of each other, but the
    /// sliver's plane stands 10 cm over the cell's far edge.
    #[test]
    fn a_sliver_does_not_tilt_the_flat_cell_beside_it() {
        let patch = MeshPatch {
            triangles: vec![
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(36.0, 4.005, 69.5),
                        Point3::new(36.5, 4.005, 69.5),
                        Point3::new(36.0, 4.0, 69.475),
                    ),
                    neighbors: [Some(1), None, None],
                    surface: SurfaceId::UNSPECIFIED,
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(36.5, 4.005, 69.5),
                        Point3::new(36.0, 4.005, 69.5),
                        Point3::new(36.0, 4.005, 70.0),
                    ),
                    neighbors: [Some(0), None, None],
                    surface: SurfaceId::UNSPECIFIED,
                },
            ],
        };
        let [sliver, cell] =
            [&patch.triangles[0], &patch.triangles[1]].map(|t| t.triangle.normal());
        assert!(
            sliver.dot(&cell) >= COPLANAR_DOT,
            "the pair should count as coplanar"
        );

        let filtered = filter_patch(&patch, COPLANAR_DOT);

        for face in &filtered.faces {
            for vertex in &face.vertices {
                let off = (vertex - face.vertices[0]).dot(&face.normal).abs();
                assert!(
                    off <= MERGE_PLANARITY,
                    "a face vertex lies {off:.4} m off its plane"
                );
            }
        }
        assert_eq!(filtered.faces.len(), 2);
    }

    #[test]
    fn boundary_edges_have_normal_a_from_owning_face() {
        let patch = MeshPatch {
            triangles: vec![PatchTriangle {
                triangle: Triangle::new(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 0.0, 1.0),
                ),
                neighbors: [None, None, None],
                surface: SurfaceId::UNSPECIFIED,
            }],
        };
        let filtered = filter_patch(&patch, 0.98);

        assert_eq!(filtered.boundary_edges.len(), 3);
        let expected_normal = patch.triangles[0].triangle.normal();
        for edge in &filtered.boundary_edges {
            let dot = edge.normal_a.dot(&expected_normal);
            assert!(
                (dot - 1.0).abs() < 1e-5,
                "normal_a should match owning triangle normal, dot={}",
                dot
            );
            assert!(
                edge.normal_b.is_none(),
                "No-neighbor edge should have normal_b = None"
            );
        }
    }

    #[test]
    fn crease_edges_have_both_normals() {
        let patch = l_shaped_patch();
        let filtered = filter_patch(&patch, 0.98);

        let crease_edges: Vec<_> = filtered
            .boundary_edges
            .iter()
            .filter(|e| e.normal_b.is_some())
            .collect();

        // The L-shaped patch has one shared crease edge, emitted from each
        // side → 2 crease-edge entries with both normals populated.
        assert_eq!(
            crease_edges.len(),
            2,
            "Crease edge should be emitted from both adjacent triangles"
        );

        let n0 = patch.triangles[0].triangle.normal();
        let n1 = patch.triangles[1].triangle.normal();

        for edge in &crease_edges {
            let na = edge.normal_a;
            let nb = edge.normal_b.unwrap();

            // normal_a and normal_b should be the two distinct triangle normals.
            let matches_n0_n1 = na.dot(&n0) > 0.99 && nb.dot(&n1) > 0.99;
            let matches_n1_n0 = na.dot(&n1) > 0.99 && nb.dot(&n0) > 0.99;
            assert!(
                matches_n0_n1 || matches_n1_n0,
                "Crease edge normals should be the two triangle normals, got na={:?} nb={:?}",
                na,
                nb
            );
        }
    }

    #[test]
    fn coplanar_suppressed_edges_have_no_entries() {
        let patch = flat_quad_patch();
        let filtered = filter_patch(&patch, 0.98);

        // The internal diagonal is suppressed — no boundary edge should have
        // both normals pointing the same direction (which would mean a
        // coplanar edge leaked through).
        for edge in &filtered.boundary_edges {
            if let Some(nb) = edge.normal_b {
                let dot = edge.normal_a.dot(&nb);
                assert!(
                    dot < 0.98,
                    "Coplanar edge should be suppressed, got normal_a·normal_b={}",
                    dot
                );
            }
        }
    }

    #[test]
    fn crease_edge_normals_are_unit_length() {
        let patch = l_shaped_patch();
        let filtered = filter_patch(&patch, 0.98);

        for edge in &filtered.boundary_edges {
            let len_a = edge.normal_a.magnitude();
            assert!(
                (len_a - 1.0).abs() < 1e-5,
                "normal_a should be unit, got {}",
                len_a
            );
            if let Some(nb) = edge.normal_b {
                let len_b = nb.magnitude();
                assert!(
                    (len_b - 1.0).abs() < 1e-5,
                    "normal_b should be unit, got {}",
                    len_b
                );
            }
        }
    }
}
