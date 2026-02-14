//! Seam filter: MeshPatch → FilteredPatch.
//!
//! Classifies each triangle edge as internal (coplanar neighbor — a mesh
//! seam) or boundary (no neighbor, or a crease/non-coplanar neighbor).
//! Each triangle is emitted as an individual ContactFace (always convex),
//! and only boundary edges are emitted as ContactEdges. This eliminates
//! phantom contacts on internal mesh seams without attempting polygon
//! merging, which can produce concave polygons that break point-in-polygon
//! and Sutherland-Hodgman clipping.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::FeatureId;
use crate::collision::mesh_patch::MeshPatch;

/// A MeshPatch after seam filtering: individual triangle faces and
/// boundary/crease edges only.
#[derive(Debug, Clone)]
pub struct FilteredPatch {
    /// Individual triangle contact faces.
    pub faces: SmallVec<[ContactFace; 4]>,

    /// Boundary and crease edges that are valid contact features.
    /// Internal mesh seam edges between coplanar neighbors are excluded.
    pub boundary_edges: SmallVec<[ContactEdge; 8]>,
}

/// A single triangle contact face.
#[derive(Debug, Clone)]
pub struct ContactFace {
    /// Vertices of the triangle.
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

/// Filter a mesh patch: emit each triangle as a face, suppress internal
/// seam edges.
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

    let mut faces = SmallVec::new();
    let mut boundary_edges = SmallVec::new();

    for (i, pt) in patch.triangles.iter().enumerate() {
        let normal = pt.triangle.normal();
        let ti = i as u32;

        faces.push(ContactFace {
            vertices: SmallVec::from_buf_and_len(
                [
                    pt.triangle.v0,
                    pt.triangle.v1,
                    pt.triangle.v2,
                    Point3::origin(),
                    Point3::origin(),
                    Point3::origin(),
                ],
                3,
            ),
            normal,
            feature_id: FeatureId::from_face(ti),
        });

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::mesh_patch::PatchTriangle;
    use crate::collision::sphere_triangle::Triangle;

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
    fn flat_quad_produces_two_faces() {
        let patch = flat_quad_patch();
        let filtered = filter_patch(&patch, 0.98);

        assert_eq!(
            filtered.faces.len(),
            2,
            "Each triangle should be a separate face"
        );
        for face in &filtered.faces {
            assert_eq!(face.vertices.len(), 3);
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

        assert_eq!(filtered.faces.len(), 2);
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
    fn feature_id_deterministic() {
        let patch = flat_quad_patch();
        let a = filter_patch(&patch, 0.98);
        let b = filter_patch(&patch, 0.98);
        assert_eq!(a.faces[0].feature_id, b.faces[0].feature_id);
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
}
