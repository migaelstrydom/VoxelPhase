use std::collections::HashSet;

use crate::collision::MeshPatch;

use super::contact_source::{ContactFeature, SourcedContact};

/// Drop contacts created from interior mesh edges on near-coplanar neighbors.
pub fn filter_internal_edge_contacts(
    contacts: Vec<SourcedContact>,
    patch: &MeshPatch,
    coplanar_dot_threshold: f32,
) -> Vec<SourcedContact> {
    contacts
        .into_iter()
        .filter(|c| should_keep_edge_contact(c, patch, coplanar_dot_threshold))
        .collect()
}

/// Drop contacts created from interior mesh vertices surrounded by coplanar
/// triangles, but only when the batch already contains a face contact from the
/// source triangle or one of its coplanar neighbors at that vertex. This
/// prevents dropping load-bearing vertex contacts when no face support exists.
pub fn filter_internal_vertex_contacts(
    contacts: Vec<SourcedContact>,
    patch: &MeshPatch,
    coplanar_dot_threshold: f32,
) -> Vec<SourcedContact> {
    let face_tri_set = build_face_triangle_set(&contacts);
    contacts
        .into_iter()
        .filter(|c| should_keep_vertex_contact(c, patch, coplanar_dot_threshold, &face_tri_set))
        .collect()
}

/// Collect the set of triangle indices that have at least one Face contact.
fn build_face_triangle_set(contacts: &[SourcedContact]) -> HashSet<u32> {
    contacts
        .iter()
        .filter(|c| matches!(c.source.feature, ContactFeature::Face))
        .map(|c| c.source.triangle_idx)
        .collect()
}

fn should_keep_edge_contact(
    contact: &SourcedContact,
    patch: &MeshPatch,
    coplanar_dot_threshold: f32,
) -> bool {
    let tri_idx = contact.source.triangle_idx as usize;
    let Some(patch_tri) = patch.triangles.get(tri_idx) else {
        return true;
    };

    let ContactFeature::Edge(edge_idx) = contact.source.feature else {
        return true;
    };
    let edge_idx = edge_idx as usize;
    if edge_idx >= 3 {
        return true;
    }

    let Some(neighbor_idx) = patch_tri.neighbors[edge_idx] else {
        return true;
    };
    let Some(neighbor_tri) = patch.triangles.get(neighbor_idx as usize) else {
        return true;
    };

    let tri_n = patch_tri.triangle.normal();
    let nbr_n = neighbor_tri.triangle.normal();
    tri_n.dot(&nbr_n) < coplanar_dot_threshold
}

/// Returns the two edge indices (into `PatchTriangle::neighbors`) that meet
/// at the given vertex index.
///
/// Edge 0 = v0→v1, Edge 1 = v1→v2, Edge 2 = v2→v0, so:
/// - Vertex 0 sits at the junction of edges 2 and 0
/// - Vertex 1 sits at the junction of edges 0 and 1
/// - Vertex 2 sits at the junction of edges 1 and 2
fn edges_incident_to_vertex(vertex_idx: usize) -> [usize; 2] {
    match vertex_idx {
        0 => [2, 0],
        1 => [0, 1],
        _ => [1, 2],
    }
}

fn should_keep_vertex_contact(
    contact: &SourcedContact,
    patch: &MeshPatch,
    coplanar_dot_threshold: f32,
    face_tri_set: &HashSet<u32>,
) -> bool {
    let tri_idx = contact.source.triangle_idx as usize;
    let Some(patch_tri) = patch.triangles.get(tri_idx) else {
        return true;
    };

    let ContactFeature::Vertex(vert_idx) = contact.source.feature else {
        return true;
    };
    let vert_idx = vert_idx as usize;
    if vert_idx >= 3 {
        return true;
    }

    let tri_n = patch_tri.triangle.normal();
    let incident = edges_incident_to_vertex(vert_idx);

    // Collect coplanar neighbor indices (if both edges have coplanar neighbors).
    let mut coplanar_neighbor_ids = Vec::new();
    for &edge_idx in &incident {
        let Some(neighbor_idx) = patch_tri.neighbors[edge_idx] else {
            return true;
        };
        let Some(neighbor_tri) = patch.triangles.get(neighbor_idx as usize) else {
            return true;
        };
        let nbr_n = neighbor_tri.triangle.normal();
        if tri_n.dot(&nbr_n) < coplanar_dot_threshold {
            return true;
        }
        coplanar_neighbor_ids.push(neighbor_idx);
    }

    // Vertex is geometrically internal. Only drop it if there is face-based
    // support from this triangle or one of its coplanar neighbors at the vertex.
    if face_tri_set.contains(&(tri_idx as u32)) {
        return false;
    }
    for &nbr_idx in &coplanar_neighbor_ids {
        if face_tri_set.contains(&nbr_idx) {
            return false;
        }
    }

    // No face support nearby — keep this vertex contact as it may be load-bearing.
    true
}

#[cfg(test)]
mod tests {
    use generational_arena::Index;
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::collision::{PatchTriangle, Triangle};
    use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
    use crate::physics::narrowphase::contact_source::{ContactSource, SourcedContact};
    use crate::physics::pipeline::solver::ContactConstraint;

    fn make_contact(feature: ContactFeature) -> SourcedContact {
        let dummy_index = Index::from_raw_parts(0, 0);
        SourcedContact {
            constraint: ContactConstraint {
                body_a: None,
                body_b: RigidBodyHandle(dummy_index),
                collider_a: None,
                collider_b: Some(ColliderHandle(dummy_index)),
                point: Point3::new(0.5, 0.0, 0.0),
                normal: Vector3::new(0.0, 1.0, 0.0),
                raw_normal: Vector3::new(0.0, 1.0, 0.0),
                depth: 0.0,
                raw_depth: 0.0,
                restitution: 0.0,
                friction: 0.0,
                warm_normal_impulse: 0.0,
                warm_tangent_impulse: [0.0, 0.0],
            },
            source: ContactSource {
                triangle_idx: 0,
                feature,
            },
        }
    }

    #[test]
    fn drops_coplanar_internal_edge_contact() {
        let patch = MeshPatch {
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
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
            ],
        };
        let filtered = filter_internal_edge_contacts(
            vec![make_contact(ContactFeature::Edge(0))],
            &patch,
            0.99,
        );
        assert!(filtered.is_empty());
    }

    #[test]
    fn keeps_boundary_edge_contact() {
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
        let filtered = filter_internal_edge_contacts(
            vec![make_contact(ContactFeature::Edge(0))],
            &patch,
            0.99,
        );
        assert_eq!(filtered.len(), 1);
    }

    #[test]
    fn drops_coplanar_internal_vertex_when_face_support_exists() {
        // Vertex 1 of tri 0 (at 1,0,0) is shared by two coplanar neighbors
        // across edges 0 and 1. A face contact exists on tri 0, so the vertex
        // contact is redundant → drop.
        let patch = MeshPatch {
            triangles: vec![
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [Some(1), Some(2), None],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
            ],
        };
        let filtered = filter_internal_vertex_contacts(
            vec![
                make_contact(ContactFeature::Face),
                make_contact(ContactFeature::Vertex(1)),
            ],
            &patch,
            0.99,
        );
        assert_eq!(filtered.len(), 1);
        assert!(matches!(filtered[0].source.feature, ContactFeature::Face));
    }

    #[test]
    fn keeps_coplanar_internal_vertex_without_face_support() {
        // Same geometry as above but no face contact in the batch.
        // The vertex is the only support → keep.
        let patch = MeshPatch {
            triangles: vec![
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [Some(1), Some(2), None],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
            ],
        };
        let filtered = filter_internal_vertex_contacts(
            vec![make_contact(ContactFeature::Vertex(1))],
            &patch,
            0.99,
        );
        assert_eq!(filtered.len(), 1);
    }

    #[test]
    fn keeps_boundary_vertex_contact() {
        // Vertex 0 has no neighbors on either incident edge → boundary.
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
        let filtered = filter_internal_vertex_contacts(
            vec![make_contact(ContactFeature::Vertex(0))],
            &patch,
            0.99,
        );
        assert_eq!(filtered.len(), 1);
    }

    #[test]
    fn keeps_crease_vertex_contact() {
        // Vertex 1 has two neighbors but one forms a crease → keep.
        let patch = MeshPatch {
            triangles: vec![
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [Some(1), Some(2), None],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
                PatchTriangle {
                    triangle: Triangle::new(
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(1.0, 1.0, 0.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
            ],
        };
        let filtered = filter_internal_vertex_contacts(
            vec![make_contact(ContactFeature::Vertex(1))],
            &patch,
            0.99,
        );
        assert_eq!(filtered.len(), 1);
    }

    #[test]
    fn keeps_vertex_with_one_missing_neighbor() {
        // Vertex 1 has one coplanar neighbor (edge 0) but edge 1 is boundary.
        let patch = MeshPatch {
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
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
            ],
        };
        let filtered = filter_internal_vertex_contacts(
            vec![make_contact(ContactFeature::Vertex(1))],
            &patch,
            0.99,
        );
        assert_eq!(filtered.len(), 1);
    }

    #[test]
    fn keeps_crease_edge_contact() {
        let patch = MeshPatch {
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
        };
        let filtered = filter_internal_edge_contacts(
            vec![make_contact(ContactFeature::Edge(0))],
            &patch,
            0.99,
        );
        assert_eq!(filtered.len(), 1);
    }
}
