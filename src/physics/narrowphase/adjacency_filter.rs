use crate::collision::MeshPatch;

use super::contact_source::{ContactFeature, SourcedContact};

/// For contacts on interior mesh edges of near-coplanar neighbors, replace the
/// contact normal with the triangle face normal. This prevents lateral impulses
/// from edge normals while keeping the contact alive (dropping would create gaps
/// when the sphere straddles a shared edge and both triangles classify it as an
/// edge contact).
pub fn fix_internal_edge_normals(
    mut contacts: Vec<SourcedContact>,
    patch: &MeshPatch,
    coplanar_dot_threshold: f32,
) -> Vec<SourcedContact> {
    for contact in &mut contacts {
        maybe_fix_edge_normal(contact, patch, coplanar_dot_threshold);
    }
    contacts
}

/// For contacts on interior mesh vertices surrounded by coplanar triangles,
/// replace the contact normal with the triangle face normal.
pub fn fix_internal_vertex_normals(
    mut contacts: Vec<SourcedContact>,
    patch: &MeshPatch,
    coplanar_dot_threshold: f32,
) -> Vec<SourcedContact> {
    for contact in &mut contacts {
        maybe_fix_vertex_normal(contact, patch, coplanar_dot_threshold);
    }
    contacts
}

fn maybe_fix_edge_normal(
    contact: &mut SourcedContact,
    patch: &MeshPatch,
    coplanar_dot_threshold: f32,
) {
    let tri_idx = contact.source.triangle_idx as usize;
    let Some(patch_tri) = patch.triangles.get(tri_idx) else {
        return;
    };

    let ContactFeature::Edge(edge_idx) = contact.source.feature else {
        return;
    };
    let edge_idx = edge_idx as usize;
    if edge_idx >= 3 {
        return;
    }

    let Some(neighbor_idx) = patch_tri.neighbors[edge_idx] else {
        return;
    };
    let Some(neighbor_tri) = patch.triangles.get(neighbor_idx as usize) else {
        return;
    };

    let tri_n = patch_tri.triangle.normal();
    let nbr_n = neighbor_tri.triangle.normal();
    if tri_n.dot(&nbr_n) >= coplanar_dot_threshold {
        contact.constraint.normal = tri_n;
        contact.constraint.raw_normal = tri_n;
    }
}

/// Returns the two edge indices (into `PatchTriangle::neighbors`) that meet
/// at the given vertex index.
///
/// Edge 0 = v0->v1, Edge 1 = v1->v2, Edge 2 = v2->v0, so:
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

fn maybe_fix_vertex_normal(
    contact: &mut SourcedContact,
    patch: &MeshPatch,
    coplanar_dot_threshold: f32,
) {
    let tri_idx = contact.source.triangle_idx as usize;
    let Some(patch_tri) = patch.triangles.get(tri_idx) else {
        return;
    };

    let ContactFeature::Vertex(vert_idx) = contact.source.feature else {
        return;
    };
    let vert_idx = vert_idx as usize;
    if vert_idx >= 3 {
        return;
    }

    let tri_n = patch_tri.triangle.normal();
    let incident = edges_incident_to_vertex(vert_idx);

    for &edge_idx in &incident {
        let Some(neighbor_idx) = patch_tri.neighbors[edge_idx] else {
            return;
        };
        let Some(neighbor_tri) = patch.triangles.get(neighbor_idx as usize) else {
            return;
        };
        let nbr_n = neighbor_tri.triangle.normal();
        if tri_n.dot(&nbr_n) < coplanar_dot_threshold {
            return;
        }
    }

    // All incident neighbors are coplanar — replace normal with face normal.
    contact.constraint.normal = tri_n;
    contact.constraint.raw_normal = tri_n;
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
                normal: Vector3::new(0.1, 0.995, 0.0),
                raw_normal: Vector3::new(0.1, 0.995, 0.0),
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

    fn coplanar_two_tri_patch() -> MeshPatch {
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
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ),
                    neighbors: [None, None, Some(0)],
                },
            ],
        }
    }

    #[test]
    fn fixes_normal_of_coplanar_internal_edge_contact() {
        let patch = coplanar_two_tri_patch();
        // Edge 0 of tri 0 is the shared edge (neighbor = Some(1)).
        let result = fix_internal_edge_normals(
            vec![make_contact(ContactFeature::Edge(0))],
            &patch,
            0.99,
        );
        assert_eq!(result.len(), 1);
        let n = result[0].constraint.normal;
        let face_n = patch.triangles[0].triangle.normal();
        assert!(
            (n - face_n).magnitude() < 1e-5,
            "normal should be replaced with face normal {face_n:?}, got {n:?}"
        );
    }

    #[test]
    fn keeps_boundary_edge_contact_unchanged() {
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
        let result = fix_internal_edge_normals(
            vec![make_contact(ContactFeature::Edge(0))],
            &patch,
            0.99,
        );
        assert_eq!(result.len(), 1);
        let n = result[0].constraint.normal;
        assert!((n.x - 0.1).abs() < 1e-5, "boundary edge normal should be unchanged");
    }

    #[test]
    fn fixes_normal_of_coplanar_internal_vertex() {
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
        let result = fix_internal_vertex_normals(
            vec![make_contact(ContactFeature::Vertex(1))],
            &patch,
            0.99,
        );
        assert_eq!(result.len(), 1);
        let n = result[0].constraint.normal;
        let face_n = patch.triangles[0].triangle.normal();
        assert!(
            (n - face_n).magnitude() < 1e-5,
            "normal should be replaced with face normal {face_n:?}, got {n:?}"
        );
    }

    #[test]
    fn keeps_boundary_vertex_contact_unchanged() {
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
        let result = fix_internal_vertex_normals(
            vec![make_contact(ContactFeature::Vertex(0))],
            &patch,
            0.99,
        );
        assert_eq!(result.len(), 1);
        let n = result[0].constraint.normal;
        assert!((n.x - 0.1).abs() < 1e-5, "boundary vertex normal should be unchanged");
    }

    #[test]
    fn keeps_crease_vertex_contact_unchanged() {
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
        let result = fix_internal_vertex_normals(
            vec![make_contact(ContactFeature::Vertex(1))],
            &patch,
            0.99,
        );
        assert_eq!(result.len(), 1);
        let n = result[0].constraint.normal;
        assert!((n.x - 0.1).abs() < 1e-5, "crease vertex normal should be unchanged");
    }

    #[test]
    fn keeps_crease_edge_contact_unchanged() {
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
        let result = fix_internal_edge_normals(
            vec![make_contact(ContactFeature::Edge(0))],
            &patch,
            0.99,
        );
        assert_eq!(result.len(), 1);
        let n = result[0].constraint.normal;
        assert!((n.x - 0.1).abs() < 1e-5, "crease edge normal should be unchanged");
    }

    #[test]
    fn keeps_vertex_with_one_missing_neighbor_unchanged() {
        let patch = coplanar_two_tri_patch();
        let result = fix_internal_vertex_normals(
            vec![make_contact(ContactFeature::Vertex(1))],
            &patch,
            0.99,
        );
        assert_eq!(result.len(), 1);
        let n = result[0].constraint.normal;
        assert!((n.x - 0.1).abs() < 1e-5, "partial-boundary vertex normal should be unchanged");
    }
}
