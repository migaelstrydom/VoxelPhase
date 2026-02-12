use crate::collision::MeshPatch;

use super::contact_source::{ContactFeature, SourcedContact};

/// Drop contacts created from interior mesh edges on near-coplanar neighbors.
pub fn filter_internal_edge_contacts(
    contacts: Vec<SourcedContact>,
    patch: &MeshPatch,
    coplanar_dot_threshold: f32,
) -> Vec<SourcedContact> {
    let mut filtered = Vec::with_capacity(contacts.len());
    for contact in contacts {
        if should_keep_contact(&contact, patch, coplanar_dot_threshold) {
            filtered.push(contact);
        }
    }
    filtered
}

fn should_keep_contact(
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
        let filtered = filter_internal_edge_contacts(vec![make_contact(ContactFeature::Edge(0))], &patch, 0.99);
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
        let filtered =
            filter_internal_edge_contacts(vec![make_contact(ContactFeature::Edge(0))], &patch, 0.99);
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
        let filtered =
            filter_internal_edge_contacts(vec![make_contact(ContactFeature::Edge(0))], &patch, 0.99);
        assert_eq!(filtered.len(), 1);
    }
}
