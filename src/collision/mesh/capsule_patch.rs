//! Capsule vs FilteredPatch manifold generation.
//!
//! For each face: find closest point on capsule segment to the face plane,
//! then do a sphere-vs-face test from that point. Multi-contact output
//! for concave terrain.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::mesh::seam_filter::{ContactEdge, ContactFace, FilteredPatch};
use crate::collision::mesh::sphere_patch::{closest_point_on_segment, point_in_convex_polygon};

/// Maximum contacts emitted before reduction.
const MAX_CAPSULE_PATCH_CONTACTS: usize = 4;

/// Generate a contact manifold for a capsule against a filtered mesh patch.
///
/// # Arguments
/// * `seg_a`, `seg_b` — world-space capsule segment endpoints
/// * `radius` — capsule radius (without margin)
/// * `patch` — seam-filtered mesh patch
/// * `contact_margin` — inflation distance for speculative contacts
pub fn capsule_patch_manifold(
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    radius: f32,
    patch: &FilteredPatch,
    contact_margin: f32,
) -> ContactManifold {
    let expanded_radius = radius + contact_margin;

    let mut face_hits: SmallVec<[CapsuleContact; 4]> = SmallVec::new();
    let mut best_boundary: Option<CapsuleContact> = None;

    for face in &patch.faces {
        if face.vertices.len() < 3 {
            continue;
        }
        if let Some(contact) = capsule_vs_face(seg_a, seg_b, expanded_radius, face) {
            if contact.is_face {
                face_hits.push(contact);
            } else if best_boundary
                .as_ref()
                .map_or(true, |b| contact.dist_sq < b.dist_sq)
            {
                best_boundary = Some(contact);
            }
        }
    }

    for edge in &patch.boundary_edges {
        if let Some(contact) = capsule_vs_edge(seg_a, seg_b, expanded_radius, edge) {
            if best_boundary
                .as_ref()
                .map_or(true, |b| contact.dist_sq < b.dist_sq)
            {
                best_boundary = Some(contact);
            }
        }
    }

    if face_hits.is_empty() {
        return match best_boundary {
            Some(hit) => {
                let raw_depth = hit.raw_depth(radius);
                ContactManifold::single(ContactPoint::new(
                    hit.point,
                    hit.normal,
                    raw_depth,
                    hit.feature_id,
                ))
            }
            None => ContactManifold::empty(),
        };
    }

    let points: SmallVec<[ContactPoint; 4]> = face_hits
        .iter()
        .map(|hit| {
            let raw_depth = hit.raw_depth(radius);
            ContactPoint::new(hit.point, hit.normal, raw_depth, hit.feature_id)
        })
        .collect();

    if points.len() > MAX_CAPSULE_PATCH_CONTACTS {
        let reducer = ContactReducer::new(MAX_CAPSULE_PATCH_CONTACTS);
        let reduced = reducer.reduce(&points);
        ContactManifold::from_vec(reduced.into_iter().collect())
    } else {
        ContactManifold::from_vec(points)
    }
}

/// Intermediate result from a capsule-vs-feature test.
struct CapsuleContact {
    point: Point3<f32>,
    normal: Vector3<f32>,
    dist_sq: f32,
    is_face: bool,
    face_signed_dist: f32,
    feature_id: FeatureId,
}

impl CapsuleContact {
    fn raw_depth(&self, radius: f32) -> f32 {
        if self.is_face {
            radius - self.face_signed_dist
        } else {
            radius - self.dist_sq.sqrt()
        }
    }
}

/// Find the closest point on the capsule segment to a face plane,
/// then test sphere-vs-face from that point.
fn capsule_vs_face(
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    expanded_radius: f32,
    face: &ContactFace,
) -> Option<CapsuleContact> {
    let verts = &face.vertices;
    let normal = face.normal;

    // Find the segment endpoint closest to the face plane (smallest signed distance).
    let signed_a = (seg_a - verts[0]).dot(&normal);
    let signed_b = (seg_b - verts[0]).dot(&normal);

    // Pick the point on the segment that minimizes signed distance to the plane.
    let (sphere_center, signed_dist) = if signed_a.abs() <= signed_b.abs() {
        // But we should actually find the true closest point on the segment.
        // The closest point is where signed_dist is minimized in abs value.
        let seg_dir = seg_b - seg_a;
        let denom = seg_dir.dot(&normal);
        if denom.abs() > 1e-6 {
            // t where signed_dist = 0
            let t = -signed_a / denom;
            let t = t.clamp(0.0, 1.0);
            let p = seg_a + seg_dir * t;
            let sd = (p - verts[0]).dot(&normal);
            (p, sd)
        } else {
            // Segment parallel to plane — both signed distances ~equal, pick smaller.
            if signed_a.abs() <= signed_b.abs() {
                (seg_a, signed_a)
            } else {
                (seg_b, signed_b)
            }
        }
    } else {
        let seg_dir = seg_b - seg_a;
        let denom = seg_dir.dot(&normal);
        if denom.abs() > 1e-6 {
            let t = (-signed_a / denom).clamp(0.0, 1.0);
            let p = seg_a + seg_dir * t;
            let sd = (p - verts[0]).dot(&normal);
            (p, sd)
        } else {
            (seg_b, signed_b)
        }
    };

    if signed_dist > expanded_radius || signed_dist < -expanded_radius {
        return None;
    }

    let projected = sphere_center - normal * signed_dist;

    if point_in_convex_polygon(&projected, verts, &normal) {
        return Some(CapsuleContact {
            point: projected,
            normal,
            dist_sq: signed_dist * signed_dist,
            is_face: true,
            face_signed_dist: signed_dist,
            feature_id: face.feature_id,
        });
    }

    // Projection outside — find closest point on face boundary.
    let mut best_dist_sq = f32::MAX;
    let mut best_point = projected;

    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        let cp = closest_point_on_segment(sphere_center, a, b);
        let d_sq = (sphere_center - cp).magnitude_squared();
        if d_sq < best_dist_sq {
            best_dist_sq = d_sq;
            best_point = cp;
        }
    }

    if best_dist_sq > expanded_radius * expanded_radius {
        return None;
    }

    let to_center = sphere_center - best_point;
    let dist = best_dist_sq.sqrt();
    let contact_normal = if dist > 1e-6 {
        to_center / dist
    } else {
        normal
    };

    Some(CapsuleContact {
        point: best_point,
        normal: contact_normal,
        dist_sq: best_dist_sq,
        is_face: false,
        face_signed_dist: signed_dist,
        feature_id: face.feature_id,
    })
}

/// Test capsule against a boundary/crease edge.
fn capsule_vs_edge(
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    expanded_radius: f32,
    edge: &ContactEdge,
) -> Option<CapsuleContact> {
    // Closest points between capsule segment and edge segment.
    let (closest_on_capsule, closest_on_edge) =
        crate::collision::segment::segment_segment_closest_points(seg_a, seg_b, edge.a, edge.b);

    let to_capsule = closest_on_capsule - closest_on_edge;
    let dist_sq = to_capsule.magnitude_squared();

    if dist_sq > expanded_radius * expanded_radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-6 {
        to_capsule / dist
    } else {
        Vector3::y()
    };

    Some(CapsuleContact {
        point: closest_on_edge,
        normal,
        dist_sq,
        is_face: false,
        face_signed_dist: 0.0,
        feature_id: edge.feature_id,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::mesh::seam_filter::FilteredPatch;
    use smallvec::SmallVec;

    fn flat_face(y: f32) -> FilteredPatch {
        FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-5.0, y, -5.0),
                        Point3::new(5.0, y, -5.0),
                        Point3::new(5.0, y, 5.0),
                        Point3::new(-5.0, y, 5.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        }
    }

    #[test]
    fn capsule_resting_on_flat_face() {
        // Upright capsule: half_height=1.0, radius=0.5, segment half=0.5
        // Bottom of capsule at y = center_y - half_height = 1.0 - 1.0 = 0.0
        let seg_a = Point3::new(0.0, 0.5, 0.0); // bottom of segment
        let seg_b = Point3::new(0.0, 1.5, 0.0); // top of segment
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.normal.y > 0.99);
        assert!(c.raw_depth.abs() < 1e-4);
    }

    #[test]
    fn capsule_penetrating_flat_face() {
        let seg_a = Point3::new(0.0, 0.3, 0.0);
        let seg_b = Point3::new(0.0, 1.3, 0.0);
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(m.len(), 1);
        assert!((m.points[0].raw_depth - 0.2).abs() < 1e-4);
    }

    #[test]
    fn capsule_above_no_contact() {
        let seg_a = Point3::new(0.0, 3.0, 0.0);
        let seg_b = Point3::new(0.0, 5.0, 0.0);
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);
        assert!(m.is_empty());
    }

    #[test]
    fn capsule_margin_only() {
        let seg_a = Point3::new(0.0, 0.55, 0.0);
        let seg_b = Point3::new(0.0, 1.55, 0.0);
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.1);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth < 0.0);
        assert_eq!(c.depth, 0.0);
    }

    #[test]
    fn horizontal_capsule_on_flat_face() {
        // Capsule lying on its side: segment along X axis
        let seg_a = Point3::new(-0.5, 0.5, 0.0);
        let seg_b = Point3::new(0.5, 0.5, 0.0);
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.normal.y > 0.99);
        assert!(c.raw_depth.abs() < 1e-4);
    }
}
