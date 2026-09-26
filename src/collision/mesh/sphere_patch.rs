//! Sphere vs FilteredPatch manifold generation.
//!
//! Tests the sphere against every face and boundary edge in the filtered
//! patch, collecting all overlapping contacts into a multi-point manifold.
//! For face contacts, depth is computed geometrically as
//! `radius - dot(center - plane_point, normal)` for stability, rather than
//! from per-triangle closest-point distances which fluctuate as edge
//! contacts enter and leave the query region.
//!
//! Multi-contact output is essential for concave terrain features (bowls,
//! valleys, crease edges) where the sphere needs simultaneous support from
//! multiple non-coplanar faces. On flat/convex terrain the seam filter
//! suppresses internal edges, and coplanar face contacts are harmless
//! (redundant same-normal support). FeatureId-based manifold caching
//! ensures smooth transitions when contacts appear or disappear.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::mesh::seam_filter::{ContactEdge, FilteredPatch};
use crate::collision::SurfaceId;

/// Maximum contacts emitted before reduction.
const MAX_SPHERE_PATCH_CONTACTS: usize = 4;

/// Generate a contact manifold for a sphere against a filtered mesh patch.
///
/// Collects face-interior contacts from every face whose plane the sphere
/// overlaps AND whose polygon contains the sphere's plane projection.
/// This produces multiple contacts for concave terrain (bowls, valleys)
/// while avoiding spurious edge-normal contacts on flat/convex terrain.
///
/// Face-interior contacts use the face normal (stable, geometric) rather
/// than edge-to-center normals (noisy, frame-dependent). If no
/// face-interior contact exists (sphere is near a patch boundary or
/// crease edge but doesn't project inside any face), falls back to the
/// single closest boundary-point or edge contact for robustness.
///
/// # Arguments
/// * `center` — world-space sphere center
/// * `radius` — sphere radius (without margin)
/// * `patch` — seam-filtered mesh patch
/// * `contact_margin` — inflation distance for speculative contacts
pub fn sphere_patch_manifold(
    center: Point3<f32>,
    radius: f32,
    patch: &FilteredPatch,
    contact_margin: f32,
) -> ContactManifold {
    let expanded_radius = radius + contact_margin;

    let mut face_hits: SmallVec<[SphereContact; 4]> = SmallVec::new();
    let mut best_boundary: Option<SphereContact> = None;

    for face in &patch.faces {
        if face.vertices.len() < 3 {
            continue;
        }
        if let Some(contact) = sphere_vs_face(center, expanded_radius, face) {
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

    // Also check boundary edges for the fallback.
    for edge in &patch.boundary_edges {
        if let Some(contact) = sphere_vs_edge(center, expanded_radius, edge) {
            if best_boundary
                .as_ref()
                .map_or(true, |b| contact.dist_sq < b.dist_sq)
            {
                best_boundary = Some(contact);
            }
        }
    }

    // Prefer face-interior contacts. Fall back to the single closest
    // boundary contact only when no face-interior contact was found.
    if face_hits.is_empty() {
        return match best_boundary {
            Some(hit) => {
                let raw_depth = hit.raw_depth(radius);
                ContactManifold::single(
                    ContactPoint::new(hit.point, hit.normal, raw_depth, hit.feature_id)
                        .on(hit.surface),
                )
            }
            None => ContactManifold::empty(),
        };
    }

    let points: SmallVec<[ContactPoint; 4]> = face_hits
        .iter()
        .map(|hit| {
            let raw_depth = hit.raw_depth(radius);
            ContactPoint::new(hit.point, hit.normal, raw_depth, hit.feature_id).on(hit.surface)
        })
        .collect();

    if points.len() > MAX_SPHERE_PATCH_CONTACTS {
        let reducer = ContactReducer::new(MAX_SPHERE_PATCH_CONTACTS);
        let reduced = reducer.reduce(&points);
        ContactManifold::from_vec(reduced.into_iter().collect())
    } else {
        ContactManifold::from_vec(points)
    }
}

/// Intermediate result from a sphere-vs-feature test.
struct SphereContact {
    /// Contact point on the surface.
    point: Point3<f32>,
    /// Contact normal (surface toward sphere).
    normal: Vector3<f32>,
    /// Squared distance from sphere center to closest surface point.
    dist_sq: f32,
    /// Whether this is a face contact (uses geometric depth) or edge contact.
    is_face: bool,
    /// Geometric face distance (only meaningful for face contacts).
    face_signed_dist: f32,
    /// Feature ID.
    feature_id: FeatureId,
    /// Surface of the face or edge the contact was found on.
    surface: SurfaceId,
}

impl SphereContact {
    fn raw_depth(&self, radius: f32) -> f32 {
        if self.is_face {
            // Geometric depth from plane distance, stable across frame-to-frame
            // fluctuations in which triangles are in the query region.
            radius - self.face_signed_dist
        } else {
            // Distance-based depth for edge/vertex contacts.
            radius - self.dist_sq.sqrt()
        }
    }
}

/// Test sphere against a merged contact face.
fn sphere_vs_face(
    center: Point3<f32>,
    expanded_radius: f32,
    face: &crate::collision::mesh::seam_filter::ContactFace,
) -> Option<SphereContact> {
    let verts = &face.vertices;

    let normal = face.normal;
    let signed_dist = (center - verts[0]).dot(&normal);

    // Sphere must overlap the face plane (within expanded radius of either side).
    // Allowing negative signed_dist (center slightly behind the plane) is
    // critical: the solver permits small penetrations, and rejecting at
    // signed_dist < 0 causes contacts to vanish when the sphere dips below
    // the surface, leading to fall-through.
    if signed_dist > expanded_radius || signed_dist < -expanded_radius {
        return None;
    }

    // Project center onto face plane.
    let projected = center - normal * signed_dist;

    // Check if projection is inside the convex polygon.
    if point_in_convex_polygon(&projected, verts, &normal) {
        return Some(SphereContact {
            point: projected,
            normal,
            dist_sq: signed_dist * signed_dist,
            is_face: true,
            face_signed_dist: signed_dist,
            feature_id: face.feature_id,
            surface: face.surface,
        });
    }

    // Projection is outside — find closest point on face boundary.
    let mut best_dist_sq = f32::MAX;
    let mut best_point = projected;

    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        let cp = closest_point_on_segment(center, a, b);
        let d_sq = (center - cp).magnitude_squared();
        if d_sq < best_dist_sq {
            best_dist_sq = d_sq;
            best_point = cp;
        }
    }

    if best_dist_sq > expanded_radius * expanded_radius {
        return None;
    }

    let to_center = center - best_point;
    let dist = best_dist_sq.sqrt();
    let contact_normal = if dist > 1e-6 {
        to_center / dist
    } else {
        normal
    };

    Some(SphereContact {
        point: best_point,
        normal: contact_normal,
        dist_sq: best_dist_sq,
        is_face: false,
        face_signed_dist: signed_dist,
        feature_id: face.feature_id,
        surface: face.surface,
    })
}

/// Test sphere against a boundary/crease edge.
fn sphere_vs_edge(
    center: Point3<f32>,
    expanded_radius: f32,
    edge: &ContactEdge,
) -> Option<SphereContact> {
    let cp = closest_point_on_segment(center, edge.a, edge.b);
    let to_center = center - cp;
    let dist_sq = to_center.magnitude_squared();

    if dist_sq > expanded_radius * expanded_radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-6 {
        to_center / dist
    } else {
        Vector3::y()
    };

    Some(SphereContact {
        point: cp,
        normal,
        dist_sq,
        is_face: false,
        face_signed_dist: 0.0,
        feature_id: edge.feature_id,
        surface: edge.surface,
    })
}

/// Test if a point lies inside a convex polygon (winding-agnostic).
///
/// Works for both CW and CCW winding: a point is inside iff it's on the
/// same side of all edges (all cross products have the same sign).
pub(crate) fn point_in_convex_polygon(
    point: &Point3<f32>,
    verts: &[Point3<f32>],
    normal: &Vector3<f32>,
) -> bool {
    let n = verts.len();
    let mut positive = 0u32;
    let mut negative = 0u32;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        let edge = b - a;
        let to_point = *point - a;
        let cross_dot = edge.cross(&to_point).dot(normal);
        if cross_dot > 1e-6 {
            positive += 1;
        } else if cross_dot < -1e-6 {
            negative += 1;
        }
    }
    positive == 0 || negative == 0
}

/// Closest point on a line segment to a query point.
pub(crate) fn closest_point_on_segment(
    p: Point3<f32>,
    a: Point3<f32>,
    b: Point3<f32>,
) -> Point3<f32> {
    let ab = b - a;
    let len_sq = ab.magnitude_squared();
    if len_sq < 1e-12 {
        return a;
    }
    let t = ((p - a).dot(&ab) / len_sq).clamp(0.0, 1.0);
    a + ab * t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::mesh::seam_filter::{ContactFace, FilteredPatch};
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
                    surface: SurfaceId::UNSPECIFIED,
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        }
    }

    #[test]
    fn sphere_resting_on_flat_face() {
        let patch = flat_face(0.0);
        let m = sphere_patch_manifold(Point3::new(0.0, 0.5, 0.0), 0.5, &patch, 0.0);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(
            c.normal.y > 0.99,
            "Normal should point up, got {:?}",
            c.normal
        );
        assert!(
            c.raw_depth.abs() < 1e-4,
            "Should be touching, got {}",
            c.raw_depth
        );
        assert!(c.depth.abs() < 1e-4);
    }

    #[test]
    fn sphere_penetrating_flat_face() {
        let patch = flat_face(0.0);
        let m = sphere_patch_manifold(Point3::new(0.0, 0.3, 0.0), 0.5, &patch, 0.0);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(
            (c.raw_depth - 0.2).abs() < 1e-4,
            "Expected ~0.2 depth, got {}",
            c.raw_depth
        );
    }

    #[test]
    fn sphere_above_face_no_contact() {
        let patch = flat_face(0.0);
        let m = sphere_patch_manifold(Point3::new(0.0, 2.0, 0.0), 0.5, &patch, 0.0);
        assert!(m.is_empty());
    }

    #[test]
    fn sphere_margin_only_contact() {
        let patch = flat_face(0.0);
        // Sphere at y=0.55, radius=0.5, margin=0.1 → expanded 0.6 catches it (dist=0.55)
        let m = sphere_patch_manifold(Point3::new(0.0, 0.55, 0.0), 0.5, &patch, 0.1);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(
            c.raw_depth < 0.0,
            "Should be margin-only, got {}",
            c.raw_depth
        );
        assert_eq!(c.depth, 0.0);
    }

    #[test]
    fn sphere_near_face_edge() {
        // Sphere positioned past the edge of the face polygon.
        let patch = FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 0.0),
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(0.0, 0.0, 1.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                    surface: SurfaceId::UNSPECIFIED,
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        };
        // Sphere center at (1.2, 0.1, 0.5) — just past the +X edge.
        // Distance to edge at (1.0, 0.0, 0.5) = sqrt(0.04 + 0.01) ≈ 0.224
        let m = sphere_patch_manifold(Point3::new(1.2, 0.1, 0.5), 0.5, &patch, 0.0);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        // Closest point should be on the edge at (1.0, 0.0, 0.5).
        assert!((c.point.x - 1.0).abs() < 0.1);
        assert!((c.point.z - 0.5).abs() < 0.1);
    }

    #[test]
    fn sphere_contacts_boundary_edge() {
        let patch = FilteredPatch {
            faces: SmallVec::new(), // No faces, just edges.
            boundary_edges: SmallVec::from_elem(
                ContactEdge {
                    a: Point3::new(0.0, 0.0, 0.0),
                    b: Point3::new(1.0, 0.0, 0.0),
                    feature_id: FeatureId::from_edge_pair(0, 0),
                    surface: SurfaceId::UNSPECIFIED,
                    normal_a: Vector3::y(),
                    normal_b: None,
                },
                1,
            ),
        };
        let m = sphere_patch_manifold(Point3::new(0.5, 0.3, 0.0), 0.5, &patch, 0.0);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth > 0.0, "Should be penetrating");
    }

    #[test]
    fn geometric_depth_stability() {
        // Verify that a sphere resting on a flat face produces consistent
        // depth regardless of which exact point is closest (face contact
        // uses plane-distance formula, not closest-point distance).
        let patch = flat_face(0.0);
        let m1 = sphere_patch_manifold(Point3::new(0.0, 0.5, 0.0), 0.5, &patch, 0.0);
        let m2 = sphere_patch_manifold(Point3::new(0.1, 0.5, 0.1), 0.5, &patch, 0.0);

        let d1 = m1.points[0].raw_depth;
        let d2 = m2.points[0].raw_depth;
        assert!(
            (d1 - d2).abs() < 1e-5,
            "Geometric depth should be stable across positions: {} vs {}",
            d1,
            d2,
        );
    }
}
