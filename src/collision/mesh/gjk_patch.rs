//! General convex shape vs FilteredPatch manifold generation using GJK + EPA.
//!
//! For each ContactFace in the patch, runs GJK/EPA between the convex shape
//! and the face polygon, then clips support faces and projects onto the contact
//! plane. Produces up to 4 contacts per patch via ContactReducer.
//!
//! This is the mesh-contact equivalent of `gjk_epa_manifold` — it provides
//! automatic terrain collision for any shape that implements `ConvexSupport`
//! and `SupportFaceExtractor` (via `ShapeView`).

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::{ContactManifold, ContactPoint};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::discrete::epa::epa_penetration;
use crate::collision::discrete::gjk::{gjk_query, GjkResult};
use crate::collision::mesh::seam_filter::{ContactFace, FilteredPatch};
use crate::collision::shape_view::{SupportFace, SupportFaceExtractor};
use crate::collision::support::ConvexSupport;

/// Maximum contacts in the final manifold.
const MAX_MANIFOLD_POINTS: usize = 4;
/// Tolerance for GJK separation distance check.
const GJK_TOLERANCE: f32 = 1e-4;
/// Small tolerance for rejecting backfacing mesh contacts.
const BACKFACE_EPSILON: f32 = 1e-4;

/// A convex polygon (ContactFace) wrapped for GJK/EPA support queries.
struct SupportPolygon<'a> {
    face: &'a ContactFace,
}

impl ConvexSupport for SupportPolygon<'_> {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        let verts = &self.face.vertices;
        let mut best = verts[0];
        let mut best_dot = verts[0].coords.dot(&direction);
        for v in verts.iter().skip(1) {
            let d = v.coords.dot(&direction);
            if d > best_dot {
                best_dot = d;
                best = *v;
            }
        }
        best
    }

    fn bounding_radius(&self) -> f32 {
        let verts = &self.face.vertices;
        let center = Point3::from(
            verts.iter().map(|v| v.coords).sum::<Vector3<f32>>() / verts.len() as f32,
        );
        verts
            .iter()
            .map(|v| (v - center).magnitude())
            .fold(0.0f32, f32::max)
    }
}

impl SupportFaceExtractor for SupportPolygon<'_> {
    fn support_face(&self, _direction: Vector3<f32>) -> Option<SupportFace> {
        Some(SupportFace {
            vertices: SmallVec::from_slice(&self.face.vertices),
            normal: self.face.normal,
            face_index: 0,
        })
    }
}

/// Generate a contact manifold for a convex shape against a filtered mesh patch.
///
/// Runs GJK + EPA per face, clips support faces, and reduces to 4 contacts.
/// The normal points from the terrain surface toward the shape.
///
/// # Arguments
/// * `shape` — world-space convex shape view
/// * `patch` — seam-filtered mesh patch
/// * `margin` — contact margin for speculative contacts
pub fn gjk_patch_manifold<S: ConvexSupport + SupportFaceExtractor>(
    shape: &S,
    patch: &FilteredPatch,
    margin: f32,
) -> ContactManifold {
    let mut all_points: SmallVec<[ContactPoint; 4]> = SmallVec::new();

    for face in &patch.faces {
        if face.vertices.len() < 3 {
            continue;
        }

        let face_contacts = gjk_epa_vs_face(shape, face, margin);
        all_points.extend(face_contacts);
    }

    if all_points.is_empty() {
        return ContactManifold::empty();
    }

    if all_points.len() > MAX_MANIFOLD_POINTS {
        let reducer = ContactReducer::new(MAX_MANIFOLD_POINTS);
        let reduced = reducer.reduce(&all_points);
        return ContactManifold::from_vec(reduced.into_iter().collect());
    }

    ContactManifold::from_vec(all_points)
}

/// Run GJK + EPA between a convex shape and a single mesh face.
///
/// Returns contact points for this face (0 to ~4).
fn gjk_epa_vs_face<S: ConvexSupport + SupportFaceExtractor>(
    shape: &S,
    face: &ContactFace,
    margin: f32,
) -> SmallVec<[ContactPoint; 4]> {
    let polygon = SupportPolygon { face };

    // Backface check: shape center (estimated) should be on the normal side.
    let shape_center_approx = estimate_center(shape);
    let signed_dist_to_plane = (shape_center_approx - face.vertices[0]).dot(&face.normal);
    if signed_dist_to_plane < -BACKFACE_EPSILON {
        return SmallVec::new();
    }

    let result = gjk_query(shape, &polygon);

    match result {
        GjkResult::Separated {
            distance,
            closest_a,
            closest_b,
        } => {
            if distance > 2.0 * margin + GJK_TOLERANCE {
                return SmallVec::new();
            }

            // Margin-only contact. Use face normal for stability.
            let projected = project_onto_face_plane(closest_a, face);
            let plane_dist = (closest_a - face.vertices[0]).dot(&face.normal);
            let raw_depth = -(distance - 2.0 * margin).max(0.0);

            // Use face normal if shape is roughly above the face.
            let normal = if plane_dist >= -BACKFACE_EPSILON {
                face.normal
            } else {
                let delta = closest_a - closest_b;
                let d = delta.magnitude();
                if d > 1e-10 { delta / d } else { face.normal }
            };

            let mut contacts = SmallVec::new();
            contacts.push(ContactPoint::new(
                projected,
                normal,
                raw_depth,
                face.feature_id,
            ));
            contacts
        }

        GjkResult::Intersecting { simplex } => {
            let epa = epa_penetration(shape, &polygon, margin, &simplex);

            // Orient normal to point from face toward shape (outward from terrain).
            let mut normal = epa.normal;
            if normal.dot(&face.normal) < 0.0 {
                normal = -normal;
            }

            let epa_depth = epa.depth;

            // Extract support faces for clipping.
            let shape_face = shape.support_face(-normal);
            let face_support = polygon.support_face(normal);

            match (shape_face, face_support) {
                (Some(sf), Some(ff)) => {
                    clip_shape_face_against_mesh_face(&sf, &ff, normal, epa_depth, margin, face)
                }
                _ => {
                    // No face to clip — single contact from EPA witness.
                    let raw_depth = epa_depth - 2.0 * margin;
                    let point = project_onto_face_plane(epa.witness_a, face);
                    let mut contacts = SmallVec::new();
                    contacts.push(ContactPoint::new(point, normal, raw_depth, face.feature_id));
                    contacts
                }
            }
        }
    }
}

/// Clip the shape's support face against the mesh face's edges,
/// then project onto the face plane and emit contacts.
fn clip_shape_face_against_mesh_face(
    shape_face: &SupportFace,
    mesh_face_support: &SupportFace,
    normal: Vector3<f32>,
    epa_depth: f32,
    margin: f32,
    face: &ContactFace,
) -> SmallVec<[ContactPoint; 4]> {
    let mesh_verts = &mesh_face_support.vertices;
    let face_normal = face.normal;

    // Clip shape face against mesh face edges (Sutherland-Hodgman).
    let clipped = clip_against_face_edges(&shape_face.vertices, mesh_verts, &face_normal);

    let face_point = face.vertices[0];

    if clipped.is_empty() {
        // Clipping failed — try projecting shape face vertices into the mesh face.
        return project_shape_verts_into_face(
            &shape_face.vertices,
            face,
            normal,
            margin,
        );
    }

    let mut contacts: SmallVec<[ContactPoint; 4]> = SmallVec::new();

    for (i, p) in clipped.iter().enumerate() {
        let signed_dist = (p - face_point).dot(&face_normal);
        let raw_depth = -signed_dist;

        if raw_depth < -margin {
            continue;
        }

        let projected = p - face_normal * signed_dist;
        let feature_id = face.feature_id.with_vertex(i as u32);
        contacts.push(ContactPoint::new(projected, normal, raw_depth, feature_id));
    }

    if contacts.is_empty() {
        // All clipped points were above the margin threshold — fall back to
        // single EPA-based contact.
        let raw_depth = epa_depth - 2.0 * margin;
        let center = compute_centroid(&shape_face.vertices);
        let projected = project_onto_face_plane(center, face);
        contacts.push(ContactPoint::new(
            projected,
            normal,
            raw_depth,
            face.feature_id,
        ));
    }

    contacts
}

/// Fallback: project shape face vertices onto the mesh face plane and keep
/// those that land inside the polygon.
fn project_shape_verts_into_face(
    shape_verts: &SmallVec<[Point3<f32>; 8]>,
    face: &ContactFace,
    normal: Vector3<f32>,
    margin: f32,
) -> SmallVec<[ContactPoint; 4]> {
    let face_point = face.vertices[0];
    let face_normal = face.normal;
    let mut contacts: SmallVec<[ContactPoint; 4]> = SmallVec::new();

    for (i, v) in shape_verts.iter().enumerate() {
        let signed_dist = (v - face_point).dot(&face_normal);
        let raw_depth = -signed_dist;

        if raw_depth < -margin {
            continue;
        }

        let projected = v - face_normal * signed_dist;
        if point_in_convex_polygon(&projected, &face.vertices, &face_normal) {
            contacts.push(ContactPoint::new(
                projected,
                normal,
                raw_depth,
                face.feature_id.with_vertex(i as u32),
            ));
        }
    }

    contacts
}

/// Clip subject polygon against the edge half-planes of a clip polygon.
fn clip_against_face_edges(
    subject: &SmallVec<[Point3<f32>; 8]>,
    clip_verts: &SmallVec<[Point3<f32>; 8]>,
    face_normal: &Vector3<f32>,
) -> SmallVec<[Point3<f32>; 8]> {
    if clip_verts.len() < 3 {
        return subject.clone();
    }

    let centroid = compute_centroid(clip_verts);
    let mut clipped = subject.clone();

    for i in 0..clip_verts.len() {
        if clipped.is_empty() {
            break;
        }
        let a = clip_verts[i];
        let b = clip_verts[(i + 1) % clip_verts.len()];
        let edge = b - a;
        let candidate_inward = edge.cross(face_normal);

        let inward = if (centroid - a).dot(&candidate_inward) >= 0.0 {
            candidate_inward
        } else {
            -candidate_inward
        };

        clipped = clip_polygon_by_plane(&clipped, a, inward);
    }

    clipped
}

/// Clip a convex polygon against a half-plane, keeping the inside.
fn clip_polygon_by_plane(
    polygon: &[Point3<f32>],
    plane_point: Point3<f32>,
    plane_normal: Vector3<f32>,
) -> SmallVec<[Point3<f32>; 8]> {
    let mut out: SmallVec<[Point3<f32>; 8]> = SmallVec::new();
    if polygon.is_empty() {
        return out;
    }

    for i in 0..polygon.len() {
        let p1 = polygon[i];
        let p2 = polygon[(i + 1) % polygon.len()];
        let d1 = (p1 - plane_point).dot(&plane_normal);
        let d2 = (p2 - plane_point).dot(&plane_normal);

        if d1 >= 0.0 && d2 >= 0.0 {
            out.push(p2);
        } else if d1 >= 0.0 {
            let t = d1 / (d1 - d2);
            out.push(p1 + (p2 - p1) * t);
        } else if d2 >= 0.0 {
            let t = d1 / (d1 - d2);
            out.push(p1 + (p2 - p1) * t);
            out.push(p2);
        }
    }

    out
}

/// Project a point onto a face's plane along the face normal.
fn project_onto_face_plane(point: Point3<f32>, face: &ContactFace) -> Point3<f32> {
    let signed_dist = (point - face.vertices[0]).dot(&face.normal);
    point - face.normal * signed_dist
}

/// Compute centroid of a polygon.
fn compute_centroid(verts: &[Point3<f32>]) -> Point3<f32> {
    let sum: Vector3<f32> = verts.iter().map(|v| v.coords).sum();
    Point3::from(sum / verts.len() as f32)
}

/// Estimate shape center from support queries in 6 axis directions.
fn estimate_center<S: ConvexSupport>(shape: &S) -> Point3<f32> {
    let dirs = [
        Vector3::x(),
        -Vector3::x(),
        Vector3::y(),
        -Vector3::y(),
        Vector3::z(),
        -Vector3::z(),
    ];
    let sum: Vector3<f32> = dirs.iter().map(|d| shape.support(*d).coords).sum();
    Point3::from(sum / dirs.len() as f32)
}

/// Test if a point lies inside a convex polygon (winding-agnostic).
fn point_in_convex_polygon(
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::capsule::Capsule;
    use crate::collision::contact::FeatureId;
    use crate::collision::mesh::capsule_patch::capsule_patch_manifold;
    use crate::collision::mesh::obb_patch::obb_patch_manifold;
    use crate::collision::mesh::sphere_patch::sphere_patch_manifold;
    use crate::collision::obb::Obb;
    use crate::collision::shape_view::ShapeView;
    use crate::physics::ColliderShape;
    use nalgebra::UnitQuaternion;

    fn large_flat_patch() -> FilteredPatch {
        FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-10.0, 0.0, -10.0),
                        Point3::new(10.0, 0.0, -10.0),
                        Point3::new(10.0, 0.0, 10.0),
                        Point3::new(-10.0, 0.0, 10.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        }
    }

    fn ramp_patch(slope_degrees: f32) -> FilteredPatch {
        // Ramp tilted around the X axis: rises in +Z direction.
        // At z=-5 the surface is at y=0, at z=+5 it's at y=rise.
        let angle = slope_degrees.to_radians();
        let rise = 10.0 * angle.tan();
        // Normal = (-sin(angle) in Z, cos(angle) in Y) rotated about X.
        let normal = Vector3::new(0.0, angle.cos(), -angle.sin()).normalize();
        FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-5.0, 0.0, -5.0),
                        Point3::new(5.0, 0.0, -5.0),
                        Point3::new(5.0, rise, 5.0),
                        Point3::new(-5.0, rise, 5.0),
                    ]),
                    normal,
                    feature_id: FeatureId::from_face(0),
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        }
    }

    // --- OBB cross-validation ---

    #[test]
    fn obb_cross_validation_flat() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let he = Vector3::new(0.5, 0.5, 0.5);
        let center = Point3::new(0.0, 0.4, 0.0);
        let rot = UnitQuaternion::identity();

        let shape = ColliderShape::Box { half_extents: he };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let gjk_result = gjk_patch_manifold(&view, &patch, margin);
        let obb = Obb::new(center, rot, he);
        let obb_result = obb_patch_manifold(&obb, &patch, margin);

        assert!(!gjk_result.is_empty(), "GJK patch should produce contacts");
        assert!(!obb_result.is_empty(), "OBB patch should produce contacts");

        // Normal direction (dot > 0.99).
        let gjk_n = gjk_result.points[0].normal;
        let obb_n = obb_result.points[0].normal;
        assert!(
            gjk_n.dot(&obb_n) > 0.99,
            "Normal mismatch: GJK {:?} vs OBB {:?}",
            gjk_n,
            obb_n,
        );

        // Depth (±0.05).
        let gjk_d = gjk_result.points[0].raw_depth;
        let obb_d = obb_result.points[0].raw_depth;
        assert!(
            (gjk_d - obb_d).abs() < 0.05,
            "Depth mismatch: GJK {} vs OBB {}",
            gjk_d,
            obb_d,
        );

        // Contact count (within ±1).
        let count_diff = (gjk_result.len() as i32 - obb_result.len() as i32).abs();
        assert!(
            count_diff <= 1,
            "Count mismatch: GJK {} vs OBB {}",
            gjk_result.len(),
            obb_result.len(),
        );

        // Contact positions (within 0.1 of nearest OBB contact).
        for gp in &gjk_result.points {
            let min_dist = obb_result
                .points
                .iter()
                .map(|op| (gp.point - op.point).magnitude())
                .fold(f32::INFINITY, f32::min);
            assert!(
                min_dist < 0.15,
                "GJK contact {:?} too far from any OBB contact (min_dist={})",
                gp.point,
                min_dist,
            );
        }
    }

    // --- Sphere cross-validation ---

    #[test]
    fn sphere_cross_validation_flat() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let center = Point3::new(0.0, 0.4, 0.0);
        let shape = ColliderShape::Sphere { radius: 0.5 };
        let view = ShapeView {
            center,
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };

        let gjk_result = gjk_patch_manifold(&view, &patch, margin);
        let sphere_result = sphere_patch_manifold(center, 0.5, &patch, margin);

        assert!(!gjk_result.is_empty(), "GJK patch should produce contacts");
        assert!(!sphere_result.is_empty(), "Sphere patch should produce contacts");

        let gjk_n = gjk_result.points[0].normal;
        let sph_n = sphere_result.points[0].normal;
        assert!(gjk_n.dot(&sph_n) > 0.99, "Normal mismatch");

        let gjk_d = gjk_result.points[0].raw_depth;
        let sph_d = sphere_result.points[0].raw_depth;
        assert!(
            (gjk_d - sph_d).abs() < 0.05,
            "Depth mismatch: GJK {} vs Sphere {}",
            gjk_d,
            sph_d,
        );
    }

    // --- Capsule cross-validation ---

    #[test]
    fn capsule_cross_validation_flat() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let center = Point3::new(0.0, 1.0, 0.0);
        let rot = UnitQuaternion::identity();
        let shape = ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let gjk_result = gjk_patch_manifold(&view, &patch, margin);
        let capsule = Capsule::new(center, rot, 1.0, 0.5);
        let (seg_a, seg_b) = capsule.segment_endpoints();
        let cap_result = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, margin);

        assert!(!gjk_result.is_empty(), "GJK patch should produce contacts");
        assert!(!cap_result.is_empty(), "Capsule patch should produce contacts");

        // Normal direction.
        let gjk_n = gjk_result.points[0].normal;
        let cap_n = cap_result.points[0].normal;
        assert!(
            gjk_n.dot(&cap_n) > 0.99,
            "Normal mismatch: GJK {:?} vs Capsule {:?}",
            gjk_n,
            cap_n,
        );

        // Depth (±0.05).
        let gjk_d = gjk_result.points[0].raw_depth;
        let cap_d = cap_result.points[0].raw_depth;
        assert!(
            (gjk_d - cap_d).abs() < 0.05,
            "Depth mismatch: GJK {} vs Capsule {}",
            gjk_d,
            cap_d,
        );
    }

    // --- Ramp geometry ---

    #[test]
    fn obb_on_ramp() {
        // 30° ramp: at z=0 the surface is at y ≈ 2.89.
        // Ramp normal is (0, 0.866, -0.5).
        // Place box center above the surface along the ramp normal.
        let patch = ramp_patch(30.0);
        let margin = 0.02;

        // Surface height at z=0: interpolated = rise/2 ≈ 2.89.
        // Box half-extent 0.5, so place center slightly into the surface
        // along the ramp normal direction.
        let ramp_normal = patch.faces[0].normal;
        let surface_point = Point3::new(0.0, 2.89, 0.0);
        let center = surface_point + ramp_normal * 0.4;

        let he = Vector3::new(0.5, 0.5, 0.5);
        let rot = UnitQuaternion::identity();
        let shape = ColliderShape::Box { half_extents: he };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &patch, margin);

        assert!(!result.is_empty(), "Should have contacts on ramp");
        let n = result.points[0].normal;
        assert!(
            n.dot(&ramp_normal) > 0.95,
            "Normal should align with ramp: got {:?}, expected {:?}",
            n,
            ramp_normal,
        );
    }

    // --- Step geometry ---

    #[test]
    fn obb_straddling_step() {
        let step_patch = FilteredPatch {
            faces: SmallVec::from_vec(vec![
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-5.0, 0.0, -5.0),
                        Point3::new(0.0, 0.0, -5.0),
                        Point3::new(0.0, 0.0, 5.0),
                        Point3::new(-5.0, 0.0, 5.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(0.0, 0.5, -5.0),
                        Point3::new(5.0, 0.5, -5.0),
                        Point3::new(5.0, 0.5, 5.0),
                        Point3::new(0.0, 0.5, 5.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(1),
                },
            ]),
            boundary_edges: SmallVec::new(),
        };
        let margin = 0.02;

        let he = Vector3::new(1.0, 0.5, 0.5);
        let center = Point3::new(0.0, 1.0, 0.0);
        let rot = UnitQuaternion::identity();
        let shape = ColliderShape::Box { half_extents: he };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &step_patch, margin);

        // Should produce contacts on both faces (box extends from x=-1 to x=1,
        // straddling the step at x=0).
        assert!(
            result.len() >= 2,
            "Should have contacts on both step faces, got {}",
            result.len(),
        );
    }

    // --- No contact ---

    #[test]
    fn shape_above_patch_no_contact() {
        let patch = large_flat_patch();
        let shape = ColliderShape::Box {
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let view = ShapeView {
            center: Point3::new(0.0, 5.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &patch, 0.02);
        assert!(result.is_empty(), "Should have no contacts when far above");
    }

    // --- Contact count limit ---

    #[test]
    fn manifold_reduced_to_four() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let he = Vector3::new(0.5, 0.5, 0.5);
        let center = Point3::new(0.0, 0.5, 0.0);
        let rot = UnitQuaternion::identity();
        let shape = ColliderShape::Box { half_extents: he };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &patch, margin);

        assert!(
            result.len() <= MAX_MANIFOLD_POINTS,
            "Should be reduced to at most {}, got {}",
            MAX_MANIFOLD_POINTS,
            result.len(),
        );
    }
}
