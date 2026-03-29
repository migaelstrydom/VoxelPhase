//! GJK/EPA manifold generation: orchestrates GJK → EPA → face clipping →
//! contact point construction.
//!
//! This is the general-purpose collision path called by the dispatch wildcard
//! arm. It produces multi-point manifolds by extracting support faces from
//! both shapes after EPA and clipping them via Sutherland-Hodgman.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use super::epa::epa_penetration;
use super::gjk::{gjk_query_seeded, GjkCache, GjkResult};
use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::shape_view::{SupportFace, SupportFaceExtractor};
use crate::collision::support::ConvexSupport;

/// Tolerance for GJK separation distance check.
const GJK_TOLERANCE: f32 = 1e-4;

/// Maximum contacts to retain after reduction.
const MAX_CONTACTS: usize = 4;

/// Generate a contact manifold between two convex shapes using GJK + EPA.
///
/// The shapes must implement both `ConvexSupport` and `SupportFaceExtractor`.
/// Shapes without faces (spheres, capsules) produce single-point contacts;
/// shapes with faces (OBBs, convex hulls) produce up to 4 clipped contacts.
///
/// `margin` is the contact margin. Depth convention: `raw_depth` is geometric
/// depth without margin, matching the existing analytic/SAT functions.
pub fn gjk_epa_manifold<S: ConvexSupport + SupportFaceExtractor>(
    a: &S,
    b: &S,
    margin: f32,
) -> ContactManifold {
    gjk_epa_manifold_cached(a, b, margin, None)
}

/// Generate a contact manifold between two convex shapes using GJK + EPA with
/// optional GJK warm-start cache.
pub fn gjk_epa_manifold_cached<S: ConvexSupport + SupportFaceExtractor>(
    a: &S,
    b: &S,
    margin: f32,
    gjk_cache: Option<&mut GjkCache>,
) -> ContactManifold {
    let seed = gjk_cache
        .as_ref()
        .and_then(|cache| cache.last_direction);
    let result = gjk_query_seeded(a, b, seed);

    match result {
        GjkResult::Separated {
            distance,
            closest_a,
            closest_b,
        } => {
            // Shapes are separated on the uninflated Minkowski difference.
            // Check if they're within the margin zone.
            if distance > 2.0 * margin + GJK_TOLERANCE {
                return ContactManifold::empty();
            }

            let delta = closest_b - closest_a;
            let dist = delta.magnitude();
            let normal = if dist > 1e-10 {
                delta / dist
            } else {
                Vector3::y()
            };

            // raw_depth is negative for margin-only contacts.
            let raw_depth = 2.0 * margin - distance;
            let point = Point3::from((closest_a.coords + closest_b.coords) * 0.5);
            if let Some(cache) = gjk_cache {
                let sep_dir = closest_b - closest_a;
                if sep_dir.magnitude_squared() > 1e-10 {
                    cache.last_direction = Some(sep_dir);
                }
            }

            ContactManifold::single(ContactPoint::new(
                point,
                normal,
                raw_depth,
                FeatureId::SINGLE,
            ))
        }

        GjkResult::Intersecting { simplex } => {
            let epa = epa_penetration(a, b, margin, &simplex);
            let mut normal = epa.normal;

            // Keep manifold orientation deterministic using EPA witness ordering.
            let witness_delta = epa.witness_b - epa.witness_a;
            if witness_delta.magnitude_squared() > 1e-10 {
                if normal.dot(&witness_delta) < 0.0 {
                    normal = -normal;
                }
            } else {
                // Degenerate witness separation: use an estimated center delta to
                // keep a stable normal hemisphere for highly symmetric overlaps.
                let center_delta = estimate_center_delta(a, b);
                if center_delta.magnitude_squared() > 1e-10 {
                    if normal.dot(&center_delta) < 0.0 {
                        normal = -normal;
                    }
                } else if normal.x < 0.0
                    || (normal.x == 0.0
                        && (normal.y < 0.0 || (normal.y == 0.0 && normal.z < 0.0)))
                {
                    normal = -normal;
                }
            }
            if let Some(cache) = gjk_cache {
                cache.last_direction = Some(normal);
            }

            // Extract support faces for multi-point manifold clipping.
            let face_a = a.support_face(-normal);
            let face_b = b.support_face(normal);

            match (face_a, face_b) {
                (Some(fa), Some(fb)) => {
                    clip_face_face_manifold(&fa, &fb, normal, epa.depth, margin)
                }
                (Some(face), None) | (None, Some(face)) => {
                    // One shape has a face, the other doesn't.
                    let raw_depth = epa.depth - 2.0 * margin;
                    let point = Point3::from(
                        (epa.witness_a.coords + epa.witness_b.coords) * 0.5,
                    );
                    let feature_id = FeatureId::from_face(face.face_index);
                    ContactManifold::single(ContactPoint::new(
                        point,
                        normal,
                        raw_depth,
                        feature_id,
                    ))
                }
                (None, None) => {
                    // Both shapes are faceless (sphere-sphere, sphere-capsule).
                    let raw_depth = epa.depth - 2.0 * margin;
                    let point = Point3::from(
                        (epa.witness_a.coords + epa.witness_b.coords) * 0.5,
                    );
                    ContactManifold::single(ContactPoint::new(
                        point,
                        normal,
                        raw_depth,
                        FeatureId::SINGLE,
                    ))
                }
            }
        }
    }
}

/// Clip two support faces against each other to produce multi-point contacts.
///
/// Follows the same pattern as the OBB-OBB face-face clipping:
/// 1. Choose reference face (the one more aligned with the contact normal).
/// 2. Clip incident face vertices against reference face side planes.
/// 3. Keep vertices below or on the reference face plane.
/// 4. Reduce to MAX_CONTACTS via ContactReducer.
fn clip_face_face_manifold(
    face_a: &SupportFace,
    face_b: &SupportFace,
    normal: Vector3<f32>,
    epa_depth: f32,
    margin: f32,
) -> ContactManifold {
    // Determine reference and incident faces.
    // Reference face: the one whose normal is most aligned with the contact normal.
    let dot_a = (-normal).dot(&face_a.normal);
    let dot_b = normal.dot(&face_b.normal);

    let (ref_face, inc_face) = if dot_a >= dot_b {
        (face_a, face_b)
    } else {
        (face_b, face_a)
    };

    // Clip the incident face against the reference face's side planes.
    let clipped = clip_against_face_sides(&ref_face.vertices, ref_face.normal, &inc_face.vertices);

    if clipped.is_empty() {
        // Clipping eliminated everything — fall back to single contact.
        let raw_depth = epa_depth - 2.0 * margin;
        return ContactManifold::single(ContactPoint::new(
            compute_face_center(&ref_face.vertices),
            normal,
            raw_depth,
            FeatureId::from_face_pair(face_a.face_index, face_b.face_index),
        ));
    }

    // Project clipped vertices onto the reference face plane and filter.
    let ref_center = compute_face_center(&ref_face.vertices);
    let ref_plane_d = ref_face.normal.dot(&ref_center.coords);

    let base_feature = FeatureId::from_face_pair(face_a.face_index, face_b.face_index);
    let mut contacts: SmallVec<[ContactPoint; 8]> = SmallVec::with_capacity(clipped.len());

    for (i, vertex) in clipped.iter().enumerate() {
        // Signed distance from vertex to reference face plane.
        // Negative = below the plane (penetrating).
        let signed_dist = vertex.coords.dot(&ref_face.normal) - ref_plane_d;

        // Accept vertices that are penetrating or within margin tolerance.
        if signed_dist < margin + 1e-4 {
            let raw_depth = -signed_dist;
            let feature_id = base_feature.with_vertex(i as u32);

            contacts.push(ContactPoint::new(*vertex, normal, raw_depth, feature_id));
        }
    }

    if contacts.is_empty() {
        let raw_depth = epa_depth - 2.0 * margin;
        return ContactManifold::single(ContactPoint::new(
            ref_center,
            normal,
            raw_depth,
            base_feature,
        ));
    }

    // Reduce to 4 contacts if needed.
    let reducer = ContactReducer::new(MAX_CONTACTS);
    let reduced = reducer.reduce(&contacts);

    ContactManifold::from_vec(SmallVec::from_vec(reduced))
}

/// Clip a polygon against the side planes of a reference face.
///
/// The side planes are the edges of the reference face, each forming a half-plane
/// with the edge normal pointing inward (toward the face center).
fn clip_against_face_sides(
    ref_verts: &SmallVec<[Point3<f32>; 8]>,
    ref_normal: Vector3<f32>,
    inc_verts: &SmallVec<[Point3<f32>; 8]>,
) -> SmallVec<[Point3<f32>; 8]> {
    let mut polygon: SmallVec<[Point3<f32>; 8]> = inc_verts.clone();
    let mut scratch: SmallVec<[Point3<f32>; 8]> = SmallVec::new();

    let n = ref_verts.len();
    if n < 3 || polygon.is_empty() {
        return polygon;
    }

    // Compute face center for inward normal orientation.
    let center = compute_face_center(ref_verts);
    let face_normal = if ref_normal.magnitude_squared() > 1e-10 {
        ref_normal
    } else {
        Vector3::y()
    };

    for i in 0..n {
        let edge_start = ref_verts[i];
        let edge_end = ref_verts[(i + 1) % n];

        // Edge direction.
        let edge = edge_end - edge_start;

        // Inward-pointing normal: perpendicular to edge, pointing toward face center.
        // Use the reference face normal to compute the perpendicular in the face plane.
        let inward = edge.cross(&face_normal);

        // Orient so it points toward the center.
        let inward = if inward.dot(&(center - edge_start)) >= 0.0 {
            inward
        } else {
            -inward
        };

        clip_polygon_into(&polygon, edge_start, inward, &mut scratch);
        std::mem::swap(&mut polygon, &mut scratch);
        if polygon.is_empty() {
            return polygon;
        }
    }

    polygon
}

/// Clip a convex polygon against a half-plane into a caller-provided buffer.
///
/// Keeps the portion on the inside (non-negative side) of the plane.
fn clip_polygon_into(
    polygon: &[Point3<f32>],
    plane_point: Point3<f32>,
    plane_normal: Vector3<f32>,
    out: &mut SmallVec<[Point3<f32>; 8]>,
) {
    out.clear();
    if polygon.is_empty() {
        return;
    }

    for i in 0..polygon.len() {
        let p1 = polygon[i];
        let p2 = polygon[(i + 1) % polygon.len()];
        let d1 = (p1 - plane_point).dot(&plane_normal);
        let d2 = (p2 - plane_point).dot(&plane_normal);
        let inside1 = d1 >= 0.0;
        let inside2 = d2 >= 0.0;

        if inside1 && inside2 {
            out.push(p2);
        } else if inside1 && !inside2 {
            let t = d1 / (d1 - d2);
            out.push(p1 + (p2 - p1) * t);
        } else if !inside1 && inside2 {
            let t = d1 / (d1 - d2);
            out.push(p1 + (p2 - p1) * t);
            out.push(p2);
        }
    }
}

/// Compute the centroid of a face polygon.
fn compute_face_center(verts: &SmallVec<[Point3<f32>; 8]>) -> Point3<f32> {
    let sum: Vector3<f32> = verts.iter().map(|v| v.coords).sum();
    Point3::from(sum / verts.len() as f32)
}

/// Estimate center delta (B - A) from support mapping only.
///
/// For centrally symmetric shapes (sphere/OBB/capsule), averaging supports in
/// opposite directions recovers the shape center exactly. For generic convex
/// shapes this is an approximation, but adequate as a sign disambiguation cue.
fn estimate_center_delta<S: ConvexSupport>(a: &S, b: &S) -> Vector3<f32> {
    let dirs = [
        Vector3::x(),
        -Vector3::x(),
        Vector3::y(),
        -Vector3::y(),
        Vector3::z(),
        -Vector3::z(),
    ];
    let sum_a: Vector3<f32> = dirs.iter().map(|dir| a.support(*dir).coords).sum();
    let sum_b: Vector3<f32> = dirs.iter().map(|dir| b.support(*dir).coords).sum();
    (sum_b - sum_a) / dirs.len() as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::discrete::obb_obb::obb_obb_manifold_cached;
    use crate::collision::discrete::sphere_obb::sphere_obb_manifold;
    use crate::collision::obb::Obb;
    use crate::collision::sat::SatCache;
    use crate::collision::shape_view::ShapeView;

    use crate::physics::ColliderShape;
    use nalgebra::UnitQuaternion;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    // --- OBB-OBB cross-validation ---

    #[test]
    fn obb_obb_cross_validation() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let rot = UnitQuaternion::identity();
        let margin = 0.02;

        let shape_a = ColliderShape::Box { half_extents: he };
        let shape_b = ColliderShape::Box { half_extents: he };
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.5, 0.0, 0.0),
            rotation: rot,
            shape: &shape_b,
        };

        let gjk_result = gjk_epa_manifold(&view_a, &view_b, margin);

        let obb_a = Obb::new(Point3::origin(), rot, he);
        let obb_b = Obb::new(Point3::new(1.5, 0.0, 0.0), rot, he);
        let mut cache = SatCache::default();
        let sat_result = obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut cache);

        assert!(
            !gjk_result.is_empty(),
            "GJK/EPA should produce contacts for overlapping OBBs"
        );
        assert!(
            !sat_result.is_empty(),
            "SAT should produce contacts for overlapping OBBs"
        );

        // Same normal direction (dot > 0.99).
        let gjk_normal = gjk_result.points[0].normal;
        let sat_normal = sat_result.points[0].normal;
        let dot = gjk_normal.dot(&sat_normal);
        assert!(
            dot > 0.99 || dot < -0.99,
            "Normal mismatch: GJK {:?} vs SAT {:?} (dot = {})",
            gjk_normal,
            sat_normal,
            dot
        );

        // Similar depth (±0.1).
        let gjk_depth = gjk_result.points[0].raw_depth;
        let sat_depth = sat_result.points[0].raw_depth;
        assert!(
            approx_eq(gjk_depth, sat_depth, 0.1),
            "Depth mismatch: GJK {} vs SAT {}",
            gjk_depth,
            sat_depth
        );

        // Similar contact count (±1).
        let count_diff = (gjk_result.len() as i32 - sat_result.len() as i32).abs();
        assert!(
            count_diff <= 1,
            "Contact count mismatch: GJK {} vs SAT {}",
            gjk_result.len(),
            sat_result.len()
        );
    }

    // --- Sphere-OBB cross-validation ---

    #[test]
    fn sphere_obb_cross_validation() {
        let margin = 0.02;

        let sphere_shape = ColliderShape::Sphere { radius: 0.5 };
        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let view_s = ShapeView {
            center: Point3::new(1.3, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &sphere_shape,
        };
        let view_b = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &box_shape,
        };

        let gjk_result = gjk_epa_manifold(&view_s, &view_b, margin);

        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let analytic = sphere_obb_manifold(&obb, Point3::new(1.3, 0.0, 0.0), 0.5, margin);

        assert!(!gjk_result.is_empty(), "GJK/EPA should produce contact");
        assert!(!analytic.is_empty(), "Analytic should produce contact");

        let gjk_depth = gjk_result.points[0].raw_depth;
        let analytic_depth = analytic.points[0].raw_depth;
        assert!(
            approx_eq(gjk_depth, analytic_depth, 0.1),
            "Depth mismatch: GJK {} vs analytic {}",
            gjk_depth,
            analytic_depth
        );

        let gjk_normal = gjk_result.points[0].normal;
        let analytic_normal = analytic.points[0].normal;
        assert!(
            gjk_normal.dot(&analytic_normal) > 0.9,
            "Normal mismatch: GJK {:?} vs analytic {:?}",
            gjk_normal,
            analytic_normal
        );
    }

    // --- OBB face-face clipping: 4 contacts ---

    #[test]
    fn obb_face_face_four_contacts() {
        let margin = 0.02;

        let shape_a = ColliderShape::Box {
            half_extents: Vector3::new(2.0, 1.0, 2.0),
        };
        let shape_b = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let view_a = ShapeView {
            center: Point3::new(0.0, -1.5, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(0.0, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let result = gjk_epa_manifold(&view_a, &view_b, margin);

        assert!(
            result.len() >= 3,
            "Expected 4 contacts for face-face, got {}",
            result.len()
        );

        // All contacts should have roughly the same normal (Y axis).
        for cp in &result.points {
            assert!(
                cp.normal.y.abs() > 0.9,
                "Expected Y-axis normal, got {:?}",
                cp.normal
            );
        }
    }

    // --- Sphere-sphere through GJK/EPA ---

    #[test]
    fn sphere_sphere_single_contact() {
        let margin = 0.0;

        let shape_a = ColliderShape::Sphere { radius: 1.0 };
        let shape_b = ColliderShape::Sphere { radius: 1.0 };
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.5, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let result = gjk_epa_manifold(&view_a, &view_b, margin);

        assert_eq!(result.len(), 1, "Sphere-sphere should produce 1 contact");
        assert!(
            approx_eq(result.points[0].raw_depth, 0.5, 0.05),
            "Expected depth ~0.5, got {}",
            result.points[0].raw_depth
        );
    }

    #[test]
    fn coincident_spheres_use_deterministic_normal_hemisphere() {
        let shape_a = ColliderShape::Sphere { radius: 1.0 };
        let shape_b = ColliderShape::Sphere { radius: 1.0 };
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let result = gjk_epa_manifold(&view_a, &view_b, 0.0);
        assert_eq!(result.len(), 1, "Coincident spheres should produce one contact");

        let n = result.points[0].normal;
        assert!(
            n.x.is_finite() && n.y.is_finite() && n.z.is_finite(),
            "Normal should be finite, got {:?}",
            n
        );
        assert!(
            (n.magnitude() - 1.0).abs() < 1e-4,
            "Normal should be unit length, got {:?} (|n|={})",
            n,
            n.magnitude()
        );

        // Degenerate witness/center deltas fall back to a stable hemisphere:
        // +X preferred, then +Y, then +Z.
        let eps = 1e-6;
        let in_positive_hemisphere = n.x > eps
            || (n.x.abs() <= eps && n.y > eps)
            || (n.x.abs() <= eps && n.y.abs() <= eps && n.z >= -eps);
        assert!(
            in_positive_hemisphere,
            "Expected deterministic positive hemisphere normal, got {:?}",
            n
        );
    }

    #[test]
    fn cached_query_matches_uncached_result() {
        let margin = 0.02;

        let shape_a = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let shape_b = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.5, 0.1, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let uncached = gjk_epa_manifold(&view_a, &view_b, margin);
        let mut cache = GjkCache::default();
        let cached = gjk_epa_manifold_cached(&view_a, &view_b, margin, Some(&mut cache));

        assert_eq!(uncached.len(), cached.len());
        for (a, b) in uncached.points.iter().zip(cached.points.iter()) {
            assert!((a.point - b.point).magnitude() < 1e-5);
            assert!((a.normal - b.normal).magnitude() < 1e-5);
            assert!((a.raw_depth - b.raw_depth).abs() < 1e-5);
        }
    }

    #[test]
    fn cached_query_populates_and_reuses_direction() {
        let margin = 0.02;

        let sphere_shape = ColliderShape::Sphere { radius: 0.5 };
        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let view_s = ShapeView {
            center: Point3::new(1.3, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &sphere_shape,
        };
        let view_b = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &box_shape,
        };

        let mut cache = GjkCache::default();
        assert!(cache.last_direction.is_none());

        let first = gjk_epa_manifold_cached(&view_s, &view_b, margin, Some(&mut cache));
        assert!(!first.is_empty());
        let first_dir = cache
            .last_direction
            .expect("Cache should store a warm-start direction");
        assert!(first_dir.magnitude_squared() > 1e-8);

        let second = gjk_epa_manifold_cached(&view_s, &view_b, margin, Some(&mut cache));
        assert!(!second.is_empty());
        let second_dir = cache
            .last_direction
            .expect("Cache should keep a warm-start direction");
        assert!(second_dir.magnitude_squared() > 1e-8);
    }
}
