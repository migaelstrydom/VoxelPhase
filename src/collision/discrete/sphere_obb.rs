//! Sphere-OBB discrete collision detection (tier 1, analytic).
//!
//! Projects the sphere center onto the OBB surface, then checks distance.
//! Handles both exterior and interior (sphere center inside box) cases.

use nalgebra::{Point3, Vector3};

use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::obb::Obb;

/// Test a sphere against an OBB with margin support.
///
/// Returns a single-point manifold if the shapes overlap (including margin).
/// Normal points from the OBB (A) toward the sphere (B).
///
/// # Arguments
/// * `obb` — the oriented bounding box in world space
/// * `sphere_center` — world-space center of the sphere
/// * `sphere_radius` — radius of the sphere (without margin)
/// * `contact_margin` — inflation distance for speculative contacts
pub fn sphere_obb_manifold(
    obb: &Obb,
    sphere_center: Point3<f32>,
    sphere_radius: f32,
    contact_margin: f32,
) -> ContactManifold {
    let expanded_radius = sphere_radius + contact_margin;
    let axes = obb.axes();
    let delta = sphere_center - obb.center;
    let local = [
        delta.dot(&axes[0]),
        delta.dot(&axes[1]),
        delta.dot(&axes[2]),
    ];

    let inside = local[0].abs() <= obb.half_extents.x + 1e-6
        && local[1].abs() <= obb.half_extents.y + 1e-6
        && local[2].abs() <= obb.half_extents.z + 1e-6;

    if !inside {
        return exterior_contact(obb, sphere_center, sphere_radius, expanded_radius);
    }

    interior_contact(obb, sphere_radius, &axes, &local)
}

/// Sphere center is outside the OBB: use closest-point projection.
fn exterior_contact(
    obb: &Obb,
    sphere_center: Point3<f32>,
    sphere_radius: f32,
    expanded_radius: f32,
) -> ContactManifold {
    let closest = obb.closest_point(sphere_center);
    let to_center = sphere_center - closest;
    let dist_sq = to_center.magnitude_squared();

    if dist_sq > expanded_radius * expanded_radius {
        return ContactManifold::empty();
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-6 {
        to_center / dist
    } else {
        (sphere_center - obb.center).normalize()
    };

    let raw_depth = sphere_radius - dist;
    let feature_id = classify_exterior_feature(obb, closest);

    ContactManifold::single(ContactPoint::new(closest, normal, raw_depth, feature_id))
}

/// Sphere center is inside the OBB: find nearest face and push out.
fn interior_contact(
    obb: &Obb,
    sphere_radius: f32,
    axes: &[Vector3<f32>; 3],
    local: &[f32; 3],
) -> ContactManifold {
    // Find the OBB face closest to the sphere center.
    let face_dists = [
        obb.half_extents.x - local[0].abs(),
        obb.half_extents.y - local[1].abs(),
        obb.half_extents.z - local[2].abs(),
    ];
    let mut min_axis = 0;
    let mut min_face_dist = face_dists[0];
    for i in 1..3 {
        if face_dists[i] < min_face_dist {
            min_face_dist = face_dists[i];
            min_axis = i;
        }
    }

    let sign = if local[min_axis] >= 0.0 { 1.0 } else { -1.0 };
    let normal = axes[min_axis] * sign;

    // Contact point on the nearest face, projected from sphere center.
    let mut contact_point = obb.center;
    let clamped = [
        if min_axis == 0 {
            sign * obb.half_extents.x
        } else {
            local[0]
        },
        if min_axis == 1 {
            sign * obb.half_extents.y
        } else {
            local[1]
        },
        if min_axis == 2 {
            sign * obb.half_extents.z
        } else {
            local[2]
        },
    ];
    for i in 0..3 {
        contact_point += axes[i] * clamped[i];
    }

    let raw_depth = sphere_radius + min_face_dist;
    // Face feature: the face index is min_axis * 2 + (0 if positive, 1 if negative).
    let face_idx = (min_axis as u32) * 2 + if sign > 0.0 { 0 } else { 1 };
    let feature_id = FeatureId::from_face(face_idx);

    ContactManifold::single(ContactPoint::new(
        contact_point,
        normal,
        raw_depth,
        feature_id,
    ))
}

/// Classify a closest point on the OBB surface into a face feature.
///
/// Returns a `FeatureId` based on which OBB face the closest point lies on.
fn classify_exterior_feature(obb: &Obb, point: Point3<f32>) -> FeatureId {
    let axes = obb.axes();
    let delta = point - obb.center;
    let local = [
        delta.dot(&axes[0]),
        delta.dot(&axes[1]),
        delta.dot(&axes[2]),
    ];

    // The closest point on the surface is on the face where |local[i]| is closest
    // to half_extents[i]. Find the axis where the projection is at the boundary.
    let mut best_axis = 0;
    let mut best_dist = f32::MAX;
    for i in 0..3 {
        let dist_to_face = (local[i].abs() - obb.half_extents[i]).abs();
        if dist_to_face < best_dist {
            best_dist = dist_to_face;
            best_axis = i;
        }
    }

    let sign = if local[best_axis] >= 0.0 { 0 } else { 1 };
    FeatureId::from_face((best_axis as u32) * 2 + sign)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    fn unit_box() -> Obb {
        Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        )
    }

    #[test]
    fn sphere_touching_face() {
        let m = sphere_obb_manifold(&unit_box(), Point3::new(2.0, 0.0, 0.0), 1.0, 0.0);
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!((c.normal - Vector3::x()).magnitude() < 0.1);
        assert!(c.raw_depth.abs() < 0.01);
    }

    #[test]
    fn sphere_overlapping() {
        let m = sphere_obb_manifold(&unit_box(), Point3::new(1.5, 0.0, 0.0), 1.0, 0.0);
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth > 0.4);
        assert!(c.depth > 0.4);
    }

    #[test]
    fn sphere_separated() {
        let m = sphere_obb_manifold(&unit_box(), Point3::new(5.0, 0.0, 0.0), 1.0, 0.0);
        assert!(m.is_empty());
    }

    #[test]
    fn sphere_near_corner() {
        let m = sphere_obb_manifold(&unit_box(), Point3::new(1.5, 1.5, 1.5), 1.0, 0.0);
        let dist_to_corner = (Point3::new(1.5, 1.5, 1.5) - Point3::new(1.0, 1.0, 1.0)).magnitude();
        if dist_to_corner < 1.0 {
            assert_eq!(m.len(), 1);
        } else {
            assert!(m.is_empty());
        }
    }

    #[test]
    fn sphere_inside_box() {
        let m = sphere_obb_manifold(&unit_box(), Point3::new(0.0, 0.0, 0.0), 0.5, 0.0);
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth > 0.0);
    }

    #[test]
    fn margin_only_contact() {
        // Sphere 0.05 away from OBB face, margin 0.1 catches it.
        let m = sphere_obb_manifold(&unit_box(), Point3::new(2.05, 0.0, 0.0), 1.0, 0.1);
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
    fn feature_id_is_face() {
        let m = sphere_obb_manifold(&unit_box(), Point3::new(2.0, 0.0, 0.0), 1.5, 0.0);
        assert_eq!(m.len(), 1);
        // +X face = axis 0, positive sign = face index 0
        assert_eq!(m.points[0].feature_id, FeatureId::from_face(0));
    }

    #[test]
    fn rotated_obb() {
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::y()),
            std::f32::consts::FRAC_PI_4,
        );
        let obb = Obb::new(Point3::origin(), rot, Vector3::new(1.0, 1.0, 1.0));
        let m = sphere_obb_manifold(&obb, Point3::new(2.0, 0.0, 0.0), 1.0, 0.0);
        // Rotated 45°: half-diagonal on X is sqrt(2), so sphere at 2.0 overlaps.
        assert_eq!(m.len(), 1);
    }
}
