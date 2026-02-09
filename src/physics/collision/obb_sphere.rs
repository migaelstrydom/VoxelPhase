//! OBB vs sphere collision using closest-point projection.
//!
//! Projects the sphere center onto the OBB surface, then checks if the
//! closest point is within the sphere radius.

use nalgebra::{Point3, Vector3};

use super::obb::Obb;

/// Contact from OBB-sphere intersection.
#[derive(Debug, Clone)]
pub struct ObbSphereContact {
    /// World-space contact point (on the OBB surface).
    pub point: Point3<f32>,
    /// Contact normal pointing from the OBB toward the sphere center.
    pub normal: Vector3<f32>,
    /// Penetration depth (positive = overlapping).
    pub depth: f32,
}

/// Test an OBB against a sphere.
///
/// Returns a single contact if the shapes overlap. Normal points from
/// the OBB surface toward the sphere center.
///
/// # Arguments
/// * `obb` - The oriented bounding box in world space
/// * `sphere_center` - World-space center of the sphere
/// * `sphere_radius` - Radius of the sphere
pub fn obb_sphere_contact(
    obb: &Obb,
    sphere_center: Point3<f32>,
    sphere_radius: f32,
) -> Option<ObbSphereContact> {
    let closest = obb.closest_point(sphere_center);
    let to_center = sphere_center - closest;
    let dist_sq = to_center.magnitude_squared();

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
        let dist = dist_sq.sqrt();
        if dist > sphere_radius {
            return None;
        }
        let normal = if dist > 1e-6 {
            to_center / dist
        } else {
            (sphere_center - obb.center).normalize()
        };
        return Some(ObbSphereContact {
            point: closest,
            normal,
            depth: sphere_radius - dist,
        });
    }

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

    Some(ObbSphereContact {
        point: contact_point,
        normal,
        depth: sphere_radius + min_face_dist,
    })
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
        let contact = obb_sphere_contact(&unit_box(), Point3::new(2.0, 0.0, 0.0), 1.0);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!((c.normal - Vector3::x()).magnitude() < 0.1);
        assert!(c.depth.abs() < 0.01, "Touching, depth should be ~0");
    }

    #[test]
    fn sphere_overlapping() {
        let contact = obb_sphere_contact(&unit_box(), Point3::new(1.5, 0.0, 0.0), 1.0);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.depth > 0.4, "Should have ~0.5 penetration, got {}", c.depth);
    }

    #[test]
    fn sphere_separated() {
        let contact = obb_sphere_contact(&unit_box(), Point3::new(5.0, 0.0, 0.0), 1.0);
        assert!(contact.is_none());
    }

    #[test]
    fn sphere_near_corner() {
        let contact = obb_sphere_contact(&unit_box(), Point3::new(1.5, 1.5, 1.5), 1.0);
        let dist_to_corner = (Point3::new(1.5, 1.5, 1.5) - Point3::new(1.0, 1.0, 1.0)).magnitude();
        if dist_to_corner < 1.0 {
            assert!(contact.is_some());
        } else {
            assert!(contact.is_none());
        }
    }

    #[test]
    fn sphere_inside_box() {
        let contact = obb_sphere_contact(&unit_box(), Point3::new(0.0, 0.0, 0.0), 0.5);
        assert!(contact.is_some(), "Sphere inside box should produce contact");
        let c = contact.unwrap();
        assert!(c.depth > 0.0);
    }
}
