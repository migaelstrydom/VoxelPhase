//! Capsule-capsule discrete collision detection (tier 2).
//!
//! Closest points between two capsule segments, then sphere-sphere test.

use nalgebra::Vector3;

use crate::collision::capsule::Capsule;
use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::segment::segment_segment_closest_points;

/// Test two capsules for overlap with margin support.
///
/// Normal points from capsule A toward capsule B.
///
/// # Arguments
/// * `cap_a`, `cap_b` — the two capsules in world space
/// * `contact_margin` — inflation distance for speculative contacts
pub fn capsule_capsule_manifold(
    cap_a: &Capsule,
    cap_b: &Capsule,
    contact_margin: f32,
) -> ContactManifold {
    let (a0, a1) = cap_a.segment_endpoints();
    let (b0, b1) = cap_b.segment_endpoints();

    let (closest_a, closest_b) = segment_segment_closest_points(a0, a1, b0, b1);

    let delta = closest_b - closest_a;
    let dist_sq = delta.magnitude_squared();
    let combined_radius = cap_a.radius + cap_b.radius;
    let expanded = combined_radius + 2.0 * contact_margin;

    if dist_sq >= expanded * expanded {
        return ContactManifold::empty();
    }

    let dist = dist_sq.sqrt();
    let normal = if dist < 1e-6 {
        Vector3::y()
    } else {
        delta / dist
    };

    let raw_depth = combined_radius - dist;
    let point = closest_a + normal * (cap_a.radius - raw_depth * 0.5);

    ContactManifold::single(ContactPoint::new(point, normal, raw_depth, FeatureId::SINGLE))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, UnitQuaternion};

    fn upright_capsule(x: f32, y: f32) -> Capsule {
        Capsule::new(
            Point3::new(x, y, 0.0),
            UnitQuaternion::identity(),
            1.0,
            0.5,
        )
    }

    #[test]
    fn parallel_capsules_overlapping() {
        let a = upright_capsule(0.0, 0.0);
        let b = upright_capsule(0.8, 0.0);
        let m = capsule_capsule_manifold(&a, &b, 0.0);
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth > 0.0);
        assert!(c.normal.x > 0.9);
    }

    #[test]
    fn parallel_capsules_separated() {
        let a = upright_capsule(0.0, 0.0);
        let b = upright_capsule(5.0, 0.0);
        let m = capsule_capsule_manifold(&a, &b, 0.0);
        assert!(m.is_empty());
    }

    #[test]
    fn perpendicular_capsules() {
        let a = Capsule::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            1.0,
            0.3,
        );
        let rot_z = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::z()),
            std::f32::consts::FRAC_PI_2,
        );
        let b = Capsule::new(
            Point3::new(0.0, 0.0, 0.0),
            rot_z,
            1.0,
            0.3,
        );
        let m = capsule_capsule_manifold(&a, &b, 0.0);
        assert_eq!(m.len(), 1);
        assert!(m.points[0].raw_depth > 0.0);
    }

    #[test]
    fn margin_only() {
        let a = upright_capsule(0.0, 0.0);
        let b = upright_capsule(1.05, 0.0);
        let m = capsule_capsule_manifold(&a, &b, 0.1);
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth < 0.0);
        assert_eq!(c.depth, 0.0);
    }
}
