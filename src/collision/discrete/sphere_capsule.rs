//! Sphere-capsule discrete collision detection (tier 1, analytic).
//!
//! Closest point on capsule segment to sphere center, then sphere-sphere test.

use nalgebra::{Point3, Vector3};

use crate::collision::capsule::Capsule;
use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::segment::segment_segment_closest_points;

/// Test a sphere against a capsule with margin support.
///
/// Normal points from the capsule (A) toward the sphere (B).
///
/// # Arguments
/// * `capsule` — the capsule in world space
/// * `sphere_center` — world-space center of the sphere
/// * `sphere_radius` — radius of the sphere (without margin)
/// * `contact_margin` — inflation distance for speculative contacts
pub fn sphere_capsule_manifold(
    capsule: &Capsule,
    sphere_center: Point3<f32>,
    sphere_radius: f32,
    contact_margin: f32,
) -> ContactManifold {
    let (seg_a, seg_b) = capsule.segment_endpoints();

    // Closest point on capsule segment to sphere center (degenerate point-vs-segment).
    let (_, closest_on_seg) = segment_segment_closest_points(
        sphere_center,
        sphere_center,
        seg_a,
        seg_b,
    );

    // Now it's a sphere-sphere test between (closest_on_seg, capsule.radius)
    // and (sphere_center, sphere_radius).
    let delta = sphere_center - closest_on_seg;
    let dist_sq = delta.magnitude_squared();
    let combined_radius = capsule.radius + sphere_radius;
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
    let point = closest_on_seg + normal * (capsule.radius - raw_depth * 0.5);

    ContactManifold::single(ContactPoint::new(point, normal, raw_depth, FeatureId::SINGLE))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    fn upright_capsule(y: f32) -> Capsule {
        Capsule::new(
            Point3::new(0.0, y, 0.0),
            UnitQuaternion::identity(),
            1.0,
            0.5,
        )
    }

    #[test]
    fn sphere_touching_capsule_side() {
        let capsule = upright_capsule(0.0);
        let m = sphere_capsule_manifold(
            &capsule,
            Point3::new(1.4, 0.0, 0.0),
            1.0,
            0.0,
        );
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth > 0.0);
        assert!(c.normal.x > 0.9);
    }

    #[test]
    fn sphere_overlapping_capsule_cap() {
        let capsule = upright_capsule(0.0);
        let m = sphere_capsule_manifold(
            &capsule,
            Point3::new(0.0, 1.8, 0.0),
            1.0,
            0.0,
        );
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth > 0.0);
        assert!(c.normal.y > 0.9);
    }

    #[test]
    fn sphere_separated_from_capsule() {
        let capsule = upright_capsule(0.0);
        let m = sphere_capsule_manifold(
            &capsule,
            Point3::new(5.0, 0.0, 0.0),
            1.0,
            0.0,
        );
        assert!(m.is_empty());
    }

    #[test]
    fn margin_only_contact() {
        let capsule = upright_capsule(0.0);
        // Sphere just beyond contact distance, but within margin.
        let m = sphere_capsule_manifold(
            &capsule,
            Point3::new(1.55, 0.0, 0.0),
            1.0,
            0.1,
        );
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth < 0.0);
        assert_eq!(c.depth, 0.0);
    }

    #[test]
    fn coincident_centers() {
        let capsule = upright_capsule(0.0);
        let m = sphere_capsule_manifold(
            &capsule,
            Point3::new(0.0, 0.0, 0.0),
            0.3,
            0.0,
        );
        assert_eq!(m.len(), 1);
        assert!(m.points[0].raw_depth > 0.0);
    }
}
