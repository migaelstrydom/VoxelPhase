//! Sphere-sphere discrete collision detection (tier 1, analytic).
//!
//! Single contact point with `FeatureId::SINGLE` (only one possible contact).

use nalgebra::{Point3, Vector3};

use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};

/// Test two spheres for overlap with margin support.
///
/// Returns a single-point manifold if the spheres overlap (including margin).
/// Normal points from A to B. `raw_depth` is geometric depth without margin
/// adjustment; the caller provides margin-expanded radii.
///
/// # Arguments
/// * `center_a`, `center_b` — world-space sphere centers
/// * `radius_a`, `radius_b` — sphere radii (without margin)
/// * `contact_margin` — inflation distance added to each sphere for speculative contacts
pub fn sphere_sphere_manifold(
    center_a: Point3<f32>,
    radius_a: f32,
    center_b: Point3<f32>,
    radius_b: f32,
    contact_margin: f32,
) -> ContactManifold {
    let delta = center_b - center_a;
    let dist_sq = delta.magnitude_squared();
    let expanded_radius = radius_a + radius_b + 2.0 * contact_margin;

    if dist_sq >= expanded_radius * expanded_radius {
        return ContactManifold::empty();
    }

    let dist = dist_sq.sqrt();

    let normal = if dist < 1e-6 {
        Vector3::y()
    } else {
        delta / dist
    };

    // Raw depth: positive means the real shapes overlap, negative means margin-only.
    let raw_depth = (radius_a + radius_b) - dist;

    // Contact point on the surface between the two spheres.
    let point = center_a + normal * (radius_a - raw_depth * 0.5);

    ContactManifold::single(ContactPoint::new(
        point,
        normal,
        raw_depth,
        FeatureId::SINGLE,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overlapping_spheres() {
        let m = sphere_sphere_manifold(
            Point3::new(0.0, 0.0, 0.0),
            1.0,
            Point3::new(1.5, 0.0, 0.0),
            1.0,
            0.0,
        );
        assert_eq!(m.len(), 1);
        let cp = &m.points[0];
        assert!(cp.raw_depth > 0.0);
        assert!(cp.depth > 0.0);
        assert!(cp.normal.x > 0.9);
        assert_eq!(cp.feature_id, FeatureId::SINGLE);
    }

    #[test]
    fn separated_spheres() {
        let m = sphere_sphere_manifold(
            Point3::new(0.0, 0.0, 0.0),
            1.0,
            Point3::new(5.0, 0.0, 0.0),
            1.0,
            0.0,
        );
        assert!(m.is_empty());
    }

    #[test]
    fn margin_only_contact() {
        // Spheres separated by 0.1, but margin of 0.1 catches them.
        let m = sphere_sphere_manifold(
            Point3::new(0.0, 0.0, 0.0),
            1.0,
            Point3::new(2.1, 0.0, 0.0),
            1.0,
            0.1,
        );
        assert_eq!(m.len(), 1);
        let cp = &m.points[0];
        assert!(
            cp.raw_depth < 0.0,
            "Should be margin-only, got raw_depth={}",
            cp.raw_depth
        );
        assert_eq!(cp.depth, 0.0, "Solver depth should be clamped to 0");
    }

    #[test]
    fn coincident_spheres() {
        let m = sphere_sphere_manifold(Point3::origin(), 1.0, Point3::origin(), 1.0, 0.0);
        assert_eq!(m.len(), 1);
        let cp = &m.points[0];
        assert_eq!(cp.normal, Vector3::y());
    }

    #[test]
    fn touching_spheres_zero_depth() {
        let m = sphere_sphere_manifold(
            Point3::new(0.0, 0.0, 0.0),
            1.0,
            Point3::new(2.0, 0.0, 0.0),
            1.0,
            0.0,
        );
        // Exactly touching: dist == combined_radius, but dist_sq == expanded_radius_sq.
        // Float equality: might be empty or have depth ~0.
        if !m.is_empty() {
            assert!(m.points[0].raw_depth.abs() < 1e-5);
        }
    }
}
