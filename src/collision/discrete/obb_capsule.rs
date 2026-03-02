//! OBB-capsule discrete collision detection (tier 2).
//!
//! Samples multiple points along the capsule segment, picks the deepest
//! sphere-OBB contact.

use crate::collision::capsule::Capsule;
use crate::collision::contact::ContactManifold;
use crate::collision::discrete::sphere_obb::sphere_obb_manifold;
use crate::collision::obb::Obb;

/// Test an OBB against a capsule with margin support.
///
/// Samples the capsule segment at both endpoints and the point closest to the
/// OBB center, then runs sphere-OBB at each sample and returns the deepest
/// contact.
///
/// Normal points from the OBB (A) toward the capsule (B).
///
/// # Arguments
/// * `obb` — the oriented bounding box in world space
/// * `capsule` — the capsule in world space
/// * `contact_margin` — inflation distance for speculative contacts
pub fn obb_capsule_manifold(
    obb: &Obb,
    capsule: &Capsule,
    contact_margin: f32,
) -> ContactManifold {
    let (seg_a, seg_b) = capsule.segment_endpoints();
    let seg_dir = seg_b - seg_a;
    let seg_len_sq = seg_dir.magnitude_squared();

    // Sample 1 & 2: segment endpoints.
    // Sample 3: point on segment closest to OBB center.
    let closest_to_center = if seg_len_sq < 1e-12 {
        seg_a
    } else {
        let t = ((obb.center - seg_a).dot(&seg_dir) / seg_len_sq).clamp(0.0, 1.0);
        seg_a + seg_dir * t
    };

    let samples = [seg_a, seg_b, closest_to_center];
    let mut best: Option<ContactManifold> = None;
    let mut best_depth = f32::NEG_INFINITY;

    for sample in &samples {
        let m = sphere_obb_manifold(obb, *sample, capsule.radius, contact_margin);
        if !m.is_empty() {
            let depth = m.points[0].raw_depth;
            if depth > best_depth {
                best_depth = depth;
                best = Some(m);
            }
        }
    }

    best.unwrap_or_else(ContactManifold::empty)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, UnitQuaternion, Vector3};

    fn unit_box() -> Obb {
        Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        )
    }

    fn upright_capsule(x: f32, y: f32) -> Capsule {
        Capsule::new(
            Point3::new(x, y, 0.0),
            UnitQuaternion::identity(),
            1.0,
            0.3,
        )
    }

    #[test]
    fn capsule_touching_box_face() {
        let obb = unit_box();
        let capsule = upright_capsule(1.3, 0.0);
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.normal.x > 0.9);
        assert!(c.raw_depth.abs() < 0.05);
    }

    #[test]
    fn capsule_overlapping_box() {
        let obb = unit_box();
        let capsule = upright_capsule(1.0, 0.0);
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert_eq!(m.len(), 1);
        assert!(m.points[0].raw_depth > 0.0);
    }

    #[test]
    fn capsule_separated_from_box() {
        let obb = unit_box();
        let capsule = upright_capsule(5.0, 0.0);
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert!(m.is_empty());
    }

    #[test]
    fn capsule_cap_touching_box() {
        let obb = unit_box();
        // Capsule above the box: center at y=2.0, half_height=1.0, radius=0.3
        // Bottom cap at y = 2.0 - 1.0 = 1.0 which is exactly the box top face.
        // Move slightly lower to ensure penetration.
        let capsule = upright_capsule(0.0, 1.9);
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert_eq!(m.len(), 1);
        assert!(m.points[0].normal.y > 0.9);
    }

    #[test]
    fn margin_only() {
        let obb = unit_box();
        let capsule = upright_capsule(1.35, 0.0);
        let m = obb_capsule_manifold(&obb, &capsule, 0.1);
        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth < 0.0);
        assert_eq!(c.depth, 0.0);
    }
}
