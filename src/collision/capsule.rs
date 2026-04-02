//! Capsule collision shape representation and utilities.
//!
//! A capsule is a cylinder with hemisphere caps. The axis is local Y, rotated
//! by the `rotation` quaternion. `half_height` includes the caps, so the
//! central segment half-length is `half_height - radius`. Total height =
//! `2 * half_height`.

use nalgebra::{Point3, UnitQuaternion, Vector3};

/// A capsule in world space.
#[derive(Debug, Clone, Copy)]
pub struct Capsule {
    /// World-space center.
    pub center: Point3<f32>,
    /// Orientation in world space.
    pub rotation: UnitQuaternion<f32>,
    /// Half of the total height (center to cap tip). Must be >= radius.
    pub half_height: f32,
    /// Radius of the cylinder and hemisphere caps.
    pub radius: f32,
}

impl Capsule {
    pub fn new(
        center: Point3<f32>,
        rotation: UnitQuaternion<f32>,
        half_height: f32,
        radius: f32,
    ) -> Self {
        debug_assert!(
            half_height >= radius,
            "Capsule half_height ({half_height}) must be >= radius ({radius})"
        );
        Self {
            center,
            rotation,
            half_height,
            radius,
        }
    }

    /// Local Y axis in world space.
    pub fn up(&self) -> Vector3<f32> {
        self.rotation * Vector3::y()
    }

    /// Half-length of the central segment (excluding caps).
    pub fn segment_half_length(&self) -> f32 {
        self.half_height - self.radius
    }

    /// World-space endpoints of the central segment.
    pub fn segment_endpoints(&self) -> (Point3<f32>, Point3<f32>) {
        let half_seg = self.segment_half_length();
        let up = self.up();
        (self.center - up * half_seg, self.center + up * half_seg)
    }

    /// Compute the AABB that fully encloses this capsule.
    pub fn enclosing_aabb(&self) -> (Point3<f32>, Point3<f32>) {
        let (a, b) = self.segment_endpoints();
        let min = Point3::new(
            a.x.min(b.x) - self.radius,
            a.y.min(b.y) - self.radius,
            a.z.min(b.z) - self.radius,
        );
        let max = Point3::new(
            a.x.max(b.x) + self.radius,
            a.y.max(b.y) + self.radius,
            a.z.max(b.z) + self.radius,
        );
        (min, max)
    }

    /// Bounding radius from center to the furthest point.
    #[allow(unused)]
    pub fn bounding_radius(&self) -> f32 {
        self.half_height
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn segment_endpoints_identity() {
        let c = Capsule::new(Point3::origin(), UnitQuaternion::identity(), 2.0, 0.5);
        let (a, b) = c.segment_endpoints();
        assert!((a - Point3::new(0.0, -1.5, 0.0)).magnitude() < 1e-6);
        assert!((b - Point3::new(0.0, 1.5, 0.0)).magnitude() < 1e-6);
    }

    #[test]
    fn segment_endpoints_rotated_90_z() {
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::z()),
            FRAC_PI_2,
        );
        let c = Capsule::new(Point3::origin(), rot, 2.0, 0.5);
        let (a, b) = c.segment_endpoints();
        // Rotated 90° around Z: local Y → world -X
        assert!((a - Point3::new(1.5, 0.0, 0.0)).magnitude() < 1e-4);
        assert!((b - Point3::new(-1.5, 0.0, 0.0)).magnitude() < 1e-4);
    }

    #[test]
    fn degenerate_sphere() {
        let c = Capsule::new(Point3::origin(), UnitQuaternion::identity(), 0.5, 0.5);
        let (a, b) = c.segment_endpoints();
        assert!((a - Point3::origin()).magnitude() < 1e-6);
        assert!((b - Point3::origin()).magnitude() < 1e-6);
    }

    #[test]
    fn enclosing_aabb_identity() {
        let c = Capsule::new(Point3::origin(), UnitQuaternion::identity(), 2.0, 0.5);
        let (min, max) = c.enclosing_aabb();
        assert!((min - Point3::new(-0.5, -2.0, -0.5)).magnitude() < 1e-6);
        assert!((max - Point3::new(0.5, 2.0, 0.5)).magnitude() < 1e-6);
    }

    #[test]
    fn bounding_radius_equals_half_height() {
        let c = Capsule::new(Point3::origin(), UnitQuaternion::identity(), 3.0, 1.0);
        assert_eq!(c.bounding_radius(), 3.0);
    }
}
