//! Axis-Aligned Bounding Box implementation.

use nalgebra::{Point3, Vector3};

/// An axis-aligned bounding box defined by min and max corners.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AABB {
    pub min: Point3<f32>,
    pub max: Point3<f32>,
}

impl AABB {
    /// Create a new AABB from min and max corners.
    pub fn new(min: Point3<f32>, max: Point3<f32>) -> Self {
        debug_assert!(min.x <= max.x && min.y <= max.y && min.z <= max.z);
        Self { min, max }
    }

    /// Create an AABB from center and half-extents.
    pub fn from_center_half_extents(center: Point3<f32>, half_extents: Vector3<f32>) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    /// Create an empty (inverted) AABB suitable for expansion.
    pub fn empty() -> Self {
        Self {
            min: Point3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            max: Point3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
        }
    }

    /// Get the center point of the AABB.
    pub fn center(&self) -> Point3<f32> {
        Point3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// Get the half-extents (half the size on each axis).
    pub fn half_extents(&self) -> Vector3<f32> {
        Vector3::new(
            (self.max.x - self.min.x) * 0.5,
            (self.max.y - self.min.y) * 0.5,
            (self.max.z - self.min.z) * 0.5,
        )
    }

    /// Get the full size on each axis.
    pub fn size(&self) -> Vector3<f32> {
        Vector3::new(
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        )
    }

    /// Check if this AABB intersects another AABB.
    pub fn intersects(&self, other: &AABB) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Check if this AABB contains a point.
    pub fn contains_point(&self, point: Point3<f32>) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Check if a sphere intersects this AABB.
    pub fn intersects_sphere(&self, center: Point3<f32>, radius: f32) -> bool {
        // Find the closest point on the AABB to the sphere center
        let closest = self.closest_point(center);
        // Check if that point is within the sphere
        let dist_sq = (closest - center).magnitude_squared();
        dist_sq <= radius * radius
    }

    /// Find the closest point on or in the AABB to a given point.
    pub fn closest_point(&self, point: Point3<f32>) -> Point3<f32> {
        Point3::new(
            point.x.clamp(self.min.x, self.max.x),
            point.y.clamp(self.min.y, self.max.y),
            point.z.clamp(self.min.z, self.max.z),
        )
    }
}

impl Default for AABB {
    fn default() -> Self {
        Self::empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aabb_intersection() {
        let a = AABB::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        let b = AABB::new(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));
        let c = AABB::new(Point3::new(5.0, 5.0, 5.0), Point3::new(6.0, 6.0, 6.0));

        assert!(a.intersects(&b));
        assert!(b.intersects(&a));
        assert!(!a.intersects(&c));
        assert!(!c.intersects(&a));
    }

    #[test]
    fn test_sphere_intersection() {
        let aabb = AABB::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));

        // Sphere inside
        assert!(aabb.intersects_sphere(Point3::new(1.0, 1.0, 1.0), 0.5));
        // Sphere touching corner
        assert!(aabb.intersects_sphere(Point3::new(3.0, 3.0, 3.0), 2.0));
        // Sphere far away
        assert!(!aabb.intersects_sphere(Point3::new(10.0, 10.0, 10.0), 1.0));
    }

    #[test]
    fn test_contains_point() {
        let aabb = AABB::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));

        assert!(aabb.contains_point(Point3::new(1.0, 1.0, 1.0)));
        assert!(aabb.contains_point(Point3::new(0.0, 0.0, 0.0))); // On boundary
        assert!(!aabb.contains_point(Point3::new(3.0, 1.0, 1.0)));
    }
}
