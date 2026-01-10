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

    /// Create an AABB that contains a single point.
    pub fn from_point(point: Point3<f32>) -> Self {
        Self {
            min: point,
            max: point,
        }
    }

    /// Create an empty (inverted) AABB suitable for expansion.
    pub fn empty() -> Self {
        Self {
            min: Point3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            max: Point3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
        }
    }

    /// Check if this AABB is valid (min <= max on all axes).
    pub fn is_valid(&self) -> bool {
        self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z
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

    /// Expand this AABB to include a point.
    pub fn expand_to_include_point(&mut self, point: Point3<f32>) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.min.z = self.min.z.min(point.z);
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
        self.max.z = self.max.z.max(point.z);
    }

    /// Expand this AABB to include another AABB.
    pub fn expand_to_include(&mut self, other: &AABB) {
        self.min.x = self.min.x.min(other.min.x);
        self.min.y = self.min.y.min(other.min.y);
        self.min.z = self.min.z.min(other.min.z);
        self.max.x = self.max.x.max(other.max.x);
        self.max.y = self.max.y.max(other.max.y);
        self.max.z = self.max.z.max(other.max.z);
    }

    /// Return a new AABB that is the union of this and another.
    pub fn union(&self, other: &AABB) -> AABB {
        AABB {
            min: Point3::new(
                self.min.x.min(other.min.x),
                self.min.y.min(other.min.y),
                self.min.z.min(other.min.z),
            ),
            max: Point3::new(
                self.max.x.max(other.max.x),
                self.max.y.max(other.max.y),
                self.max.z.max(other.max.z),
            ),
        }
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

    /// Check if this AABB fully contains another AABB.
    pub fn contains(&self, other: &AABB) -> bool {
        self.min.x <= other.min.x
            && self.max.x >= other.max.x
            && self.min.y <= other.min.y
            && self.max.y >= other.max.y
            && self.min.z <= other.min.z
            && self.max.z >= other.max.z
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

    /// Expand the AABB by a margin on all sides.
    pub fn expanded(&self, margin: f32) -> AABB {
        AABB {
            min: Point3::new(
                self.min.x - margin,
                self.min.y - margin,
                self.min.z - margin,
            ),
            max: Point3::new(
                self.max.x + margin,
                self.max.y + margin,
                self.max.z + margin,
            ),
        }
    }

    /// Get the surface area of the AABB (useful for BVH heuristics).
    pub fn surface_area(&self) -> f32 {
        let size = self.size();
        2.0 * (size.x * size.y + size.y * size.z + size.z * size.x)
    }

    /// Get the volume of the AABB.
    pub fn volume(&self) -> f32 {
        let size = self.size();
        size.x * size.y * size.z
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
