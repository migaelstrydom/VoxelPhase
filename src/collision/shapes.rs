//! Collision shapes and the CollisionShape trait.

use nalgebra::{Point3, Vector3};

use super::AABB;

/// Trait for collision shapes. Designed to be shape-agnostic so we can
/// swap sphere → capsule → convex hull as needed.
pub trait CollisionShape: Send + Sync {
    /// Get the axis-aligned bounding box in world space.
    fn world_aabb(&self, position: Point3<f32>) -> AABB;

    /// Support function: returns the furthest point on the shape in the given direction.
    /// Used by GJK algorithm for general convex collision detection.
    fn support(&self, position: Point3<f32>, direction: Vector3<f32>) -> Point3<f32>;

    /// Quick test: does this shape intersect an AABB?
    fn intersects_aabb(&self, position: Point3<f32>, aabb: &AABB) -> bool;
}

/// A sphere collision shape.
#[derive(Debug, Clone, Copy)]
pub struct Sphere {
    pub radius: f32,
}

impl Sphere {
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }
}

impl CollisionShape for Sphere {
    fn world_aabb(&self, position: Point3<f32>) -> AABB {
        AABB::from_center_half_extents(
            position,
            Vector3::new(self.radius, self.radius, self.radius),
        )
    }

    fn support(&self, position: Point3<f32>, direction: Vector3<f32>) -> Point3<f32> {
        if direction.magnitude_squared() > 1e-6 {
            position + direction.normalize() * self.radius
        } else {
            position
        }
    }

    fn intersects_aabb(&self, position: Point3<f32>, aabb: &AABB) -> bool {
        aabb.intersects_sphere(position, self.radius)
    }
}

/// A capsule collision shape (for future humanoid character).
/// A capsule is a cylinder with hemispherical caps.
#[derive(Debug, Clone, Copy)]
pub struct Capsule {
    /// Radius of the capsule (cylinder and hemispheres).
    pub radius: f32,
    /// Half-height of the cylindrical portion (total height = 2*half_height + 2*radius).
    pub half_height: f32,
}

impl Capsule {
    pub fn new(radius: f32, half_height: f32) -> Self {
        Self {
            radius,
            half_height,
        }
    }

    /// Total height of the capsule.
    pub fn total_height(&self) -> f32 {
        2.0 * (self.half_height + self.radius)
    }

    /// Get the top hemisphere center relative to capsule center.
    pub fn top_center(&self, position: Point3<f32>) -> Point3<f32> {
        Point3::new(position.x, position.y + self.half_height, position.z)
    }

    /// Get the bottom hemisphere center relative to capsule center.
    pub fn bottom_center(&self, position: Point3<f32>) -> Point3<f32> {
        Point3::new(position.x, position.y - self.half_height, position.z)
    }
}

impl CollisionShape for Capsule {
    fn world_aabb(&self, position: Point3<f32>) -> AABB {
        let half_extents = Vector3::new(self.radius, self.half_height + self.radius, self.radius);
        AABB::from_center_half_extents(position, half_extents)
    }

    fn support(&self, position: Point3<f32>, direction: Vector3<f32>) -> Point3<f32> {
        if direction.magnitude_squared() < 1e-6 {
            return position;
        }

        // The support point is on one of the hemisphere caps
        let normalized = direction.normalize();
        let center = if normalized.y >= 0.0 {
            self.top_center(position)
        } else {
            self.bottom_center(position)
        };
        center + normalized * self.radius
    }

    fn intersects_aabb(&self, position: Point3<f32>, aabb: &AABB) -> bool {
        // For capsule-AABB, we need to check if the line segment between
        // the two hemisphere centers is within radius of the AABB.
        // This is a simplification that expands the AABB by the radius.
        let expanded = aabb.expanded(self.radius);

        let top = self.top_center(position);
        let bottom = self.bottom_center(position);

        // Check if either endpoint is inside, or if the segment crosses
        if expanded.contains_point(top) || expanded.contains_point(bottom) {
            return true;
        }

        // Check if the line segment from bottom to top intersects the expanded AABB
        // This is a simplified check - a full implementation would do segment-AABB intersection
        expanded.contains_point(position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sphere_aabb() {
        let sphere = Sphere::new(1.0);
        let pos = Point3::new(5.0, 5.0, 5.0);
        let aabb = sphere.world_aabb(pos);

        assert_eq!(aabb.min, Point3::new(4.0, 4.0, 4.0));
        assert_eq!(aabb.max, Point3::new(6.0, 6.0, 6.0));
    }

    #[test]
    fn test_sphere_support() {
        let sphere = Sphere::new(2.0);
        let pos = Point3::origin();

        let support = sphere.support(pos, Vector3::new(1.0, 0.0, 0.0));
        assert!((support.x - 2.0).abs() < 1e-6);
        assert!(support.y.abs() < 1e-6);
        assert!(support.z.abs() < 1e-6);
    }

    #[test]
    fn test_capsule_height() {
        let capsule = Capsule::new(0.5, 1.0);
        assert!((capsule.total_height() - 3.0).abs() < 1e-6);
    }
}
