//! Sphere-triangle collision detection.
//!
//! This is the core narrowphase test for collision against terrain meshes.

use nalgebra::{Point3, Vector3};

use super::ContactPoint;
use crate::physics::ContactFeature;

/// A triangle represented by three vertices.
#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    pub v0: Point3<f32>,
    pub v1: Point3<f32>,
    pub v2: Point3<f32>,
}

impl Triangle {
    pub fn new(v0: Point3<f32>, v1: Point3<f32>, v2: Point3<f32>) -> Self {
        Self { v0, v1, v2 }
    }

    /// Compute the face normal (not normalized).
    pub fn normal_unnormalized(&self) -> Vector3<f32> {
        let e1 = self.v1 - self.v0;
        let e2 = self.v2 - self.v0;
        e1.cross(&e2)
    }

    /// Compute the unit face normal.
    pub fn normal(&self) -> Vector3<f32> {
        let n = self.normal_unnormalized();
        let len_sq = n.magnitude_squared();
        if len_sq > 1e-10 {
            n / len_sq.sqrt()
        } else {
            Vector3::new(0.0, 1.0, 0.0) // Degenerate triangle fallback
        }
    }

    /// Get a vertex by index (0, 1, or 2).
    #[inline]
    pub fn vertex(&self, index: usize) -> Point3<f32> {
        match index {
            0 => self.v0,
            1 => self.v1,
            _ => self.v2,
        }
    }

    /// Get an edge by index (0, 1, or 2).
    /// Returns (start, end) of the edge.
    #[inline]
    pub fn edge(&self, index: usize) -> (Point3<f32>, Point3<f32>) {
        match index {
            0 => (self.v0, self.v1),
            1 => (self.v1, self.v2),
            _ => (self.v2, self.v0),
        }
    }
}

/// Test for collision between a sphere and a triangle.
///
/// Returns a contact point if there's a collision, None otherwise.
/// The contact normal points from the triangle toward the sphere center.
pub fn sphere_triangle_collision(
    sphere_center: Point3<f32>,
    sphere_radius: f32,
    triangle: &Triangle,
) -> Option<ContactPoint> {
    sphere_triangle_collision_with_feature(sphere_center, sphere_radius, triangle)
        .map(|(contact, _)| contact)
}

/// Test for collision between a sphere and triangle with feature provenance.
pub fn sphere_triangle_collision_with_feature(
    sphere_center: Point3<f32>,
    sphere_radius: f32,
    triangle: &Triangle,
) -> Option<(ContactPoint, ContactFeature)> {
    // Find the closest point on the triangle to the sphere center
    let (closest, feature) = closest_point_on_triangle_with_feature(sphere_center, triangle);

    // Vector from closest point to sphere center
    let to_center = sphere_center - closest;
    let dist_sq = to_center.magnitude_squared();

    // Check if within radius
    if dist_sq > sphere_radius * sphere_radius {
        return None;
    }

    let dist = dist_sq.sqrt();

    // Determine the contact normal
    let normal = if dist > 1e-6 {
        to_center / dist
    } else {
        // Sphere center is exactly on the triangle, use face normal
        triangle.normal()
    };

    // Penetration depth
    let depth = sphere_radius - dist;

    Some((ContactPoint::new(closest, normal, depth), feature))
}

/// Find the closest point on a triangle to a given point.
/// Uses barycentric coordinates and Voronoi regions.
#[cfg(test)]
fn closest_point_on_triangle(point: Point3<f32>, tri: &Triangle) -> Point3<f32> {
    closest_point_on_triangle_with_feature(point, tri).0
}

/// Find closest point and classify its triangle feature.
fn closest_point_on_triangle_with_feature(
    point: Point3<f32>,
    tri: &Triangle,
) -> (Point3<f32>, ContactFeature) {
    let ab = tri.v1 - tri.v0;
    let ac = tri.v2 - tri.v0;
    let ap = point - tri.v0;

    // Check if P is in vertex region outside A
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (tri.v0, ContactFeature::Vertex(0)); // Closest to vertex A
    }

    // Check if P is in vertex region outside B
    let bp = point - tri.v1;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (tri.v1, ContactFeature::Vertex(1)); // Closest to vertex B
    }

    // Check if P is in edge region of AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (tri.v0 + ab * v, ContactFeature::Edge(0)); // On edge AB
    }

    // Check if P is in vertex region outside C
    let cp = point - tri.v2;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (tri.v2, ContactFeature::Vertex(2)); // Closest to vertex C
    }

    // Check if P is in edge region of AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (tri.v0 + ac * w, ContactFeature::Edge(2)); // On edge AC
    }

    // Check if P is in edge region of BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (tri.v1 + (tri.v2 - tri.v1) * w, ContactFeature::Edge(1)); // On edge BC
    }

    // P is inside the face region
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (tri.v0 + ab * v + ac * w, ContactFeature::Face)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    #[test]
    fn test_sphere_above_triangle() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 2.0),
        );

        // Sphere directly above the triangle center
        let center = Point3::new(1.0, 0.5, 0.5);
        let radius = 1.0;

        let contact = sphere_triangle_collision(center, radius, &tri);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.depth > 0.0);
        assert!(c.normal.y > 0.9); // Should point mostly up
    }

    #[test]
    fn test_sphere_no_collision() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 2.0),
        );

        // Sphere far above the triangle
        let center = Point3::new(1.0, 5.0, 0.5);
        let radius = 1.0;

        let contact = sphere_triangle_collision(center, radius, &tri);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sphere_on_edge() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 2.0),
        );

        // Sphere positioned at edge AB
        let center = Point3::new(1.0, 0.5, -0.5);
        let radius = 1.0;

        let contact = sphere_triangle_collision(center, radius, &tri);
        assert!(contact.is_some());
    }

    #[test]
    fn test_closest_point_vertex() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 2.0),
        );

        // Point closest to vertex A
        let point = Point3::new(-1.0, 0.0, -1.0);
        let closest = closest_point_on_triangle(point, &tri);

        assert!(approx_eq(closest.x, 0.0));
        assert!(approx_eq(closest.y, 0.0));
        assert!(approx_eq(closest.z, 0.0));
    }

    #[test]
    fn test_feature_face() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
        );
        let center = Point3::new(0.5, 0.2, 0.5);
        let (_, feature) = sphere_triangle_collision_with_feature(center, 1.0, &tri).unwrap();
        assert_eq!(feature, ContactFeature::Face);
    }

    #[test]
    fn test_feature_edge() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
        );
        let center = Point3::new(1.0, 0.2, -0.1);
        let (_, feature) = sphere_triangle_collision_with_feature(center, 1.0, &tri).unwrap();
        assert_eq!(feature, ContactFeature::Edge(0));
    }

    #[test]
    fn test_feature_vertex() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
        );
        let center = Point3::new(-0.1, 0.2, -0.1);
        let (_, feature) = sphere_triangle_collision_with_feature(center, 0.5, &tri).unwrap();
        assert_eq!(feature, ContactFeature::Vertex(0));
    }
}
