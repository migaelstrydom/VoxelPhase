//! Sphere-triangle collision detection.

use nalgebra::{Point3, Vector3};

use super::triangle::Triangle;
use super::triangle_closest::closest_on_triangle;

/// Contact from a sphere-triangle overlap test.
#[derive(Debug, Clone, Copy)]
pub struct SphereTriangleContact {
    /// Contact normal pointing from the triangle toward the sphere center.
    pub normal: Vector3<f32>,
}

/// Test for collision between a sphere and a triangle.
///
/// Returns contact information if the sphere overlaps the triangle.
/// The contact normal points from the triangle toward the sphere center.
pub fn sphere_triangle_collision(
    sphere_center: Point3<f32>,
    sphere_radius: f32,
    triangle: &Triangle,
) -> Option<SphereTriangleContact> {
    let closest = closest_point_on_triangle(sphere_center, triangle);

    let to_center = sphere_center - closest;
    let dist_sq = to_center.magnitude_squared();

    if dist_sq > sphere_radius * sphere_radius {
        return None;
    }

    let dist = dist_sq.sqrt();

    let normal = if dist > 1e-6 {
        to_center / dist
    } else {
        triangle.normal()
    };

    Some(SphereTriangleContact { normal })
}

/// The point of `tri` closest to `point`.
pub fn closest_point_on_triangle(point: Point3<f32>, tri: &Triangle) -> Point3<f32> {
    let corners = [tri.v0, tri.v1, tri.v2];
    closest_on_triangle(point, corners).point(corners)
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

        let center = Point3::new(1.0, 0.5, 0.5);
        let radius = 1.0;

        let contact = sphere_triangle_collision(center, radius, &tri);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.normal.y > 0.9);
    }

    #[test]
    fn test_sphere_no_collision() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 2.0),
        );

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

        let point = Point3::new(-1.0, 0.0, -1.0);
        let closest = closest_point_on_triangle(point, &tri);

        assert!(approx_eq(closest.x, 0.0));
        assert!(approx_eq(closest.y, 0.0));
        assert!(approx_eq(closest.z, 0.0));
    }

    #[test]
    fn test_closest_point_face() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
        );
        let center = Point3::new(0.5, 0.2, 0.5);
        let contact = sphere_triangle_collision(center, 1.0, &tri);
        assert!(contact.is_some());
        assert!(contact.unwrap().normal.y > 0.9);
    }

    #[test]
    fn test_closest_point_edge() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
        );
        let center = Point3::new(1.0, 0.2, -0.1);
        let contact = sphere_triangle_collision(center, 1.0, &tri);
        assert!(contact.is_some());
    }
}
