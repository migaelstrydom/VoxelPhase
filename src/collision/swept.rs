//! Swept collision detection for continuous collision detection (CCD).
//!
//! This module implements swept sphere vs triangle intersection tests,
//! which find the exact time of first contact as an object moves through space.
//! This prevents fast-moving objects from tunneling through geometry.

use nalgebra::{Point3, Vector3};

use super::sphere_triangle::Triangle;
use super::ContactPoint;

/// Result of a swept collision test.
#[derive(Debug, Clone)]
pub struct SweptContact {
    /// Time of contact in range [0, 1] where 0 = start position, 1 = end position.
    pub t: f32,
    /// Contact point on the surface.
    pub point: Point3<f32>,
    /// Surface normal at contact.
    pub normal: Vector3<f32>,
}

impl SweptContact {
    pub fn new(t: f32, point: Point3<f32>, normal: Vector3<f32>) -> Self {
        Self { t, point, normal }
    }

    /// Convert to a ContactPoint for collision response.
    pub fn to_contact_point(&self, penetration: f32) -> ContactPoint {
        ContactPoint::new(self.point, self.normal, penetration)
    }
}

/// Swept sphere vs triangle collision test.
///
/// Tests if a sphere moving from `start` to `end` with given `radius`
/// intersects the triangle. Returns the earliest contact if any.
///
/// The sphere's center moves linearly: P(t) = start + t * (end - start)
/// We find the smallest t in [0, 1] where the sphere touches the triangle.
pub fn swept_sphere_triangle(
    start: Point3<f32>,
    end: Point3<f32>,
    radius: f32,
    triangle: &Triangle,
) -> Option<SweptContact> {
    let velocity = end - start;
    let speed_sq = velocity.magnitude_squared();

    // If not moving, fall back to point check
    if speed_sq < 1e-10 {
        return None;
    }

    let mut earliest_t = f32::MAX;
    let mut earliest_contact: Option<SweptContact> = None;

    // Check collision with triangle plane (interior)
    if let Some(contact) = swept_sphere_plane(start, velocity, radius, triangle) {
        if contact.t >= 0.0 && contact.t <= 1.0 && contact.t < earliest_t {
            // Verify the contact point is inside the triangle
            if point_in_triangle(contact.point, triangle) {
                earliest_t = contact.t;
                earliest_contact = Some(contact);
            }
        }
    }

    // Check collision with triangle edges
    for i in 0..3 {
        let (p0, p1) = triangle.edge(i);
        if let Some(contact) = swept_sphere_edge(start, velocity, radius, p0, p1) {
            if contact.t >= 0.0 && contact.t <= 1.0 && contact.t < earliest_t {
                earliest_t = contact.t;
                earliest_contact = Some(contact);
            }
        }
    }

    // Check collision with triangle vertices
    for i in 0..3 {
        let vertex = triangle.vertex(i);
        if let Some(contact) = swept_sphere_point(start, velocity, radius, vertex) {
            if contact.t >= 0.0 && contact.t <= 1.0 && contact.t < earliest_t {
                earliest_t = contact.t;
                earliest_contact = Some(contact);
            }
        }
    }

    earliest_contact
}

/// Swept sphere vs infinite plane.
/// Returns contact if the sphere crosses the plane.
fn swept_sphere_plane(
    start: Point3<f32>,
    velocity: Vector3<f32>,
    radius: f32,
    triangle: &Triangle,
) -> Option<SweptContact> {
    let normal = triangle.normal();
    let plane_d = normal.dot(&triangle.v0.coords);

    // Signed distance from sphere center to plane
    let dist_start = normal.dot(&start.coords) - plane_d;

    // Rate of change of distance
    let vel_dot_n = normal.dot(&velocity);

    // Moving parallel to plane or away from it
    if vel_dot_n.abs() < 1e-10 {
        return None;
    }

    // Time when sphere surface touches plane
    // dist_start + t * vel_dot_n = radius (approaching from positive side)
    // dist_start + t * vel_dot_n = -radius (approaching from negative side)
    let t = if dist_start > 0.0 {
        // Approaching from front
        (dist_start - radius) / -vel_dot_n
    } else {
        // Approaching from back
        (dist_start + radius) / -vel_dot_n
    };

    if t < 0.0 || t > 1.0 {
        return None;
    }

    // Contact point on the plane
    let sphere_center_at_t = start + velocity * t;
    let contact_point = sphere_center_at_t - normal * radius * dist_start.signum();

    // Normal points away from the side we're approaching from
    let contact_normal = if dist_start > 0.0 { normal } else { -normal };

    Some(SweptContact::new(t, contact_point, contact_normal))
}

/// Swept sphere vs line segment (edge).
/// Solves the quadratic equation for sphere-cylinder intersection.
fn swept_sphere_edge(
    start: Point3<f32>,
    velocity: Vector3<f32>,
    radius: f32,
    edge_start: Point3<f32>,
    edge_end: Point3<f32>,
) -> Option<SweptContact> {
    let edge = edge_end - edge_start;
    let edge_len_sq = edge.magnitude_squared();

    if edge_len_sq < 1e-10 {
        // Degenerate edge, treat as point
        return swept_sphere_point(start, velocity, radius, edge_start);
    }

    let edge_dir = edge / edge_len_sq.sqrt();

    // Project velocity and start-to-edge onto perpendicular plane
    let to_edge = start - edge_start;

    // Remove edge-parallel component
    let vel_perp = velocity - edge_dir * velocity.dot(&edge_dir);
    let to_edge_perp = to_edge - edge_dir * to_edge.dot(&edge_dir);

    // Solve: |to_edge_perp + t * vel_perp|^2 = radius^2
    let a = vel_perp.magnitude_squared();
    let b = 2.0 * to_edge_perp.dot(&vel_perp);
    let c = to_edge_perp.magnitude_squared() - radius * radius;

    let t = solve_quadratic_smallest_positive(a, b, c)?;

    if t > 1.0 {
        return None;
    }

    // Check if contact point is within edge bounds
    let sphere_center_at_t = start + velocity * t;
    let to_contact = sphere_center_at_t - edge_start;
    let edge_param = to_contact.dot(&edge_dir) / edge_len_sq.sqrt();

    if edge_param < 0.0 || edge_param > 1.0 {
        return None;
    }

    // Contact point on the edge
    let contact_point = edge_start + edge_dir * edge_param * edge_len_sq.sqrt();
    let normal = (sphere_center_at_t - contact_point).normalize();

    Some(SweptContact::new(t, contact_point, normal))
}

/// Swept sphere vs point (vertex).
/// Solves the quadratic equation for sphere-point intersection.
fn swept_sphere_point(
    start: Point3<f32>,
    velocity: Vector3<f32>,
    radius: f32,
    point: Point3<f32>,
) -> Option<SweptContact> {
    let to_point = start - point;

    // Solve: |to_point + t * velocity|^2 = radius^2
    let a = velocity.magnitude_squared();
    let b = 2.0 * to_point.dot(&velocity);
    let c = to_point.magnitude_squared() - radius * radius;

    let t = solve_quadratic_smallest_positive(a, b, c)?;

    if t > 1.0 {
        return None;
    }

    let sphere_center_at_t = start + velocity * t;
    let normal = (sphere_center_at_t - point).normalize();

    Some(SweptContact::new(t, point, normal))
}

/// Solve quadratic equation ax² + bx + c = 0, return smallest positive root.
fn solve_quadratic_smallest_positive(a: f32, b: f32, c: f32) -> Option<f32> {
    if a.abs() < 1e-10 {
        // Linear equation: bx + c = 0
        if b.abs() < 1e-10 {
            return None;
        }
        let t = -c / b;
        return if t >= 0.0 { Some(t) } else { None };
    }

    let discriminant = b * b - 4.0 * a * c;

    if discriminant < 0.0 {
        return None;
    }

    let sqrt_d = discriminant.sqrt();
    let t1 = (-b - sqrt_d) / (2.0 * a);
    let t2 = (-b + sqrt_d) / (2.0 * a);

    // Return smallest non-negative root
    if t1 >= 0.0 {
        Some(t1)
    } else if t2 >= 0.0 {
        Some(t2)
    } else {
        None
    }
}

/// Check if a point lies inside a triangle (on the triangle's plane).
fn point_in_triangle(p: Point3<f32>, triangle: &Triangle) -> bool {
    let v0 = triangle.v0;
    let v1 = triangle.v1;
    let v2 = triangle.v2;

    // Compute barycentric coordinates
    let v0v1 = v1 - v0;
    let v0v2 = v2 - v0;
    let v0p = p - v0;

    let dot00 = v0v1.dot(&v0v1);
    let dot01 = v0v1.dot(&v0v2);
    let dot02 = v0v1.dot(&v0p);
    let dot11 = v0v2.dot(&v0v2);
    let dot12 = v0v2.dot(&v0p);

    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // Check if point is in triangle
    u >= 0.0 && v >= 0.0 && (u + v) <= 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_swept_sphere_hits_plane() {
        let triangle = Triangle::new(
            Point3::new(-10.0, 0.0, -10.0),
            Point3::new(10.0, 0.0, -10.0),
            Point3::new(0.0, 0.0, 10.0),
        );

        // Sphere falling straight down onto triangle
        let start = Point3::new(0.0, 5.0, 0.0);
        let end = Point3::new(0.0, -5.0, 0.0);
        let radius = 0.5;

        let contact = swept_sphere_triangle(start, end, radius, &triangle);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Should hit at t ≈ 0.45 (when sphere at y=0.5, surface at y=0)
        assert!(c.t > 0.4 && c.t < 0.5);
        assert!(c.normal.y > 0.9); // Normal pointing up
    }

    #[test]
    fn test_swept_sphere_misses() {
        let triangle = Triangle::new(
            Point3::new(-1.0, 0.0, -1.0),
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 1.0),
        );

        // Sphere moving parallel, missing the triangle
        let start = Point3::new(5.0, 5.0, 0.0);
        let end = Point3::new(5.0, -5.0, 0.0);
        let radius = 0.5;

        let contact = swept_sphere_triangle(start, end, radius, &triangle);
        assert!(contact.is_none());
    }

    #[test]
    fn test_swept_sphere_hits_edge() {
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 2.0),
        );

        // Sphere moving to hit edge from the side
        let start = Point3::new(-2.0, 0.5, 1.0);
        let end = Point3::new(2.0, 0.5, 1.0);
        let radius = 0.5;

        let contact = swept_sphere_triangle(start, end, radius, &triangle);
        // Should detect collision with the edge
        assert!(contact.is_some());
    }

    #[test]
    fn test_swept_sphere_hits_vertex() {
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 2.0),
        );

        // Sphere moving to hit vertex
        let start = Point3::new(-2.0, 0.0, 0.0);
        let end = Point3::new(2.0, 0.0, 0.0);
        let radius = 0.5;

        let contact = swept_sphere_triangle(start, end, radius, &triangle);
        assert!(contact.is_some());
    }

    #[test]
    fn test_fast_projectile() {
        let triangle = Triangle::new(
            Point3::new(-10.0, 0.0, -10.0),
            Point3::new(10.0, 0.0, -10.0),
            Point3::new(0.0, 0.0, 10.0),
        );

        // Very fast projectile (100 units travel)
        let start = Point3::new(0.0, 50.0, 0.0);
        let end = Point3::new(0.0, -50.0, 0.0);
        let radius = 0.1;

        let contact = swept_sphere_triangle(start, end, radius, &triangle);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Should still detect the hit
        assert!(c.t > 0.0 && c.t < 1.0);
    }
}
