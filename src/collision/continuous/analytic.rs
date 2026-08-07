//! Analytic continuous collision detection (CCD).
//!
//! Closed-form swept tests for common shape pairs:
//! - `swept_sphere_sphere`: quadratic for two moving spheres
//! - `swept_sphere_triangle`: plane + edge + vertex tests for a moving sphere vs triangle
//! - `swept_sphere_plane`: sphere vs infinite plane (used internally and as standalone fast-path)

use nalgebra::{Point3, Vector3};

use crate::collision::sphere_triangle;
use crate::collision::triangle::Triangle;

/// Result of a swept collision test.
#[derive(Debug, Clone)]
pub struct SweptContact {
    /// Time of contact in [0, 1] where 0 = start position, 1 = end position.
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
}

/// Swept sphere-sphere collision detection.
///
/// Tests if two moving spheres collide during a timestep.
/// Returns the time of first contact in [0, 1] if they collide.
///
/// Uses relative motion: treats A as moving, B as stationary, then solves
/// the quadratic `|rel_start + t * rel_motion|² = combined_radius²`.
pub fn swept_sphere_sphere(
    start_a: Point3<f32>,
    end_a: Point3<f32>,
    radius_a: f32,
    start_b: Point3<f32>,
    end_b: Point3<f32>,
    radius_b: f32,
) -> Option<f32> {
    let combined_radius = radius_a + radius_b;
    let combined_radius_sq = combined_radius * combined_radius;

    let rel_start = start_a - start_b;
    let rel_end = end_a - end_b;
    let rel_motion = rel_end - rel_start;

    let initial_dist_sq = rel_start.magnitude_squared();

    if initial_dist_sq < combined_radius_sq {
        return Some(0.0);
    }

    let a = rel_motion.magnitude_squared();
    let b = 2.0 * rel_start.dot(&rel_motion);
    let c = initial_dist_sq - combined_radius_sq;

    if a < 1e-10 {
        return None;
    }

    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }

    let sqrt_disc = discriminant.sqrt();
    let t = (-b - sqrt_disc) / (2.0 * a);

    if t >= 0.0 && t <= 1.0 {
        Some(t)
    } else {
        None
    }
}

/// Swept sphere vs triangle collision test.
///
/// Tests if a sphere moving from `start` to `end` with given `radius`
/// intersects the triangle. Returns the earliest contact if any.
///
/// The sphere's center moves linearly: `P(t) = start + t * (end - start)`.
/// Tests against the triangle plane interior, all 3 edges, and all 3 vertices,
/// returning the earliest hit.
pub fn swept_sphere_triangle(
    start: Point3<f32>,
    end: Point3<f32>,
    radius: f32,
    triangle: &Triangle,
) -> Option<SweptContact> {
    let velocity = end - start;
    let speed_sq = velocity.magnitude_squared();

    if let Some(contact) = sphere_triangle::sphere_triangle_collision(start, radius, triangle) {
        let point = start - contact.normal * radius;
        return Some(SweptContact::new(0.0, point, contact.normal));
    }

    if speed_sq < 1e-10 {
        return None;
    }

    let mut earliest_t = f32::MAX;
    let mut earliest_contact: Option<SweptContact> = None;

    if let Some(contact) = swept_sphere_plane(start, velocity, radius, triangle) {
        if contact.t >= 0.0 && contact.t <= 1.0 && contact.t < earliest_t {
            if point_in_triangle(contact.point, triangle) {
                earliest_t = contact.t;
                earliest_contact = Some(contact);
            }
        }
    }

    for i in 0..3 {
        let (p0, p1) = triangle.edge(i);
        if let Some(contact) = swept_sphere_edge(start, velocity, radius, p0, p1) {
            if contact.t >= 0.0 && contact.t <= 1.0 && contact.t < earliest_t {
                earliest_t = contact.t;
                earliest_contact = Some(contact);
            }
        }
    }

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
fn swept_sphere_plane(
    start: Point3<f32>,
    velocity: Vector3<f32>,
    radius: f32,
    triangle: &Triangle,
) -> Option<SweptContact> {
    let normal = triangle.normal();
    let plane_d = normal.dot(&triangle.v0.coords);

    let dist_start = normal.dot(&start.coords) - plane_d;
    let vel_dot_n = normal.dot(&velocity);

    if vel_dot_n.abs() < 1e-10 {
        return None;
    }

    let t = if dist_start > 0.0 {
        (dist_start - radius) / -vel_dot_n
    } else {
        (dist_start + radius) / -vel_dot_n
    };

    if t < 0.0 || t > 1.0 {
        return None;
    }

    let sphere_center_at_t = start + velocity * t;
    let contact_point = sphere_center_at_t - normal * radius * dist_start.signum();
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
        return swept_sphere_point(start, velocity, radius, edge_start);
    }

    let edge_dir = edge / edge_len_sq.sqrt();

    let to_edge = start - edge_start;

    let vel_perp = velocity - edge_dir * velocity.dot(&edge_dir);
    let to_edge_perp = to_edge - edge_dir * to_edge.dot(&edge_dir);

    let a = vel_perp.magnitude_squared();
    let b = 2.0 * to_edge_perp.dot(&vel_perp);
    let c = to_edge_perp.magnitude_squared() - radius * radius;

    let t = solve_quadratic_smallest_positive(a, b, c)?;

    if t > 1.0 {
        return None;
    }

    let sphere_center_at_t = start + velocity * t;
    let to_contact = sphere_center_at_t - edge_start;
    let edge_param = to_contact.dot(&edge_dir) / edge_len_sq.sqrt();

    if edge_param < 0.0 || edge_param > 1.0 {
        return None;
    }

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

    u >= 0.0 && v >= 0.0 && (u + v) <= 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_swept_sphere_sphere_hit() {
        let t = swept_sphere_sphere(
            Point3::new(-5.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            1.0,
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(-5.0, 0.0, 0.0),
            1.0,
        );
        assert!(t.is_some());
        let t = t.unwrap();
        assert!(t > 0.0 && t < 1.0);
    }

    #[test]
    fn test_swept_sphere_sphere_miss() {
        let t = swept_sphere_sphere(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            0.5,
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(10.0, 5.0, 0.0),
            0.5,
        );
        assert!(t.is_none());
    }

    #[test]
    fn test_swept_sphere_sphere_already_overlapping() {
        let t = swept_sphere_sphere(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            1.0,
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.5, 0.0, 0.0),
            1.0,
        );
        assert_eq!(t, Some(0.0));
    }

    #[test]
    fn test_swept_sphere_sphere_zero_relative_velocity() {
        let t = swept_sphere_sphere(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            0.5,
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(5.0, 3.0, 0.0),
            0.5,
        );
        assert!(t.is_none());
    }

    #[test]
    fn test_swept_sphere_hits_plane() {
        let triangle = Triangle::new(
            Point3::new(-10.0, 0.0, -10.0),
            Point3::new(10.0, 0.0, -10.0),
            Point3::new(0.0, 0.0, 10.0),
        );

        let start = Point3::new(0.0, 5.0, 0.0);
        let end = Point3::new(0.0, -5.0, 0.0);
        let radius = 0.5;

        let contact = swept_sphere_triangle(start, end, radius, &triangle);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.t > 0.4 && c.t < 0.5);
        assert!(c.normal.y > 0.9);
    }

    #[test]
    fn test_swept_sphere_misses() {
        let triangle = Triangle::new(
            Point3::new(-1.0, 0.0, -1.0),
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 1.0),
        );

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

        let start = Point3::new(-2.0, 0.5, 1.0);
        let end = Point3::new(2.0, 0.5, 1.0);
        let radius = 0.5;

        let contact = swept_sphere_triangle(start, end, radius, &triangle);
        assert!(contact.is_some());
    }

    #[test]
    fn test_swept_sphere_hits_vertex() {
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 2.0),
        );

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

        let start = Point3::new(0.0, 50.0, 0.0);
        let end = Point3::new(0.0, -50.0, 0.0);
        let radius = 0.1;

        let contact = swept_sphere_triangle(start, end, radius, &triangle);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.t > 0.0 && c.t < 1.0);
    }
}
