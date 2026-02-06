//! Sphere-sphere collision detection.

use nalgebra::{Point3, Vector3};

/// Contact information from a sphere-sphere collision.
#[derive(Debug, Clone)]
pub struct SphereSphereContact {
    /// Contact point (midpoint between sphere surfaces).
    pub point: Point3<f32>,
    /// Contact normal pointing from sphere A to sphere B.
    pub normal: Vector3<f32>,
    /// Penetration depth (positive means overlapping).
    pub depth: f32,
}

/// Test for collision between two spheres.
///
/// Returns contact information if the spheres overlap.
pub fn sphere_sphere_collision(
    center_a: Point3<f32>,
    radius_a: f32,
    center_b: Point3<f32>,
    radius_b: f32,
) -> Option<SphereSphereContact> {
    let delta = center_b - center_a;
    let dist_sq = delta.magnitude_squared();
    let combined_radius = radius_a + radius_b;

    if dist_sq >= combined_radius * combined_radius {
        return None;
    }

    let dist = dist_sq.sqrt();

    // Handle coincident spheres
    let normal = if dist < 1e-6 {
        Vector3::y() // Arbitrary direction
    } else {
        delta / dist
    };

    let depth = combined_radius - dist;

    // Contact point is on the surface between the two spheres
    let point = center_a + normal * (radius_a - depth * 0.5);

    Some(SphereSphereContact {
        point,
        normal,
        depth,
    })
}

/// Swept sphere-sphere collision detection (CCD).
///
/// Tests if two moving spheres collide during a timestep.
/// Returns the time of first contact in [0, 1] if they collide.
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

    // Relative motion: treat A as moving, B as stationary
    // rel_start and rel_motion are vectors (Point3 - Point3 = Vector3)
    let rel_start = start_a - start_b;
    let rel_end = end_a - end_b;
    let rel_motion = rel_end - rel_start;

    let initial_dist_sq = rel_start.magnitude_squared();

    // Already overlapping at start?
    if initial_dist_sq < combined_radius_sq {
        return Some(0.0);
    }

    // Solve quadratic: |rel_start + t * rel_motion|² = combined_radius²
    let a = rel_motion.magnitude_squared();
    let b = 2.0 * rel_start.dot(&rel_motion);
    let c = initial_dist_sq - combined_radius_sq;

    // Not moving relative to each other
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
