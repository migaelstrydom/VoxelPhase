//! Small geometry the placer's parts share: facings and yaws, distances
//! over the ground, blends of points and of unit vectors.

use nalgebra::{Point3, Vector3};

#[inline]
pub(crate) fn planar_distance(a: Point3<f32>, b: Point3<f32>) -> f32 {
    let dx = a.x - b.x;
    let dz = a.z - b.z;
    (dx * dx + dz * dz).sqrt()
}

#[inline]
pub(super) fn lerp_point(a: Point3<f32>, b: Point3<f32>, t: f32) -> Point3<f32> {
    a + (b - a) * t.clamp(0.0, 1.0)
}

#[inline]
pub(crate) fn facing_from_yaw(yaw: f32) -> Vector3<f32> {
    Vector3::new(yaw.sin(), 0.0, yaw.cos())
}

#[inline]
pub(super) fn yaw_from_facing(facing: Vector3<f32>) -> f32 {
    facing.x.atan2(facing.z)
}

#[inline]
pub(super) fn normalise_facing(facing: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(facing.x, 0.0, facing.z)
        .try_normalize(1e-4)
        .unwrap_or_else(|| Vector3::new(0.0, 0.0, 1.0))
}

/// Linearly interpolate two unit vectors and renormalise. Good enough for
/// the small angles between world-Y and typical ground normals; avoids
/// pulling in a full quaternion slerp for what is visually a tilt.
#[inline]
pub(super) fn lerp_unit(a: Vector3<f32>, b: Vector3<f32>, t: f32) -> Vector3<f32> {
    let v = a * (1.0 - t) + b * t;
    v.try_normalize(1e-4).unwrap_or(Vector3::y())
}

/// Shortest signed angle from `a` to `b`, in `(-PI, PI]`.
#[inline]
pub(crate) fn angle_diff(b: f32, a: f32) -> f32 {
    let two_pi = std::f32::consts::TAU;
    let mut d = (b - a) % two_pi;
    if d > std::f32::consts::PI {
        d -= two_pi;
    } else if d < -std::f32::consts::PI {
        d += two_pi;
    }
    d
}
