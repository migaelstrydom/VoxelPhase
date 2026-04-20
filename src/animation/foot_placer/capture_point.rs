//! Capture-point math for procedural foot placement.
//!
//! For a linear inverted pendulum of effective height `h` with
//! horizontal CoM velocity `v`, the capture point is the xz location a
//! foot must be planted at to bring the body to rest:
//!
//! ```text
//! capture_point = CoM_xz + v_xz * sqrt(h / g)
//! ```
//!
//! Pratt et al. 2006, Koolen et al. 2012.

use nalgebra::{Point3, Vector2, Vector3};

/// Standard gravitational acceleration, matching the physics engine's
/// default world gravity magnitude (`PhysicsConfig::gravity.y` = -9.81).
pub const GRAVITY: f32 = 9.81;

/// Capture point in world xz, given a pelvis position, horizontal
/// velocity, and effective pendulum height. Y of `pelvis` is ignored —
/// the returned point's y is set to `pelvis.y` so callers have a
/// complete Point3 to work with.
#[inline]
pub fn capture_point_xz(
    pelvis: Point3<f32>,
    horizontal_velocity: Vector3<f32>,
    effective_height: f32,
) -> Point3<f32> {
    let h = effective_height.max(0.01);
    let k = (h / GRAVITY).sqrt();
    Point3::new(
        pelvis.x + horizontal_velocity.x * k,
        pelvis.y,
        pelvis.z + horizontal_velocity.z * k,
    )
}

/// Lateral pull applied to each foot proportional to yaw rate. The foot
/// on the outside of the turn is driven further out, mirroring how a
/// human pivots.
#[inline]
pub fn turn_in_place_offset(
    facing: Vector3<f32>,
    yaw_rate: f32,
    hip_width: f32,
    k_yaw: f32,
    foot_side_sign: f32,
) -> Vector2<f32> {
    let right = facing.cross(&Vector3::y());
    let magnitude = yaw_rate * hip_width * k_yaw * foot_side_sign;
    Vector2::new(right.x * magnitude, right.z * magnitude)
}

/// Neutral stance offset: each foot sits `hip_width` to its own side of
/// the CoM. `foot_side_sign` is +1 for right, -1 for left.
#[inline]
pub fn stance_offset(facing: Vector3<f32>, hip_width: f32, foot_side_sign: f32) -> Vector2<f32> {
    let right = facing.cross(&Vector3::y());
    Vector2::new(
        right.x * hip_width * foot_side_sign,
        right.z * hip_width * foot_side_sign,
    )
}
