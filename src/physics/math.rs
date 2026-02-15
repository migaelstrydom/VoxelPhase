//! Math utilities for physics calculations.

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

/// Compute the inertia tensor for a solid sphere.
///
/// Formula: I = (2/5) * m * r² for all axes
pub fn sphere_inertia_tensor(mass: f32, radius: f32) -> Matrix3<f32> {
    let i = (2.0 / 5.0) * mass * radius * radius;
    Matrix3::from_diagonal(&Vector3::new(i, i, i))
}

/// Compute the inertia tensor for a solid box.
///
/// Formula:
/// - I_xx = (1/12) * m * (h² + d²)
/// - I_yy = (1/12) * m * (w² + d²)
/// - I_zz = (1/12) * m * (w² + h²)
///
/// Where w, h, d are the full widths (2 * half_extents).
pub fn box_inertia_tensor(mass: f32, half_extents: Vector3<f32>) -> Matrix3<f32> {
    let w = 2.0 * half_extents.x;
    let h = 2.0 * half_extents.y;
    let d = 2.0 * half_extents.z;
    let factor = mass / 12.0;

    let i_xx = factor * (h * h + d * d);
    let i_yy = factor * (w * w + d * d);
    let i_zz = factor * (w * w + h * h);

    Matrix3::from_diagonal(&Vector3::new(i_xx, i_yy, i_zz))
}

/// Transform a world-space inertia tensor to local frame.
/// I_world = R * I_local * R^T
pub fn transform_inertia_tensor(
    local_inertia: &Matrix3<f32>,
    rotation: &UnitQuaternion<f32>,
) -> Matrix3<f32> {
    let r = rotation.to_rotation_matrix();
    r.matrix() * local_inertia * r.matrix().transpose()
}

/// Integrate a quaternion by angular velocity.
///
/// q' = q + (dt/2) * Quaternion(0, ω) * q
/// q' = normalize(q')
pub fn integrate_orientation(
    orientation: UnitQuaternion<f32>,
    angular_velocity: Vector3<f32>,
    dt: f32,
) -> UnitQuaternion<f32> {
    // Create quaternion from angular velocity (pure quaternion)
    let omega_quat = nalgebra::Quaternion::new(
        0.0,
        angular_velocity.x,
        angular_velocity.y,
        angular_velocity.z,
    );

    // q' = q + (dt/2) * omega_quat * q
    let delta = omega_quat * orientation.into_inner() * (dt * 0.5);
    let new_quat = orientation.into_inner() + delta;

    // Normalize to maintain unit quaternion
    UnitQuaternion::new_normalize(new_quat)
}
