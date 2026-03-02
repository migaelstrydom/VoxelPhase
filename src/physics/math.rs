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

/// Compute the inertia tensor for a solid capsule (cylinder + hemisphere caps).
///
/// The capsule axis is Y. `half_height` includes the caps, so the cylinder
/// half-length is `half_height - radius`.
///
/// Uses standard formulas with the parallel axis theorem for hemisphere
/// contributions shifted along Y.
pub fn capsule_inertia_tensor(mass: f32, half_height: f32, radius: f32) -> Matrix3<f32> {
    let r = radius;
    let r2 = r * r;
    let cyl_h = 2.0 * (half_height - r); // full cylinder length
    let pi = std::f32::consts::PI;

    // Volumes
    let v_cyl = pi * r2 * cyl_h;
    let v_caps = (4.0 / 3.0) * pi * r2 * r; // two hemispheres = one sphere
    let v_total = v_cyl + v_caps;

    // Mass distribution
    let m_cyl = mass * v_cyl / v_total;
    let m_caps = mass * v_caps / v_total;

    // Cylinder moment about its own center (axis = Y)
    let cyl_iy = 0.5 * m_cyl * r2;
    let cyl_ix = m_cyl * (3.0 * r2 + cyl_h * cyl_h) / 12.0;

    // Sphere moment about its own center
    let sphere_iy = 0.4 * m_caps * r2;
    let sphere_ix = sphere_iy; // sphere is symmetric

    // Parallel axis theorem: each hemisphere center is at ±(cyl_h/2 + 3r/8)
    // from the capsule center. But for a full sphere split into two hemispheres,
    // the combined CoM offset simplifies. Using the standard approach:
    // each hemisphere mass = m_caps/2, offset = cyl_h/2 + 3r/8
    let half_m_caps = m_caps * 0.5;
    let offset = cyl_h * 0.5 + 3.0 * r / 8.0;
    let offset2 = offset * offset;

    // Two hemispheres contribute to transverse (X, Z) axes via parallel axis
    let caps_ix = sphere_ix + 2.0 * half_m_caps * offset2;
    // Axial (Y) needs no parallel axis for symmetric hemispheres
    let caps_iy = sphere_iy;

    let i_y = cyl_iy + caps_iy;
    let i_xz = cyl_ix + caps_ix;

    Matrix3::from_diagonal(&Vector3::new(i_xz, i_y, i_xz))
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
