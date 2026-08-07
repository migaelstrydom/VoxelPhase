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
/// Each hemisphere's transverse inertia is computed about its own center of
/// mass, then shifted to the capsule center via the parallel axis theorem.
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
    let m_hemi = mass * v_caps / v_total * 0.5; // mass of one hemisphere

    // Cylinder moment about its own center (axis = Y)
    let cyl_iy = 0.5 * m_cyl * r2;
    let cyl_ix = m_cyl * (3.0 * r2 + cyl_h * cyl_h) / 12.0;

    // Hemisphere axial (Y) inertia — same as half a sphere, no offset needed.
    let hemi_iy = (2.0 / 5.0) * m_hemi * r2;

    // Hemisphere transverse inertia about its own center of mass.
    // A hemisphere's CoM is 3r/8 from the flat face. Starting from the
    // flat-face inertia (2/5·m·r²) and shifting inward by 3r/8:
    //   I_cm = (2/5)·m·r² − m·(3r/8)² = (83/320)·m·r²
    let hemi_ix_cm = (83.0 / 320.0) * m_hemi * r2;

    // Parallel axis theorem: each hemisphere CoM is at cyl_h/2 + 3r/8
    // from the capsule center.
    let offset = cyl_h * 0.5 + 3.0 * r / 8.0;

    // Two hemispheres contribute to transverse (X, Z) axes
    let caps_ix = 2.0 * (hemi_ix_cm + m_hemi * offset * offset);
    // Axial (Y) needs no parallel axis for symmetric hemispheres
    let caps_iy = 2.0 * hemi_iy;

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capsule_inertia_degenerates_to_sphere() {
        // When half_height == radius, the cylinder vanishes and the capsule is a sphere.
        let r = 1.0;
        let mass = 10.0;
        let capsule = capsule_inertia_tensor(mass, r, r);
        let sphere = sphere_inertia_tensor(mass, r);

        for i in 0..3 {
            assert!(
                (capsule[(i, i)] - sphere[(i, i)]).abs() < 1e-4,
                "axis {i}: capsule={} sphere={}",
                capsule[(i, i)],
                sphere[(i, i)],
            );
        }
    }

    #[test]
    fn capsule_inertia_hemisphere_correction() {
        // Verify the hemisphere CoM correction is applied.
        // For a squat capsule (half_height=1.0, radius=0.5), the r² coefficient
        // for hemisphere transverse inertia should be 83/320, not 2/5 = 128/320.
        let r = 0.5;
        let half_height = 1.0;
        let mass = 1.0;

        let inertia = capsule_inertia_tensor(mass, half_height, r);
        let i_xz = inertia[(0, 0)];
        let i_y = inertia[(1, 1)];

        // Transverse inertia should be > axial inertia for a non-sphere
        assert!(i_xz > i_y, "I_xz={i_xz} should be > I_y={i_y}");

        // Compare against hand-computed value:
        // cyl_h = 1.0, r = 0.5, r² = 0.25
        // v_cyl = π·0.25·1.0 = 0.7854, v_caps = (4/3)·π·0.125 = 0.5236
        // v_total = 1.3090
        // m_cyl = 0.5999, m_hemi = 0.2001
        // cyl_ix = 0.5999·(0.75+1)/12 = 0.0875
        // hemi_ix_cm = 83/320·0.2001·0.25 = 0.01298
        // offset = 0.5 + 0.1875 = 0.6875
        // caps_ix = 2·(0.01298 + 0.2001·0.4727) = 2·(0.01298 + 0.09457) = 0.2151
        // i_xz = 0.0875 + 0.2151 = 0.3026
        assert!(
            (i_xz - 0.3026).abs() < 0.001,
            "I_xz={i_xz}, expected ~0.3026"
        );
    }

    #[test]
    fn capsule_inertia_tall_capsule() {
        // Tall capsule: verify transverse >> axial
        let inertia = capsule_inertia_tensor(1.0, 5.0, 0.5);
        let i_xz = inertia[(0, 0)];
        let i_y = inertia[(1, 1)];

        assert!(
            i_xz / i_y > 50.0,
            "tall capsule I_xz/I_y={:.1}, expected > 50",
            i_xz / i_y
        );
    }
}
