//! KeepUpright constraint: row expansion and error computation.
//!
//! Constrains a body's local Y-axis to align with an arbitrary target direction
//! using two angular constraint rows perpendicular to the target.

use nalgebra::{UnitVector3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::types::ConstraintRow;

/// Build an orthonormal basis perpendicular to `target_up`.
///
/// Returns two unit vectors (perp1, perp2) such that (perp1, target_up, perp2)
/// form a right-handed basis. The seed axis is chosen as the cardinal axis
/// least aligned with `target_up` to avoid degeneracy.
fn perpendicular_basis(target_up: &UnitVector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    let t = target_up.into_inner();
    let abs_t = t.map(|c| c.abs());
    let seed = if abs_t.x <= abs_t.y && abs_t.x <= abs_t.z {
        Vector3::x()
    } else if abs_t.y <= abs_t.z {
        Vector3::y()
    } else {
        Vector3::z()
    };
    let perp1 = t.cross(&seed).normalize();
    let perp2 = t.cross(&perp1);
    (perp1, perp2)
}

/// Expand a KeepUpright constraint into two solver rows.
///
/// `beta` is the position correction factor (from `PositionCorrectionConfig::correction_factor`).
pub fn expand(
    body: &RigidBody,
    body_handle: RigidBodyHandle,
    target_up: &UnitVector3<f32>,
    compliance: f32,
    dt: f32,
    beta: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> [ConstraintRow; 2] {
    let local_up = body.rotation() * Vector3::y();
    let (perp1, perp2) = perpendicular_basis(target_up);

    // Tilt errors: project local_up onto each perpendicular axis.
    // For small angles, sin(θ) ≈ θ.
    let error_1 = local_up.dot(&perp1);
    let error_2 = local_up.dot(&perp2);

    // Effective mass: 1 / (J · I⁻¹ · Jᵀ + compliance/dt²)
    let inv_inertia_ws = body.world_inv_inertia();
    let compliance_term = if compliance > 0.0 {
        compliance / (dt * dt)
    } else {
        0.0
    };
    let eff_mass_1 =
        1.0 / ((inv_inertia_ws * perp1).dot(&perp1) + compliance_term);
    let eff_mass_2 =
        1.0 / ((inv_inertia_ws * perp2).dot(&perp2) + compliance_term);

    // Bias: position correction drives error toward zero
    let bias_1 = -(beta / dt) * error_1;
    let bias_2 = -(beta / dt) * error_2;

    let zeros = Vector3::zeros();

    [
        ConstraintRow {
            body_a: Some(body_handle),
            body_b: None,
            lin_jac_a: zeros,
            ang_jac_a: perp1,
            lin_jac_b: zeros,
            ang_jac_b: zeros,
            effective_mass_inv: eff_mass_1,
            bias: bias_1,
            accumulated_impulse: warm_impulses[0],
            bounds: (-f32::MAX, f32::MAX),
            constraint_index,
            row_index: 0,
        },
        ConstraintRow {
            body_a: Some(body_handle),
            body_b: None,
            lin_jac_a: zeros,
            ang_jac_a: perp2,
            lin_jac_b: zeros,
            ang_jac_b: zeros,
            effective_mass_inv: eff_mass_2,
            bias: bias_2,
            accumulated_impulse: warm_impulses[1],
            bounds: (-f32::MAX, f32::MAX),
            constraint_index,
            row_index: 1,
        },
    ]
}
