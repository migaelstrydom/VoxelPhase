//! KeepUpright constraint: row expansion and error computation.
//!
//! Constrains a body's local Y-axis to align with an arbitrary target direction
//! using two angular constraint rows perpendicular to the target.

use nalgebra::{UnitVector3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::primitives::{self, BodySide, RowParams};
use super::types::{ConstraintRow, CorrectionMode, Enforcement};

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
///
/// `max_impulse` bounds the angular impulse each row may accumulate. A finite
/// bound rules out `Enforcement::HardProjection`, which rewrites velocity
/// directly and would spend unlimited impulse doing it; the rows fall back to
/// iterative solving, where the bound is honoured. `PositionCorrectionSolver`
/// skips its NGS pass for bounded constraints for the same reason.
pub fn expand(
    body: &RigidBody,
    body_handle: RigidBodyHandle,
    target_up: &UnitVector3<f32>,
    compliance: f32,
    max_impulse: f32,
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

    let compliance_term = if compliance > 0.0 {
        compliance / (dt * dt)
    } else {
        0.0
    };

    let bias_1 = -(beta / dt) * error_1;
    let bias_2 = -(beta / dt) * error_2;

    let enforcement = if compliance > 0.0 || max_impulse.is_finite() {
        Enforcement::Iterative
    } else {
        Enforcement::HardProjection
    };

    let side_a = BodySide {
        handle: Some(body_handle),
        inv_mass: 0.0,
        inv_inertia: body.world_inv_inertia(),
        lever_arm: Vector3::zeros(),
    };
    let side_b = BodySide::world();

    [
        primitives::lock_angular_axis(
            &side_a,
            &side_b,
            perp1,
            bias_1,
            compliance_term,
            max_impulse,
            CorrectionMode::PositionAndVelocity,
            enforcement,
            &RowParams {
                constraint_index,
                row_index: 0,
                warm_impulse: warm_impulses[0],
            },
        ),
        primitives::lock_angular_axis(
            &side_a,
            &side_b,
            perp2,
            bias_2,
            compliance_term,
            max_impulse,
            CorrectionMode::PositionAndVelocity,
            enforcement,
            &RowParams {
                constraint_index,
                row_index: 1,
                warm_impulse: warm_impulses[1],
            },
        ),
    ]
}
