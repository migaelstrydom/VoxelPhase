//! AnchorPoint constraint: pins a body to a fixed world-space position.
//!
//! Produces 3 positional constraint rows (X, Y, Z) that drive a local
//! point on the body toward a fixed world-space anchor. The world side
//! has `body_a: None`, so the solver treats it as immovable.
//!
//! Optional angular rows lock rotation around world Y (yaw) and/or
//! world X (roll). Pair with a KeepUpright constraint to fully pin
//! orientation, or use lock_yaw + lock_roll to allow tilt around only
//! one axis (e.g. seesaw tilts around Z only).

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::types::ConstraintRow;

/// Expand an AnchorPoint constraint into 3–5 solver rows.
///
/// Rows 0–2: positional (X, Y, Z).
/// Row 3+ (optional): angular, constraining spin around world Y and/or X.
///
/// `beta` is the position correction factor (from solver config).
pub fn expand(
    body: &RigidBody,
    body_handle: RigidBodyHandle,
    local_anchor: &Vector3<f32>,
    world_anchor: &Point3<f32>,
    compliance: f32,
    max_impulse: f32,
    lock_yaw: bool,
    lock_roll: bool,
    dt: f32,
    beta: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> SmallVec<[ConstraintRow; 5]> {
    let inv_mass = body.inv_mass();
    let inv_inertia = body.world_inv_inertia();

    let r = body.rotation() * local_anchor;
    let body_point: Point3<f32> = body.position() + r;
    let error = body_point - world_anchor;

    let compliance_term = if compliance > 0.0 {
        compliance / (dt * dt)
    } else {
        0.0
    };

    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
    let zeros = Vector3::zeros();

    let mut rows = SmallVec::new();

    // Positional rows (0–2).
    for i in 0..3 {
        let axis = axes[i];
        let ang_jac = r.cross(&axis);
        let ang_term = (inv_inertia * ang_jac).dot(&ang_jac);
        let eff_mass = 1.0 / (inv_mass + ang_term + compliance_term);

        let positional_error = error.dot(&axis);
        let bias = -(beta / dt) * positional_error;

        rows.push(ConstraintRow {
            body_a: None,
            body_b: Some(body_handle),
            lin_jac_a: zeros,
            ang_jac_a: zeros,
            lin_jac_b: axis,
            ang_jac_b: ang_jac,
            effective_mass_inv: eff_mass,
            bias,
            accumulated_impulse: warm_impulses[i],
            bounds: (-max_impulse, max_impulse),
            constraint_index,
            row_index: i,
        });
    }

    // Optional angular lock rows.
    let mut row_idx = 3;

    if lock_yaw {
        let y_axis = Vector3::y();
        let ang_term = (inv_inertia * y_axis).dot(&y_axis);
        let eff_mass = 1.0 / (ang_term + compliance_term);

        rows.push(ConstraintRow {
            body_a: None,
            body_b: Some(body_handle),
            lin_jac_a: zeros,
            ang_jac_a: zeros,
            lin_jac_b: zeros,
            ang_jac_b: y_axis,
            effective_mass_inv: eff_mass,
            bias: 0.0,
            accumulated_impulse: warm_impulses.get(row_idx).copied().unwrap_or(0.0),
            bounds: (-max_impulse, max_impulse),
            constraint_index,
            row_index: row_idx,
        });
        row_idx += 1;
    }

    if lock_roll {
        let x_axis = Vector3::x();
        let ang_term = (inv_inertia * x_axis).dot(&x_axis);
        let eff_mass = 1.0 / (ang_term + compliance_term);

        rows.push(ConstraintRow {
            body_a: None,
            body_b: Some(body_handle),
            lin_jac_a: zeros,
            ang_jac_a: zeros,
            lin_jac_b: zeros,
            ang_jac_b: x_axis,
            effective_mass_inv: eff_mass,
            bias: 0.0,
            accumulated_impulse: warm_impulses.get(row_idx).copied().unwrap_or(0.0),
            bounds: (-max_impulse, max_impulse),
            constraint_index,
            row_index: row_idx,
        });
    }

    rows
}
