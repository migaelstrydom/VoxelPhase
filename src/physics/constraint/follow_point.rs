//! FollowPoint constraint: drives a point on body_b toward a point on body_a,
//! with angular rows that resist relative rotation.
//!
//! Produces 6 constraint rows:
//! - 3 positional (X, Y, Z) with linear + angular Jacobians on both bodies.
//! - 3 angular (X, Y, Z) with pure angular Jacobians that lock relative
//!   orientation to the snapshot captured at grab time.
//!
//! Newton's third law is automatic — the solver applies equal and opposite
//! impulses, so heavy held objects resist the anchor body's movement and turning.

use nalgebra::{UnitQuaternion, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::types::ConstraintRow;

/// Expand a two-body FollowPoint constraint into six solver rows.
///
/// Rows 0–2: positional (X, Y, Z). Error = grab_point_b - hold_point_a.
/// Rows 3–5: angular (X, Y, Z). Error = axis-angle of the orientation drift
/// from the relative orientation captured at grab time.
pub fn expand(
    body_a: &RigidBody,
    handle_a: RigidBodyHandle,
    body_b: &RigidBody,
    handle_b: RigidBodyHandle,
    local_anchor_a: &Vector3<f32>,
    local_anchor_b: &Vector3<f32>,
    compliance: f32,
    max_impulse: f32,
    relative_orientation: &UnitQuaternion<f32>,
    angular_compliance: f32,
    angular_max_impulse: f32,
    dt: f32,
    beta: f32,
    constraint_index: generational_arena::Index,
) -> [ConstraintRow; 6] {
    let inv_mass_a = body_a.inv_mass();
    let inv_inertia_a = body_a.world_inv_inertia();
    let inv_mass_b = body_b.inv_mass();
    let inv_inertia_b = body_b.world_inv_inertia();

    // --- Positional rows (0–2) ---

    let r_a = body_a.rotation() * local_anchor_a;
    let hold_point = body_a.position() + r_a;
    let r_b = body_b.rotation() * local_anchor_b;
    let grab_point = body_b.position() + r_b;

    let pos_compliance_term = if compliance > 0.0 {
        compliance / (dt * dt)
    } else {
        0.0
    };

    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
    let zeros = Vector3::zeros();

    // --- Angular rows (3–5) ---

    // Compute angular error in world space so it matches the world-space
    // Jacobian axes. target_b_rot is where B *should* be oriented given A's
    // current rotation and the grab-time snapshot.
    let target_b_rot = body_a.rotation() * relative_orientation;
    let error_q = body_b.rotation() * target_b_rot.inverse();
    // Extract axis-angle error vector (world-space). The vector part of a
    // unit quaternion scaled by 2 is a good approximation of the rotation
    // vector, exact for small angles and well-behaved at larger ones.
    let angular_error = error_q.vector() * 2.0 * error_q.w.signum();

    let ang_compliance_term = if angular_compliance > 0.0 {
        angular_compliance / (dt * dt)
    } else {
        0.0
    };

    core::array::from_fn(|i| {
        if i < 3 {
            // Positional row
            let axis = axes[i];
            let ang_jac_a = -(r_a.cross(&axis));
            let ang_jac_b = r_b.cross(&axis);
            let error = grab_point[i] - hold_point[i];
            let ang_term_a = (inv_inertia_a * ang_jac_a).dot(&ang_jac_a);
            let ang_term_b = (inv_inertia_b * ang_jac_b).dot(&ang_jac_b);
            let eff_mass =
                1.0 / (inv_mass_a + ang_term_a + inv_mass_b + ang_term_b + pos_compliance_term);
            let bias = (beta / dt) * error;

            ConstraintRow {
                body_a: Some(handle_a),
                body_b: Some(handle_b),
                lin_jac_a: -axis,
                ang_jac_a,
                lin_jac_b: axis,
                ang_jac_b,
                effective_mass_inv: eff_mass,
                bias,
                accumulated_impulse: 0.0,
                bounds: (-max_impulse, max_impulse),
                constraint_index,
                row_index: i,
            }
        } else {
            // Angular row (i-3 maps to X, Y, Z)
            let axis_idx = i - 3;
            let axis = axes[axis_idx];
            // Pure angular: d(error)/d(ω_a) = -axis, d(error)/d(ω_b) = +axis
            let ang_term_a = (inv_inertia_a * axis).dot(&axis);
            let ang_term_b = (inv_inertia_b * axis).dot(&axis);
            let eff_mass = 1.0 / (ang_term_a + ang_term_b + ang_compliance_term);
            let bias = (beta / dt) * angular_error[axis_idx];

            ConstraintRow {
                body_a: Some(handle_a),
                body_b: Some(handle_b),
                lin_jac_a: zeros,
                ang_jac_a: -axis,
                lin_jac_b: zeros,
                ang_jac_b: axis,
                effective_mass_inv: eff_mass,
                bias,
                accumulated_impulse: 0.0,
                bounds: (-angular_max_impulse, angular_max_impulse),
                constraint_index,
                row_index: i,
            }
        }
    })
}
