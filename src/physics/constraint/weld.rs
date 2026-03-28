//! Weld constraint: locks two bodies at a fixed relative position and orientation.
//!
//! Produces 6 constraint rows:
//! - 3 positional (X, Y, Z): drive anchor points together.
//! - 3 angular (X, Y, Z): drive relative orientation to the creation-time snapshot.
//!
//! Unlike FollowPoint, weld joints use unbounded impulse limits and warm-start
//! from cached impulses to achieve maximum stiffness within the PGS budget.

use nalgebra::{UnitQuaternion, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::types::ConstraintRow;

/// Expand a weld constraint into six solver rows.
///
/// `warm_impulses` should have exactly 6 entries (one per row). They are copied
/// into each row's `accumulated_impulse` for warm-starting.
pub fn expand(
    body_a: &RigidBody,
    handle_a: RigidBodyHandle,
    body_b: &RigidBody,
    handle_b: RigidBodyHandle,
    local_anchor_a: &Vector3<f32>,
    local_anchor_b: &Vector3<f32>,
    compliance: f32,
    _relative_orientation: &UnitQuaternion<f32>,
    angular_compliance: f32,
    dt: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> [ConstraintRow; 6] {
    let inv_mass_a = body_a.inv_mass();
    let inv_inertia_a = body_a.world_inv_inertia();
    let inv_mass_b = body_b.inv_mass();
    let inv_inertia_b = body_b.world_inv_inertia();

    // --- Positional rows (0–2) ---

    let r_a = body_a.rotation() * local_anchor_a;
    let r_b = body_b.rotation() * local_anchor_b;

    let pos_compliance_term = if compliance > 0.0 {
        compliance / (dt * dt)
    } else {
        0.0
    };

    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
    let zeros = Vector3::zeros();

    // --- Angular rows (3–5) ---

    let ang_compliance_term = if angular_compliance > 0.0 {
        angular_compliance / (dt * dt)
    } else {
        0.0
    };

    core::array::from_fn(|i| {
        let warm = warm_impulses.get(i).copied().unwrap_or(0.0);

        if i < 3 {
            let axis = axes[i];
            let ang_jac_a = -(r_a.cross(&axis));
            let ang_jac_b = r_b.cross(&axis);
            let ang_term_a = (inv_inertia_a * ang_jac_a).dot(&ang_jac_a);
            let ang_term_b = (inv_inertia_b * ang_jac_b).dot(&ang_jac_b);
            let eff_mass =
                1.0 / (inv_mass_a + ang_term_a + inv_mass_b + ang_term_b + pos_compliance_term);

            ConstraintRow {
                body_a: Some(handle_a),
                body_b: Some(handle_b),
                lin_jac_a: -axis,
                ang_jac_a,
                lin_jac_b: axis,
                ang_jac_b,
                effective_mass_inv: eff_mass,
                bias: 0.0,
                accumulated_impulse: warm,
                bounds: (-f32::MAX, f32::MAX),
                constraint_index,
                row_index: i,
            }
        } else {
            let axis_idx = i - 3;
            let axis = axes[axis_idx];
            let ang_term_a = (inv_inertia_a * axis).dot(&axis);
            let ang_term_b = (inv_inertia_b * axis).dot(&axis);
            let eff_mass = 1.0 / (ang_term_a + ang_term_b + ang_compliance_term);

            ConstraintRow {
                body_a: Some(handle_a),
                body_b: Some(handle_b),
                lin_jac_a: zeros,
                ang_jac_a: -axis,
                lin_jac_b: zeros,
                ang_jac_b: axis,
                effective_mass_inv: eff_mass,
                bias: 0.0,
                accumulated_impulse: warm,
                bounds: (-f32::MAX, f32::MAX),
                constraint_index,
                row_index: i,
            }
        }
    })
}
