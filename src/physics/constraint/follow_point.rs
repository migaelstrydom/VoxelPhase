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

use super::primitives::{self, BodySide, RowParams};
use super::types::{ConstraintRow, CorrectionMode, Enforcement};

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
    _relative_orientation: &UnitQuaternion<f32>,
    angular_compliance: f32,
    angular_max_impulse: f32,
    dt: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> [ConstraintRow; 6] {
    let r_a = body_a.rotation() * local_anchor_a;
    let r_b = body_b.rotation() * local_anchor_b;

    let pos_compliance_term = if compliance > 0.0 {
        compliance / (dt * dt)
    } else {
        0.0
    };

    let ang_compliance_term = if angular_compliance > 0.0 {
        angular_compliance / (dt * dt)
    } else {
        0.0
    };

    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];

    let side_a = BodySide {
        handle: Some(handle_a),
        inv_mass: body_a.inv_mass(),
        inv_inertia: body_a.world_inv_inertia(),
        lever_arm: r_a,
    };
    let side_b = BodySide {
        handle: Some(handle_b),
        inv_mass: body_b.inv_mass(),
        inv_inertia: body_b.world_inv_inertia(),
        lever_arm: r_b,
    };

    core::array::from_fn(|i| {
        if i < 3 {
            primitives::lock_linear_axis(
                &side_a,
                &side_b,
                axes[i],
                0.0,
                pos_compliance_term,
                max_impulse,
                &RowParams {
                    constraint_index,
                    row_index: i,
                    warm_impulse: warm_impulses[i],
                },
            )
        } else {
            let axis_idx = i - 3;
            primitives::lock_angular_axis(
                &side_a,
                &side_b,
                axes[axis_idx],
                0.0,
                ang_compliance_term,
                angular_max_impulse,
                CorrectionMode::VelocityOnly,
                Enforcement::Iterative,
                &RowParams {
                    constraint_index,
                    row_index: i,
                    warm_impulse: warm_impulses[i],
                },
            )
        }
    })
}
