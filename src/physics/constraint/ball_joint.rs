//! BallJoint constraint: constrains two anchor points to coincide.
//!
//! Produces 3 constraint rows (`lock_linear_axis` x3) that pin two anchor
//! points together. 3 DOF locked (translation), 3 DOF free (rotation).
//!
//! Use cases: pendulums, chain links, swinging platforms, rope anchors.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::primitives::{self, BodySide, RowParams};
use super::types::ConstraintRow;

/// Expand a BallJoint constraint into 3 solver rows.
///
/// Rows 0-2: positional (X, Y, Z). Error = anchor_b_world - anchor_a_world.
///
/// `beta` is the position correction factor (from solver config).
#[allow(clippy::too_many_arguments)]
pub fn expand(
    body_a: Option<(&RigidBody, RigidBodyHandle)>,
    body_b: &RigidBody,
    handle_b: RigidBodyHandle,
    local_anchor_a: &Vector3<f32>,
    local_anchor_b: &Vector3<f32>,
    compliance: f32,
    max_impulse: f32,
    dt: f32,
    beta: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> SmallVec<[ConstraintRow; 3]> {
    let rot_b = body_b.rotation();

    let (anchor_a_world, side_a) = if let Some((ba, ha)) = body_a {
        let r = ba.rotation() * local_anchor_a;
        let anchor = ba.position() + r;
        let side = BodySide {
            handle: Some(ha),
            inv_mass: ba.inv_mass(),
            inv_inertia: ba.world_inv_inertia(),
            lever_arm: r,
        };
        (anchor, side)
    } else {
        let anchor = Point3::from(*local_anchor_a);
        (anchor, BodySide::world())
    };

    let r_b = rot_b * local_anchor_b;
    let anchor_b_world: Point3<f32> = body_b.position() + r_b;

    let side_b = BodySide {
        handle: Some(handle_b),
        inv_mass: body_b.inv_mass(),
        inv_inertia: body_b.world_inv_inertia(),
        lever_arm: r_b,
    };

    let error = anchor_b_world - anchor_a_world;

    let compliance_term = if compliance > 0.0 {
        compliance / (dt * dt)
    } else {
        0.0
    };

    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
    let mut rows = SmallVec::new();

    for i in 0..3 {
        let positional_error = error.dot(&axes[i]);
        let bias = -(beta / dt) * positional_error;

        rows.push(primitives::lock_linear_axis(
            &side_a,
            &side_b,
            axes[i],
            bias,
            compliance_term,
            max_impulse,
            &RowParams {
                constraint_index,
                row_index: i,
                warm_impulse: warm_impulses[i],
            },
        ));
    }

    rows
}
