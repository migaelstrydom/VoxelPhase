//! Fixed constraint: locks all 6 DOF between two bodies (or body to world).
//!
//! Produces 6 constraint rows:
//! - Rows 0-2: `lock_linear_axis` x3 — pin anchor points together.
//! - Rows 3-5: `lock_angular_axis` x3 — hold body_b at the joint's reference
//!   rotation relative to body_a (or to the world).

use nalgebra::{Point3, UnitQuaternion, Vector3};
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::primitives::{self, BodySide, RowParams};
use super::types::{ConstraintRow, CorrectionMode, Enforcement};

/// Expand a Fixed constraint into 6 solver rows.
///
/// Rows 0-2: positional (X, Y, Z). Error = anchor_b_world - anchor_a_world.
/// Rows 3-5: angular (X, Y, Z). Error = [`orientation_error`].
///
/// `beta` is the position correction factor (from solver config).
#[allow(clippy::too_many_arguments)]
pub fn expand(
    body_a: Option<(&RigidBody, RigidBodyHandle)>,
    body_b: &RigidBody,
    handle_b: RigidBodyHandle,
    local_anchor_a: &Vector3<f32>,
    local_anchor_b: &Vector3<f32>,
    reference: &UnitQuaternion<f32>,
    compliance: f32,
    max_impulse: f32,
    dt: f32,
    beta: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> SmallVec<[ConstraintRow; 6]> {
    let rot_b = body_b.rotation();

    // World-space anchor positions and lever arms.
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

    // Rows 0-2: positional — pin anchor points together.
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

    // Rows 3-5: angular — hold the reference rotation.
    let rotation_error = orientation_error(body_a.map(|(ba, _)| ba.rotation()), rot_b, reference);
    for (i, axis) in axes.into_iter().enumerate() {
        let bias = -(beta / dt) * rotation_error.dot(&axis);
        rows.push(primitives::lock_angular_axis(
            &side_a,
            &side_b,
            axis,
            bias,
            compliance_term,
            max_impulse,
            CorrectionMode::PositionAndVelocity,
            Enforcement::Iterative,
            &RowParams {
                constraint_index,
                row_index: 3 + i,
                warm_impulse: warm_impulses[3 + i],
            },
        ));
    }

    rows
}

/// How far body_b is turned from where the joint holds it, as a world-space
/// rotation vector: turning body_b by its negative puts it back.
///
/// `rot_a` is `None` for a joint anchored to the world.
pub fn orientation_error(
    rot_a: Option<UnitQuaternion<f32>>,
    rot_b: UnitQuaternion<f32>,
    reference: &UnitQuaternion<f32>,
) -> Vector3<f32> {
    let held = rot_a.map_or(*reference, |a| a * reference);
    let off = rot_b * held.inverse();
    // q and -q are the same rotation; the one with w ≥ 0 is the short way round.
    let off = if off.w < 0.0 {
        UnitQuaternion::new_unchecked(-off.into_inner())
    } else {
        off
    };
    off.scaled_axis()
}
