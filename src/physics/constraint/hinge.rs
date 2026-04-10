//! Hinge constraint: constrains two anchor points to coincide and restricts
//! rotation to one axis (the hinge axis). 5 DOF locked, 1 DOF free.
//!
//! Produces 5 constraint rows:
//! - Rows 0-2: `lock_linear_axis` x3 — pin anchor points together.
//! - Rows 3-4: `lock_angular_axis` x2 — lock the two axes perpendicular to
//!   the hinge axis, preventing relative rotation around anything other than
//!   the hinge axis.
//!
//! For world-anchored hinges (`body_a = None`), the world side has zero
//! inverse mass and inertia.

use nalgebra::{Point3, UnitVector3, Vector3};
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::primitives::{self, BodySide, RowParams};
use super::types::{ConstraintRow, CorrectionMode, Enforcement};

/// Expand a Hinge constraint into 5 solver rows.
///
/// Rows 0-2: positional (X, Y, Z). Error = anchor_b_world - anchor_a_world.
/// Rows 3-4: angular. Error = projection of body_a's hinge axis onto the two
/// perpendicular axes derived from body_b's reference frame.
///
/// `beta` is the position correction factor (from solver config).
#[allow(clippy::too_many_arguments)]
pub fn expand(
    body_a: Option<(&RigidBody, RigidBodyHandle)>,
    body_b: &RigidBody,
    handle_b: RigidBodyHandle,
    local_anchor_a: &Vector3<f32>,
    local_anchor_b: &Vector3<f32>,
    _local_axis_a: &UnitVector3<f32>,
    local_axis_b: &UnitVector3<f32>,
    local_ref_b: &UnitVector3<f32>,
    compliance: f32,
    max_impulse: f32,
    dt: f32,
    beta: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> SmallVec<[ConstraintRow; 5]> {
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

    // Rows 3-4: angular — lock perpendicular-to-hinge axes.
    //
    // Build two perpendicular axes from body_b's reference frame:
    // n1 = rot_b * local_ref_b  (perpendicular to hinge in body_b's frame)
    // n2 = world_axis_b x n1    (the other perpendicular direction)
    let world_axis_b = rot_b * local_axis_b.into_inner();
    let n1 = rot_b * local_ref_b.into_inner();
    let n2 = world_axis_b.cross(&n1);

    // Angular rows use PositionAndVelocity so Phase 7's angular NGS will
    // apply position correction. Until then, only PGS velocity solving
    // constrains the locked axes (zero bias, no Baumgarte).
    rows.push(primitives::lock_angular_axis(
        &side_a,
        &side_b,
        n1,
        0.0,
        compliance_term,
        max_impulse,
        CorrectionMode::PositionAndVelocity,
        Enforcement::Iterative,
        &RowParams {
            constraint_index,
            row_index: 3,
            warm_impulse: warm_impulses[3],
        },
    ));

    rows.push(primitives::lock_angular_axis(
        &side_a,
        &side_b,
        n2,
        0.0,
        compliance_term,
        max_impulse,
        CorrectionMode::PositionAndVelocity,
        Enforcement::Iterative,
        &RowParams {
            constraint_index,
            row_index: 4,
            warm_impulse: warm_impulses[4],
        },
    ));

    rows
}
