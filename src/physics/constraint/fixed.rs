//! Fixed constraint: locks all 6 DOF between two bodies (or body to world).
//!
//! Produces 6 constraint rows:
//! - Rows 0-2: `lock_linear_axis` x3 — pin anchor points together.
//! - Rows 3-5: `lock_angular_axis` x3 — lock all rotation.
//!
//! For world-anchored Fixed with compliance=0, the two tilt angular rows
//! (perpendicular to world Y) get `Enforcement::HardProjection`, matching
//! the KeepUpright projection behavior. The spin row (around Y) uses
//! iterative solving.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::primitives::{self, BodySide, RowParams};
use super::types::{ConstraintRow, CorrectionMode, Enforcement};

/// Expand a Fixed constraint into 6 solver rows.
///
/// Rows 0-2: positional (X, Y, Z). Error = anchor_b_world - anchor_a_world.
/// Rows 3-5: angular (X, Y, Z). Lock all relative rotation.
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

    // Rows 3-5: angular — lock all rotation.
    //
    // For world-anchored Fixed with zero compliance, the two tilt axes
    // (perpendicular to world Y) get HardProjection so the projection pass
    // handles tilt correction with the same cross-product recovery as
    // KeepUpright. The spin axis (Y) uses iterative solving.
    let is_world_anchored_rigid = body_a.is_none() && compliance == 0.0;

    // Use the same perpendicular basis as KeepUpright: perp1 and perp2 are
    // perpendicular to world Y, so cross(perp1, perp2) recovers +Y for the
    // projection pass.
    let world_y = Vector3::y();
    let perp1 = world_y.cross(&Vector3::x()).normalize(); // = -Z
    let perp2 = world_y.cross(&perp1); // = -X

    // Row 3: first tilt axis (perp1).
    let enforcement_tilt = if is_world_anchored_rigid {
        Enforcement::HardProjection
    } else {
        Enforcement::Iterative
    };

    rows.push(primitives::lock_angular_axis(
        &side_a,
        &side_b,
        perp1,
        0.0,
        compliance_term,
        max_impulse,
        CorrectionMode::PositionAndVelocity,
        enforcement_tilt,
        &RowParams {
            constraint_index,
            row_index: 3,
            warm_impulse: warm_impulses[3],
        },
    ));

    // Row 4: second tilt axis (perp2).
    rows.push(primitives::lock_angular_axis(
        &side_a,
        &side_b,
        perp2,
        0.0,
        compliance_term,
        max_impulse,
        CorrectionMode::PositionAndVelocity,
        enforcement_tilt,
        &RowParams {
            constraint_index,
            row_index: 4,
            warm_impulse: warm_impulses[4],
        },
    ));

    // Row 5: spin axis (world Y). Iterative only — PGS handles spin locking.
    rows.push(primitives::lock_angular_axis(
        &side_a,
        &side_b,
        world_y,
        0.0,
        compliance_term,
        max_impulse,
        CorrectionMode::VelocityOnly,
        Enforcement::Iterative,
        &RowParams {
            constraint_index,
            row_index: 5,
            warm_impulse: warm_impulses[5],
        },
    ));

    rows
}
