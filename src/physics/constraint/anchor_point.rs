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

use super::primitives::{self, BodySide, RowParams};
use super::types::{ConstraintRow, CorrectionMode, Enforcement};

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
    let r = body.rotation() * local_anchor;
    let body_point: Point3<f32> = body.position() + r;
    let error = body_point - world_anchor;

    let compliance_term = if compliance > 0.0 {
        compliance / (dt * dt)
    } else {
        0.0
    };

    let side_a = BodySide::world();
    let side_b = BodySide {
        handle: Some(body_handle),
        inv_mass: body.inv_mass(),
        inv_inertia: body.world_inv_inertia(),
        lever_arm: r,
    };

    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
    let mut rows = SmallVec::new();

    // Positional rows (0–2).
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

    // Optional angular lock rows.
    let mut row_idx = 3;

    if lock_yaw {
        rows.push(primitives::lock_angular_axis(
            &side_a,
            &side_b,
            Vector3::y(),
            0.0,
            compliance_term,
            max_impulse,
            CorrectionMode::VelocityOnly,
            Enforcement::Iterative,
            &RowParams {
                constraint_index,
                row_index: row_idx,
                warm_impulse: warm_impulses.get(row_idx).copied().unwrap_or(0.0),
            },
        ));
        row_idx += 1;
    }

    if lock_roll {
        rows.push(primitives::lock_angular_axis(
            &side_a,
            &side_b,
            Vector3::x(),
            0.0,
            compliance_term,
            max_impulse,
            CorrectionMode::VelocityOnly,
            Enforcement::Iterative,
            &RowParams {
                constraint_index,
                row_index: row_idx,
                warm_impulse: warm_impulses.get(row_idx).copied().unwrap_or(0.0),
            },
        ));
    }

    rows
}
