//! Generic constraint row solving: the PGS step for a single Jacobian row.
//!
//! These functions work for any constraint type because the Jacobian is
//! stored explicitly on the row.

use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::constraint::ConstraintRow;

/// Compute the constraint-space velocity: Cdot = J · v.
fn jacobian_dot_velocity(bodies: &Arena<RigidBody>, row: &ConstraintRow) -> f32 {
    let mut cdot = 0.0;

    if let Some(handle) = row.body_a {
        if let Some(body) = bodies.get(handle.0) {
            cdot += row.lin_jac_a.dot(&body.linear_velocity());
            cdot += row.ang_jac_a.dot(&body.angular_velocity());
        }
    }

    if let Some(handle) = row.body_b {
        if let Some(body) = bodies.get(handle.0) {
            cdot += row.lin_jac_b.dot(&body.linear_velocity());
            cdot += row.ang_jac_b.dot(&body.angular_velocity());
        }
    }

    cdot
}

/// Apply a scalar impulse along the constraint's Jacobian transpose: v += M⁻¹ · Jᵀ · impulse.
///
/// Linear: v += inv_mass * J_lin * impulse
/// Angular: ω += I_world_inv * J_ang * impulse
fn apply_constraint_impulse(
    bodies: &mut Arena<RigidBody>,
    row: &ConstraintRow,
    impulse: f32,
) {
    if let Some(handle) = row.body_a {
        if let Some(body) = bodies.get_mut(handle.0) {
            body.apply_impulse(row.lin_jac_a * impulse);
            let ang = body.world_inv_inertia() * (row.ang_jac_a * impulse);
            body.set_angular_velocity(body.angular_velocity() + ang);
        }
    }

    if let Some(handle) = row.body_b {
        if let Some(body) = bodies.get_mut(handle.0) {
            body.apply_impulse(row.lin_jac_b * impulse);
            let ang = body.world_inv_inertia() * (row.ang_jac_b * impulse);
            body.set_angular_velocity(body.angular_velocity() + ang);
        }
    }
}

/// Solve one constraint row: compute impulse, clamp, apply to bodies.
pub fn solve_constraint_row(
    bodies: &mut Arena<RigidBody>,
    row: &mut ConstraintRow,
) {
    let cdot = jacobian_dot_velocity(bodies, row);

    let lambda = row.effective_mass_inv * -(cdot + row.bias);

    let old_accumulated = row.accumulated_impulse;
    row.accumulated_impulse = (old_accumulated + lambda).clamp(row.bounds.0, row.bounds.1);
    let delta_lambda = row.accumulated_impulse - old_accumulated;

    if delta_lambda.abs() > 1e-12 {
        apply_constraint_impulse(bodies, row, delta_lambda);
    }
}

/// Warm-start a constraint row by applying its cached impulse scaled by a factor.
pub fn warm_start_constraint_row(
    bodies: &mut Arena<RigidBody>,
    row: &mut ConstraintRow,
    scale: f32,
) {
    let impulse = row.accumulated_impulse * scale;
    if impulse.abs() > 1e-12 {
        apply_constraint_impulse(bodies, row, impulse);
    }
}
