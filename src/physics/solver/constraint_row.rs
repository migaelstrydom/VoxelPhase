//! Generic constraint row solving: the PGS step for a single Jacobian row.
//!
//! These functions work for any constraint type because the Jacobian is
//! stored explicitly on the row.

use nalgebra::Vector3;

use crate::physics::constraint::ConstraintRow;

use super::solver_bodies::{SolverBodies, SolverBody};

/// The solver slots of a constraint row's two bodies, `None` for a
/// world-anchored side or a body that is gone.
pub(crate) type RowSlots = (Option<usize>, Option<usize>);

/// Compute the constraint-space velocity: Cdot = J · v.
fn jacobian_dot_velocity(bodies: &SolverBodies, slots: RowSlots, row: &ConstraintRow) -> f32 {
    let mut cdot = 0.0;

    if let Some(slot) = slots.0 {
        let body = bodies.get(slot);
        cdot += row.lin_jac_a.dot(&body.linear_velocity);
        cdot += row.ang_jac_a.dot(&body.angular_velocity);
    }

    if let Some(slot) = slots.1 {
        let body = bodies.get(slot);
        cdot += row.lin_jac_b.dot(&body.linear_velocity);
        cdot += row.ang_jac_b.dot(&body.angular_velocity);
    }

    cdot
}

/// Apply a scalar impulse along the constraint's Jacobian transpose: v += M⁻¹ · Jᵀ · impulse.
///
/// Linear: v += inv_mass * J_lin * impulse
/// Angular: ω += I_world_inv * J_ang * impulse
fn apply_constraint_impulse(
    bodies: &mut SolverBodies,
    slots: RowSlots,
    row: &ConstraintRow,
    impulse: f32,
) {
    if let Some(slot) = slots.0 {
        apply_along_jacobian(bodies.get_mut(slot), row.lin_jac_a, row.ang_jac_a, impulse);
    }

    if let Some(slot) = slots.1 {
        apply_along_jacobian(bodies.get_mut(slot), row.lin_jac_b, row.ang_jac_b, impulse);
    }
}

/// One side of [`apply_constraint_impulse`].
///
/// The angular half is added even to a body with no inverse mass, whose world
/// inverse inertia is zero: adding that zero is not a no-op on a velocity
/// component of `-0.0`, and the rows have always added it.
fn apply_along_jacobian(
    body: &mut SolverBody,
    linear_jacobian: Vector3<f32>,
    angular_jacobian: Vector3<f32>,
    impulse: f32,
) {
    body.apply_linear_impulse(linear_jacobian * impulse);
    body.angular_velocity += body.world_inv_inertia * (angular_jacobian * impulse);
}

/// Solve one constraint row: compute impulse, clamp, apply to bodies.
///
/// A rigid row (`softness == 0`) converges where `J·v + bias` is zero, which
/// is to say where the error is gone. A soft row carries its accumulated
/// impulse into the residual, so it converges where the impulse balances the
/// softness instead — a spring, whose steady deflection under a constant load
/// `tau` is `compliance · tau / beta`. Without this term the softness would
/// only slow the row's convergence, and it would still end up rigid.
pub(crate) fn solve_constraint_row(
    bodies: &mut SolverBodies,
    slots: RowSlots,
    row: &mut ConstraintRow,
) {
    let cdot = jacobian_dot_velocity(bodies, slots, row);

    let lambda =
        row.effective_mass_inv * -(cdot + row.bias + row.softness * row.accumulated_impulse);

    let old_accumulated = row.accumulated_impulse;
    row.accumulated_impulse = (old_accumulated + lambda).clamp(row.bounds.0, row.bounds.1);
    let delta_lambda = row.accumulated_impulse - old_accumulated;

    if delta_lambda.abs() > 1e-12 {
        apply_constraint_impulse(bodies, slots, row, delta_lambda);
    }
}

/// Warm-start a constraint row by applying its cached impulse scaled by a factor.
///
/// The accumulator is reset to what was actually applied, matching the contact
/// warm start. Leaving it at the full cached value credits the row with impulse
/// the body never received. An unbounded row corrects that on its next solve;
/// a row at its bound cannot, so its clamp works off a stale number and the
/// impulse it delivers per substep stops meaning what the bound says.
pub(crate) fn warm_start_constraint_row(
    bodies: &mut SolverBodies,
    slots: RowSlots,
    row: &mut ConstraintRow,
    scale: f32,
) {
    row.accumulated_impulse *= scale;
    if row.accumulated_impulse.abs() > 1e-12 {
        apply_constraint_impulse(bodies, slots, row, row.accumulated_impulse);
    }
}
