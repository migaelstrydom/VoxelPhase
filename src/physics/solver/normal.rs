//! Normal impulse resolution for contact constraints.

use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::contact_row::ContactRow;
use super::diagnostics::log_impulse_torque_diag;

/// Minimum effective inverse mass allowed in the normal solver.
///
/// Prevents near-zero denominators (degenerate geometry or deeply wedged bodies)
/// from producing runaway impulses.
pub(crate) const MIN_EFFECTIVE_INV_MASS: f32 = 1.0e-8;

/// Solve one contact's normal row against the rows prepared for this substep.
///
/// `row` is `None` when a body of the pair is gone, which leaves nothing to do.
pub(crate) fn solve_normal_impulse(
    bodies: &mut Arena<RigidBody>,
    row: Option<&ContactRow>,
    header: &PairHeader,
    contact: &mut SolverContact,
    restitution_velocity_threshold: f32,
    pre_solve_vn: f32,
) {
    let Some(row) = row else {
        return;
    };

    let vel_along_normal = row.relative_velocity(bodies).dot(&contact.normal);
    if vel_along_normal > 0.0 && contact.accumulated_normal_impulse <= 1e-8 {
        return;
    }

    let effective_inv_mass = row.normal_inv_mass;
    if effective_inv_mass <= MIN_EFFECTIVE_INV_MASS || !effective_inv_mass.is_finite() {
        return;
    }

    let speed = pre_solve_vn.abs();
    let restitution_scale =
        ((speed - restitution_velocity_threshold) / restitution_velocity_threshold).clamp(0.0, 1.0);
    let restitution = header.restitution * restitution_scale;
    let restitution_velocity = if pre_solve_vn < 0.0 {
        restitution * pre_solve_vn
    } else {
        0.0
    };

    let delta = -(vel_along_normal + restitution_velocity) / effective_inv_mass;
    let old = contact.accumulated_normal_impulse;
    let new = (old + delta).max(0.0);
    let applied = new - old;
    contact.accumulated_normal_impulse = new;

    if applied.abs() > 1e-10 {
        let impulse = contact.normal * applied;
        log_impulse_torque_diag("normal", header, contact, row, &impulse);
        row.apply_impulse(bodies, impulse);
    }
}
