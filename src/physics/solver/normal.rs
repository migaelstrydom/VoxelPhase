//! Normal impulse resolution for contact constraints.

use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::contact_row::ContactRow;
use super::diagnostics::log_impulse_torque_diag;
use super::solver_bodies::SolverBodies;

/// Minimum effective inverse mass allowed in the normal solver.
///
/// Prevents near-zero denominators (degenerate geometry or deeply wedged bodies)
/// from producing runaway impulses.
pub(crate) const MIN_EFFECTIVE_INV_MASS: f32 = 1.0e-8;

/// Solve one contact's normal row against the rows prepared for this substep.
///
/// `row` is `None` when a body of the pair is gone, which leaves nothing to do.
pub(crate) fn solve_normal_impulse(
    bodies: &mut SolverBodies,
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

    let target_velocity = normal_target_velocity(
        header,
        contact,
        restitution_velocity_threshold,
        pre_solve_vn,
    );

    let delta = (target_velocity - vel_along_normal) / effective_inv_mass;
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

/// The normal velocity a contact's row drives its pair towards: the rebound
/// restitution asks for, or, for a speculative contact whose pair has not
/// arrived, the approach it still permits.
pub(crate) fn normal_target_velocity(
    header: &PairHeader,
    contact: &SolverContact,
    restitution_velocity_threshold: f32,
    pre_solve_vn: f32,
) -> f32 {
    let speed = pre_solve_vn.abs();
    let restitution_scale =
        ((speed - restitution_velocity_threshold) / restitution_velocity_threshold).clamp(0.0, 1.0);
    let rebound = header.restitution * restitution_scale * (-pre_solve_vn).max(0.0);

    // A speculative contact whose pair has not arrived permits approach up to
    // the gap that remains. On the substep it arrives there is an impact to
    // answer, and the rebound is taken from the approach it arrived with —
    // arrested on arrival, the pair would otherwise start the next substep
    // with no approach left for restitution to reflect. It leaves up to the
    // remaining gap unclosed, less than one substep of travel.
    let allowance = contact.closing_allowance;
    let arrives = -pre_solve_vn > allowance;
    if allowance > 0.0 && !(arrives && rebound > 0.0) {
        -allowance
    } else {
        rebound
    }
}
