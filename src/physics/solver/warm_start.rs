//! Warm-starting for PGS solvers.

use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::contact_row::ContactRow;
use super::solver_bodies::SolverBodies;

/// Apply cached impulse for a single contact and initialize its accumulated impulses.
///
/// `row` is `None` when a body of the pair is gone: the accumulators are still
/// initialised, and nothing is applied.
pub(crate) fn warm_start_contact(
    bodies: &mut SolverBodies,
    row: Option<&ContactRow>,
    header: &PairHeader,
    contact: &mut SolverContact,
    scale: f32,
) {
    contact.accumulated_normal_impulse = contact.warm_normal_impulse * scale;

    // Project cached world-space friction onto the current tangent plane
    // so that small normal drift does not rotate the friction direction.
    let warm_friction_scaled = contact.warm_friction_impulse_ws * scale;
    let projected =
        warm_friction_scaled - contact.normal * warm_friction_scaled.dot(&contact.normal);

    // Clamp to Coulomb limit with the warm normal impulse
    let mag = projected.magnitude();
    let max_friction = header.friction * contact.accumulated_normal_impulse;
    let friction_ws = if mag > max_friction && mag > 1e-8 {
        projected * (max_friction / mag)
    } else {
        projected
    };

    contact.accumulated_friction_impulse_ws = friction_ws;
    contact.accumulated_torsional_impulse = 0.0;

    if scale <= 0.0 {
        return;
    }
    if contact.warm_normal_impulse.abs() < 1e-8
        && contact.warm_friction_impulse_ws.magnitude_squared() < 1e-16
    {
        return;
    }

    let normal_impulse = contact.normal * contact.accumulated_normal_impulse;
    let total = normal_impulse + friction_ws;

    if let Some(row) = row {
        row.apply_impulse(bodies, total);
    }
}
