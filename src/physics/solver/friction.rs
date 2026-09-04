//! Friction impulse resolution for contact constraints.

use generational_arena::Arena;
use nalgebra::Vector3;
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::body_pair::BodyPairState;
use super::diagnostics::log_impulse_torque_diag;
use super::impulse::{apply_impulse_pair, compute_tangent_basis};
use super::normal::MIN_EFFECTIVE_INV_MASS;

/// The tangential coefficient in force at one contact: the pair's combined
/// friction, scaled by what its participants are allowed to draw there.
fn tangential_coefficient(header: &PairHeader, contact: &SolverContact) -> f32 {
    header.friction * contact.tangential_scale
}

pub(crate) fn solve_friction_impulse(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    contact: &mut SolverContact,
    shock_scales: (f32, f32),
) {
    let mu = tangential_coefficient(header, contact);
    if contact.accumulated_normal_impulse <= 0.0 || mu <= 0.0 {
        contact.accumulated_friction_impulse_ws = Vector3::zeros();
        return;
    }

    let Some(state) = BodyPairState::extract(bodies, header, contact.point, shock_scales) else {
        return;
    };

    let rel_vel = state.relative_velocity_at(contact.point);
    let (t1, t2) = compute_tangent_basis(&contact.normal);

    let effective_mass_t1 = state.effective_inv_mass(contact.point, &t1);
    let effective_mass_t2 = state.effective_inv_mass(contact.point, &t2);
    if effective_mass_t1 <= MIN_EFFECTIVE_INV_MASS
        || !effective_mass_t1.is_finite()
        || effective_mass_t2 <= MIN_EFFECTIVE_INV_MASS
        || !effective_mass_t2.is_finite()
    {
        return;
    }

    // Decompose world-space accumulator into current tangent basis
    let curr_t1 = contact.accumulated_friction_impulse_ws.dot(&t1);
    let curr_t2 = contact.accumulated_friction_impulse_ws.dot(&t2);

    let delta_t1 = -rel_vel.dot(&t1) / effective_mass_t1;
    let delta_t2 = -rel_vel.dot(&t2) / effective_mass_t2;

    let mut new_t1 = curr_t1 + delta_t1;
    let mut new_t2 = curr_t2 + delta_t2;

    let max_friction = mu * contact.accumulated_normal_impulse;
    let mag = (new_t1 * new_t1 + new_t2 * new_t2).sqrt();
    if mag > max_friction {
        let scale = max_friction / mag;
        new_t1 *= scale;
        new_t2 *= scale;
    }

    let applied_t1 = new_t1 - curr_t1;
    let applied_t2 = new_t2 - curr_t2;

    // Reconstruct world-space accumulator from current basis
    contact.accumulated_friction_impulse_ws = t1 * new_t1 + t2 * new_t2;

    if applied_t1.abs() > 1e-10 || applied_t2.abs() > 1e-10 {
        let impulse = t1 * applied_t1 + t2 * applied_t2;
        log_impulse_torque_diag("friction", header, contact, &state, &impulse);
        apply_impulse_pair(bodies, header, contact.point, impulse, shock_scales);
    }
}

/// Enforce a shared friction budget across all contacts in a manifold.
///
/// After per-contact friction solve, the total friction effort (sum of individual
/// friction impulse magnitudes) must not exceed the sum of the per-contact
/// budgets `mu_i * lambda_n_i`. If it does, all contacts' friction impulses are
/// scaled down proportionally. This prevents individual contacts from each
/// maxing out their friction cones and producing oscillating net torque.
pub(crate) fn manifold_friction_projection(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    contacts: &mut SmallVec<[SolverContact; 4]>,
    shock_scales: (f32, f32),
) {
    let budget: f32 = contacts
        .iter()
        .map(|c| tangential_coefficient(header, c) * c.accumulated_normal_impulse.max(0.0))
        .sum();
    if budget <= 0.0 {
        return;
    }

    let total_friction_mag: f32 = contacts
        .iter()
        .map(|c| c.accumulated_friction_impulse_ws.magnitude())
        .sum();

    if total_friction_mag <= budget || total_friction_mag < 1e-8 {
        return;
    }

    let scale = budget / total_friction_mag;
    for contact in contacts.iter_mut() {
        let old = contact.accumulated_friction_impulse_ws;
        contact.accumulated_friction_impulse_ws = old * scale;
        let delta = contact.accumulated_friction_impulse_ws - old;
        if delta.magnitude_squared() > 1e-20 {
            apply_impulse_pair(bodies, header, contact.point, delta, shock_scales);
        }
    }
}
