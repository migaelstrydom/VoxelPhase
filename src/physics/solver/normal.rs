//! Normal impulse resolution for contact constraints.

use generational_arena::Arena;
use nalgebra::Matrix3;

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::body_pair::{is_kinematic_static, BodyPairState};
use super::diagnostics::log_impulse_torque_diag;
use super::impulse::apply_impulse_pair;

/// Minimum effective inverse mass allowed in the normal solver.
///
/// Prevents near-zero denominators (degenerate geometry or deeply wedged bodies)
/// from producing runaway impulses.
pub(crate) const MIN_EFFECTIVE_INV_MASS: f32 = 1.0e-8;

pub(crate) fn solve_normal_impulse(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    contact: &mut SolverContact,
    restitution_velocity_threshold: f32,
    pre_solve_vn: f32,
    shock_scales: (f32, f32),
) {
    let Some(state) = BodyPairState::extract(bodies, header, contact.point, shock_scales) else {
        return;
    };

    let vel_along_normal = state.relative_normal_velocity(contact.point, &contact.normal);
    if vel_along_normal > 0.0 && contact.accumulated_normal_impulse <= 1e-8 {
        return;
    }

    // Kinematic-vs-static contacts use fake unit mass so the normal solver
    // can push the kinematic body out of static geometry. Friction does NOT
    // use this override — kinematic bodies should move freely along surfaces.
    let kinematic_static = bodies
        .get(header.body_b.0)
        .is_some_and(|b| is_kinematic_static(b, header));

    let (eff_inv_mass_b, eff_inv_inertia_b) = if kinematic_static {
        (1.0, Matrix3::zeros())
    } else {
        (state.inv_mass_b, state.inv_inertia_b)
    };

    let effective_inv_mass = state.effective_inv_mass_with_overrides(
        contact.point,
        &contact.normal,
        eff_inv_mass_b,
        eff_inv_inertia_b,
    );
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
        log_impulse_torque_diag("normal", header, contact, &state, &impulse);
        apply_impulse_pair(bodies, header, contact.point, impulse, shock_scales);
    }
}
