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
    is_persisted: bool,
    shock_scales: (f32, f32),
) {
    let Some(state) = BodyPairState::extract(bodies, header, contact.point, shock_scales) else {
        return;
    };

    // Sanity check: for dynamic-dynamic pairs, the contact normal should
    // roughly point from body_a toward body_b (the solver convention).
    // A flipped normal inverts the impulse direction, causing penetration
    // instead of separation. Threshold is generous (-0.5 ≈ 120°) to avoid
    // false positives from edge/corner contacts where the normal is
    // perpendicular to the center-to-center axis.
    debug_assert!(
        header.body_a.is_none() || {
            let ab = state.pos_b - state.pos_a;
            let ab_len_sq = ab.magnitude_squared();
            ab_len_sq < 1e-6 || contact.normal.dot(&ab) / ab_len_sq.sqrt() > -0.5
        },
        "Contact normal appears to point from B toward A (dot={:.3}). \
         This usually means the body_a/body_b ordering in PairHeader \
         doesn't match the manifold's normal convention.",
        {
            let ab = state.pos_b - state.pos_a;
            let len = ab.magnitude();
            if len > 1e-6 {
                contact.normal.dot(&ab) / len
            } else {
                0.0
            }
        }
    );

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

    // Persisted contacts suppress restitution: the high approach velocity
    // is from an external velocity drive, not a new impact.
    let restitution = if is_persisted {
        0.0
    } else {
        let speed = pre_solve_vn.abs();
        let restitution_scale = ((speed - restitution_velocity_threshold)
            / restitution_velocity_threshold)
            .clamp(0.0, 1.0);
        header.restitution * restitution_scale
    };
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
