//! Contact solving for transient CCD contacts (no warm-starting).

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::SolverManifold;

use super::body_pair::BodyPairState;
use super::friction::solve_friction_impulse;
use super::normal::solve_normal_impulse;

/// Solve contacts without warm-starting (for transient contacts like CCD).
pub(crate) fn solve_contacts(
    bodies: &mut Arena<RigidBody>,
    manifolds: &mut [SolverManifold],
    restitution_velocity_threshold: f32,
) {
    let no_shock = (1.0, 1.0);
    for manifold in manifolds.iter_mut() {
        let header = &manifold.header;
        for contact in manifold.contacts.iter_mut() {
            let pre_solve_vn = BodyPairState::extract(bodies, header, contact.point, no_shock)
                .map(|state| state.relative_normal_velocity(contact.point, &contact.normal))
                .unwrap_or(0.0);
            solve_normal_impulse(
                bodies,
                header,
                contact,
                restitution_velocity_threshold,
                pre_solve_vn,
                false,
                no_shock,
            );
            solve_friction_impulse(bodies, header, contact, &Vector3::zeros(), no_shock);
        }
    }
}
