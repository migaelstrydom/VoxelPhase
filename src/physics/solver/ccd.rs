//! Contact solving for transient CCD contacts (no warm-starting).

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::SolverManifold;
use generational_arena::Arena;

use super::contact_row::ContactRow;
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
            let row = ContactRow::prepare(bodies, header, contact, no_shock);
            let pre_solve_vn = row.map_or(0.0, |row| {
                row.relative_velocity(bodies).dot(&contact.normal)
            });
            solve_normal_impulse(
                bodies,
                row.as_ref(),
                header,
                contact,
                restitution_velocity_threshold,
                pre_solve_vn,
            );
            // A swept contact is built mid-substep, after the frame's plan,
            // so its traction row is the default: hold still, honest budget.
            solve_friction_impulse(bodies, row.as_ref(), header, contact);
        }
    }
}
