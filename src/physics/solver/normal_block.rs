//! A manifold's normal impulses, solved together and exactly.
//!
//! Every contact of a manifold acts on the same two bodies, so a push at one
//! changes the velocity at all the others. Solved one contact at a time, the
//! pushes converge at a rate set by how alike the contacts are: two contacts a
//! few centimetres apart under a tall body are nearly the same constraint and
//! take hundreds of passes to share its weight evenly. Stopped short, one of
//! them carries more than its share, and the difference is a torque that
//! nothing applied.
//!
//! A manifold has at most four contacts, so its pushes are found exactly
//! instead: the one set of impulses under which every contact either pushes
//! and meets its target velocity, or does not push and is already at or past
//! it.
//!
//! ```text
//!   contact rows ──measure (once per substep)──▶ NormalCoupling
//!                                                     │
//!   live velocities, targets ──▶ solve ◀──────────────┘
//!                                  │  tries sets of pushing contacts,
//!                                  │  last iteration's first
//!                                  ▼
//!                               impulses ──▶ SolverBodies
//! ```
//!
//! Manifolds are still solved one after another, iterated, so a stack
//! converges as before; only the contacts within a manifold are solved as one.

use nalgebra::{Matrix4, Vector3, Vector4};
use smallvec::SmallVec;

use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::contact_row::ContactRow;
use super::diagnostics::log_impulse_torque_diag;
use super::normal::{normal_target_velocity, MIN_EFFECTIVE_INV_MASS};
use super::solver_bodies::{SolverBodies, SolverBody};

/// Most contacts a block holds. Every contact generator keeps a manifold to
/// four.
pub(crate) const MAX_BLOCK_CONTACTS: usize = 4;

/// How much every contact's own response is strengthened, as a share of the
/// largest.
///
/// Four contacts on a flat face constrain only three motions — lift and two
/// tilts — so many splits of the load between them move the body alike, and
/// the exact equations have no one answer. A touch on the diagonal picks the
/// most even split, which keeps friction, limited at each contact by how hard
/// it pushes, the same at every corner.
///
/// The measured coupling carries rounding of about a ten-millionth of its
/// entries, and the split between the corners strays from even by that over
/// this: here, a thousandth. It is still far below the difference between
/// contacts that really are distinct: two eight centimetres apart under a
/// dodecahedron resist tilting with over two hundred times this.
const REGULARISATION: f32 = 1.0e-4;

/// How fast, in m/s, a contact that is not pushing may still be approaching
/// its target and have the solution accepted: rounding, not a real approach.
const APPROACH_TOLERANCE: f32 = 1.0e-5;

/// How an impulse at each contact of a manifold changes the normal velocity
/// at every contact of it, for one substep.
#[derive(Debug, Clone, Copy)]
pub(crate) struct NormalCoupling {
    /// How many contacts the block has.
    size: usize,
    /// Entry `(i, j)`: the change in contact `i`'s normal velocity per unit of
    /// normal impulse at contact `j`, regularised. Past `size` it is the
    /// identity, so every system solved from it is four by four.
    matrix: Matrix4<f32>,
}

impl NormalCoupling {
    /// Measure a manifold's coupling by applying a unit impulse through each
    /// contact's row, from rest, and reading the velocity it gives every
    /// contact. The rows' own arithmetic — shock scales, a kinematic body's
    /// override — is in it, so it cannot disagree with how they apply impulses.
    ///
    /// `None` for a single contact, which needs no block, for more contacts
    /// than a block holds, when a body of the pair is gone, or when a contact
    /// cannot be moved by an impulse at all.
    pub fn measure(
        rows: &[Option<ContactRow>],
        contacts: &[SolverContact],
        bodies: &mut SolverBodies,
    ) -> Option<Self> {
        let size = rows.len();
        if !(2..=MAX_BLOCK_CONTACTS).contains(&size) || rows.iter().any(Option::is_none) {
            return None;
        }
        let rows = rows.iter().flatten();

        // Every row of a manifold is between the same two bodies.
        let (slot_a, slot_b) = rows.clone().next()?.slots();
        let pair: SmallVec<[(usize, SolverBody); 2]> = slot_a
            .into_iter()
            .chain([slot_b])
            .map(|slot| (slot, *bodies.get(slot)))
            .collect();

        let mut matrix = Matrix4::identity();
        for (j, (pushed, contact)) in rows.clone().zip(contacts).enumerate() {
            for &(slot, _) in &pair {
                let body = bodies.get_mut(slot);
                body.linear_velocity = Vector3::zeros();
                body.angular_velocity = Vector3::zeros();
            }
            pushed.apply_impulse(bodies, contact.normal);
            for (i, (row, contact)) in rows.clone().zip(contacts).enumerate() {
                matrix[(i, j)] = row.relative_velocity(bodies).dot(&contact.normal);
            }
        }
        for (slot, body) in pair {
            *bodies.get_mut(slot) = body;
        }

        let largest = (0..size).map(|i| matrix[(i, i)]).fold(0.0, f32::max);
        if (0..size).any(|i| !(matrix[(i, i)] > MIN_EFFECTIVE_INV_MASS)) || !largest.is_finite() {
            return None;
        }
        // The same response measured both ways round; averaging drops the
        // rounding that tells them apart.
        let mut matrix = (matrix + matrix.transpose()) * 0.5;
        for i in 0..size {
            matrix[(i, i)] += largest * REGULARISATION;
        }
        Some(Self { size, matrix })
    }

    /// The impulses, one per contact, that leave each contact's velocity
    /// `w = matrix · impulses + bias` meeting three conditions: no impulse
    /// pulls (`impulses ≥ 0`), no contact is left approaching its target
    /// (`w ≥ 0`), and no contact both pushes and moves past its target.
    ///
    /// Tries sets of pushing contacts until one meets those conditions, the
    /// `hint` first. `None` if rounding leaves every set just short of them.
    fn solve(&self, bias: &Vector4<f32>, hint: u8) -> Option<Vector4<f32>> {
        let every_set = (0..1u8 << self.size).rev();
        std::iter::once(hint)
            .chain(every_set)
            .find_map(|pushing| self.solve_with(pushing, bias))
    }

    /// The impulses when exactly the contacts in `pushing` (a bit per
    /// contact) push, each to exactly its target, if that is consistent: none
    /// of them pulls, and none of the rest is left approaching its target.
    fn solve_with(&self, pushing: u8, bias: &Vector4<f32>) -> Option<Vector4<f32>> {
        let pushes = |i: usize| pushing & (1 << i) != 0;

        // The coupling among the pushing contacts, with every other contact
        // reduced to `impulse = 0`.
        let mut system = Matrix4::identity();
        let mut rhs = Vector4::zeros();
        for i in (0..self.size).filter(|&i| pushes(i)) {
            rhs[i] = -bias[i];
            for j in (0..self.size).filter(|&j| pushes(j)) {
                system[(i, j)] = self.matrix[(i, j)];
            }
        }
        let impulses = system.cholesky()?.solve(&rhs);

        let velocities = self.matrix * impulses + bias;
        let consistent = (0..self.size).all(|i| {
            if pushes(i) {
                impulses[i] >= 0.0
            } else {
                velocities[i] >= -APPROACH_TOLERANCE
            }
        });
        consistent.then_some(impulses)
    }
}

/// Solve a manifold's normal rows together, through its measured coupling.
///
/// Each contact aims for the same velocity its own row would, and keeps the
/// same accumulated impulse. `false`, with nothing applied, when no
/// consistent set of pushing contacts was found; the caller then solves the
/// rows one at a time.
pub(crate) fn solve_normal_block(
    bodies: &mut SolverBodies,
    rows: &[Option<ContactRow>],
    coupling: &NormalCoupling,
    header: &PairHeader,
    contacts: &mut [SolverContact],
    restitution_velocity_threshold: f32,
    pre_solve_vn: &[f32],
) -> bool {
    let rows: SmallVec<[&ContactRow; MAX_BLOCK_CONTACTS]> = rows.iter().flatten().collect();
    let mut accumulated = Vector4::zeros();
    let mut bias = Vector4::zeros();
    let mut hint = 0u8;
    for (i, (row, contact)) in rows.iter().zip(contacts.iter()).enumerate() {
        let velocity = row.relative_velocity(bodies).dot(&contact.normal);
        let mut target = normal_target_velocity(
            header,
            contact,
            restitution_velocity_threshold,
            pre_solve_vn[i],
        );
        // A contact already moving apart that holds no impulse is left alone,
        // as its own row leaves it: it is not pushed to leave faster.
        if velocity > 0.0 && contact.accumulated_normal_impulse <= 1e-8 {
            target = target.min(velocity);
        }
        accumulated[i] = contact.accumulated_normal_impulse;
        bias[i] = velocity - target;
        if accumulated[i] > 0.0 {
            hint |= 1 << i;
        }
    }
    // How far past its target each contact would be moving without the
    // impulses it already holds: those are taken back out, and the solve
    // finds the whole of each impulse afresh.
    let bias = bias - coupling.matrix * accumulated;

    let Some(impulses) = coupling.solve(&bias, hint) else {
        return false;
    };
    for (i, (row, contact)) in rows.iter().zip(contacts.iter_mut()).enumerate() {
        let applied = impulses[i] - contact.accumulated_normal_impulse;
        contact.accumulated_normal_impulse = impulses[i];
        if applied.abs() > 1e-10 {
            let impulse = contact.normal * applied;
            log_impulse_torque_diag("normal", header, contact, row, &impulse);
            row.apply_impulse(bodies, impulse);
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use generational_arena::Arena;
    use nalgebra::{Matrix3, Point3, Vector3};

    use super::*;
    use crate::collision::contact::FeatureId;
    use crate::physics::body::{RigidBody, RigidBodyDesc};
    use crate::physics::drive::TractionRow;
    use crate::physics::handle::RigidBodyHandle;
    use crate::physics::solver::contact_row::ContactRows;

    /// A unit-mass body at the origin over static ground, moving at
    /// `velocity` and turning at `spin`, with the given principal inertias.
    fn body_on_ground(
        inertia: Vector3<f32>,
        velocity: Vector3<f32>,
        spin: Vector3<f32>,
    ) -> (Arena<RigidBody>, PairHeader) {
        let mut bodies = Arena::new();
        let mut body = RigidBody::new(RigidBodyDesc::dynamic());
        body.set_mass_properties(1.0, Matrix3::from_diagonal(&inertia));
        body.set_linear_velocity(velocity);
        body.set_angular_velocity(spin);
        let index = bodies.insert(body);
        let header = PairHeader {
            body_a: None,
            body_b: RigidBodyHandle(index),
            collider_a: None,
            collider_b: None,
            restitution: 0.0,
            friction: 0.5,
        };
        (bodies, header)
    }

    /// A resting contact on level ground at `point`, holding no impulse yet.
    fn contact(point: Point3<f32>) -> SolverContact {
        SolverContact {
            point,
            normal: Vector3::y(),
            raw_normal: Vector3::y(),
            depth: 0.0,
            raw_depth: 0.0,
            gap: 0.0,
            closing_allowance: 0.0,
            feature_id: FeatureId(0),
            warm_normal_impulse: 0.0,
            warm_friction_impulse_ws: Vector3::zeros(),
            accumulated_normal_impulse: 0.0,
            accumulated_friction_impulse_ws: Vector3::zeros(),
            accumulated_torsional_impulse: 0.0,
            tangential_scale: 1.0,
            traction: TractionRow::default(),
        }
    }

    /// Solve the contacts' normal rows as one block, once, and write the
    /// result back to the body. Returns each contact's impulse.
    fn solve_block(
        bodies: &mut Arena<RigidBody>,
        header: &PairHeader,
        contacts: &mut [SolverContact],
    ) -> Vec<f32> {
        let mut solver_bodies = SolverBodies::default();
        let mut rows = ContactRows::default();
        rows.prepare(
            bodies,
            &mut solver_bodies,
            std::iter::once((header, &*contacts, (1.0, 1.0))),
        );
        let coupling = *rows.coupling(0).expect("a block of contacts");
        let solved = solve_normal_block(
            &mut solver_bodies,
            rows.manifold(0),
            &coupling,
            header,
            contacts,
            0.3,
            rows.pre_solve_normal_velocities(0),
        );
        assert!(solved, "no consistent set of pushing contacts");
        solver_bodies.scatter(bodies);
        contacts
            .iter()
            .map(|c| c.accumulated_normal_impulse)
            .collect()
    }

    /// The body's linear and angular velocity.
    fn motion(bodies: &Arena<RigidBody>, header: &PairHeader) -> (Vector3<f32>, Vector3<f32>) {
        let body = bodies.get(header.body_b.0).unwrap();
        (body.linear_velocity(), body.angular_velocity())
    }

    /// A box landing flat is stopped by its four corners equally: the face
    /// constrains only three motions, and the even split is the one taken.
    #[test]
    fn a_face_landing_flat_is_stopped_evenly_at_every_corner() {
        let (mut bodies, header) = body_on_ground(
            Vector3::repeat(1.0 / 6.0),
            Vector3::new(0.0, -1.0, 0.0),
            Vector3::zeros(),
        );
        let mut contacts: Vec<_> = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]
            .map(|(x, z)| contact(Point3::new(x, -0.5, z)))
            .into();

        let impulses = solve_block(&mut bodies, &header, &mut contacts);
        let (velocity, spin) = motion(&bodies, &header);

        for impulse in &impulses {
            assert!(
                (impulse - 0.25).abs() < 0.25e-3,
                "split further from even than a thousandth: {impulses:?}"
            );
        }
        assert!(velocity.norm() < 1e-4 && spin.norm() < 1e-4);
    }

    /// Two contacts eight centimetres apart under a body sixty centimetres
    /// tall — a dodecahedron balanced on a ridge — are nearly the same
    /// constraint. Solved one at a time they share its weight unevenly and
    /// set it turning; solved together, it stops without a turn.
    #[test]
    fn two_close_contacts_under_a_tall_body_stop_it_without_turning_it() {
        let (mut bodies, header) = body_on_ground(
            Vector3::repeat(0.123),
            Vector3::new(0.0, -0.05, 0.0),
            Vector3::zeros(),
        );
        let mut contacts = vec![
            contact(Point3::new(0.0, -0.589, -0.038)),
            contact(Point3::new(0.0, -0.589, 0.038)),
        ];

        let impulses = solve_block(&mut bodies, &header, &mut contacts);
        let (velocity, spin) = motion(&bodies, &header);

        assert!(
            (impulses[0] - impulses[1]).abs() < 1e-5,
            "uneven split: {impulses:?}"
        );
        assert!(velocity.norm() < 1e-5 && spin.norm() < 1e-5);
    }

    /// A box tipping onto one edge lands on that edge alone: the corners
    /// lifting away do not pull it back down.
    #[test]
    fn a_face_tipping_onto_an_edge_is_held_by_that_edge_alone() {
        // Falling and turning about z, so the +x edge comes down and the
        // -x edge goes up.
        let (mut bodies, header) = body_on_ground(
            Vector3::repeat(1.0 / 6.0),
            Vector3::new(0.0, -0.1, 0.0),
            Vector3::new(0.0, 0.0, -2.0),
        );
        let mut contacts: Vec<_> = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]
            .map(|(x, z)| contact(Point3::new(x, -0.5, z)))
            .into();

        let impulses = solve_block(&mut bodies, &header, &mut contacts);
        let (velocity, spin) = motion(&bodies, &header);

        assert!(
            impulses[0] == 0.0 && impulses[3] == 0.0,
            "the lifting edge pulled: {impulses:?}"
        );
        assert!(
            impulses[1] > 0.0 && impulses[2] > 0.0,
            "the landing edge did not push: {impulses:?}"
        );
        for c in &contacts {
            let lever = c.point.coords;
            let corner = velocity + spin.cross(&lever);
            assert!(
                corner.y > -1e-4,
                "a corner at {:?} is still coming down at {:.5} m/s",
                c.point,
                corner.y
            );
        }
    }

    /// The coupling is measured through the rows, so each contact's own
    /// response is the effective mass its row was prepared with.
    #[test]
    fn a_contacts_own_response_is_its_rows_effective_mass() {
        let (bodies, header) = body_on_ground(
            Vector3::new(0.2, 0.1, 0.3),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let contacts = vec![
            contact(Point3::new(0.3, -0.5, 0.1)),
            contact(Point3::new(-0.2, -0.5, 0.4)),
            contact(Point3::new(0.1, -0.5, -0.3)),
        ];
        let mut solver_bodies = SolverBodies::default();
        let mut rows = ContactRows::default();
        rows.prepare(
            &bodies,
            &mut solver_bodies,
            std::iter::once((&header, contacts.as_slice(), (1.0, 1.0))),
        );
        let coupling = rows.coupling(0).expect("a block of contacts");
        let largest = (0..3)
            .map(|i| rows.manifold(0)[i].unwrap().normal_inv_mass)
            .fold(0.0, f32::max);

        for (i, row) in rows.manifold(0).iter().enumerate() {
            let own = coupling.matrix[(i, i)] - largest * REGULARISATION;
            let expected = row.unwrap().normal_inv_mass;
            assert!(
                (own - expected).abs() < 1e-5 * expected,
                "contact {i}: measured {own}, row has {expected}"
            );
        }
    }
}
