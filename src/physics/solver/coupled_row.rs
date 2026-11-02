//! Coupled rows: a joint row bounded by a sibling row's live impulse.
//!
//! Friction is bounded by load, and a joint's load is another of its rows, so
//! the bound cannot be fixed when the rows are expanded: it moves with the
//! load row's accumulated impulse inside the iteration loop. A row carrying a
//! `CoupledBound` names that sibling by `row_index`; the island resolves it
//! to a slot once per substep ([`resolve_coupled_slots`]), and the row is then
//! solved against the sibling's impulse as it stands.
//!
//! ```text
//!   load row (solved first) ──λ_load──┐
//!                                     ├──► |λ + λ_load·share| ≤ μ·λ_load
//!   coupled row(s) ──λ (+ partner)────┘      one row: a clamp
//!                                            a pair:  the Coulomb cone (friction.rs)
//! ```
//!
//! A pair naming each other through `cone_with` is solved together, from the
//! velocities before either is applied, as a contact's two tangent rows are.

use crate::physics::constraint::types::ConstraintRow;

use super::constraint_row::{apply_constraint_impulse, jacobian_dot_velocity, RowSlots};
use super::friction::project_onto_cone;
use super::solver_bodies::SolverBodies;

/// Where a coupled row's siblings sit in its island's row slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CoupledSlots {
    /// Slot of the row whose impulse is the load.
    pub load: usize,
    /// Slot of the row sharing this one's cone, if any.
    pub cone_partner: Option<usize>,
}

/// Resolve every row's coupling to slots in `rows`. `None` for a row with no
/// coupling, and for one whose load row is not in the slice, which is then
/// solved against its own fixed bounds.
pub(crate) fn resolve_coupled_slots<'a>(
    rows: &'a [&mut ConstraintRow],
) -> impl Iterator<Item = Option<CoupledSlots>> + 'a {
    rows.iter().map(move |row| {
        let coupling = row.coupling?;
        let find = |row_index: u32| {
            rows.iter().position(|sibling| {
                sibling.constraint_index == row.constraint_index
                    && sibling.row_index == row_index as usize
            })
        };
        Some(CoupledSlots {
            load: find(coupling.coupled_to)?,
            cone_partner: coupling.cone_with.and_then(find),
        })
    })
}

/// Solve the coupled row at `index`, with its cone partner if it has one.
///
/// A row whose partner sits earlier in the slice was solved with it and is
/// skipped.
pub(crate) fn solve_coupled_row(
    bodies: &mut SolverBodies,
    slots: &[RowSlots],
    rows: &mut [&mut ConstraintRow],
    index: usize,
    coupled: CoupledSlots,
) {
    let Some(bound) = rows[index].coupling else {
        return;
    };
    let load = rows[coupled.load].accumulated_impulse;
    let radius = bound.radius(load);

    match coupled.cone_partner {
        Some(partner) if partner < index => {}
        Some(partner) => {
            let delta_first = unbounded_delta(bodies, slots[index], rows[index]);
            let delta_second = unbounded_delta(bodies, slots[partner], rows[partner]);
            let share_first = load * bound.load_share;
            let share_second = load * load_share(rows[partner]);
            let cone = project_onto_cone(
                rows[index].accumulated_impulse + delta_first + share_first,
                rows[partner].accumulated_impulse + delta_second + share_second,
                radius,
            );
            set_accumulated(bodies, slots[index], rows[index], cone.t1 - share_first);
            set_accumulated(
                bodies,
                slots[partner],
                rows[partner],
                cone.t2 - share_second,
            );
        }
        None => {
            let delta = unbounded_delta(bodies, slots[index], rows[index]);
            let share = load * bound.load_share;
            let total = (rows[index].accumulated_impulse + delta + share).clamp(-radius, radius);
            set_accumulated(bodies, slots[index], rows[index], total - share);
        }
    }
}

/// Warm-start the coupled row at `index`, with its cone partner if it has
/// one: the cached impulse is scaled, held inside the bound of the load row's
/// own warm impulse, and applied. The load row is warm-started first, being
/// earlier in the slice, so a friction warm impulse never outgrows the load
/// it is warm-started against.
pub(crate) fn warm_start_coupled_row(
    bodies: &mut SolverBodies,
    slots: &[RowSlots],
    rows: &mut [&mut ConstraintRow],
    index: usize,
    coupled: CoupledSlots,
    scale: f32,
) {
    let Some(bound) = rows[index].coupling else {
        return;
    };
    let load = rows[coupled.load].accumulated_impulse;
    let radius = bound.radius(load);

    let mut held = [(index, 0.0f32); 2];
    let count = match coupled.cone_partner {
        Some(partner) if partner < index => return,
        Some(partner) => {
            let share_first = load * bound.load_share;
            let share_second = load * load_share(rows[partner]);
            let cone = project_onto_cone(
                rows[index].accumulated_impulse * scale + share_first,
                rows[partner].accumulated_impulse * scale + share_second,
                radius,
            );
            held[0] = (index, cone.t1 - share_first);
            held[1] = (partner, cone.t2 - share_second);
            2
        }
        None => {
            let share = load * bound.load_share;
            let total = (rows[index].accumulated_impulse * scale + share).clamp(-radius, radius);
            held[0] = (index, total - share);
            1
        }
    };
    for &(slot, impulse) in &held[..count] {
        rows[slot].accumulated_impulse = impulse;
        if impulse.abs() > 1e-12 {
            apply_constraint_impulse(bodies, slots[slot], rows[slot], impulse);
        }
    }
}

/// The load row's share along `row`'s own direction: each row of a pair
/// carries its own.
fn load_share(row: &ConstraintRow) -> f32 {
    row.coupling.map_or(0.0, |bound| bound.load_share)
}

/// The impulse that would satisfy the row with no bound at all.
fn unbounded_delta(bodies: &SolverBodies, slots: RowSlots, row: &ConstraintRow) -> f32 {
    let cdot = jacobian_dot_velocity(bodies, slots, row);
    row.effective_mass * -(cdot + row.bias + row.softness * row.accumulated_impulse)
}

/// Move the row's accumulated impulse to `accumulated`, applying the
/// difference to its bodies.
fn set_accumulated(
    bodies: &mut SolverBodies,
    slots: RowSlots,
    row: &mut ConstraintRow,
    accumulated: f32,
) {
    let delta = accumulated - row.accumulated_impulse;
    row.accumulated_impulse = accumulated;
    if delta.abs() > 1e-12 {
        apply_constraint_impulse(bodies, slots, row, delta);
    }
}

#[cfg(test)]
mod tests {
    use generational_arena::{Arena, Index};
    use nalgebra::{Matrix3, Vector3};

    use super::*;
    use crate::physics::body::{RigidBody, RigidBodyDesc};
    use crate::physics::constraint::types::{CorrectionMode, CoupledBound, Enforcement, RowKind};

    const MASS: f32 = 2.0;
    const LOAD: f32 = 3.0;
    const MU: f32 = 0.5;

    /// A row on one body, world on the other side, acting through its centre
    /// of mass along `axis`.
    fn row(axis: Vector3<f32>, row_index: usize, coupling: Option<CoupledBound>) -> ConstraintRow {
        ConstraintRow {
            body_a: None,
            body_b: Some(crate::physics::handle::RigidBodyHandle(
                Index::from_raw_parts(0, 0),
            )),
            lin_jac_a: Vector3::zeros(),
            ang_jac_a: Vector3::zeros(),
            lin_jac_b: axis,
            ang_jac_b: Vector3::zeros(),
            effective_mass: MASS,
            bias: 0.0,
            softness: 0.0,
            accumulated_impulse: 0.0,
            bounds: (0.0, 0.0),
            constraint_index: Index::from_raw_parts(0, 0),
            row_index,
            correction_mode: CorrectionMode::VelocityOnly,
            row_kind: RowKind::Linear,
            enforcement: Enforcement::Iterative,
            refresh: None,
            coupling,
        }
    }

    fn friction(partner: u32, load_share: f32) -> CoupledBound {
        CoupledBound {
            coupled_to: 0,
            cone_with: Some(partner),
            coefficient: MU,
            load_share,
        }
    }

    /// A load row holding `LOAD` and a friction pair along x and z coupled to
    /// it, on one body moving at `velocity`.
    struct Rig {
        bodies: SolverBodies,
        slots: Vec<RowSlots>,
        rows: Vec<ConstraintRow>,
    }

    impl Rig {
        fn new(velocity: Vector3<f32>, shares: (f32, f32)) -> Self {
            let mut arena = Arena::new();
            let mut body = RigidBody::new(RigidBodyDesc::dynamic().linear_velocity(velocity));
            body.set_mass_properties(MASS, Matrix3::identity());
            let index = arena.insert(body);
            let mut bodies = SolverBodies::default();
            let slot = bodies.gather(&arena, index);
            let mut load = row(Vector3::y(), 0, None);
            load.accumulated_impulse = LOAD;
            let rows = vec![
                load,
                row(Vector3::x(), 1, Some(friction(2, shares.0))),
                row(Vector3::z(), 2, Some(friction(1, shares.1))),
            ];
            Self {
                bodies,
                slots: vec![(None, slot); 3],
                rows,
            }
        }

        fn coupled(&mut self) -> Vec<Option<CoupledSlots>> {
            let refs: Vec<&mut ConstraintRow> = self.rows.iter_mut().collect();
            resolve_coupled_slots(&refs).collect()
        }

        fn solve(&mut self) {
            let coupled = self.coupled();
            let mut refs: Vec<&mut ConstraintRow> = self.rows.iter_mut().collect();
            for index in 1..3 {
                solve_coupled_row(
                    &mut self.bodies,
                    &self.slots,
                    &mut refs,
                    index,
                    coupled[index].unwrap(),
                );
            }
        }

        fn warm_start(&mut self, scale: f32) {
            let coupled = self.coupled();
            let mut refs: Vec<&mut ConstraintRow> = self.rows.iter_mut().collect();
            for index in 1..3 {
                warm_start_coupled_row(
                    &mut self.bodies,
                    &self.slots,
                    &mut refs,
                    index,
                    coupled[index].unwrap(),
                    scale,
                );
            }
        }

        fn friction(&self) -> Vector3<f32> {
            Vector3::new(
                self.rows[1].accumulated_impulse,
                0.0,
                self.rows[2].accumulated_impulse,
            )
        }

        fn velocity(&self) -> Vector3<f32> {
            self.bodies.get(0).linear_velocity
        }
    }

    #[test]
    fn siblings_resolve_to_their_slots() {
        let mut rig = Rig::new(Vector3::zeros(), (0.0, 0.0));
        let coupled = rig.coupled();
        assert_eq!(coupled[0], None);
        assert_eq!(
            coupled[1],
            Some(CoupledSlots {
                load: 0,
                cone_partner: Some(2)
            })
        );
        assert_eq!(
            coupled[2],
            Some(CoupledSlots {
                load: 0,
                cone_partner: Some(1)
            })
        );
    }

    /// A body sliding fast enough to slip is slowed by exactly `μ·N`,
    /// straight against its motion, along a tangent and on every diagonal
    /// alike: a disc, not the square two separately clamped rows would make,
    /// which would grip √2 harder at 45°.
    #[test]
    fn a_coupled_pair_slips_at_mu_n_in_any_direction() {
        for degrees in [0.0f32, 30.0, 45.0, 90.0, 135.0, 225.0, 300.0] {
            let angle = degrees.to_radians();
            let direction = Vector3::new(angle.cos(), 0.0, angle.sin());
            let mut rig = Rig::new(direction * 10.0, (0.0, 0.0));
            rig.solve();

            let impulse = rig.friction();
            let bound = MU * LOAD;
            assert!(
                (impulse.norm() - bound).abs() < 1e-5 * bound,
                "at {degrees}° the pair gripped {} of {bound}",
                impulse.norm()
            );
            assert!(
                impulse.normalize().dot(&-direction) > 1.0 - 1e-6,
                "at {degrees}° the grip is not against the motion: {impulse:?}"
            );
            let expected = direction * (10.0 - bound / MASS);
            assert!((rig.velocity() - expected).norm() < 1e-5);
        }
    }

    /// Within the bound the pair holds: the body stops.
    #[test]
    fn within_the_bound_a_coupled_pair_holds() {
        let mut rig = Rig::new(Vector3::new(0.3, 0.0, -0.2), (0.0, 0.0));
        rig.solve();
        assert!(rig.velocity().norm() < 1e-6, "{:?}", rig.velocity());
    }

    /// The bound covers the load row's own share along the surface: a load
    /// whose sideways part already exceeds `μ·N` is a slope steeper than
    /// `atan μ`, and the pair must push back against it rather than add to it.
    #[test]
    fn the_cone_bounds_the_load_rows_share_too() {
        let mut rig = Rig::new(Vector3::zeros(), (0.6, 0.0));
        rig.solve();
        let total = rig.rows[1].accumulated_impulse + LOAD * 0.6;
        assert!((total - MU * LOAD).abs() < 1e-5, "total {total}");
        assert!(rig.rows[2].accumulated_impulse.abs() < 1e-6);

        let mut held = Rig::new(Vector3::zeros(), (0.4, 0.0));
        held.solve();
        assert!(held.rows[1].accumulated_impulse.abs() < 1e-6);
    }

    /// A friction warm impulse cached under a heavier load is clamped to the
    /// cone of the load it is warm-started against.
    #[test]
    fn a_warm_impulse_is_clamped_to_the_cone_of_the_loads() {
        let mut rig = Rig::new(Vector3::zeros(), (0.0, 0.0));
        rig.rows[1].accumulated_impulse = 4.0;
        rig.rows[2].accumulated_impulse = 4.0;
        rig.warm_start(1.0);

        let impulse = rig.friction();
        assert!((impulse.norm() - MU * LOAD).abs() < 1e-5, "{impulse:?}");
        assert!((impulse.x - impulse.z).abs() < 1e-6);
        assert!((rig.velocity() - impulse / MASS).norm() < 1e-6);
    }
}
