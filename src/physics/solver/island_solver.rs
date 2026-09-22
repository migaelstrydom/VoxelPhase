//! The velocity phase of one island, self-contained so islands can run on
//! separate threads.
//!
//! ```text
//!   &Arena<RigidBody> ──gather──▶ IslandSolver ──▶ warm start ──▶ iterations
//!   (shared, read-only)            own SolverBodies,               │
//!                                  ContactRows, joint slots        ▼
//!                                                    velocities, scattered to
//!                                                    the arena after every
//!                                                    island is done
//! ```
//!
//! An island touches no movable body another island touches, so what it
//! reads from the arena is not changed by any other island's solve, and its
//! results do not depend on which island runs first or on which thread.

use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::constraint::types::ConstraintRow;
use crate::physics::pipeline::pair::SolverManifold;

use super::conditioning::ManifoldConditions;
use super::constraint_row::{solve_constraint_row, warm_start_constraint_row, RowSlots};
use super::contact_row::ContactRows;
use super::friction::{manifold_friction_projection, solve_friction_impulse};
use super::iteration_budget::IterationBudget;
use super::normal::solve_normal_impulse;
use super::pgs_ngs::PgsNgsConfig;
use super::solver_bodies::SolverBodies;
use super::torsional::solve_torsional_impulse;
use super::warm_start::warm_start_contact;

/// A manifold handed to an island, with its index in the substep's manifold
/// slice — the index its manifold conditions are kept under.
pub(crate) type IslandManifold<'a> = (usize, &'a mut SolverManifold);

/// Working state for solving one island's velocities. Kept between substeps
/// so its buffers are reused.
#[derive(Debug, Default)]
pub(crate) struct IslandSolver {
    /// Velocities of every body the island's rows touch.
    solver_bodies: SolverBodies,
    /// The island's contact rows, in the order of its manifolds.
    contact_rows: ContactRows,
    /// Solver slots of each of the island's joint rows' bodies.
    constraint_slots: Vec<RowSlots>,
    /// The island's contact measurements, which set its iteration count.
    budget: IterationBudget,
}

impl IslandSolver {
    /// Warm-start and iterate the island's rows. The resulting velocities stay
    /// in the island solver until [`IslandSolver::scatter`].
    pub fn solve(
        &mut self,
        bodies: &Arena<RigidBody>,
        manifolds: &mut [IslandManifold],
        constraint_rows: &mut [&mut ConstraintRow],
        conditions: &ManifoldConditions,
        config: &PgsNgsConfig,
    ) {
        self.prepare(bodies, manifolds, constraint_rows, conditions);
        self.warm_start(manifolds, constraint_rows, config);

        self.budget.measure(
            manifolds.iter().map(|(_, m)| &**m),
            &self.contact_rows,
            self.solver_bodies.len(),
        );
        for _ in 0..self.budget.iterations(config.solver_iterations) {
            self.iterate(manifolds, constraint_rows, config);
        }
    }

    /// Write the island's velocities back to the arena.
    pub fn scatter(&self, bodies: &mut Arena<RigidBody>) {
        self.solver_bodies.scatter(bodies);
    }

    /// Gather the bodies every row touches, prepare every contact's rows, and
    /// capture the pre-solve normal velocities restitution and warm-starting
    /// are decided from.
    fn prepare(
        &mut self,
        bodies: &Arena<RigidBody>,
        manifolds: &[IslandManifold],
        constraint_rows: &[&mut ConstraintRow],
        conditions: &ManifoldConditions,
    ) {
        let solver_bodies = &mut self.solver_bodies;
        solver_bodies.clear();
        self.contact_rows.prepare(
            bodies,
            solver_bodies,
            manifolds.iter().map(|(mi, m)| {
                (
                    &m.header,
                    m.contacts.as_slice(),
                    conditions.shock_scales_for(*mi),
                )
            }),
        );

        self.constraint_slots.clear();
        self.constraint_slots
            .extend(constraint_rows.iter().map(|row| {
                (
                    row.body_a
                        .and_then(|handle| solver_bodies.gather(bodies, handle.0)),
                    row.body_b
                        .and_then(|handle| solver_bodies.gather(bodies, handle.0)),
                )
            }));
    }

    /// Apply the impulses cached from the previous frame: joints first, then
    /// contacts, the order they are solved in.
    fn warm_start(
        &mut self,
        manifolds: &mut [IslandManifold],
        constraint_rows: &mut [&mut ConstraintRow],
        config: &PgsNgsConfig,
    ) {
        let solver_bodies = &mut self.solver_bodies;
        for (row, &slots) in constraint_rows.iter_mut().zip(&self.constraint_slots) {
            warm_start_constraint_row(solver_bodies, slots, row, config.warm_start_scale);
        }
        for (i, (_, manifold)) in manifolds.iter_mut().enumerate() {
            let header = &manifold.header;
            let rows = self.contact_rows.manifold(i);
            let pre_solve_vn = self.contact_rows.pre_solve_normal_velocities(i);
            for (ci, contact) in manifold.contacts.iter_mut().enumerate() {
                let warm_scale = if pre_solve_vn[ci].abs() > config.restitution_velocity_threshold {
                    0.0
                } else {
                    config.warm_start_scale
                };
                warm_start_contact(
                    solver_bodies,
                    rows[ci].as_ref(),
                    header,
                    contact,
                    warm_scale,
                );
            }
        }
    }

    /// One sequential-impulse pass over every row of the island.
    fn iterate(
        &mut self,
        manifolds: &mut [IslandManifold],
        constraint_rows: &mut [&mut ConstraintRow],
        config: &PgsNgsConfig,
    ) {
        let solver_bodies = &mut self.solver_bodies;

        // Joint constraints first — solved early so contacts get the last
        // word for penetration prevention.
        for (row, &slots) in constraint_rows.iter_mut().zip(&self.constraint_slots) {
            solve_constraint_row(solver_bodies, slots, row);
        }

        for (i, (_, manifold)) in manifolds.iter_mut().enumerate() {
            let rows = self.contact_rows.manifold(i);
            let pre_solve_vn = self.contact_rows.pre_solve_normal_velocities(i);

            // Block normal solve: multi-contact manifolds get extra local
            // iterations to capture cross-contact coupling.
            let normal_passes = if manifold.contacts.len() > 1 {
                config.block_normal_micro_iterations
            } else {
                1
            };
            for _ in 0..normal_passes {
                for ci in 0..manifold.contacts.len() {
                    solve_normal_impulse(
                        solver_bodies,
                        rows[ci].as_ref(),
                        &manifold.header,
                        &mut manifold.contacts[ci],
                        config.restitution_velocity_threshold,
                        pre_solve_vn[ci],
                    );
                }
            }

            for ci in 0..manifold.contacts.len() {
                solve_friction_impulse(
                    solver_bodies,
                    rows[ci].as_ref(),
                    &manifold.header,
                    &mut manifold.contacts[ci],
                );
            }

            // Inert unless a body driving through the contact declared a
            // patch for it to bear on.
            for ci in 0..manifold.contacts.len() {
                solve_torsional_impulse(
                    solver_bodies,
                    rows[ci].as_ref(),
                    &manifold.header,
                    &mut manifold.contacts[ci],
                );
            }

            // Manifold-level friction budget projection
            if manifold.contacts.len() > 1 {
                manifold_friction_projection(
                    solver_bodies,
                    rows,
                    &manifold.header,
                    &mut manifold.contacts,
                );
            }
        }
    }
}
