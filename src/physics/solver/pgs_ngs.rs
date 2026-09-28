//! PGS + NGS solver: Projected Gauss-Seidel velocity solve with
//! nonlinear Gauss-Seidel position correction.

use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use smallvec::SmallVec;

use generational_arena::{Arena, Index};
use nalgebra::{Point3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::constraint::expand::{
    check_constraint_breakage, expand_constraints, write_back_constraints,
};
use crate::physics::constraint::types::{Constraint, ConstraintRow, Enforcement, RowKind};
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

use super::closing_allowance::set_closing_allowances;
use super::conditioning::ManifoldConditions;
use super::island_solver::{IslandManifold, IslandSolver};
use super::position_correction::{self, PositionCorrectionConfig};
use super::solver_islands::SolverIslands;
use super::ConstraintSolver;

/// Configuration for the PGS+NGS solver.
#[derive(Debug, Clone)]
pub struct PgsNgsConfig {
    /// Number of velocity solver iterations per step.
    pub solver_iterations: u32,
    /// Minimum approach speed for restitution to apply. Below this threshold,
    /// restitution is zeroed to prevent micro-bouncing at resting contacts.
    pub restitution_velocity_threshold: f32,
    /// Scale factor applied to warm-start impulses (0..=1).
    pub warm_start_scale: f32,
    /// Number of PGS micro-iterations for block normal solve on multi-contact
    /// manifolds. Extra local iterations capture cross-contact coupling within
    /// a single outer solver pass, reducing rocking in stacks and eccentric loads.
    pub block_normal_micro_iterations: u32,
    /// Position correction configuration.
    pub position_correction: PositionCorrectionConfig,
    /// Position correction factor for joint constraints (beta).
    /// Controls how aggressively constraint drift is corrected via Baumgarte
    /// bias and KeepUpright hard projection.
    pub constraint_position_beta: f32,
}

impl Default for PgsNgsConfig {
    fn default() -> Self {
        Self {
            solver_iterations: 3,
            restitution_velocity_threshold: 0.3,
            warm_start_scale: 0.6,
            block_normal_micro_iterations: 4,
            position_correction: PositionCorrectionConfig::default(),
            constraint_position_beta: 0.2,
        }
    }
}

/// PGS+NGS contact constraint solver.
///
/// Velocity phase: warm-started Projected Gauss-Seidel with accumulated impulse
/// clamping, block normal micro-iterations, and manifold-level friction projection.
///
/// Position phase: nonlinear Gauss-Seidel (NGS) direct position correction,
/// or Baumgarte stabilization as a fallback.
pub struct PgsNgsSolver {
    config: PgsNgsConfig,
    /// Body positions at the time contacts were generated. Used as the reference
    /// for position correction across multiple substeps so that stale
    /// `contact.depth` values don't cause re-correction.
    contact_generation_positions: FxHashMap<Index, Point3<f32>>,
    /// Solver-ready constraint rows, expanded once per frame in `prepare`.
    /// Reused across substeps within the same frame.
    cached_constraint_rows: Vec<ConstraintRow>,
    /// Rows grouped by the movable bodies they share, rebuilt every `solve`.
    islands: SolverIslands,
    /// One velocity-phase workspace per island, kept to reuse its buffers.
    island_solvers: Vec<IslandSolver>,
}

impl PgsNgsSolver {
    pub fn new(config: PgsNgsConfig) -> Self {
        Self {
            config,
            contact_generation_positions: FxHashMap::default(),
            cached_constraint_rows: Vec::new(),
            islands: SolverIslands::default(),
            island_solvers: Vec::new(),
        }
    }
}

impl Default for PgsNgsSolver {
    fn default() -> Self {
        Self::new(PgsNgsConfig::default())
    }
}

impl ConstraintSolver for PgsNgsSolver {
    fn prepare(
        &mut self,
        bodies: &Arena<RigidBody>,
        constraints: &Arena<Constraint>,
        sleeping: Option<&FxHashSet<RigidBodyHandle>>,
        dt: f32,
    ) {
        self.contact_generation_positions.clear();
        for (idx, body) in bodies.iter() {
            if !body.is_static() {
                self.contact_generation_positions
                    .insert(idx, body.position());
            }
        }

        expand_constraints(
            constraints,
            bodies,
            sleeping,
            dt,
            self.config.constraint_position_beta,
            &mut self.cached_constraint_rows,
        );
    }

    fn solve(
        &mut self,
        bodies: &mut Arena<RigidBody>,
        manifolds: &mut [SolverManifold],
        conditions: &ManifoldConditions,
        constraints: &Arena<Constraint>,
        dt: f32,
    ) {
        let constraint_rows = &mut self.cached_constraint_rows;

        if manifolds.is_empty() && constraint_rows.is_empty() {
            return;
        }

        set_closing_allowances(bodies, manifolds, &self.contact_generation_positions, dt);

        // Phases 1–3: the velocity phase, island by island. Islands share no
        // movable body, so they run in parallel, each gathering, warm-starting
        // and iterating on its own copy of its bodies; their velocities reach
        // the arena only once every island is done.
        let islands = &mut self.islands;
        islands.build(
            bodies,
            manifolds
                .iter()
                .map(|m| (m.header.body_a, Some(m.header.body_b))),
            constraint_rows.iter().map(|row| (row.body_a, row.body_b)),
        );
        if self.island_solvers.len() < islands.len() {
            self.island_solvers
                .resize_with(islands.len(), IslandSolver::default);
        }
        let island_solvers = &mut self.island_solvers[..islands.len()];

        let mut arranged_manifolds: Vec<IslandManifold> =
            islands.arrange_manifolds(manifolds.iter_mut().enumerate());
        let mut arranged_constraints: Vec<&mut ConstraintRow> =
            islands.arrange_constraints(constraint_rows.iter_mut());
        let work: Vec<_> = island_solvers
            .iter_mut()
            .zip(islands.split_manifolds(&mut arranged_manifolds))
            .zip(islands.split_constraints(&mut arranged_constraints))
            .collect();
        let arena: &Arena<RigidBody> = bodies;
        let config = &self.config;
        work.into_par_iter()
            .for_each(|((solver, manifolds), constraints)| {
                solver.solve(arena, manifolds, constraints, conditions, config);
            });
        drop(arranged_manifolds);
        drop(arranged_constraints);

        for solver in island_solvers.iter() {
            solver.scatter(bodies);
        }

        // Phase 4: Position correction after velocity solving.
        // NGS uses real masses (no shock propagation) — position correction
        // is linear-only and doesn't benefit from mass scaling.
        position_correction::apply_position_correction(
            bodies,
            manifolds,
            constraints,
            constraint_rows,
            &self.config.position_correction,
            dt,
            &self.contact_generation_positions,
        );
    }

    fn write_back(&self, constraints: &mut Arena<Constraint>) {
        write_back_constraints(constraints, &self.cached_constraint_rows);
        check_constraint_breakage(constraints, &self.cached_constraint_rows);
    }

    fn project_velocities(&self, bodies: &mut Arena<RigidBody>, dt: f32) {
        let beta = self.config.constraint_position_beta;

        // Collect HardProjection angular rows grouped by constraint index.
        // Each group's Jacobian axes span the plane perpendicular to the
        // constraint's target direction, so cross(perp1, perp2) recovers it.
        let mut groups: SmallVec<[(Index, SmallVec<[Vector3<f32>; 2]>); 4]> = SmallVec::new();
        for row in &self.cached_constraint_rows {
            if row.enforcement != Enforcement::HardProjection || row.row_kind != RowKind::Angular {
                continue;
            }
            let jac = if row.body_a.is_some() {
                row.ang_jac_a
            } else {
                row.ang_jac_b
            };
            if let Some(entry) = groups
                .iter_mut()
                .find(|(idx, _)| *idx == row.constraint_index)
            {
                entry.1.push(jac);
            } else {
                let body_handle = row.body_a.or(row.body_b);
                if body_handle.is_none() {
                    continue;
                }
                let mut axes = SmallVec::new();
                axes.push(jac);
                groups.push((row.constraint_index, axes));
            }
        }

        for (constraint_index, axes) in &groups {
            debug_assert!(
                axes.len() >= 2,
                "HardProjection constraint {constraint_index:?} has only {} angular row(s), \
                 expected at least 2 — likely a constraint expansion bug",
                axes.len(),
            );
            if axes.len() < 2 {
                continue;
            }

            // Recover the target up direction from the perpendicular Jacobian axes.
            let up_raw = axes[0].cross(&axes[1]);
            let mag = up_raw.magnitude();
            if mag < 1e-6 {
                continue;
            }
            let up = up_raw / mag;

            // Find the body handle from any HardProjection row of this constraint.
            let body_handle = self
                .cached_constraint_rows
                .iter()
                .find(|r| {
                    r.constraint_index == *constraint_index
                        && r.enforcement == Enforcement::HardProjection
                })
                .and_then(|r| r.body_a.or(r.body_b));

            let Some(handle) = body_handle else {
                continue;
            };
            let Some(body) = bodies.get_mut(handle.0) else {
                continue;
            };

            let local_up = body.rotation() * Vector3::y();

            // Preserve only the spin component (rotation around target up).
            let omega = body.angular_velocity();
            let spin = omega.dot(&up) * up;

            // Corrective angular velocity to reduce tilt error.
            // The cross product local_up x target_up gives the rotation
            // axis and its magnitude equals sin(theta), which works correctly
            // at all angles (unlike the linearized dot-product Jacobian).
            let correction_axis = local_up.cross(&up);
            let correction = correction_axis * (beta / dt);

            body.set_angular_velocity(spin + correction);
        }
    }
}
