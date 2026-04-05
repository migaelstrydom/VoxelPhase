//! PGS + NGS solver: Projected Gauss-Seidel velocity solve with
//! nonlinear Gauss-Seidel position correction.

use rustc_hash::FxHashMap;

use generational_arena::{Arena, Index};
use nalgebra::{Point3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::constraint::expand::{expand_constraints, write_back_constraints};
use crate::physics::constraint::types::{Constraint, ConstraintKind, ConstraintRow};
use crate::physics::pipeline::pair::SolverManifold;

use super::body_pair::BodyPairState;
use super::conditioning::ManifoldConditions;
use super::constraint_row::{solve_constraint_row, warm_start_constraint_row};
use super::friction::{manifold_friction_projection, solve_friction_impulse};
use super::normal::solve_normal_impulse;
use super::position_correction::{self, PositionCorrectionConfig};
use super::warm_start::{effective_solver_iterations, warm_start_contact};
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
}

impl PgsNgsSolver {
    pub fn new(config: PgsNgsConfig) -> Self {
        Self {
            config,
            contact_generation_positions: FxHashMap::default(),
            cached_constraint_rows: Vec::new(),
        }
    }
}

impl Default for PgsNgsSolver {
    fn default() -> Self {
        Self::new(PgsNgsConfig::default())
    }
}

impl ConstraintSolver for PgsNgsSolver {
    fn prepare(&mut self, bodies: &Arena<RigidBody>, constraints: &Arena<Constraint>, dt: f32) {
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

        // Phase 1: Capture pre-solve normal velocities and warm-start scales.
        //
        // Persisted contacts (warm_normal_impulse > 0) always get warm-started
        // regardless of approach velocity. This prevents velocity-driven bodies
        // (e.g. player characters) from having warm-start disabled every frame
        // due to their externally-set velocity exceeding the threshold.
        //
        // Pre-solve extraction uses identity shock scales — we need the real
        // relative velocity for restitution decisions, not the shock-adjusted one.
        let no_shock = (1.0, 1.0);
        let pre_solve: Vec<Vec<(f32, f32)>> = manifolds
            .iter()
            .map(|m| {
                m.contacts
                    .iter()
                    .map(|c| {
                        let vn = BodyPairState::extract(bodies, &m.header, c.point, no_shock)
                            .map(|s| s.relative_normal_velocity(c.point, &c.normal))
                            .unwrap_or(0.0);
                        let is_persisted = c.warm_normal_impulse > 0.0;
                        let warm_scale = if is_persisted {
                            self.config.warm_start_scale
                        } else if vn.abs() > self.config.restitution_velocity_threshold {
                            0.0
                        } else {
                            self.config.warm_start_scale
                        };
                        (vn, warm_scale)
                    })
                    .collect()
            })
            .collect();

        // Phase 2: Warm-start — apply cached impulses from previous frame.
        for (mi, manifold) in manifolds.iter_mut().enumerate() {
            let header = &manifold.header;
            let shock = conditions.shock_scales_for(mi);
            for (ci, contact) in manifold.contacts.iter_mut().enumerate() {
                warm_start_contact(bodies, header, contact, pre_solve[mi][ci].1, shock);
            }
        }
        for row in constraint_rows.iter_mut() {
            warm_start_constraint_row(bodies, row, self.config.warm_start_scale);
        }

        // Phase 3: Iterative sequential-impulse solving.
        let iterations = effective_solver_iterations(manifolds, self.config.solver_iterations);
        for _ in 0..iterations {
            // Contacts first — normal + friction impulses.
            for (mi, manifold) in manifolds.iter_mut().enumerate() {
                let shock = conditions.shock_scales_for(mi);

                // Block normal solve: multi-contact manifolds get extra local
                // iterations to capture cross-contact coupling.
                let normal_passes = if manifold.contacts.len() > 1 {
                    self.config.block_normal_micro_iterations
                } else {
                    1
                };
                for _ in 0..normal_passes {
                    for ci in 0..manifold.contacts.len() {
                        let is_persisted = manifold.contacts[ci].warm_normal_impulse > 0.0;
                        solve_normal_impulse(
                            bodies,
                            &manifold.header,
                            &mut manifold.contacts[ci],
                            self.config.restitution_velocity_threshold,
                            pre_solve[mi][ci].0,
                            is_persisted,
                            shock,
                        );
                    }
                }

                // Per-contact friction solve
                for ci in 0..manifold.contacts.len() {
                    solve_friction_impulse(
                        bodies,
                        &manifold.header,
                        &mut manifold.contacts[ci],
                        shock,
                    );
                }

                // Manifold-level friction budget projection
                if manifold.contacts.len() > 1 {
                    manifold_friction_projection(
                        bodies,
                        &manifold.header,
                        &mut manifold.contacts,
                        shock,
                    );
                }
            }

            // Joint constraints last within each iteration — higher priority
            // than contacts, so friction can't undo constraint corrections.
            for row in constraint_rows.iter_mut() {
                solve_constraint_row(bodies, row);
            }
        }

        // Phase 4: Position correction after velocity solving.
        // NGS uses real masses (no shock propagation) — position correction
        // is linear-only and doesn't benefit from mass scaling.
        position_correction::apply_position_correction(
            bodies,
            manifolds,
            constraints,
            &self.config.position_correction,
            dt,
            &self.contact_generation_positions,
        );
    }

    fn write_back(&self, constraints: &mut Arena<Constraint>) {
        write_back_constraints(constraints, &self.cached_constraint_rows);
    }

    fn project_velocities(
        &self,
        constraints: &Arena<Constraint>,
        bodies: &mut Arena<RigidBody>,
        dt: f32,
    ) {
        let beta = self.config.constraint_position_beta;

        for (_index, constraint) in constraints.iter() {
            if !constraint.active {
                continue;
            }

            match &constraint.kind {
                ConstraintKind::KeepUpright {
                    body,
                    target_up,
                    compliance,
                } => {
                    if *compliance > 0.0 {
                        continue;
                    }

                    let Some(body) = bodies.get_mut(body.0) else {
                        continue;
                    };

                    let up = target_up.into_inner();
                    let local_up = body.rotation() * Vector3::y();

                    // Preserve only the spin component (rotation around target up)
                    let omega = body.angular_velocity();
                    let spin = omega.dot(&up) * up;

                    // Corrective angular velocity to reduce tilt error.
                    // The cross product local_up × target_up gives the rotation
                    // axis and its magnitude equals sin(θ), which works correctly
                    // at all angles (unlike the linearized dot-product Jacobian).
                    let correction_axis = local_up.cross(&up);
                    let correction = correction_axis * (beta / dt);

                    body.set_angular_velocity(spin + correction);
                }

                ConstraintKind::AnchorPoint { .. } | ConstraintKind::FollowPoint { .. } => {}
            }
        }
    }
}
