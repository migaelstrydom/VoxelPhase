//! PGS + NGS solver: Projected Gauss-Seidel velocity solve with
//! nonlinear Gauss-Seidel position correction.

use std::collections::HashMap;

use generational_arena::{Arena, Index};
use nalgebra::Point3;

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::SolverManifold;

use super::body_pair::BodyPairState;
use super::conditioning::ManifoldConditions;
use super::friction::{manifold_friction_projection, solve_friction_impulse};
use super::normal::solve_normal_impulse;
use super::position_correction::{self, PositionCorrectionConfig};
use super::warm_start::{effective_solver_iterations, warm_start_contact};
use super::ContactSolver;

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
}

impl Default for PgsNgsConfig {
    fn default() -> Self {
        Self {
            solver_iterations: 3,
            restitution_velocity_threshold: 0.3,
            warm_start_scale: 0.6,
            block_normal_micro_iterations: 4,
            position_correction: PositionCorrectionConfig::default(),
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
    contact_generation_positions: HashMap<Index, Point3<f32>>,
}

impl PgsNgsSolver {
    pub fn new(config: PgsNgsConfig) -> Self {
        Self {
            config,
            contact_generation_positions: HashMap::new(),
        }
    }
}

impl Default for PgsNgsSolver {
    fn default() -> Self {
        Self::new(PgsNgsConfig::default())
    }
}

impl ContactSolver for PgsNgsSolver {
    fn prepare(&mut self, bodies: &Arena<RigidBody>) {
        self.contact_generation_positions.clear();
        for (idx, body) in bodies.iter() {
            if !body.is_static() {
                self.contact_generation_positions
                    .insert(idx, body.position());
            }
        }
    }

    fn solve(
        &mut self,
        bodies: &mut Arena<RigidBody>,
        manifolds: &mut [SolverManifold],
        conditions: &ManifoldConditions,
        dt: f32,
    ) {
        if manifolds.is_empty() {
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

        // Phase 3: Iterative sequential-impulse solving.
        let iterations = effective_solver_iterations(manifolds, self.config.solver_iterations);
        for _ in 0..iterations {
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
        }

        // Phase 4: Position correction after velocity solving.
        // NGS uses real masses (no shock propagation) — position correction
        // is linear-only and doesn't benefit from mass scaling.
        position_correction::apply_position_correction(
            bodies,
            manifolds,
            &self.config.position_correction,
            dt,
            &self.contact_generation_positions,
        );
    }
}
