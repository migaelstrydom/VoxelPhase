use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::constraint::types::Constraint;
use crate::physics::pipeline::pair::SolverManifold;

use super::conditioning::ManifoldConditions;

/// Trait for constraint solvers (contacts + joints).
///
/// The solver owns the entire constraint pipeline: expansion into internal
/// representation, warm-start caching, velocity solving, position correction,
/// write-back, and post-solve projection. `PhysicsWorld` passes constraint
/// definitions through but never interprets them.
pub trait ConstraintSolver {
    /// Called once per frame after contact generation, before substeps.
    /// The solver can snapshot body state, pre-process constraints,
    /// expand into internal representations, etc.
    fn prepare(&mut self, bodies: &Arena<RigidBody>, constraints: &Arena<Constraint>, dt: f32);

    /// Solve all constraints (contacts + joints) for one substep.
    ///
    /// `conditions` carries per-manifold data produced by the active
    /// `ManifoldConditioner` (e.g. shock propagation mass-scaling factors).
    fn solve(
        &mut self,
        bodies: &mut Arena<RigidBody>,
        manifolds: &mut [SolverManifold],
        conditions: &ManifoldConditions,
        constraints: &Arena<Constraint>,
        dt: f32,
    );

    /// Write solver state back to persistent constraints after each substep.
    /// The solver owns what gets cached (impulses, factorizations, nothing).
    fn write_back(&self, constraints: &mut Arena<Constraint>);

    /// Post-solve velocity projection for rows marked `Enforcement::HardProjection`.
    /// Called after write_back, before position integration.
    fn project_velocities(&self, bodies: &mut Arena<RigidBody>, dt: f32);
}
