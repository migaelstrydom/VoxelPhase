use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::constraint::ConstraintRow;
use crate::physics::pipeline::pair::SolverManifold;

use super::conditioning::ManifoldConditions;

/// Trait for constraint solvers (contacts + joints).
///
/// Different solver strategies (PGS+NGS, TGS, XPBD, etc.) implement this trait
/// to provide different trade-offs between stability, performance, and accuracy.
pub trait ConstraintSolver {
    /// Called once after contact generation and constraint expansion,
    /// before substeps begin.
    /// Solvers can capture any per-frame state they need (e.g., body position snapshots).
    fn prepare(&mut self, bodies: &Arena<RigidBody>);

    /// Solve all constraints (contacts + joints) for one substep.
    ///
    /// `conditions` carries per-manifold data produced by the active
    /// `ManifoldConditioner` (e.g. shock propagation mass-scaling factors).
    ///
    /// `constraint_rows` carries joint constraint rows expanded from
    /// user-defined constraints. Solvers that don't support joints can
    /// ignore this slice.
    fn solve(
        &mut self,
        bodies: &mut Arena<RigidBody>,
        manifolds: &mut [SolverManifold],
        conditions: &ManifoldConditions,
        constraint_rows: &mut [ConstraintRow],
        dt: f32,
    );
}
