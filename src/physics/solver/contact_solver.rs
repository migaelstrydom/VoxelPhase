use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::SolverManifold;

use super::conditioning::ManifoldConditions;

/// Trait for contact constraint solvers.
///
/// Different solver strategies (PGS+NGS, TGS, XPBD, etc.) implement this trait
/// to provide different trade-offs between stability, performance, and accuracy.
pub trait ContactSolver {
    /// Called once after contact generation, before substeps begin.
    /// Solvers can capture any per-frame state they need (e.g., body position snapshots).
    fn prepare(&mut self, bodies: &Arena<RigidBody>);

    /// Solve contact constraints for one substep.
    ///
    /// `conditions` carries per-manifold data produced by the active
    /// `ManifoldConditioner` (e.g. shock propagation mass-scaling factors).
    fn solve(
        &mut self,
        bodies: &mut Arena<RigidBody>,
        manifolds: &mut [SolverManifold],
        conditions: &ManifoldConditions,
        dt: f32,
    );
}
