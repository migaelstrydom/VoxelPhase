use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::SolverManifold;

/// Trait for contact constraint solvers.
///
/// Different solver strategies (PGS+NGS, TGS, XPBD, etc.) implement this trait
/// to provide different trade-offs between stability, performance, and accuracy.
pub trait ContactSolver {
    /// Called once after contact generation, before substeps begin.
    /// Solvers can capture any per-frame state they need (e.g., body position snapshots).
    fn prepare(&mut self, bodies: &Arena<RigidBody>);

    /// Solve contact constraints for one substep.
    fn solve(
        &mut self,
        bodies: &mut Arena<RigidBody>,
        manifolds: &mut [SolverManifold],
        dt: f32,
    );
}
