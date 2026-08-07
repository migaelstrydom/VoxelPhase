use crate::debug::DebugLines;
use crate::physics::force_provider::SubstepForceProvider;
use crate::physics::impulses::PhysicsImpulse;
use crate::physics::static_geometry::StaticGeometry;
use crate::physics::world::PhysicsWorld;

/// Result of a single `Stepper::step()` call.
pub struct StepResult {
    /// Number of fixed-timestep substeps executed this frame.
    pub substeps: u32,
}

/// Controls the overall physics step loop structure.
///
/// Different solvers require different stepping patterns:
/// - **Sequential** (PGS, PGS+NGS): generate contacts once, substep N times.
/// - **Interleaved** (TGS): regenerate contacts each substep.
/// - **Position-based** (XPBD): predict positions, solve constraints, derive velocities.
///
/// The `Stepper` orchestrates when to call `PhysicsWorld::update_contacts()`
/// and `PhysicsWorld::substep()`, while the `ConstraintSolver` inside the world
/// handles the actual constraint solving.
pub trait Stepper: Send + Sync {
    fn step(
        &mut self,
        world: &mut PhysicsWorld,
        frame_dt: f32,
        static_geometry: &dyn StaticGeometry,
        impulses: &[PhysicsImpulse],
        force_providers: &[&dyn SubstepForceProvider],
        debug_lines: &mut DebugLines,
    ) -> StepResult;

    /// The fixed timestep size used by this stepper.
    fn fixed_dt(&self) -> f32;
}
