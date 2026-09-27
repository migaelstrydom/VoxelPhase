use crate::physics::{PhysicsConfig, PhysicsImpulse, PhysicsWorld};

use crate::perf::Ground;

/// A one-shot impulse the runner delivers partway through a run, the way the
/// game's impulse queue delivers it: once, at the start of a frame.
#[derive(Debug, Clone, Copy)]
pub struct Disturbance {
    /// Simulated time it goes off, in seconds; it lands on the first frame
    /// that starts at or after it.
    pub at: f32,
    pub impulse: PhysicsImpulse,
}

/// A population of bodies whose simulation cost is worth measuring.
///
/// A scenario builds the world's initial state and says what disturbs it
/// later. The runner steps it at the game's own cadence and does the timing,
/// so every scenario is measured the same way.
pub trait PerfScenario {
    fn name(&self) -> &str;

    /// One line saying what was built, for the report header.
    fn describe(&self) -> String;

    /// The world the scenario is simulated in; the game's own by default.
    fn physics_config(&self) -> PhysicsConfig {
        PhysicsConfig::default()
    }

    /// Create the scenario's bodies in `world`, standing on `ground`.
    fn populate(&self, world: &mut PhysicsWorld, ground: &Ground);

    /// Impulses to deliver during the run; none by default.
    fn disturbances(&self, ground: &Ground) -> Vec<Disturbance> {
        let _ = ground;
        Vec::new()
    }
}
