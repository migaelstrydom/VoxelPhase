use crate::physics::PhysicsWorld;

use super::ground::Ground;

/// A population of bodies whose simulation cost is worth measuring.
///
/// A scenario only builds the world's initial state. The runner steps it at
/// the game's own cadence and does the timing, so every scenario is measured
/// the same way.
pub trait PerfScenario {
    fn name(&self) -> &str;

    /// One line saying what was built, for the report header.
    fn describe(&self) -> String;

    /// Create the scenario's bodies in `world`, standing on `ground`.
    fn populate(&self, world: &mut PhysicsWorld, ground: &Ground);
}
