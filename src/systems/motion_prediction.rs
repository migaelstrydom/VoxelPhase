//! Motion prediction system for CCD collision detection.

use nalgebra::Point3;
use specs::{Join, ReadExpect, ReadStorage, System, WriteStorage};

use crate::components::{MotionState, Position, Velocity};
use crate::time::Time;

/// Computes predicted positions for all entities with Position, Velocity, and MotionState.
///
/// This system captures the current position as `prev` and computes `predicted = prev + vel * dt`.
/// Collision systems then sweep from `prev` to `predicted` and write the resolved position back.
pub struct MotionPredictionSystem;

impl<'a> System<'a> for MotionPredictionSystem {
    type SystemData = (
        ReadExpect<'a, Time>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Velocity>,
        WriteStorage<'a, MotionState>,
    );

    fn run(&mut self, (time, positions, velocities, mut motion_states): Self::SystemData) {
        let dt = time.delta_seconds();

        for (pos, vel, motion) in (&positions, &velocities, &mut motion_states).join() {
            // Capture current position as prev
            motion.prev = Point3::new(pos.0.x, pos.0.y, pos.0.z);

            // Compute predicted position based on current velocity
            motion.predicted = Point3::new(
                pos.0.x + vel.0.x * dt,
                pos.0.y + vel.0.y * dt,
                pos.0.z + vel.0.z * dt,
            );
        }
    }
}
