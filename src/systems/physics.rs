use crate::components::{Acceleration, Gravity, Position, Velocity};
use crate::time::Time;
use specs::{Join, ReadExpect, ReadStorage, System, WriteStorage};

/// Applies gravity to entities that have the Gravity component.
/// Sets the Y component of acceleration based on the gravity value.
pub struct GravitySystem;

impl<'a> System<'a> for GravitySystem {
    type SystemData = (
        ReadStorage<'a, Gravity>,
        WriteStorage<'a, Acceleration>,
    );

    fn run(&mut self, (gravities, mut accelerations): Self::SystemData) {
        for (gravity, accel) in (&gravities, &mut accelerations).join() {
            // Gravity is a downward force (negative Y in our coordinate system)
            accel.0.y = -gravity.0;
        }
    }
}

/// Integrates physics: applies acceleration to velocity, and velocity to position.
/// Uses semi-implicit Euler integration for stability.
pub struct PhysicsSystem;

impl<'a> System<'a> for PhysicsSystem {
    type SystemData = (
        ReadExpect<'a, Time>,
        ReadStorage<'a, Acceleration>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Position>,
    );

    fn run(&mut self, (time, accelerations, mut velocities, mut positions): Self::SystemData) {
        let dt = time.delta_seconds();

        // Update velocity from acceleration, then position from velocity
        // (Semi-implicit Euler: v += a*dt, then p += v*dt)
        for (accel, vel) in (&accelerations, &mut velocities).join() {
            vel.0 += accel.0 * dt;
        }

        for (vel, pos) in (&velocities, &mut positions).join() {
            pos.0 += vel.0 * dt;
        }
    }
}
