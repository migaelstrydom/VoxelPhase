use crate::components::{Acceleration, Gravity, Velocity};
use crate::player::PlayerConfig;
use crate::time::Time;
use specs::{Join, ReadExpect, ReadStorage, System, WriteStorage};

/// Applies gravity to entities that have the Gravity component.
/// Sets the Y component of acceleration based on the gravity value.
pub struct GravitySystem;

impl<'a> System<'a> for GravitySystem {
    type SystemData = (ReadStorage<'a, Gravity>, WriteStorage<'a, Acceleration>);

    fn run(&mut self, (gravities, mut accelerations): Self::SystemData) {
        for (gravity, accel) in (&gravities, &mut accelerations).join() {
            // Gravity is a downward force (negative Y in our coordinate system)
            accel.0.y = -gravity.0;
        }
    }
}

/// Integrates acceleration into velocity only.
///
/// Position updates are handled by collision systems (SpringBipedCollisionSystem
/// for biped entities, DynamicTerrainCollisionSystem for dynamic bodies).
/// This ensures collision resolution is authoritative for position.
pub struct VelocityIntegrationSystem;

impl<'a> System<'a> for VelocityIntegrationSystem {
    type SystemData = (
        ReadExpect<'a, Time>,
        ReadExpect<'a, PlayerConfig>,
        ReadStorage<'a, Acceleration>,
        WriteStorage<'a, Velocity>,
    );

    fn run(&mut self, (time, config, accelerations, mut velocities): Self::SystemData) {
        let dt = time.delta_seconds();
        let terminal_vel = config.terminal_velocity;

        // for (accel, vel) in (&accelerations, &mut velocities).join() {
        //     vel.0 += accel.0 * dt;

        //     // Clamp vertical velocity to terminal velocity
        //     if vel.0.y < -terminal_vel {
        //         vel.0.y = -terminal_vel;
        //     }
        // }
    }
}
