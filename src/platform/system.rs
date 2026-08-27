//! Drives moving platforms along their patrol.

use specs::{Join, ReadStorage, System, WriteStorage};

use crate::components::{Position, Velocity, VelocityDriven};
use crate::platform::components::MovingPlatform;

/// Turns each platform's shuttle state into a velocity drive target.
///
/// Runs before `PhysicsSyncSystem`, which copies `Velocity` into the physics
/// body as a drive target. This system therefore never touches the body
/// directly — it only states an intent, and the solver decides how much of it
/// survives contact with the world. A platform carrying more than its motor can
/// argue with simply falls short of its target speed.
pub struct MovingPlatformSystem;

impl<'a> System<'a> for MovingPlatformSystem {
    type SystemData = (
        WriteStorage<'a, MovingPlatform>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        ReadStorage<'a, VelocityDriven>,
    );

    fn run(&mut self, (mut platforms, positions, mut velocities, driven): Self::SystemData) {
        for (platform, position, velocity, _) in
            (&mut platforms, &positions, &mut velocities, &driven).join()
        {
            platform.update_heading(&position.0);
            velocity.0 = platform.target_velocity(&position.0);
        }
    }
}
