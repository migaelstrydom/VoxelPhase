//! Drives moving platforms along their patrol.

use specs::{Join, ReadStorage, System, WriteStorage};

use crate::components::Position;
use crate::drive::{Actuator, DriveIntent};
use crate::platform::components::MovingPlatform;

/// Turns each platform's shuttle state into a drive command.
///
/// Runs before `PhysicsSyncSystem`, which pushes `DriveIntent` into the
/// physics body as a drive target. This system therefore never touches the body
/// directly — it only states an intent, and the solver decides how much of it
/// survives contact with the world. A platform carrying more than its motor can
/// argue with simply falls short of its target speed.
pub struct MovingPlatformSystem;

impl<'a> System<'a> for MovingPlatformSystem {
    type SystemData = (
        WriteStorage<'a, MovingPlatform>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, DriveIntent>,
        ReadStorage<'a, Actuator>,
    );

    fn run(&mut self, (mut platforms, positions, mut intents, actuators): Self::SystemData) {
        for (platform, position, intent, _) in
            (&mut platforms, &positions, &mut intents, &actuators).join()
        {
            platform.update_heading(&position.0);
            intent.linear_target = platform.target_velocity(&position.0);
        }
    }
}
