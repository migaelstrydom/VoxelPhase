//! Drives moving platforms along their patrol.

use nalgebra::Vector3;
use specs::{Join, LendJoin, ReadExpect, ReadStorage, System, WriteStorage};

use crate::components::{Orientation, Position};
use crate::drive::{Actuator, DriveIntent};
use crate::platform::components::MovingPlatform;
use crate::platform::heading::FacesTravel;
use crate::time::Time;

/// Turns each platform's shuttle state into a drive command.
///
/// Runs before `PhysicsSyncSystem`, which pushes `DriveIntent` into the
/// physics body as a drive target. This system therefore never touches the body
/// directly — it only states an intent, and the solver decides how much of it
/// survives contact with the world. A platform carrying more than its motor can
/// argue with simply falls short of its target speed.
///
/// A platform that [`FacesTravel`] also asks for a rate of turn toward the
/// way its patrol is taking it; any other holds still about every axis.
pub struct MovingPlatformSystem;

impl<'a> System<'a> for MovingPlatformSystem {
    type SystemData = (
        WriteStorage<'a, MovingPlatform>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Orientation>,
        ReadStorage<'a, FacesTravel>,
        WriteStorage<'a, DriveIntent>,
        ReadStorage<'a, Actuator>,
        ReadExpect<'a, Time>,
    );

    fn run(
        &mut self,
        (mut platforms, positions, orientations, turning, mut intents, actuators, time): Self::SystemData,
    ) {
        let dt = time.delta_seconds();
        for (platform, position, orientation, turning, intent, _) in (
            &mut platforms,
            &positions,
            orientations.maybe(),
            turning.maybe(),
            &mut intents,
            &actuators,
        )
            .join()
        {
            platform.update_heading(&position.0);
            intent.linear_target = platform.target_velocity(&position.0, dt);
            if let (Some(turning), Some(orientation)) = (turning, orientation) {
                let rate = turning.yaw_rate(&orientation.0, platform.seek.commanded);
                intent.angular_target = Vector3::y() * rate;
            }
        }
    }
}
