//! ECS systems for biped animation.
//!
//! These are thin wrappers that call into the BipedController.

use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use super::controller::BipedController;
use crate::components::{Position, Rotation, Velocity};
use crate::debug::{DebugLines, DebugOverlays};
use crate::player::Player;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::time::Time;

/// System that configures probes based on biped animation state.
///
/// Runs BEFORE SensorProbeSystem.
/// Reads current position/rotation and writes probe configuration.
pub struct BipedProbeConfigSystem;

impl<'a> System<'a> for BipedProbeConfigSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, BipedController>,
        WriteStorage<'a, SensorSet>,
    );

    fn run(
        &mut self,
        (entities, players, positions, rotations, controllers, mut sensors): Self::SystemData,
    ) {
        for (entity, _player, pos, rot, controller) in
            (&entities, &players, &positions, &rotations, &controllers).join()
        {
            let pelvis_pos = nalgebra::Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let yaw = rot.0;

            // Configure probes based on current animation state
            let probes = controller.configure_probes(pelvis_pos, yaw);

            // Write to SensorSet component
            let sensor_set = SensorSet::new(probes);
            let _ = sensors.insert(entity, sensor_set);
        }
    }
}

/// System that updates biped animation from probe results.
///
/// Runs AFTER SensorProbeSystem.
/// Reads probe results and updates the controller.
pub struct BipedAnimationSystem;

impl<'a> System<'a> for BipedAnimationSystem {
    type SystemData = (
        Read<'a, Time>,
        Entities<'a>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, ContactCandidates>,
        WriteStorage<'a, BipedController>,
        Write<'a, DebugLines>,
        Write<'a, DebugOverlays>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            entities,
            players,
            positions,
            rotations,
            velocities,
            candidates,
            mut controllers,
            mut _debug_lines,
            mut _debug_overlays,
        ) = data;

        let dt = time.delta_seconds();

        for (entity, _player, pos, rot, vel, controller) in (
            &entities,
            &players,
            &positions,
            &rotations,
            &velocities,
            &mut controllers,
        )
            .join()
        {
            let pelvis_pos = nalgebra::Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let yaw = rot.0;

            // Get probe results
            let contacts = candidates
                .get(entity)
                .map(|c| c.candidates.as_slice())
                .unwrap_or(&[]);

            // Update the controller
            let velocity = nalgebra::Vector3::new(vel.0.x, vel.0.y, vel.0.z);
            controller.update(dt, pelvis_pos, yaw, velocity, contacts);
            // debug_lines.add(
            //     "WheelAngle",
            //     &format!(
            //         "{:.2} rad ({:.0}°)",
            //         controller.state.wheel_angle,
            //         controller.state.wheel_angle.to_degrees()
            //     ),
            // );
            // debug_overlays.add_sphere(controller.state.pelvis_position, 0.1, Colour::GREEN);

            // Visualize stride wheel rim point
            // let wheel_radius = controller.config.body_radius;
            // let wheel_center = controller.state.pelvis_position;
            // let facing = controller.state.facing;
            // let angle = controller.state.wheel_angle;

            // Rim point for a wheel rolling forward:
            // - vertical: -cos(angle) (bottom at angle=0)
            // - forward: -sin(angle) (bottom point moves backward as wheel rolls forward)
            // let rim_offset = nalgebra::Vector3::new(
            //     -facing.x * angle.sin() * wheel_radius,
            //     -angle.cos() * wheel_radius,
            //     -facing.z * angle.sin() * wheel_radius,
            // );
            // let rim_point = wheel_center + rim_offset;
            // debug_overlays.add_sphere(rim_point, 0.05, Colour::YELLOW);
        }
    }
}
