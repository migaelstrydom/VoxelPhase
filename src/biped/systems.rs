//! ECS systems for biped animation.
//!
//! These are thin wrappers that call into the BipedController.

use specs::{Entities, Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

use super::controller::BipedController;
use crate::components::{Position, Rotation, Velocity};
use crate::debug::{DebugLines, DebugOverlays};
use crate::player::grab::GrabConfig;
use crate::player::{ArmState, Player, PlayerState, PlayerTargetState};
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
        ReadExpect<'a, GrabConfig>,
        Entities<'a>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, PlayerState>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, PlayerTargetState>,
        ReadStorage<'a, ContactCandidates>,
        WriteStorage<'a, BipedController>,
        Write<'a, DebugLines>,
        Write<'a, DebugOverlays>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            grab_config,
            entities,
            players,
            player_states,
            positions,
            rotations,
            velocities,
            target_states,
            candidates,
            mut controllers,
            mut _debug_lines,
            mut _debug_overlays,
        ) = data;

        let dt = time.delta_seconds();

        for (entity, _player, player_state, pos, rot, vel, target, controller) in (
            &entities,
            &players,
            &player_states,
            &positions,
            &rotations,
            &velocities,
            &target_states,
            &mut controllers,
        )
            .join()
        {
            let pelvis_pos = nalgebra::Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let yaw = rot.0;

            // Set grab hand target before animation update
            controller.state.grab_hand_target =
                compute_grab_hand_target(&player_state.arm, pelvis_pos, yaw, &grab_config);
            // Get probe results
            let contacts = candidates
                .get(entity)
                .map(|c| c.candidates.as_slice())
                .unwrap_or(&[]);

            // Update the controller
            let velocity = nalgebra::Vector3::new(vel.0.x, vel.0.y, vel.0.z);
            let wants_to_walk = target.direction.magnitude_squared() > 0.001;
            controller.update(dt, pelvis_pos, yaw, velocity, wants_to_walk, contacts);
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

/// Compute the right hand target for grab animation, if applicable.
fn compute_grab_hand_target(
    arm: &ArmState,
    pelvis_pos: nalgebra::Point3<f32>,
    yaw: f32,
    config: &GrabConfig,
) -> Option<nalgebra::Point3<f32>> {
    let facing = nalgebra::Vector3::new(yaw.sin(), 0.0, yaw.cos());

    match arm {
        ArmState::Idle => None,
        ArmState::Reaching { elapsed, target } => {
            // Reach toward the hit point's height if we have one, otherwise
            // use the probe direction (hold_distance forward at pelvis level).
            let reach_height = target
                .map(|(_body, hit)| hit.y - pelvis_pos.y)
                .unwrap_or(0.0);
            let reach_target =
                pelvis_pos + facing * config.hold_distance + nalgebra::Vector3::y() * reach_height;
            let t = (elapsed / config.reach_duration).min(1.0);
            let rest_hand = pelvis_pos + nalgebra::Vector3::y() * 0.1;
            Some(nalgebra::Point3::from(
                rest_hand.coords.lerp(&reach_target.coords, t),
            ))
        }
        ArmState::Holding {
            current_hold_height,
            ..
        } => {
            let point = pelvis_pos
                + facing * config.hold_distance
                + nalgebra::Vector3::y() * *current_hold_height;
            Some(point)
        }
    }
}
