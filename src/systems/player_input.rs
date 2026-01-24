use crate::components::{Acceleration, CameraComponent, Position, Velocity};
use crate::input::GameplayActions;
use crate::player::{Player, PlayerConfig, PlayerState};
use crate::skeleton::SpringBipedCharacter;
use nalgebra::Vector3;
use specs::{Join, ReadExpect, ReadStorage, System, WriteStorage};

/// Reads input and updates player velocity based on movement keys.
/// Movement is relative to camera direction (forward = toward where camera looks).
pub struct PlayerInputSystem;

impl<'a> System<'a> for PlayerInputSystem {
    type SystemData = (
        ReadExpect<'a, GameplayActions>,
        ReadExpect<'a, PlayerConfig>,
        ReadStorage<'a, Player>,
        WriteStorage<'a, PlayerState>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Acceleration>,
        ReadStorage<'a, CameraComponent>,
        ReadStorage<'a, SpringBipedCharacter>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            actions,
            config,
            players,
            mut player_states,
            positions,
            mut velocities,
            mut accelerations,
            cameras,
            spring_bipeds,
        ) = data;

        // Get camera position for calculating movement direction
        let camera = cameras.join().next();
        let camera_pos = camera
            .map(|c| c.0.position)
            .unwrap_or_else(|| nalgebra::Point3::new(0.0, 0.0, 5.0));

        for (_player, state, pos, vel, accel, spring_biped) in (
            &players,
            &mut player_states,
            &positions,
            &mut velocities,
            &mut accelerations,
            &spring_bipeds,
        )
            .join()
        {
            // Debug: show player position
            // debug.add(
            //     "Position",
            //     format!("{:.1}, {:.1}, {:.1}", pos.0.x, pos.0.y, pos.0.z),
            // );
            // debug.add(
            //     "Velocity",
            //     format!("{:.1}, {:.1}, {:.1}", vel.0.x, vel.0.y, vel.0.z),
            // );
            // debug.add("On Ground", format!("{}", on_ground.grounded));
            // debug.add("Facing Direction", format!("{:.1}", state.facing_direction));

            let is_grounded = spring_biped.grounded;

            // Calculate forward direction from player toward camera (XZ plane only)
            // In BallDude, forward was from player toward camera position
            let to_camera = Vector3::new(
                camera_pos.x - pos.0.x,
                0.0, // Ignore Y for horizontal movement
                camera_pos.z - pos.0.z,
            );

            // Normalize, or use a default if camera is directly above player
            let forward = if to_camera.magnitude() > 0.001 {
                -to_camera.normalize()
            } else {
                Vector3::new(0.0, 0.0, 1.0)
            };

            // Right vector is perpendicular to forward (rotate 90 degrees in XZ plane)
            let right = Vector3::new(-forward.z, 0.0, forward.x);

            // Build movement direction from input actions
            let mut move_dir = Vector3::zeros();

            if actions.move_forward {
                move_dir += forward;
            }
            if actions.move_backward {
                move_dir -= forward;
            }
            if actions.move_left {
                move_dir -= right;
            }
            if actions.move_right {
                move_dir += right;
            }

            // Normalize diagonal movement to prevent faster diagonal speed
            if move_dir.magnitude() > 0.001 {
                move_dir = move_dir.normalize();

                // Update facing direction based on movement
                state.facing_direction = -move_dir.z.atan2(move_dir.x) + std::f32::consts::PI / 2.0;
            }

            // Reset horizontal acceleration each frame (prevents stale air accel)
            accel.0.x = 0.0;
            accel.0.z = 0.0;

            // Apply horizontal velocity (preserve vertical velocity for gravity/jumping)
            if is_grounded {
                vel.0.x = move_dir.x * config.walk_speed;
                vel.0.z = move_dir.z * config.walk_speed;
            } else {
                accel.0.x = move_dir.x * config.air_acceleration;
                accel.0.z = move_dir.z * config.air_acceleration;
            }

            // Handle jumping - only when grounded
            if actions.jump && is_grounded {
                vel.0.y = config.jump_speed;
            }
        }
    }
}
