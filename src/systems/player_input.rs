use crate::components::{CameraComponent, Position, Velocity};
use crate::input::InputState;
use crate::player::{Player, PlayerConfig, PlayerState};
use nalgebra::Vector3;
use specs::{Join, ReadExpect, ReadStorage, System, WriteStorage};
use winit::event::MouseButton;
use winit::keyboard::KeyCode;

/// Reads input and updates player velocity based on movement keys.
/// Movement is relative to camera direction (forward = toward where camera looks).
pub struct PlayerInputSystem;

impl<'a> System<'a> for PlayerInputSystem {
    type SystemData = (
        ReadExpect<'a, InputState>,
        ReadExpect<'a, PlayerConfig>,
        ReadStorage<'a, Player>,
        WriteStorage<'a, PlayerState>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        ReadStorage<'a, CameraComponent>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (input, config, players, mut player_states, positions, mut velocities, cameras) = data;

        // Get camera position for calculating movement direction
        let camera = cameras.join().next();
        let camera_pos = camera
            .map(|c| c.0.position)
            .unwrap_or_else(|| nalgebra::Point3::new(0.0, 0.0, 5.0));

        for (_player, state, pos, vel) in
            (&players, &mut player_states, &positions, &mut velocities).join()
        {
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

            // Determine movement speed based on ground state
            let speed = if state.on_ground {
                config.walk_speed
            } else {
                config.air_speed
            };

            // Build movement direction from WASD input
            let mut move_dir = Vector3::zeros();

            if input.is_key_pressed(KeyCode::KeyW) || input.is_key_pressed(KeyCode::ArrowUp) {
                move_dir += forward;
            }
            if input.is_key_pressed(KeyCode::KeyS) || input.is_key_pressed(KeyCode::ArrowDown) {
                move_dir -= forward;
            }
            if input.is_key_pressed(KeyCode::KeyA) || input.is_key_pressed(KeyCode::ArrowLeft) {
                move_dir -= right;
            }
            if input.is_key_pressed(KeyCode::KeyD) || input.is_key_pressed(KeyCode::ArrowRight) {
                move_dir += right;
            }

            // Normalize diagonal movement to prevent faster diagonal speed
            if move_dir.magnitude() > 0.001 {
                move_dir = move_dir.normalize();

                // Update facing direction based on movement
                state.facing_direction = -move_dir.z.atan2(move_dir.x) + std::f32::consts::PI / 2.0;
            }

            // Apply horizontal velocity (preserve vertical velocity for gravity/jumping)
            vel.0.x = move_dir.x * speed;
            vel.0.z = move_dir.z * speed;

            // Handle jumping
            if (input.is_mouse_button_just_pressed(MouseButton::Right)
                || input.is_key_just_pressed(KeyCode::Space))
                && state.on_ground
            {
                vel.0.y = config.jump_speed;
                state.on_ground = false;
            }
        }
    }
}
