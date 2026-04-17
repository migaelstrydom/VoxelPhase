use crate::components::{CameraComponent, Position};
use crate::input::GameplayActions;
use crate::player::{Player, PlayerTargetState};
use nalgebra::Vector3;
use specs::{Join, ReadExpect, ReadStorage, System, WriteStorage};

/// Processes player input and calculates desired movement target.
/// Runs early in the frame to convert raw input into movement intent.
/// Movement is relative to camera direction (forward = toward where camera looks).
pub struct PlayerInputSystem;

impl<'a> System<'a> for PlayerInputSystem {
    type SystemData = (
        ReadExpect<'a, GameplayActions>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, CameraComponent>,
        WriteStorage<'a, PlayerTargetState>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (actions, players, positions, cameras, mut player_targets) = data;

        // Get camera position for calculating movement direction
        let camera = cameras.join().next();
        let camera_pos = camera
            .map(|c| c.0.position)
            .unwrap_or_else(|| nalgebra::Point3::new(0.0, 0.0, 5.0));

        for (_player, pos, target) in (&players, &positions, &mut player_targets).join() {
            // Calculate forward direction from player toward camera (XZ plane only)
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
            }

            // Write target state for later systems to use
            target.direction = move_dir;
            target.jump = actions.jump;
            target.jump_held = actions.jump_held;
            target.jump_released = actions.jump_released;
            target.crouch = actions.crouch_held;
            target.crouch_just_pressed = actions.crouch_just_pressed;
            target.sprint = actions.sprint_held;
            target.grab_held = actions.grab_held;
            target.grab_just_pressed = actions.grab_just_pressed;
            target.grab_just_released = actions.grab_just_released;
            target.throw = actions.throw;
            target.throw_grenade = false;
        }
    }
}
