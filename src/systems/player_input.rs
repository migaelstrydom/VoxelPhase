use crate::biped::BipedController;
use crate::components::{CameraComponent, Position, Rotation, Velocity};
use crate::debug::DebugLines;
use crate::input::GameplayActions;
use crate::player::{Player, PlayerConfig, PlayerMoveState, PlayerTargetState};
use crate::time::Time;
use nalgebra::Vector3;
use specs::{Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

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
        }
    }
}

/// Applies player movement target to actual physics state.
/// Runs after animation systems to apply intended movement based on grounded state.
///
/// Uses a four-state FSM:
///   Grounded ──(jump)──────────► Launching ──(!is_grounded)──► Airborne
///   Grounded ──(!is_grounded)──► CoyoteTime ──(timer expired)──► Airborne
///   CoyoteTime ──(jump)──────────────────────────────────────► Airborne
///   CoyoteTime ──(is_grounded)──► Grounded
///   Airborne ──(is_grounded)────► Grounded
pub struct PlayerMotionSystem;

impl<'a> System<'a> for PlayerMotionSystem {
    type SystemData = (
        Read<'a, Time>,
        ReadExpect<'a, PlayerConfig>,
        ReadStorage<'a, Player>,
        WriteStorage<'a, PlayerTargetState>,
        ReadStorage<'a, BipedController>,
        WriteStorage<'a, Rotation>,
        WriteStorage<'a, Velocity>,
        Write<'a, DebugLines>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            config,
            players,
            mut player_targets,
            controllers,
            mut rotations,
            mut velocities,
            mut _debug_lines,
        ) = data;
        let dt = time.delta_seconds();

        for (_player, target, controller, rotation, vel) in (
            &players,
            &mut player_targets,
            &controllers,
            &mut rotations,
            &mut velocities,
        )
            .join()
        {
            let is_grounded = controller.is_grounded();
            let move_dir = target.direction;

            // --- State transitions ---
            target.move_state = match target.move_state {
                PlayerMoveState::Grounded => {
                    if target.jump {
                        vel.0.y = config.jump_speed;
                        PlayerMoveState::Launching
                    } else if !is_grounded {
                        PlayerMoveState::CoyoteTime(config.ground_grace_period)
                    } else {
                        PlayerMoveState::Grounded
                    }
                }
                PlayerMoveState::Launching => {
                    if !is_grounded {
                        PlayerMoveState::Airborne
                    } else {
                        PlayerMoveState::Launching
                    }
                }
                PlayerMoveState::CoyoteTime(remaining) => {
                    if target.jump {
                        vel.0.y = config.jump_speed;
                        PlayerMoveState::Airborne
                    } else if is_grounded {
                        PlayerMoveState::Grounded
                    } else {
                        let t = remaining - dt;
                        if t <= 0.0 {
                            PlayerMoveState::Airborne
                        } else {
                            PlayerMoveState::CoyoteTime(t)
                        }
                    }
                }
                PlayerMoveState::Airborne => {
                    if is_grounded {
                        PlayerMoveState::Grounded
                    } else {
                        PlayerMoveState::Airborne
                    }
                }
            };

            // --- Per-state behaviour ---

            // Update facing direction based on movement
            if move_dir.magnitude() > 0.001 {
                rotation.0 = -move_dir.z.atan2(move_dir.x) + std::f32::consts::PI / 2.0;
            }

            match target.move_state {
                PlayerMoveState::Grounded | PlayerMoveState::Launching => {
                    vel.0.x = move_dir.x * config.walk_speed;
                    vel.0.z = move_dir.z * config.walk_speed;
                }
                PlayerMoveState::CoyoteTime(_) => {
                    apply_air_steering(
                        vel,
                        move_dir,
                        config.walk_speed,
                        config.air_steer_speed,
                        dt,
                    );
                    if vel.0.y > 0.0 {
                        vel.0.y = 0.0;
                    }
                }
                PlayerMoveState::Airborne => {
                    apply_air_steering(
                        vel,
                        move_dir,
                        config.walk_speed,
                        config.air_steer_speed,
                        dt,
                    );
                }
            }

            // Debug display
            // let state_str = match target.move_state {
            //     PlayerMoveState::Grounded => "Grounded".to_string(),
            //     PlayerMoveState::Launching => "Launching".to_string(),
            //     PlayerMoveState::CoyoteTime(t) => format!("CoyoteTime({:.3})", t),
            //     PlayerMoveState::Airborne => "Airborne".to_string(),
            // };
            // debug_lines.add("Player/MoveState", state_str);
            // debug_lines.add("Player/VelY", format!("{:.2}", vel.0.y));
            // debug_lines.add("Player/Grounded", format!("{}", is_grounded));
        }
    }
}

fn apply_air_steering(
    vel: &mut Velocity,
    move_dir: Vector3<f32>,
    walk_speed: f32,
    air_steer_speed: f32,
    dt: f32,
) {
    let target_x = move_dir.x * walk_speed;
    let target_z = move_dir.z * walk_speed;
    let max_delta = air_steer_speed * dt;
    vel.0.x = move_toward(vel.0.x, target_x, max_delta);
    vel.0.z = move_toward(vel.0.z, target_z, max_delta);
}

fn move_toward(current: f32, target: f32, max_delta: f32) -> f32 {
    let diff = target - current;
    if diff.abs() <= max_delta {
        target
    } else {
        current + diff.signum() * max_delta
    }
}
