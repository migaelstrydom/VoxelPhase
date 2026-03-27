use crate::biped::BipedController;
use crate::components::{Position, RigidBodyComponent, Rotation, Velocity, VelocityDriven};
use crate::debug::{DebugLines, DebugOverlays};
use crate::player::grab::{self, GrabConfig};
use crate::player::{
    ArmState, LocomotionState, Player, PlayerConfig, PlayerState, PlayerTargetState,
};
use crate::rendering::colour::Colour;
use crate::systems::PhysicsResource;
use crate::time::Time;
use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

/// Owns all `PlayerState` transitions (locomotion and arm) and applies physics
/// effects. Reads `PlayerTargetState` (intent) as input.
///
/// Locomotion FSM:
///   Grounded ──(jump)──────────► Launching ──(!is_grounded)──► Airborne
///   Grounded ──(!is_grounded)──► CoyoteTime ──(timer expired)──► Airborne
///   CoyoteTime ──(jump)──────────────────────────────────────► Airborne
///   CoyoteTime ──(is_grounded)──► Grounded
///   Airborne ──(is_grounded)────► Grounded
pub struct PlayerControlSystem;

impl<'a> System<'a> for PlayerControlSystem {
    type SystemData = (
        Read<'a, Time>,
        ReadExpect<'a, PlayerConfig>,
        ReadExpect<'a, GrabConfig>,
        Write<'a, PhysicsResource>,
        ReadStorage<'a, Player>,
        WriteStorage<'a, PlayerTargetState>,
        WriteStorage<'a, PlayerState>,
        ReadStorage<'a, BipedController>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, Rotation>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, VelocityDriven>,
        Write<'a, DebugLines>,
        Write<'a, DebugOverlays>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            config,
            grab_config,
            mut physics_res,
            players,
            mut player_targets,
            mut player_states,
            controllers,
            positions,
            rigid_bodies,
            mut rotations,
            mut velocities,
            mut velocity_driven,
            mut _debug_lines,
            mut debug_overlays,
        ) = data;
        let dt = time.delta_seconds();

        for (_player, target, state, controller, pos, rb, rotation, vel, vd) in (
            &players,
            &mut player_targets,
            &mut player_states,
            &controllers,
            &positions,
            &rigid_bodies,
            &mut rotations,
            &mut velocities,
            &mut velocity_driven,
        )
            .join()
        {
            let is_grounded = controller.is_grounded();
            let move_dir = target.direction;
            let player_body = rb.0;

            // --- Locomotion state transitions ---
            state.locomotion = match state.locomotion {
                LocomotionState::Grounded => {
                    if target.jump {
                        vel.0.y = config.jump_speed;
                        LocomotionState::Launching
                    } else if !is_grounded {
                        LocomotionState::CoyoteTime(config.ground_grace_period)
                    } else {
                        LocomotionState::Grounded
                    }
                }
                LocomotionState::Launching => {
                    if !is_grounded {
                        LocomotionState::Airborne
                    } else {
                        LocomotionState::Launching
                    }
                }
                LocomotionState::CoyoteTime(remaining) => {
                    if target.jump {
                        vel.0.y = config.jump_speed;
                        LocomotionState::Airborne
                    } else if is_grounded {
                        LocomotionState::Grounded
                    } else {
                        let t = remaining - dt;
                        if t <= 0.0 {
                            LocomotionState::Airborne
                        } else {
                            LocomotionState::CoyoteTime(t)
                        }
                    }
                }
                LocomotionState::Airborne => {
                    if is_grounded {
                        LocomotionState::Grounded
                    } else {
                        LocomotionState::Airborne
                    }
                }
            };

            // --- Per-state locomotion behaviour ---

            // Extract current yaw from the physics body's quaternion.
            let current_yaw = {
                let body = physics_res.world.body(player_body);
                body.map(|b| {
                    let forward = b.rotation() * Vector3::z();
                    forward.x.atan2(forward.z)
                })
                .unwrap_or(rotation.0)
            };

            // Write physics yaw to Rotation so biped/grab systems track the
            // physics body's actual facing, not a stale input-driven value.
            rotation.0 = current_yaw;

            // Compute target yaw from movement input and apply angular velocity
            // drive to turn the physics body. When not moving, angular velocity
            // is zero and the body holds its current facing.
            if move_dir.magnitude() > 0.001 {
                let target_yaw = -move_dir.z.atan2(move_dir.x) + std::f32::consts::PI / 2.0;
                let yaw_error = wrap_angle(target_yaw - current_yaw);
                vd.angular_velocity = Vector3::new(0.0, yaw_error * config.turn_aggression, 0.0);
            } else {
                vd.angular_velocity = Vector3::zeros();
            }

            match state.locomotion {
                LocomotionState::Grounded | LocomotionState::Launching => {
                    vel.0.x = move_dir.x * config.walk_speed;
                    vel.0.z = move_dir.z * config.walk_speed;
                }
                LocomotionState::CoyoteTime(_) => {
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
                LocomotionState::Airborne => {
                    apply_air_steering(
                        vel,
                        move_dir,
                        config.walk_speed,
                        config.air_steer_speed,
                        dt,
                    );
                }
            }

            // --- Arm state transitions ---
            let facing = facing_from_rotation(rotation.0);
            let player_pos = Point3::new(pos.0.x, pos.0.y, pos.0.z);

            let was_holding = matches!(state.arm, ArmState::Holding { .. });
            state.arm = match state.arm {
                ArmState::Idle => {
                    if target.grab_just_pressed {
                        grab::begin_reach(
                            &physics_res.world,
                            player_pos,
                            facing,
                            player_body,
                            &grab_config,
                        )
                    } else {
                        ArmState::Idle
                    }
                }

                ArmState::Reaching {
                    mut elapsed,
                    target: probe_target,
                } => {
                    elapsed += dt;
                    if elapsed >= grab_config.reach_duration {
                        if let Some((body, hit_point)) = probe_target {
                            if target.grab_held {
                                grab::finalize_grab(
                                    &mut physics_res.world,
                                    player_body,
                                    body,
                                    hit_point,
                                    &grab_config,
                                )
                                .unwrap_or(ArmState::Idle)
                            } else {
                                ArmState::Idle
                            }
                        } else {
                            ArmState::Idle
                        }
                    } else {
                        ArmState::Reaching {
                            elapsed,
                            target: probe_target,
                        }
                    }
                }

                ArmState::Holding {
                    target_body,
                    constraint,
                    ..
                } => {
                    if !grab::is_body_alive(&physics_res.world, target_body) {
                        grab::release(&mut physics_res.world, constraint);
                        ArmState::Idle
                    } else if target.grab_just_released || !target.grab_held {
                        grab::release(&mut physics_res.world, constraint);
                        ArmState::Idle
                    } else if target.throw {
                        grab::throw(
                            &mut physics_res.world,
                            target_body,
                            constraint,
                            facing,
                            grab_config.throw_impulse,
                        );
                        ArmState::Idle
                    } else {
                        let new_height = grab::update_lift(
                            &mut physics_res.world,
                            player_body,
                            target_body,
                            constraint,
                            dt,
                            &grab_config,
                        );
                        ArmState::Holding {
                            target_body,
                            constraint,
                            current_hold_height: new_height,
                        }
                    }
                }
            };

            // Resolve throw intent: if throw wasn't consumed by a grab-throw
            // (arm was Holding), pass it through as a grenade throw.
            if target.throw && !was_holding {
                target.throw_grenade = true;
            }

            // --- Debug visualization ---
            if grab_config.debug_draw {
                draw_grab_debug(
                    &mut debug_overlays,
                    &state.arm,
                    player_pos,
                    facing,
                    &grab_config,
                    &physics_res.world,
                );
            }
        }
    }
}

fn draw_grab_debug(
    overlays: &mut DebugOverlays,
    arm: &ArmState,
    player_pos: Point3<f32>,
    facing: Vector3<f32>,
    config: &GrabConfig,
    physics: &crate::physics::PhysicsWorld,
) {
    let probe_end = player_pos + facing * config.grab_range;
    let hold_point = grab::desired_hold_point(player_pos, facing, config);

    // Probe ray (cyan line from player to max grab range)
    overlays.add_line_with_radius(player_pos, probe_end, 0.01, Colour::rgb(0.0, 0.8, 0.8));

    // Probe tip sphere (shows swept-sphere radius at end)
    overlays.add_sphere(probe_end, config.probe_radius, Colour::rgb(0.0, 0.5, 0.5));

    // Hold point (where the object is pulled toward)
    overlays.add_sphere(hold_point, 0.05, Colour::YELLOW);

    match arm {
        ArmState::Idle => {}
        ArmState::Reaching { target, .. } => {
            if let Some((body_handle, hit_point)) = target {
                // Hit point on body surface (orange)
                overlays.add_sphere(*hit_point, 0.05, Colour::rgb(1.0, 0.5, 0.0));
                if let Some(body) = physics.body(*body_handle) {
                    // Targeted body CoM
                    overlays.add_sphere(body.position(), 0.04, Colour::rgb(1.0, 0.3, 0.0));
                }
            }
        }
        ArmState::Holding {
            target_body,
            constraint,
            ..
        } => {
            if let Some(c) = physics.constraint(*constraint) {
                if let crate::physics::ConstraintKind::FollowPoint {
                    body_a,
                    local_anchor_a,
                    local_anchor_b,
                    ..
                } = &c.kind
                {
                    // Hold point on player body (magenta)
                    if let Some(player) = physics.body(*body_a) {
                        let r_a = player.rotation() * local_anchor_a;
                        let hold = player.position() + r_a;
                        overlays.add_sphere(hold, 0.06, Colour::rgb(1.0, 0.0, 1.0));

                        // Grab point on held body (green)
                        if let Some(body) = physics.body(*target_body) {
                            let r_b = body.rotation() * local_anchor_b;
                            let grab_point = body.position() + r_b;
                            overlays.add_sphere(grab_point, 0.05, Colour::GREEN);
                            // Line from hold point to grab point (constraint stretch)
                            overlays.add_line_with_radius(
                                hold, grab_point, 0.01, Colour::GREEN,
                            );
                        }
                    }
                }
            }
        }
    }
}

/// Convert a Y-axis rotation angle to a facing direction vector (unit, XZ plane).
fn facing_from_rotation(rotation_y: f32) -> Vector3<f32> {
    Vector3::new(rotation_y.sin(), 0.0, rotation_y.cos())
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

/// Wrap an angle to the range [-π, π].
#[inline]
fn wrap_angle(angle: f32) -> f32 {
    let pi = std::f32::consts::PI;
    let mut a = angle % (2.0 * pi);
    if a > pi {
        a -= 2.0 * pi;
    } else if a < -pi {
        a += 2.0 * pi;
    }
    a
}
