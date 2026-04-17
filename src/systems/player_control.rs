use crate::biped::BipedController;
use crate::components::{Position, RigidBodyComponent, Rotation, Velocity, VelocityDriven};
use crate::debug::{DebugLines, DebugOverlays};
use crate::player::grab::{self, GrabConfig};
use crate::player::{
    ArmState, LocomotionInput, LocomotionState, MovementRule, Player, PlayerConfig, PlayerState,
    PlayerTargetState,
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
            let horizontal_speed = (vel.0.x * vel.0.x + vel.0.z * vel.0.z).sqrt();

            // Tick all input-grace timers once per frame before use.
            state.jump_buffer.tick(dt);
            state.crouch_buffer.tick(dt);
            state.crouch_lockout.tick(dt);
            if target.jump {
                state.jump_buffer.arm(config.jump_buffer_window);
            }
            if target.crouch_just_pressed {
                state.crouch_buffer.arm(config.crouch_buffer_window);
            }

            // Snapshot "was in a committed maneuver" before the tick overwrites state.
            let was_committed = state.locomotion.is_committed();

            // --- Locomotion state transition ---
            let outcome = state.locomotion.tick(&LocomotionInput {
                dt,
                is_grounded,
                jump_pressed: state.jump_buffer.active(),
                horizontal_speed,
                move_dir,
                long_jump_armed: state.crouch_buffer.active(),
                config: &config,
            });
            state.locomotion = outcome.next_state;
            if let Some(vy) = outcome.set_vy {
                vel.0.y = vy;
            }
            if let Some(air) = outcome.set_air_speed {
                state.air_speed = air;
            }
            // Landing from a committed maneuver (long jump) arms the crouch
            // lockout so a still-held Ctrl doesn't instantly slow the player.
            if was_committed && matches!(state.locomotion, LocomotionState::Grounded) {
                state.crouch_lockout.arm(config.long_jump_crouch_lockout);
            }

            if outcome.consumed_jump {
                state.jump_buffer.clear();
                // Tap-then-land (buffered jump, button already released): apply
                // cutoff up-front so the hop is short. Skip committed maneuvers.
                if !target.jump_held && state.locomotion.allows_jump_cutoff() {
                    vel.0.y *= config.jump_cutoff_factor;
                }
            }

            // Variable-height jump: cut upward velocity on early release.
            if target.jump_released && state.locomotion.allows_jump_cutoff() && vel.0.y > 0.0 {
                vel.0.y *= config.jump_cutoff_factor;
            }

            // Compute ground speed AFTER lockout tick + landing so a long-jump
            // landing frame already sees crouch suppressed.
            let ground_speed = resolve_ground_speed(target, &config, &state.crouch_lockout);

            // --- Yaw drive (physics body → Rotation, input → angular velocity) ---
            let current_yaw = {
                let body = physics_res.world.body(player_body);
                body.map(|b| {
                    let forward = b.rotation() * Vector3::z();
                    forward.x.atan2(forward.z)
                })
                .unwrap_or(rotation.0)
            };
            rotation.0 = current_yaw;

            if move_dir.magnitude() > 0.001 {
                let target_yaw = -move_dir.z.atan2(move_dir.x) + std::f32::consts::PI / 2.0;
                let yaw_error = wrap_angle(target_yaw - current_yaw);
                vd.angular_velocity = Vector3::new(0.0, yaw_error * config.turn_aggression, 0.0);
            } else {
                vd.angular_velocity = Vector3::zeros();
            }

            // --- Apply the movement rule for the resolved locomotion state ---
            apply_movement_rule(
                vel,
                state.locomotion.movement_rule(
                    move_dir,
                    ground_speed,
                    config.ground_accel,
                    state.air_speed,
                    config.air_steer_speed,
                ),
                dt,
            );

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
                            overlays.add_line_with_radius(hold, grab_point, 0.01, Colour::GREEN);
                        }
                    }
                }
            }
        }
    }
}

/// Resolve effective ground speed from gait intent. Crouch wins over sprint,
/// except while the crouch lockout is active (post-long-jump recovery), in
/// which case crouch is ignored so holding Ctrl doesn't brake the player.
fn resolve_ground_speed(
    target: &PlayerTargetState,
    config: &PlayerConfig,
    crouch_lockout: &crate::player::Timer,
) -> f32 {
    let crouch_active = target.crouch && !crouch_lockout.active();
    let mul = if crouch_active {
        config.crouch_speed_mul
    } else if target.sprint {
        config.sprint_speed_mul
    } else {
        1.0
    };
    config.walk_speed * mul
}

/// Apply a `MovementRule` to a velocity: steer planar velocity toward the
/// rule's target at `accel` (infinite = snap), and optionally cancel any
/// positive y component.
fn apply_movement_rule(vel: &mut Velocity, rule: MovementRule, dt: f32) {
    if rule.accel.is_infinite() {
        vel.0.x = rule.target.x;
        vel.0.z = rule.target.z;
    } else {
        let max_delta = rule.accel * dt;
        vel.0.x = move_toward(vel.0.x, rule.target.x, max_delta);
        vel.0.z = move_toward(vel.0.z, rule.target.z, max_delta);
    }
    if rule.clamp_up && vel.0.y > 0.0 {
        vel.0.y = 0.0;
    }
}

/// Convert a Y-axis rotation angle to a facing direction vector (unit, XZ plane).
fn facing_from_rotation(rotation_y: f32) -> Vector3<f32> {
    Vector3::new(rotation_y.sin(), 0.0, rotation_y.cos())
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
