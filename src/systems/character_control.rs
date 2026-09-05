use crate::character::grab::{self, GrabConfig};
use crate::character::{
    ArmState, CharacterIntent, CharacterState, Grounding, LocomotionConfig, LocomotionInput,
    LocomotionState, MovementRule,
};
use crate::components::{Position, RigidBodyComponent, Rotation};
use crate::debug::DebugOverlays;
use crate::drive::{Actuator, BodyMotion, DriveIntent};
use crate::rendering::colour::Colour;
use crate::systems::PhysicsResource;
use crate::time::Time;
use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

/// Owns all `CharacterState` transitions (locomotion and arm) and applies
/// physics effects. Reads `CharacterIntent` as input and `Grounding` as the
/// world's answer.
///
/// This system is deliberately blind to *who* it is driving: it joins on the
/// locomotion components alone, so the player and every AI creature carrying
/// them go through exactly the same FSM, coyote time, jump buffering and
/// grabbing. Whoever fills the intent decides the behaviour.
///
/// Locomotion FSM:
///   Grounded ──(jump)──────────► Launching ──(!is_grounded)──► Airborne
///   Grounded ──(!is_grounded)──► CoyoteTime ──(timer expired)──► Airborne
///   CoyoteTime ──(jump)──────────────────────────────────────► Airborne
///   CoyoteTime ──(is_grounded)──► Grounded
///   Airborne ──(is_grounded)────► Grounded
pub struct CharacterControlSystem;

impl<'a> System<'a> for CharacterControlSystem {
    type SystemData = (
        Read<'a, Time>,
        ReadExpect<'a, GrabConfig>,
        Write<'a, PhysicsResource>,
        WriteStorage<'a, CharacterIntent>,
        WriteStorage<'a, CharacterState>,
        ReadStorage<'a, LocomotionConfig>,
        ReadStorage<'a, Grounding>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, Rotation>,
        WriteStorage<'a, DriveIntent>,
        ReadStorage<'a, Actuator>,
        ReadStorage<'a, BodyMotion>,
        Write<'a, DebugOverlays>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            grab_config,
            mut physics_res,
            mut intents,
            mut character_states,
            configs,
            groundings,
            positions,
            rigid_bodies,
            mut rotations,
            mut drive_intents,
            actuators,
            body_motions,
            mut debug_overlays,
        ) = data;
        let dt = time.delta_seconds();

        // The axis the vertical verbs and the gait's speed measure are taken
        // along. A world with no gravity has no up, and then every axis is a
        // walking axis.
        let up = physics_res
            .world
            .config()
            .gravity_direction()
            .map(|down| -down.into_inner())
            .unwrap_or_else(Vector3::zeros);

        for (target, state, config, grounding, pos, rb, rotation, drive, motion, _) in (
            &mut intents,
            &mut character_states,
            &configs,
            &groundings,
            &positions,
            &rigid_bodies,
            &mut rotations,
            &mut drive_intents,
            &body_motions,
            &actuators,
        )
            .join()
        {
            // Grounding is one contact-derived answer for every body, and a
            // contact set chatters where a probe's reach did not: a walking
            // capsule leaves the floor between footfalls. The forgiveness
            // window is where that leniency now lives, named and tunable.
            let support =
                state
                    .ground_forgiveness
                    .observe(grounding, dt, config.ground_forgiveness_window);
            let is_grounded = support.is_grounded;
            let move_dir = target.direction;
            let character_body = rb.0;

            let horizontal_speed = motion.speed_across(&up);

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
                config,
            });
            state.locomotion = outcome.next_state;
            if let Some(vy) = outcome.set_vy {
                drive.jump(vy);
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
                // Leaving the ground on purpose is not a loss of support to
                // forgive. Left armed, the window would report the takeoff
                // contact into the air.
                state.ground_forgiveness.cancel();
                // Tap-then-land (buffered jump, button already released): apply
                // cutoff up-front so the hop is short. Skip committed maneuvers.
                if !target.jump_held && state.locomotion.allows_jump_cutoff() {
                    drive.cut_normal(config.jump_cutoff_factor);
                }
            }

            // Variable-height jump: cut upward velocity on early release. The
            // cut acts on a rise only, so there is nothing to test here — a
            // falling character has no jump left to shorten.
            if target.jump_released && state.locomotion.allows_jump_cutoff() {
                drive.cut_normal(config.jump_cutoff_factor);
            }

            // Compute ground speed AFTER lockout tick + landing so a long-jump
            // landing frame already sees crouch suppressed.
            let ground_speed = resolve_ground_speed(target, config, &state.crouch_lockout);

            // --- Yaw drive (physics body → Rotation, input → angular velocity) ---
            let current_yaw = {
                let body = physics_res.world.body(character_body);
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
                drive.angular_target = Vector3::new(0.0, yaw_error * config.turn_aggression, 0.0);
            } else {
                drive.angular_target = Vector3::zeros();
            }

            // --- Apply the movement rule for the resolved locomotion state ---
            //
            // The rule states a speed relative to whatever is holding the
            // character up, because that is what a walk is: asking for 0 m/s
            // while standing on a moving platform is asking to be dragged off
            // the back of it. On static ground the two frames are the same and
            // this adds nothing.
            let mut rule = state.locomotion.movement_rule(
                move_dir,
                ground_speed,
                state.air_speed,
                config.air_steer_speed,
            );
            if is_grounded {
                rule.target += across(support.surface_velocity, &up);
            }
            apply_movement_rule(drive, rule);

            // --- Arm state transitions ---
            let facing = facing_from_rotation(rotation.0);
            let character_pos = Point3::new(pos.0.x, pos.0.y, pos.0.z);

            let was_holding = matches!(state.arm, ArmState::Holding { .. });
            state.arm = match state.arm {
                ArmState::Idle => {
                    if target.grab_just_pressed {
                        grab::begin_reach(
                            &physics_res.world,
                            character_pos,
                            facing,
                            character_body,
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
                                    character_body,
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
                            character_body,
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
                    character_pos,
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
    character_pos: Point3<f32>,
    facing: Vector3<f32>,
    config: &GrabConfig,
    physics: &crate::physics::PhysicsWorld,
) {
    let probe_end = character_pos + facing * config.grab_range;
    let hold_point = grab::desired_hold_point(character_pos, facing, config);

    // Probe ray (cyan line from character to max grab range)
    overlays.add_line_with_radius(character_pos, probe_end, 0.01, Colour::rgb(0.0, 0.8, 0.8));

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
                    // Hold point on holder body (magenta)
                    if let Some(holder) = physics.body(*body_a) {
                        let r_a = holder.rotation() * local_anchor_a;
                        let hold = holder.position() + r_a;
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
pub fn resolve_ground_speed(
    target: &CharacterIntent,
    config: &LocomotionConfig,
    crouch_lockout: &crate::character::Timer,
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

/// Turn a `MovementRule` into a linear drive target: steer the planar axes
/// from where the body actually is toward the rule's target at `accel`
/// (infinite = snap), and optionally cancel any rise.
///
/// The vertical axis is not this system's to command. The target carries the
/// measured value forward unchanged, so the drive asks for no vertical
/// authority at all and gravity keeps the axis to itself — only a jump verb
/// says otherwise.
/// Command the movement rule's target, as a velocity **relative to whatever is
/// holding the character up**.
///
/// The rule's target goes down untouched. It is not blended with the measured
/// velocity and it is not rate-limited here, because both jobs moved into the
/// engine: the tangential row at each supporting contact drives relative
/// velocity toward this target under the contact's own `μ·N`, so the ramp from
/// standstill to walk speed is the traction budget rather than a number this
/// system counts out. Reading the measurement to steer a target back toward
/// itself was the read-modify-write R5 exists to remove, and with the target
/// now support-relative it would also be the wrong quantity: an idle passenger
/// on a running deck asks for zero and is carried.
///
/// The rate travels with it for the one case the contacts cannot answer: with
/// nothing underneath, the same target is chased out of the actuator's
/// allowance instead, at this rate and no faster.
fn apply_movement_rule(drive: &mut DriveIntent, rule: MovementRule) {
    drive.linear_target = rule.target;
    drive.steer_accel = rule.steer_accel;
    if rule.clamp_up {
        drive.clamp_normal_rise();
    }
}

/// The part of `velocity` that lies across `up` — the plane a gait is walked
/// in. Zero `up` (a world with no gravity) leaves the velocity untouched,
/// which is the same answer every other planar term gives there.
fn across(velocity: Vector3<f32>, up: &Vector3<f32>) -> Vector3<f32> {
    velocity - up * velocity.dot(up)
}

/// Convert a Y-axis rotation angle to a facing direction vector (unit, XZ plane).
fn facing_from_rotation(rotation_y: f32) -> Vector3<f32> {
    Vector3::new(rotation_y.sin(), 0.0, rotation_y.cos())
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::Grounding;
    use crate::components::Orientation;
    use crate::physics::RigidBodyDesc;
    use specs::{Builder, Entity, RunNow, World, WorldExt};

    /// The gait rule the FSM hands to the driver while walking east.
    fn walk_east(steer_accel: f32) -> MovementRule {
        MovementRule {
            target: Vector3::new(5.0, 0.0, 0.0),
            steer_accel: Some(steer_accel),
            clamp_up: false,
        }
    }

    /// The gait's target goes down as commanded. Under the traction drive the
    /// ramp toward it is the contact's own budget, so there is nothing for this
    /// system to steer from and no measurement it needs to read.
    #[test]
    fn the_gaits_target_is_commanded_as_it_stands() {
        let mut drive = DriveIntent::default();
        apply_movement_rule(&mut drive, walk_east(8.0));
        assert_eq!(drive.linear_target, Vector3::new(5.0, 0.0, 0.0));
    }

    /// The rate rides along with the target for the airborne case, and a
    /// committed arc asks for none.
    #[test]
    fn the_steering_rate_crosses_with_the_target() {
        let mut drive = DriveIntent::default();
        apply_movement_rule(&mut drive, walk_east(8.0));
        assert_eq!(drive.steer_accel, Some(8.0));

        apply_movement_rule(
            &mut drive,
            MovementRule {
                steer_accel: None,
                ..walk_east(8.0)
            },
        );
        assert_eq!(drive.steer_accel, None);
    }

    /// The target is support-relative, so a zero is "hold station on whatever
    /// I am standing on" rather than "come to rest in the world". That is the
    /// whole of R4, and it is why an idle passenger rides a moving deck.
    #[test]
    fn a_released_stick_asks_to_hold_station_on_the_support() {
        let mut drive = DriveIntent::default();
        apply_movement_rule(
            &mut drive,
            MovementRule {
                target: Vector3::zeros(),
                ..walk_east(8.0)
            },
        );
        assert_eq!(drive.linear_target, Vector3::zeros());
    }

    #[test]
    fn a_walk_off_asks_for_the_rise_to_be_cancelled() {
        let mut drive = DriveIntent::default();
        apply_movement_rule(
            &mut drive,
            MovementRule {
                clamp_up: true,
                ..walk_east(8.0)
            },
        );
        // The verb is a command, not an edit: the projection is what cancels
        // the rise, and the planar target is untouched by it.
        assert_eq!(drive.linear_target.x, 5.0);
        assert!(drive.normal_projection.clamp_positive);
    }

    /// A world holding one grounded character, with everything
    /// `CharacterControlSystem` reads present.
    fn character_world(actuated: bool) -> (World, Entity) {
        let mut world = World::new();
        world.register::<CharacterIntent>();
        world.register::<CharacterState>();
        world.register::<LocomotionConfig>();
        world.register::<Grounding>();
        world.register::<Position>();
        world.register::<RigidBodyComponent>();
        world.register::<Rotation>();
        world.register::<DriveIntent>();
        world.register::<Actuator>();
        world.register::<BodyMotion>();
        world.register::<Orientation>();
        world.insert(Time::new());
        world.insert(GrabConfig::default());
        world.insert(DebugOverlays::default());

        let mut physics = PhysicsResource::default();
        let body = physics.world.create_body(RigidBodyDesc::dynamic());
        world.insert(physics);

        let mut builder = world
            .create_entity()
            .with(CharacterIntent::default())
            .with(CharacterState::default())
            .with(LocomotionConfig::player())
            .with(Grounding::on(Vector3::y()))
            .with(Position(Vector3::zeros()))
            .with(RigidBodyComponent(body))
            .with(Rotation(0.0))
            .with(DriveIntent::default())
            .with(BodyMotion::default());
        if actuated {
            builder = builder.with(Actuator::character());
        }
        let entity = builder.build();
        (world, entity)
    }

    fn press_jump(world: &World, entity: Entity) {
        world
            .write_storage::<CharacterIntent>()
            .get_mut(entity)
            .unwrap()
            .jump = true;
    }

    fn intent_of(world: &World, entity: Entity) -> DriveIntent {
        world
            .read_storage::<DriveIntent>()
            .get(entity)
            .expect("the character keeps its drive intent")
            .clone()
    }

    #[test]
    fn a_jump_travels_as_a_verb_and_not_as_a_velocity() {
        let (world, entity) = character_world(true);
        world
            .write_storage::<BodyMotion>()
            .get_mut(entity)
            .unwrap()
            .linear = Vector3::new(0.0, -0.5, 0.0);
        press_jump(&world, entity);

        CharacterControlSystem.run_now(&world);

        let drive = intent_of(&world, entity);
        let config = LocomotionConfig::player();
        assert_eq!(drive.normal_impulse, Some(config.jump_speed));
        // The continuous channel never learns about the jump. It no longer
        // carries the measured fall either: the target is the gait's planar
        // ask, stated relative to the support, and the vertical axis belongs
        // to gravity and to the verb.
        assert_eq!(drive.linear_target.y, 0.0);
    }

    fn set_grounding(world: &World, entity: Entity, grounding: Grounding) {
        let _ = world
            .write_storage::<Grounding>()
            .insert(entity, grounding)
            .expect("the character keeps its grounding");
    }

    /// The passenger case, end to end: no input, standing on a deck running
    /// east. Asking for a world-frame standstill would have the drive brake
    /// against the platform until the character slid off the back of it.
    #[test]
    fn an_idle_passenger_is_commanded_to_ride_the_deck() {
        let (world, entity) = character_world(true);
        let deck = Vector3::new(3.0, 0.0, 0.0);
        set_grounding(&world, entity, Grounding::on(Vector3::y()).carried_by(deck));

        CharacterControlSystem.run_now(&world);

        assert_eq!(intent_of(&world, entity).linear_target, deck);
    }

    /// And walking on it is walking *on it*: the gait's speed is added to the
    /// deck's, so the same stick input produces the same gait wherever it is
    /// walked.
    #[test]
    fn a_walk_on_a_moving_deck_is_the_walk_plus_the_deck() {
        let (world, entity) = character_world(true);
        let deck = Vector3::new(3.0, 0.0, 0.0);
        set_grounding(&world, entity, Grounding::on(Vector3::y()).carried_by(deck));
        world
            .write_storage::<CharacterIntent>()
            .get_mut(entity)
            .unwrap()
            .direction = Vector3::new(0.0, 0.0, 1.0);

        CharacterControlSystem.run_now(&world);

        let walk = LocomotionConfig::player().walk_speed;
        let target = intent_of(&world, entity).linear_target;
        assert!((target - Vector3::new(3.0, 0.0, walk)).magnitude() < 1e-4);
    }

    /// Nothing carries an airborne character. A deck's velocity reaching a
    /// jump would be free momentum granted every frame of the flight.
    #[test]
    fn a_character_with_nothing_under_it_is_carried_by_nothing() {
        let (world, entity) = character_world(true);
        set_forgiveness_window(&world, entity, 0.0);
        set_grounding(
            &world,
            entity,
            Grounding::airborne().carried_by(Vector3::new(3.0, 0.0, 0.0)),
        );

        CharacterControlSystem.run_now(&world);

        assert_eq!(intent_of(&world, entity).linear_target, Vector3::zeros());
    }

    fn set_forgiveness_window(world: &World, entity: Entity, window: f32) {
        world
            .write_storage::<LocomotionConfig>()
            .get_mut(entity)
            .unwrap()
            .ground_forgiveness_window = window;
    }

    fn locomotion_of(world: &World, entity: Entity) -> LocomotionState {
        world
            .read_storage::<CharacterState>()
            .get(entity)
            .unwrap()
            .locomotion
    }

    #[test]
    fn a_frame_without_contact_does_not_leave_the_ground() {
        // What the forgiveness window is for: a walking capsule loses its
        // contacts between footfalls, and the state machine must not read
        // that as walking off a ledge.
        let (world, entity) = character_world(true);
        CharacterControlSystem.run_now(&world);
        set_grounding(&world, entity, Grounding::airborne());

        CharacterControlSystem.run_now(&world);

        assert_eq!(locomotion_of(&world, entity), LocomotionState::Grounded);
    }

    #[test]
    fn without_a_window_the_same_frame_is_a_walk_off() {
        // The same input with the window closed, so the test above is
        // measuring the window rather than something else.
        let (world, entity) = character_world(true);
        set_forgiveness_window(&world, entity, 0.0);
        CharacterControlSystem.run_now(&world);
        set_grounding(&world, entity, Grounding::airborne());

        CharacterControlSystem.run_now(&world);

        assert!(matches!(
            locomotion_of(&world, entity),
            LocomotionState::CoyoteTime(_)
        ));
    }

    #[test]
    fn a_jump_spends_no_forgiveness_on_the_way_up() {
        // Leaving on purpose is not a loss of support: the window must not
        // hold a jump in `Launching`, or report ground once it is airborne.
        let (world, entity) = character_world(true);
        press_jump(&world, entity);
        CharacterControlSystem.run_now(&world);
        assert!(matches!(
            locomotion_of(&world, entity),
            LocomotionState::Launching { .. }
        ));

        set_grounding(&world, entity, Grounding::airborne());
        CharacterControlSystem.run_now(&world);

        assert!(matches!(
            locomotion_of(&world, entity),
            LocomotionState::Airborne { .. }
        ));
    }

    #[test]
    fn an_unactuated_body_is_not_driven() {
        // What `DeathSystem` relies on: take the actuator away and the FSM
        // stops writing commands, without the entity losing anything else.
        let (world, entity) = character_world(false);
        press_jump(&world, entity);

        CharacterControlSystem.run_now(&world);

        let drive = intent_of(&world, entity);
        assert_eq!(drive.normal_impulse, None);
        assert_eq!(drive.linear_target, Vector3::zeros());
    }
}
