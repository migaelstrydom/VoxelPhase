use nalgebra::{Point3, Vector3};
use specs::{Component, DenseVecStorage};

use super::config::LocomotionConfig;
use super::forgiveness::GroundForgiveness;
use crate::physics::RigidBodyHandle;

/// Countdown timer for input-grace windows (jump buffer, crouch buffer, lockouts).
/// Arm with a duration; it decays each frame until it hits zero.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Timer(f32);

impl Timer {
    pub fn arm(&mut self, duration: f32) {
        self.0 = duration;
    }
    pub fn tick(&mut self, dt: f32) {
        self.0 = (self.0 - dt).max(0.0);
    }
    pub fn active(&self) -> bool {
        self.0 > 0.0
    }
    pub fn clear(&mut self) {
        self.0 = 0.0;
    }
}

/// Air steering policy — how the character's horizontal velocity responds to
/// input while airborne.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AirSteering {
    /// Input-responsive: velocity steers toward move_dir at `air_steer_speed`.
    Responsive,
    /// Committed planar velocity (e.g. long jump). `remaining` seconds until
    /// the lock lapses and the character regains air control.
    Locked {
        velocity: Vector3<f32>,
        remaining: f32,
    },
}

impl AirSteering {
    /// Advance a committed lock by `dt`, lapsing to `Responsive` on expiry.
    ///
    /// Every state that can carry a lock must call this. A state that holds a
    /// `Locked` without decaying it pins the character to the committed
    /// velocity for as long as that state lasts.
    pub fn decayed(self, dt: f32) -> Self {
        match self {
            AirSteering::Locked {
                velocity,
                remaining,
            } => {
                let r = remaining - dt;
                if r <= 0.0 {
                    AirSteering::Responsive
                } else {
                    AirSteering::Locked {
                        velocity,
                        remaining: r,
                    }
                }
            }
            AirSteering::Responsive => AirSteering::Responsive,
        }
    }
}

/// Locomotion layer — how the character is moving through the world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LocomotionState {
    /// On the ground. Direct X/Z control, can jump.
    Grounded,
    /// Jump initiated but still in contact with ground. Holds the template
    /// for the air state it will transition into.
    ///
    /// `remaining` bounds the wait. Leaving the ground is the normal exit, but
    /// it cannot be the *only* one: this state snaps a committed velocity and
    /// handles no input, so a grounding signal that never goes false would
    /// strand the character here with the controls dead.
    Launching {
        steering: AirSteering,
        allow_cutoff: bool,
        remaining: f32,
    },
    /// Just walked off an edge. Air steering, Y clamped, can coyote-jump.
    /// The f32 is the remaining grace time.
    CoyoteTime(f32),
    /// In the air. `steering` decides input responsiveness (Responsive or a
    /// time-limited committed Lock). `allow_cutoff` gates variable-height jump.
    Airborne {
        steering: AirSteering,
        allow_cutoff: bool,
    },
}

impl Default for LocomotionState {
    fn default() -> Self {
        Self::Grounded
    }
}

/// Per-frame inputs to `LocomotionState::tick`.
pub struct LocomotionInput<'a> {
    pub dt: f32,
    pub is_grounded: bool,
    pub jump_pressed: bool,
    /// Current horizontal speed — latched into `air_speed` when leaving ground.
    pub horizontal_speed: f32,
    /// Character's current movement intent direction (unit, planar). Used by
    /// maneuvers that need a takeoff direction (e.g. long jump).
    pub move_dir: Vector3<f32>,
    /// True when sprint is held and crouch was pressed within the buffer
    /// window — the precondition for a long jump on this frame's jump press.
    pub long_jump_armed: bool,
    pub config: &'a LocomotionConfig,
}

/// Outcome of a locomotion tick. The driver applies these effects.
#[derive(Default)]
pub struct LocomotionOutcome {
    pub next_state: LocomotionState,
    /// If set, overwrite `vel.y` (used for jump impulses).
    pub set_vy: Option<f32>,
    /// If set, latch this as the new `air_speed` cap.
    pub set_air_speed: Option<f32>,
    /// True when this tick consumed the jump input (driver clears the buffer).
    pub consumed_jump: bool,
}

/// Horizontal movement rule for the current locomotion state. Every state
/// collapses to "ask for a planar target velocity, say whether the body may
/// be steered toward it while nothing holds it up, and optionally cancel
/// upward vy."
pub struct MovementRule {
    /// Planar target velocity (y unused; gravity owns vertical).
    pub target: Vector3<f32>,
    /// Rate the body may be steered toward `target` at while unsupported, in
    /// m/s². Spent against the actuator's allowance and no further.
    ///
    /// Nothing reads it while a character is supported: the traction rows ramp
    /// the body toward the target at the contact's own `μ·N`, so how quickly a
    /// walk starts is a property of the surface rather than a number the
    /// controller counts out. `None` is a state that wants no air authority at
    /// all — a committed long jump, whose arc is ballistic and stays that way
    /// because nothing corrects it, which is what snapping to the committed
    /// velocity every frame used to achieve the long way round.
    pub steer_accel: Option<f32>,
    /// True for the CoyoteTime walk-off state: cancel positive vy so the
    /// character doesn't suddenly rise off a ramp at the edge.
    pub clamp_up: bool,
}

impl LocomotionState {
    /// Compute state transition + entry effects. Pure w.r.t. world state — the
    /// driver applies `set_vy`/`set_air_speed` to the components.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionOutcome {
        let cfg = input.config;
        let mut out = LocomotionOutcome {
            next_state: self,
            ..Default::default()
        };
        match self {
            LocomotionState::Grounded => {
                if input.jump_pressed && input.long_jump_armed && input.move_dir.magnitude() > 0.001
                {
                    let dir = input.move_dir.normalize();
                    let speed = cfg.walk_speed * cfg.long_jump_speed_mul;
                    let planar = Vector3::new(dir.x * speed, 0.0, dir.z * speed);
                    out.next_state = LocomotionState::Launching {
                        steering: AirSteering::Locked {
                            velocity: planar,
                            remaining: cfg.long_jump_air_lock_duration,
                        },
                        allow_cutoff: false,
                        remaining: cfg.launch_window,
                    };
                    out.set_vy = Some(cfg.jump_speed * cfg.long_jump_vertical_mul);
                    out.set_air_speed = Some(speed);
                    out.consumed_jump = true;
                } else if input.jump_pressed {
                    out.next_state = LocomotionState::Launching {
                        steering: AirSteering::Responsive,
                        allow_cutoff: true,
                        remaining: cfg.launch_window,
                    };
                    out.set_vy = Some(cfg.jump_speed);
                    out.set_air_speed = Some(input.horizontal_speed.max(cfg.walk_speed));
                    out.consumed_jump = true;
                } else if !input.is_grounded {
                    out.next_state = LocomotionState::CoyoteTime(cfg.ground_grace_period);
                    out.set_air_speed = Some(input.horizontal_speed.max(cfg.walk_speed));
                }
            }
            LocomotionState::Launching {
                steering,
                allow_cutoff,
                remaining,
            } => {
                let steering = steering.decayed(input.dt);
                let r = remaining - input.dt;
                // The window expiring while still grounded is a jump that never
                // got off the floor. Going airborne anyway is right: the next
                // tick sees `is_grounded` and drops straight back to Grounded,
                // which restores control. Staying here would not.
                out.next_state = if !input.is_grounded || r <= 0.0 {
                    LocomotionState::Airborne {
                        steering,
                        allow_cutoff,
                    }
                } else {
                    LocomotionState::Launching {
                        steering,
                        allow_cutoff,
                        remaining: r,
                    }
                };
            }
            LocomotionState::CoyoteTime(remaining) => {
                if input.jump_pressed {
                    out.next_state = LocomotionState::Airborne {
                        steering: AirSteering::Responsive,
                        allow_cutoff: true,
                    };
                    out.set_vy = Some(cfg.jump_speed);
                    out.consumed_jump = true;
                } else if input.is_grounded {
                    out.next_state = LocomotionState::Grounded;
                } else {
                    let t = remaining - input.dt;
                    out.next_state = if t <= 0.0 {
                        LocomotionState::Airborne {
                            steering: AirSteering::Responsive,
                            allow_cutoff: true,
                        }
                    } else {
                        LocomotionState::CoyoteTime(t)
                    };
                }
            }
            LocomotionState::Airborne {
                steering,
                allow_cutoff,
            } => {
                if input.is_grounded {
                    out.next_state = LocomotionState::Grounded;
                } else {
                    out.next_state = LocomotionState::Airborne {
                        steering: steering.decayed(input.dt),
                        allow_cutoff,
                    };
                }
            }
        }
        out
    }

    /// Whether jump-cutoff (variable-height jump) applies in this state.
    /// Set by the jump variant at takeoff (regular jump: true, long jump: false).
    pub fn allows_jump_cutoff(&self) -> bool {
        match self {
            LocomotionState::Launching { allow_cutoff, .. }
            | LocomotionState::Airborne { allow_cutoff, .. } => *allow_cutoff,
            _ => false,
        }
    }

    /// True for states with a locked (committed) steering — long-jump arc etc.
    /// Used by the driver to detect landings that deserve follow-up effects.
    pub fn is_committed(&self) -> bool {
        matches!(
            self,
            LocomotionState::Launching {
                steering: AirSteering::Locked { .. },
                ..
            } | LocomotionState::Airborne {
                steering: AirSteering::Locked { .. },
                ..
            }
        )
    }

    /// Compute this tick's movement rule. `ground_speed` is the gait-resolved
    /// speed; `air_speed` is the latched takeoff cap. Planar target is
    /// `move_dir * speed` for steered states; for locked steering it is the
    /// committed velocity, with no authority to steer it anywhere else.
    pub fn movement_rule(
        &self,
        move_dir: Vector3<f32>,
        ground_speed: f32,
        air_speed: f32,
        air_accel: f32,
    ) -> MovementRule {
        // Common helper: steered-toward-input rule at the given speed. The
        // rate is the airborne one whatever the state, because it is only ever
        // spent while nothing is holding the character up.
        let steered = |speed: f32| MovementRule {
            target: Vector3::new(move_dir.x * speed, 0.0, move_dir.z * speed),
            steer_accel: Some(air_accel),
            clamp_up: false,
        };
        // Common helper: locked rule from a committed planar velocity. It asks
        // for no steering at all, which is what keeps the arc committed.
        let locked = |velocity: Vector3<f32>| MovementRule {
            target: velocity,
            steer_accel: None,
            clamp_up: false,
        };

        match self {
            LocomotionState::Grounded => steered(ground_speed),
            LocomotionState::Launching { steering, .. } => match steering {
                AirSteering::Locked { velocity, .. } => locked(*velocity),
                AirSteering::Responsive => steered(ground_speed),
            },
            // Coyote time bridges one-frame ground-contact losses (terrain
            // seams) as well as real ledge walk-offs, so it keeps the ground
            // speed: asking for the airborne cap here injects a velocity
            // perturbation at the seam-crossing rate, strong enough to entrain
            // gait timing. `clamp_up` cancels any upward velocity so a walk-off
            // starts falling immediately.
            LocomotionState::CoyoteTime(_) => MovementRule {
                clamp_up: true,
                ..steered(ground_speed)
            },
            LocomotionState::Airborne { steering, .. } => match steering {
                AirSteering::Locked { velocity, .. } => locked(*velocity),
                AirSteering::Responsive => steered(air_speed),
            },
        }
    }
}

/// Arm/action layer — what the character's hands are doing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArmState {
    /// Normal arm swing / idle.
    Idle,
    /// Right hand reaching toward grab point (animation plays regardless of hit).
    Reaching {
        elapsed: f32,
        /// Body and world-space hit point from the probe (if any object was in range).
        target: Option<(RigidBodyHandle, Point3<f32>)>,
    },
    /// Object attached via two-body constraint (character ↔ held body).
    Holding {
        target_body: RigidBodyHandle,
        constraint: crate::physics::ConstraintHandle,
        /// Current height of the hold point above pelvis (tracks the lift).
        /// Used by animation to position the hand at the actual hold height
        /// rather than the final target height.
        current_hold_height: f32,
    },
}

impl Default for ArmState {
    fn default() -> Self {
        Self::Idle
    }
}

/// Layered character state machine holding two orthogonal state enums.
///
/// Locomotion and arm states are independent but subject to a compatibility
/// table — e.g. a future Swimming locomotion state would force-drop held objects.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct CharacterState {
    pub locomotion: LocomotionState,
    pub arm: ArmState,
    /// Horizontal speed cap used by air steering. Latched at takeoff so
    /// sprint-jumps keep their speed even if Shift is released in the air.
    pub air_speed: f32,
    /// Jump-press grace: set when jump is pressed; decays; consumed on fire.
    pub jump_buffer: Timer,
    /// Crouch-press grace: while active, a jump triggers a long jump.
    pub crouch_buffer: Timer,
    /// Post-long-jump crouch suppression: while active, held Ctrl is ignored.
    pub crouch_lockout: Timer,
    /// Holds grounding true briefly after contact is lost, so the chatter of a
    /// real contact set does not reach the state machine.
    pub ground_forgiveness: GroundForgiveness,
}

impl Default for CharacterState {
    fn default() -> Self {
        Self {
            locomotion: LocomotionState::Grounded,
            arm: ArmState::Idle,
            air_speed: 0.0,
            jump_buffer: Timer::default(),
            crouch_buffer: Timer::default(),
            crouch_lockout: Timer::default(),
            ground_forgiveness: GroundForgiveness::default(),
        }
    }
}

impl CharacterState {
    /// Whether the given arm state is permitted with the current locomotion.
    pub fn arm_state_allowed(&self, arm: &ArmState) -> bool {
        match self.locomotion {
            // Swimming would force arm to Idle (drop held objects).
            // LocomotionState::Swimming => matches!(arm, ArmState::Idle),
            _ => {
                let _ = arm;
                true
            }
        }
    }
}

/// Pure intent struct — carries what the character *wants* to do.
///
/// This is the seam that makes a character source-agnostic: `PlayerInputSystem`
/// fills it from the keyboard, a creature's `BrainSystem` fills it from AI, and
/// `CharacterControlSystem` consumes it without knowing which wrote it. Fields
/// named after keys (`jump_held`, `crouch_just_pressed`) describe the *shape* of
/// the intent — a brain synthesises them just as legitimately as a keyboard.
#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct CharacterIntent {
    pub direction: Vector3<f32>,
    pub jump: bool,
    /// Jump key held this frame.
    pub jump_held: bool,
    /// Jump key released this frame (variable-height jump cutoff).
    pub jump_released: bool,
    /// Crouch modifier held (gait intent).
    pub crouch: bool,
    /// Crouch just pressed this frame (edge event for long-jump buffer).
    pub crouch_just_pressed: bool,
    /// Sprint modifier held (gait intent). Crouch wins over sprint.
    pub sprint: bool,
    /// Grab button held this frame.
    pub grab_held: bool,
    /// Grab button just pressed this frame.
    pub grab_just_pressed: bool,
    /// Grab button just released this frame.
    pub grab_just_released: bool,
    /// Throw action (just pressed this frame). Written by PlayerInputSystem.
    pub throw: bool,
    /// Resolved by CharacterControlSystem: true when throw should spawn a grenade.
    pub throw_grenade: bool,
}

impl Default for CharacterIntent {
    fn default() -> Self {
        Self {
            direction: Vector3::zeros(),
            jump: false,
            jump_held: false,
            jump_released: false,
            crouch: false,
            crouch_just_pressed: false,
            sprint: false,
            grab_held: false,
            grab_just_pressed: false,
            grab_just_released: false,
            throw: false,
            throw_grenade: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    fn input<'a>(
        config: &'a LocomotionConfig,
        is_grounded: bool,
        move_dir: Vector3<f32>,
    ) -> LocomotionInput<'a> {
        LocomotionInput {
            dt: DT,
            is_grounded,
            jump_pressed: false,
            horizontal_speed: 0.0,
            move_dir,
            long_jump_armed: false,
            config,
        }
    }

    fn long_jump(config: &LocomotionConfig) -> LocomotionState {
        let dir = Vector3::new(1.0, 0.0, 0.0);
        let mut start = input(config, true, dir);
        start.jump_pressed = true;
        start.long_jump_armed = true;
        let out = LocomotionState::Grounded.tick(&start);
        assert!(matches!(out.next_state, LocomotionState::Launching { .. }));
        out.next_state
    }

    /// A long jump that never reports leaving the ground used to be an
    /// absorbing state: `Launching` exited only on `!is_grounded`, so a
    /// grounding source stuck true pinned the character there forever. That
    /// state snaps horizontal velocity to the committed vector and handles no
    /// jump input, which is a total loss of control — the flat long-jump arc
    /// keeps the feet inside foot-probe range, so this is reachable in play.
    #[test]
    fn launching_cannot_be_held_by_grounding_that_never_goes_false() {
        let config = LocomotionConfig::player();
        let mut state = long_jump(&config);

        // Two full seconds of "still grounded" — far past any launch window.
        for _ in 0..120 {
            state = state
                .tick(&input(&config, true, Vector3::zeros()))
                .next_state;
        }

        assert!(
            !matches!(state, LocomotionState::Launching { .. }),
            "launch must time out even while grounding reports contact"
        );
    }

    /// Timing out into `Airborne` while still grounded is not a dead end:
    /// the very next tick lands, which is what restores input control.
    #[test]
    fn a_launch_that_never_left_the_ground_lands_again() {
        let config = LocomotionConfig::player();
        let mut state = long_jump(&config);

        for _ in 0..120 {
            state = state
                .tick(&input(&config, true, Vector3::zeros()))
                .next_state;
        }

        assert_eq!(state, LocomotionState::Grounded);
    }

    /// The steering lock is a duration, not a state: time spent in `Launching`
    /// has to count against it. Only `Airborne` used to decay it.
    #[test]
    fn the_committed_steering_lock_decays_during_launch() {
        let config = LocomotionConfig::player();
        let state = long_jump(&config);

        let after = state
            .tick(&input(&config, true, Vector3::zeros()))
            .next_state;

        match after {
            LocomotionState::Launching {
                steering: AirSteering::Locked { remaining, .. },
                ..
            } => assert!(
                remaining < config.long_jump_air_lock_duration,
                "lock did not decay: {remaining}"
            ),
            other => panic!("expected a still-locked launch, got {other:?}"),
        }
    }

    /// While locked, the rule asks for no steering authority at all and keeps
    /// the committed velocity — that is the mechanism by which a stuck launch
    /// reads as "keeps moving in a fixed direction, keys do nothing".
    #[test]
    fn a_locked_launch_ignores_the_movement_input() {
        let config = LocomotionConfig::player();
        let state = long_jump(&config);

        let rule = state.movement_rule(Vector3::new(-1.0, 0.0, 0.0), 5.0, 5.0, 8.0);

        assert_eq!(rule.steer_accel, None);
        assert!(rule.target.x > 0.0, "target follows takeoff, not input");
    }

    /// A plain jump is `Responsive`, so the launch window is the only thing
    /// holding it — and it must still let go.
    #[test]
    fn a_plain_jump_also_times_out_of_launch() {
        let config = LocomotionConfig::player();
        let mut start = input(&config, true, Vector3::zeros());
        start.jump_pressed = true;
        let mut state = LocomotionState::Grounded.tick(&start).next_state;

        for _ in 0..60 {
            state = state
                .tick(&input(&config, true, Vector3::zeros()))
                .next_state;
        }

        assert_eq!(state, LocomotionState::Grounded);
    }

    /// The normal path is unchanged: leaving the ground promotes to Airborne
    /// on the first ungrounded tick, carrying the lock across.
    #[test]
    fn leaving_the_ground_still_promotes_a_launch_immediately() {
        let config = LocomotionConfig::player();
        let state = long_jump(&config);

        let after = state
            .tick(&input(&config, false, Vector3::zeros()))
            .next_state;

        assert!(matches!(
            after,
            LocomotionState::Airborne {
                steering: AirSteering::Locked { .. },
                ..
            }
        ));
    }
}
