use nalgebra::{Point3, Vector3};
use specs::{Component, DenseVecStorage};

use crate::physics::RigidBodyHandle;
use crate::player::config::PlayerConfig;

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

/// Marker component identifying the player entity.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Player;

/// Air steering policy — how the player's horizontal velocity responds to
/// input while airborne.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AirSteering {
    /// Input-responsive: velocity steers toward move_dir at `air_steer_speed`.
    Responsive,
    /// Committed planar velocity (e.g. long jump). `remaining` seconds until
    /// the lock lapses and the player regains air control.
    Locked {
        velocity: Vector3<f32>,
        remaining: f32,
    },
}

/// Locomotion layer — how the player is moving through the world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LocomotionState {
    /// On the ground. Direct X/Z control, can jump.
    Grounded,
    /// Jump initiated but still in contact with ground. Holds the template
    /// for the air state it will transition into when `!is_grounded`.
    Launching {
        steering: AirSteering,
        allow_cutoff: bool,
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
    /// Player's current movement intent direction (unit, planar). Used by
    /// maneuvers that need a takeoff direction (e.g. long jump).
    pub move_dir: Vector3<f32>,
    /// True when sprint is held and crouch was pressed within the buffer
    /// window — the precondition for a long jump on this frame's jump press.
    pub long_jump_armed: bool,
    pub config: &'a PlayerConfig,
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
/// collapses to "steer toward a planar target velocity at some accel, and
/// optionally cancel upward vy." Set `accel = f32::INFINITY` to snap.
pub struct MovementRule {
    /// Planar target velocity (y unused; gravity owns vertical).
    pub target: Vector3<f32>,
    /// Units/s² toward the target. Infinity = snap instantly (committed lock).
    pub accel: f32,
    /// True for the CoyoteTime walk-off state: cancel positive vy so the
    /// player doesn't suddenly rise off a ramp at the edge.
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
                    };
                    out.set_vy = Some(cfg.jump_speed * cfg.long_jump_vertical_mul);
                    out.set_air_speed = Some(speed);
                    out.consumed_jump = true;
                } else if input.jump_pressed {
                    out.next_state = LocomotionState::Launching {
                        steering: AirSteering::Responsive,
                        allow_cutoff: true,
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
            } => {
                if !input.is_grounded {
                    out.next_state = LocomotionState::Airborne {
                        steering,
                        allow_cutoff,
                    };
                }
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
                    // Decay any locked-steering timer; lapse to Responsive on expiry.
                    let next_steering = match steering {
                        AirSteering::Locked {
                            velocity,
                            remaining,
                        } => {
                            let r = remaining - input.dt;
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
                    };
                    out.next_state = LocomotionState::Airborne {
                        steering: next_steering,
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
    /// `move_dir * speed` for steered states; for locked steering it's the
    /// committed velocity and `accel` is infinity (snap).
    pub fn movement_rule(
        &self,
        move_dir: Vector3<f32>,
        ground_speed: f32,
        ground_accel: f32,
        air_speed: f32,
        air_accel: f32,
    ) -> MovementRule {
        // Common helper: steered-toward-input rule with given speed/accel.
        let steered = |speed: f32, accel: f32| MovementRule {
            target: Vector3::new(move_dir.x * speed, 0.0, move_dir.z * speed),
            accel,
            clamp_up: false,
        };
        // Common helper: locked (snap) rule from a committed planar velocity.
        let locked = |velocity: Vector3<f32>| MovementRule {
            target: velocity,
            accel: f32::INFINITY,
            clamp_up: false,
        };

        match self {
            LocomotionState::Grounded => steered(ground_speed, ground_accel),
            LocomotionState::Launching { steering, .. } => match steering {
                AirSteering::Locked { velocity, .. } => locked(*velocity),
                AirSteering::Responsive => steered(ground_speed, ground_accel),
            },
            // Coyote time bridges one-frame ground-contact losses (terrain
            // seams) as well as real ledge walk-offs, so it must keep
            // ground handling. Steering with the air model here injects a
            // velocity perturbation at the seam-crossing rate — strong
            // enough to entrain gait timing. `clamp_up` still cancels any
            // upward velocity so a walk-off starts falling immediately.
            LocomotionState::CoyoteTime(_) => MovementRule {
                clamp_up: true,
                ..steered(ground_speed, ground_accel)
            },
            LocomotionState::Airborne { steering, .. } => match steering {
                AirSteering::Locked { velocity, .. } => locked(*velocity),
                AirSteering::Responsive => steered(air_speed, air_accel),
            },
        }
    }
}

/// Arm/action layer — what the player's hands are doing.
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
    /// Object attached via two-body constraint (player ↔ held body).
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

/// Layered player state machine holding two orthogonal state enums.
///
/// Locomotion and arm states are independent but subject to a compatibility
/// table — e.g. a future Swimming locomotion state would force-drop held objects.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct PlayerState {
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
}

impl Default for PlayerState {
    fn default() -> Self {
        Self {
            locomotion: LocomotionState::Grounded,
            arm: ArmState::Idle,
            air_speed: 0.0,
            jump_buffer: Timer::default(),
            crouch_buffer: Timer::default(),
            crouch_lockout: Timer::default(),
        }
    }
}

impl PlayerState {
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

/// Pure intent struct — carries what the player *wants* to do.
/// Written by `PlayerInputSystem`, read by `PlayerControlSystem`.
#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct PlayerTargetState {
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
    /// Resolved by PlayerControlSystem: true when throw should spawn a grenade.
    pub throw_grenade: bool,
}

impl Default for PlayerTargetState {
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
