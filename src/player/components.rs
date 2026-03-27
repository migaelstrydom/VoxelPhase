use nalgebra::{Point3, Vector3};
use specs::{Component, DenseVecStorage};

use crate::physics::RigidBodyHandle;

/// Marker component identifying the player entity.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Player;

/// Locomotion layer — how the player is moving through the world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LocomotionState {
    /// On the ground. Direct X/Z control, can jump.
    Grounded,
    /// Jump initiated but still in contact with ground. Direct X/Z control.
    Launching,
    /// Just walked off an edge. Air steering, Y clamped, can coyote-jump.
    /// The f32 is the remaining grace time.
    CoyoteTime(f32),
    /// In the air after jumping or after coyote time expired. Air steering, no Y clamp.
    Airborne,
}

impl Default for LocomotionState {
    fn default() -> Self {
        Self::Grounded
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
}

impl Default for PlayerState {
    fn default() -> Self {
        Self {
            locomotion: LocomotionState::Grounded,
            arm: ArmState::Idle,
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
            grab_held: false,
            grab_just_pressed: false,
            grab_just_released: false,
            throw: false,
            throw_grenade: false,
        }
    }
}
