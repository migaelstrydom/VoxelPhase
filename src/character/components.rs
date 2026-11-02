use nalgebra::{Point3, Vector3};
use specs::{Component, DenseVecStorage};

use super::ledge::{Grip, SensedEdge};
use super::locomotion::{Conditioning, LocomotionState, PlannedLanding, PlannedTakeoff};
use crate::physics::RigidBodyHandle;

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
/// table (`arm_state_allowed`): a swimmer's arms hold nothing.
#[derive(Component, Debug, Clone, Default)]
#[storage(DenseVecStorage)]
pub struct CharacterState {
    pub locomotion: LocomotionState,
    pub arm: ArmState,
    /// Horizontal speed cap used by air steering. Latched at takeoff so
    /// sprint-jumps keep their speed even if Shift is released in the air.
    pub air_speed: f32,
    /// The buffers, ground forgiveness and lockout that turn the intent and
    /// the contacts into what the state machine reads.
    pub conditioning: Conditioning,
    /// How far the body is asked to lie over from upright, toward its
    /// heading, in radians. Eased toward the swimming state's pitch and back;
    /// handed to the engine by `AttitudeControl`.
    pub body_pitch: f32,
    /// The hands' hold on a ledge: the joint, while the state is a Ledge.
    pub grip: Option<Grip>,
}

impl CharacterState {
    /// Whether the given arm state is permitted with the current locomotion.
    pub fn arm_state_allowed(&self, arm: &ArmState) -> bool {
        match self.locomotion {
            // A swimmer's arms are stroking and a hanger's are on the edge:
            // nothing can be held.
            LocomotionState::Swim(_) | LocomotionState::Ledge(_) => matches!(arm, ArmState::Idle),
            LocomotionState::OnFoot(_)
            | LocomotionState::Hop(_)
            | LocomotionState::Jump(_)
            | LocomotionState::Fall(_) => true,
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
    /// Horizontal direction to throw in, for a brain that aims somewhere
    /// other than where it walks. `None` leaves the aim to the camera when
    /// it follows this character, and to the facing otherwise.
    pub aim: Option<Vector3<f32>>,
    /// A takeoff planned for this frame: a traversal plan's hop or roll-off.
    /// Consumed by `CharacterControlSystem`; a jump pressed the same frame
    /// wins.
    pub takeoff: Option<PlannedTakeoff>,
    /// Where the plan now lands a Hop under way. Consumed by
    /// `CharacterControlSystem`.
    pub landing: Option<PlannedLanding>,
    /// The edge at the hands, from `LedgeSensorSystem`. Consumed by
    /// `CharacterControlSystem`.
    pub edge: Option<SensedEdge>,
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
            aim: None,
            takeoff: None,
            landing: None,
            edge: None,
        }
    }
}
