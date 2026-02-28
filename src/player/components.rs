use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

/// Marker component identifying the player entity.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Player;

/// Player movement state machine.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlayerMoveState {
    /// On the ground. Direct X/Z control, can jump.
    Grounded,
    /// Jump initiated but still in contact with ground. Direct X/Z control.
    /// Future home of jump wind-up animation.
    Launching,
    /// Just walked off an edge. Air steering, Y clamped, can coyote-jump.
    /// The f32 is the remaining grace time.
    CoyoteTime(f32),
    /// In the air after jumping or after coyote time expired. Air steering, no Y clamp.
    Airborne,
}

impl Default for PlayerMoveState {
    fn default() -> Self {
        Self::Grounded
    }
}

#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct PlayerTargetState {
    pub direction: Vector3<f32>,
    pub jump: bool,
    pub move_state: PlayerMoveState,
}

impl Default for PlayerTargetState {
    fn default() -> Self {
        Self {
            direction: Vector3::zeros(),
            jump: false,
            move_state: PlayerMoveState::Grounded,
        }
    }
}
