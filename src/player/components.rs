use specs::{Component, DenseVecStorage};

/// Marker component identifying the player entity.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Player;

/// Tracks the player's physical state for gameplay logic.
#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct PlayerState {
    /// True when the player is standing on solid ground
    pub on_ground: bool,
    /// Direction the player is facing (radians around Y axis)
    pub facing_direction: f32,
}

impl Default for PlayerState {
    fn default() -> Self {
        Self {
            on_ground: false,
            facing_direction: 0.0,
        }
    }
}
