use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

/// Marker component identifying the player entity.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Player;

#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct PlayerTargetState {
    pub direction: Vector3<f32>,
    pub jump: bool,
}

impl Default for PlayerTargetState {
    fn default() -> Self {
        Self {
            direction: Vector3::zeros(),
            jump: false,
        }
    }
}
