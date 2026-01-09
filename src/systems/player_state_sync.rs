//! Syncs player state to rendering components.
//!
//! This keeps PlayerState as the authoritative source for player-specific
//! values while updating generic components used by the render system.

use crate::components::Rotation;
use crate::player::{Player, PlayerState};
use specs::{Join, ReadStorage, System, WriteStorage};

/// Copies player-specific state to generic rendering components.
pub struct PlayerStateSyncSystem;

impl<'a> System<'a> for PlayerStateSyncSystem {
    type SystemData = (
        ReadStorage<'a, Player>,
        ReadStorage<'a, PlayerState>,
        WriteStorage<'a, Rotation>,
    );

    fn run(&mut self, (players, player_states, mut rotations): Self::SystemData) {
        for (_player, state, rotation) in (&players, &player_states, &mut rotations).join() {
            rotation.0 = state.facing_direction;
        }
    }
}
