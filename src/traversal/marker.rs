use specs::{Component, NullStorage};

/// Marks the character whose way across the ground is planned: the player.
/// Apart from [`Player`](crate::player::Player), which says whose intent the
/// keyboard writes, so a tool that drives the player by script keeps it.
#[derive(Component, Debug, Default)]
#[storage(NullStorage)]
pub struct PlansTraversal;
