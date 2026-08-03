use specs::{Component, DenseVecStorage};

/// Marker component identifying the player entity.
///
/// Purely a selector: it tells `PlayerInputSystem` whose intent to fill and the
/// camera whom to follow. It confers no movement behaviour — that comes from
/// [`CharacterIntent`](crate::character::CharacterIntent) and
/// [`CharacterState`](crate::character::CharacterState), which creatures carry
/// too.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Player;
