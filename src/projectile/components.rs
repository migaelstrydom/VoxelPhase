//! Projectile-related ECS components.

use specs::{Component, DenseVecStorage, VecStorage};

/// Marker component for projectile entities.
///
/// Entities with this component are subject to projectile physics
/// and collision detection.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Projectile;

/// Component for grenade-specific state.
///
/// Grenades explode either on impact with terrain or after a timeout.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Grenade {
    /// Whether the grenade is armed (will explode on impact).
    pub armed: bool,
}

impl Grenade {
    /// Create a new grenade that's immediately armed.
    pub fn new() -> Self {
        Self { armed: true }
    }
}

impl Default for Grenade {
    fn default() -> Self {
        Self::new()
    }
}

/// Generic lifetime component for automatic entity despawn.
///
/// When the remaining time reaches zero, the entity should be deleted.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Lifetime {
    /// Time remaining before despawn (seconds).
    pub remaining: f32,
}

impl Lifetime {
    /// Create a new lifetime with the given duration.
    pub fn new(seconds: f32) -> Self {
        Self { remaining: seconds }
    }
}
