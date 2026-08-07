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
/// A grenade detonates when either of two clocks runs out. The fuse is the
/// primary mechanism and always fires; impact detonation is an override for
/// hits hard enough to matter, and only arms once the grenade has left the
/// thrower's vicinity.
///
/// Deciding "hard enough" from the impulse the solver actually applied — rather
/// than from what was hit — keeps the rule material-agnostic: breakable and
/// yielding surfaces absorb the grenade because they transfer little momentum,
/// not because anything declares a grenade-specific response.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Grenade {
    /// Seconds left before the fuse burns down and the grenade detonates.
    pub fuse: f32,
    /// Seconds left before impact detonation arms.
    ///
    /// Stops a grenade fumbled against a nearby wall from going off in the
    /// thrower's face.
    pub arm_delay: f32,
}

impl Grenade {
    /// Create a grenade with the given fuse and arming delay, both in seconds.
    pub fn new(fuse: f32, arm_delay: f32) -> Self {
        Self { fuse, arm_delay }
    }

    /// Advance both clocks by `dt` seconds.
    pub fn tick(&mut self, dt: f32) {
        self.fuse -= dt;
        self.arm_delay -= dt;
    }

    /// Whether impact detonation is live yet.
    pub fn is_armed(&self) -> bool {
        self.arm_delay <= 0.0
    }

    /// Whether the fuse has burned down.
    pub fn fuse_expired(&self) -> bool {
        self.fuse <= 0.0
    }
}

impl Default for Grenade {
    fn default() -> Self {
        Self::new(2.5, 0.15)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_grenade_is_not_yet_armed_for_impact_detonation() {
        let grenade = Grenade::new(2.5, 0.15);
        assert!(!grenade.is_armed());
        assert!(!grenade.fuse_expired());
    }

    #[test]
    fn the_arming_delay_elapses_long_before_the_fuse() {
        let mut grenade = Grenade::new(2.5, 0.15);
        grenade.tick(0.2);
        assert!(grenade.is_armed());
        assert!(!grenade.fuse_expired());
    }

    #[test]
    fn the_fuse_expires_once_burned_down() {
        let mut grenade = Grenade::new(2.5, 0.15);
        // One tick past 2.5s; accumulated float error leaves the fuse a hair
        // above zero at exactly 150.
        for _ in 0..151 {
            grenade.tick(1.0 / 60.0);
        }
        assert!(grenade.fuse_expired());
    }
}
