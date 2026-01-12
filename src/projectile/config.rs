//! Configuration for grenade throwing mechanics.

/// Configuration for grenade throwing and physics.
///
/// This is an ECS resource that can be tuned without code changes.
#[derive(Debug, Clone)]
pub struct GrenadeConfig {
    /// Initial throw speed (units/second).
    pub throw_speed: f32,
    /// Upward arc factor added to throw velocity.
    pub arc_factor: f32,
    /// Grenade collision radius.
    pub radius: f32,
    /// Gravity applied to grenades.
    pub gravity: f32,
    /// Maximum time before grenade despawns if it doesn't explode (seconds).
    pub max_lifetime: f32,
    /// Cooldown between throws (seconds).
    pub cooldown: f32,
}

impl Default for GrenadeConfig {
    fn default() -> Self {
        Self {
            throw_speed: 30.0,
            arc_factor: 2.0,
            radius: 0.15,
            gravity: 20.0,
            max_lifetime: 10.0,
            cooldown: 0.01,
        }
    }
}
