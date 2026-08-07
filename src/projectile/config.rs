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
    ///
    /// A backstop only: the fuse normally detonates the grenade well before
    /// this.
    pub max_lifetime: f32,
    /// Seconds the fuse burns before the grenade detonates on its own.
    pub fuse_time: f32,
    /// Seconds after the throw before impact detonation arms.
    pub arm_delay: f32,
    /// How hard an impact must be to detonate the grenade, expressed as the
    /// head-on approach speed (m/s) that would produce it.
    ///
    /// Compared against the normal impulse the solver delivered over the frame,
    /// scaled by the grenade's mass — the impulse needed to arrest the grenade
    /// from this speed. Glancing blows, slow rolls and yielding surfaces stay
    /// below it; a direct throw at `throw_speed` clears it comfortably.
    pub detonation_impact_speed: f32,
    /// Cooldown between throws (seconds).
    pub cooldown: f32,
    /// Base upward angle offset for throws (radians). Added to throw direction.
    pub upward_offset: f32,
    /// How much camera pitch influences throw direction (0.0 = none, 1.0 = full).
    pub pitch_influence: f32,
}

impl Default for GrenadeConfig {
    fn default() -> Self {
        Self {
            throw_speed: 20.0,
            arc_factor: 0.0,
            radius: 0.2,
            gravity: 9.81,
            max_lifetime: 10.0,
            fuse_time: 2.5,
            arm_delay: 0.15,
            detonation_impact_speed: 6.0,
            cooldown: 0.01,
            upward_offset: 0.1,
            pitch_influence: 0.5,
        }
    }
}
