/// Configuration for player movement physics.
/// This is an ECS resource that can be tuned without code changes.
#[derive(Debug, Clone)]
pub struct PlayerConfig {
    /// Movement speed when on the ground
    pub walk_speed: f32,
    /// How quickly horizontal velocity steers toward input direction while airborne (units/s)
    pub air_steer_speed: f32,
    /// Upward velocity applied when jumping
    pub jump_speed: f32,
    /// Grace period after leaving ground where the character still behaves as grounded (seconds).
    /// Prevents ramp launches and enables coyote-time jumping.
    pub ground_grace_period: f32,
}

impl Default for PlayerConfig {
    fn default() -> Self {
        Self {
            walk_speed: 5.0,
            air_steer_speed: 8.0,
            jump_speed: 7.0,
            ground_grace_period: 0.08,
        }
    }
}
