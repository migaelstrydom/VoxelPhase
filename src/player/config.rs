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
    /// Gravity acceleration (positive value, applied downward)
    pub gravity: f32,
}

impl Default for PlayerConfig {
    fn default() -> Self {
        Self {
            walk_speed: 5.0,
            air_steer_speed: 8.0,
            jump_speed: 7.0,
            gravity: 9.81,
        }
    }
}
