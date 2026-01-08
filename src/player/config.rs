/// Configuration for player movement physics.
/// This is an ECS resource that can be tuned without code changes.
#[derive(Debug, Clone)]
pub struct PlayerConfig {
    /// Movement speed when on the ground
    pub walk_speed: f32,
    /// Movement speed when in the air (typically reduced for limited air control)
    pub air_speed: f32,
    /// Upward velocity applied when jumping
    pub jump_speed: f32,
    /// Gravity acceleration (positive value, applied downward)
    pub gravity: f32,
    /// Player collision radius
    pub radius: f32,
}

impl Default for PlayerConfig {
    fn default() -> Self {
        Self {
            walk_speed: 8.0,
            air_speed: 2.0,
            jump_speed: 12.0,
            gravity: 0.0,
            radius: 0.5,
        }
    }
}
