/// Configuration for player movement physics.
/// This is an ECS resource that can be tuned without code changes.
#[derive(Debug, Clone)]
pub struct PlayerConfig {
    /// Movement speed when on the ground
    pub walk_speed: f32,
    /// Movement speed when in the air (typically reduced for limited air control)
    pub air_acceleration: f32,
    /// Upward velocity applied when jumping
    pub jump_speed: f32,
    /// Gravity acceleration (positive value, applied downward)
    pub gravity: f32,
    /// Maximum falling speed (terminal velocity) - prevents tunneling through terrain
    pub terminal_velocity: f32,
}

impl Default for PlayerConfig {
    fn default() -> Self {
        Self {
            walk_speed: 1.0,
            air_acceleration: 8.0,
            jump_speed: 12.0,
            gravity: 20.0,
            terminal_velocity: 30.0, // Safe value: should be < radius / typical_frame_time
        }
    }
}
