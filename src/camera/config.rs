use super::clearance::CameraClearance;

/// Configuration for camera behavior.
/// This is an ECS resource that can be tuned without code changes.
#[derive(Debug, Clone)]
pub struct CameraConfig {
    /// How quickly the camera follows the target (higher = snappier)
    /// This is used as an exponential decay factor
    pub follow_speed: f32,

    /// Minimum distance from target
    pub min_distance: f32,

    /// Maximum distance from target
    pub max_distance: f32,

    /// Default/starting distance from target
    pub default_distance: f32,

    /// Mouse sensitivity for horizontal orbit (radians per pixel)
    pub orbit_sensitivity: f32,

    /// Mouse sensitivity for vertical height adjustment
    pub height_sensitivity: f32,

    /// Keyboard zoom speed (units per second)
    pub zoom_speed: f32,

    /// Height gradient - how much the camera rises as it moves away from target
    /// 0.0 = camera stays at target height, 1.0 = camera rises 1 unit per unit of distance
    pub height_gradient: f32,

    /// Minimum height angle (radians). May be below the horizon, but must stay
    /// above straight down (-π/2).
    pub min_pitch: f32,

    /// Maximum height angle (radians) - prevents camera from going directly overhead
    pub max_pitch: f32,

    /// How the camera keeps clear of scenery between it and the target.
    pub clearance: CameraClearance,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            follow_speed: 10.0,
            min_distance: 3.0,
            max_distance: 20.0,
            default_distance: 8.0,
            orbit_sensitivity: 0.003,
            height_sensitivity: 0.003,
            zoom_speed: 5.0,
            height_gradient: 0.3,
            // Below the horizon, to look up from low; short of straight down,
            // past which the orbit crosses under the target and turns the
            // wrong way, and at which the view has no up.
            min_pitch: -1.4,                              // ~80 degrees below
            max_pitch: std::f32::consts::FRAC_PI_2 - 0.5, // ~60 degrees
            clearance: CameraClearance::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use super::*;

    /// The orbit's spherical coordinates hold only strictly between straight
    /// down and straight up.
    #[test]
    fn the_pitch_range_stays_short_of_the_poles() {
        let config = CameraConfig::default();
        assert!(config.min_pitch > -FRAC_PI_2);
        assert!(config.max_pitch < FRAC_PI_2);
        assert!(config.min_pitch < config.max_pitch);
    }
}
