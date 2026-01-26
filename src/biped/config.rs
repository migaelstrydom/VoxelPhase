//! Configuration for biped characters.
//!
//! All tuning parameters in one place.

use crate::rendering::Colour;

/// Configuration for a biped character.
#[derive(Clone, Debug)]
pub struct BipedConfig {
    // === Skeleton geometry ===
    /// Upper leg length (hip to knee).
    pub upper_leg_length: f32,
    /// Lower leg length (knee to foot).
    pub lower_leg_length: f32,
    /// Lateral distance from pelvis center to each hip.
    pub hip_width: f32,
    /// Collision radius for pelvis sphere.
    pub pelvis_radius: f32,

    // === Collision radii ===
    /// Collision radius for foot spheres.
    pub foot_radius: f32,
    /// Collision radius for knee spheres.
    pub knee_radius: f32,
    /// Collision radius for body sphere.
    pub body_radius: f32,

    // === Gait parameters ===
    /// Speed below which the character is considered standing still.
    pub idle_threshold: f32,
    /// Height of the step arc when foot is swinging.
    pub step_height: f32,
    /// Total stride length (distance covered in one full gait cycle).
    pub stride_length: f32,
    /// Pelvis height as fraction of leg length (1.0 = fully extended).
    pub standing_height_ratio: f32,

    // === Probe parameters ===
    /// Probe length as multiplier of leg length.
    pub probe_length_factor: f32,
    /// Probe radius for ground detection.
    pub probe_radius: f32,

    // === Mesh/rendering ===
    /// Color of the pelvis/body.
    pub body_colour: Colour,
    /// Color of the legs.
    pub leg_colour: Colour,
    /// Color of the feet.
    pub foot_colour: Colour,
    /// Segments around capsules (higher = smoother).
    pub mesh_segments: u32,
}

impl BipedConfig {
    /// Total leg length (upper + lower).
    #[inline]
    pub fn leg_length(&self) -> f32 {
        self.upper_leg_length + self.lower_leg_length
    }

    /// Standing pelvis height above feet.
    #[inline]
    pub fn standing_height(&self) -> f32 {
        self.leg_length() * self.standing_height_ratio
    }

    /// Probe length in world units.
    #[inline]
    pub fn probe_length(&self) -> f32 {
        self.leg_length() * self.probe_length_factor
    }
}

impl Default for BipedConfig {
    fn default() -> Self {
        Self {
            // Skeleton geometry
            upper_leg_length: 0.25,
            lower_leg_length: 0.25,
            hip_width: 0.12,
            pelvis_radius: 0.1,

            // Collision radii
            foot_radius: 0.06,
            knee_radius: 0.04,
            body_radius: 0.5,

            // Gait parameters
            idle_threshold: 0.1,
            step_height: 0.15,
            stride_length: 0.4,
            standing_height_ratio: 0.85,

            // Probe parameters
            probe_length_factor: 1.5,
            probe_radius: 0.05,

            // Mesh/rendering
            body_colour: Colour::new(0.8, 0.6, 0.4, 1.0),
            leg_colour: Colour::new(0.3, 0.3, 0.5, 1.0),
            foot_colour: Colour::new(0.2, 0.2, 0.3, 1.0),
            mesh_segments: 8,
        }
    }
}
