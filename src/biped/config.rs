//! Configuration for biped characters.
//!
//! All tuning parameters in one place.

use crate::rendering::Colour;

/// Configuration for a biped character.
#[derive(Clone, Debug)]
pub struct BipedConfig {
    // === Lower body geometry ===
    /// Upper leg length (hip to knee).
    pub upper_leg_length: f32,
    /// Lower leg length (knee to foot).
    pub lower_leg_length: f32,
    /// Lateral distance from pelvis center to each hip.
    pub hip_width: f32,
    /// Collision radius for pelvis sphere.
    pub pelvis_radius: f32,

    // === Upper body geometry ===
    /// Height from pelvis to chest/shoulders.
    pub torso_height: f32,
    /// Lateral distance from chest center to each shoulder.
    pub shoulder_width: f32,
    /// Upper arm length (shoulder to elbow).
    pub upper_arm_length: f32,
    /// Lower arm length (elbow to hand).
    pub lower_arm_length: f32,
    /// Neck length (chest to head base).
    pub neck_length: f32,
    /// Head sphere radius.
    pub head_radius: f32,

    // === Lower body radii ===
    /// Collision radius for foot spheres.
    pub foot_radius: f32,
    /// Collision radius for knee spheres.
    pub knee_radius: f32,
    /// Collision radius for body sphere (also stride wheel radius).
    pub body_radius: f32,
    /// Collision radius for hip spheres.
    pub hip_radius: f32,

    // === Upper body radii ===
    /// Torso cylinder radius.
    pub torso_radius: f32,
    /// Shoulder joint radius.
    pub shoulder_radius: f32,
    /// Elbow joint radius.
    pub elbow_radius: f32,
    /// Hand sphere radius.
    pub hand_radius: f32,

    // === Gait parameters ===
    /// Speed below which the character is considered standing still.
    pub idle_threshold: f32,
    /// Height of the step arc when foot is swinging.
    pub step_height: f32,
    /// Total stride length (distance covered in one full gait cycle).
    pub stride_length: f32,
    /// Pelvis height as fraction of leg length (1.0 = fully extended).
    pub standing_height_ratio: f32,

    // === Upper body animation ===
    /// Forward/backward swing amplitude for arms.
    pub arm_swing_amplitude: f32,
    /// Maximum shoulder twist angle in radians.
    pub shoulder_twist_max: f32,
    /// How much head tilts with velocity (radians per unit speed).
    pub head_tilt_factor: f32,
    /// Vertical head bob amplitude during walking.
    pub head_bob_amplitude: f32,

    // === Probe parameters ===
    /// Probe length as multiplier of leg length.
    pub probe_length_factor: f32,
    /// Probe radius for ground detection.
    pub probe_radius: f32,

    // === Mesh/rendering colors ===
    /// Color of the torso and head.
    pub body_colour: Colour,
    /// Color of the head.
    pub head_colour: Colour,
    /// Color of the legs and arms.
    pub leg_colour: Colour,
    /// Color of the feet and hands.
    pub foot_colour: Colour,
    /// Color of the hips.
    pub hip_colour: Colour,
    /// Segments around capsules (higher = smoother).
    pub mesh_segments: u32,
}

impl BipedConfig {
    /// Total leg length (upper + lower).
    #[inline]
    pub fn leg_length(&self) -> f32 {
        self.upper_leg_length + self.lower_leg_length
    }

    /// Total arm length (upper + lower).
    #[inline]
    pub fn arm_length(&self) -> f32 {
        self.upper_arm_length + self.lower_arm_length
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
            // Lower body geometry
            upper_leg_length: 0.25,
            lower_leg_length: 0.25,
            hip_width: 0.12,
            pelvis_radius: 0.05,

            // Upper body geometry
            torso_height: 0.2,
            shoulder_width: 0.18,
            upper_arm_length: 0.15,
            lower_arm_length: 0.12,
            neck_length: 0.05,
            head_radius: 0.1,

            // Lower body radii
            foot_radius: 0.06,
            knee_radius: 0.04,
            body_radius: 0.5,
            hip_radius: 0.04,

            // Upper body radii
            torso_radius: 0.05,
            shoulder_radius: 0.04,
            elbow_radius: 0.03,
            hand_radius: 0.04,

            // Gait parameters
            idle_threshold: 0.1,
            step_height: 0.15,
            stride_length: 0.4,
            standing_height_ratio: 0.85,

            // Upper body animation
            arm_swing_amplitude: 0.3,
            shoulder_twist_max: 0.15,
            head_tilt_factor: 0.01,
            head_bob_amplitude: 0.02,

            // Probe parameters
            probe_length_factor: 1.5,
            probe_radius: 0.05,

            // Colors - metallic blue
            body_colour: Colour::new(0.2, 0.4, 0.85, 1.0),
            head_colour: Colour::new(0.14, 0.22, 0.5, 1.0),
            leg_colour: Colour::new(0.18, 0.38, 0.8, 1.0),
            foot_colour: Colour::new(0.14, 0.22, 0.5, 1.0),
            hip_colour: Colour::new(0.18, 0.38, 0.8, 1.0),
            mesh_segments: 8,
        }
    }
}
