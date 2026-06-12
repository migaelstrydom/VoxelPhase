//! Configuration for character rigs.
//!
//! All tuning parameters in one place.

use super::foot_placer::FootPlacerConfig;
use crate::rendering::Colour;

/// Per-gait animation parameters. Values that differ between Walk / Sprint /
/// Crouch live here; rig-level constants (arm length, torso height, etc.)
/// stay on `CharacterRigConfig`.
#[derive(Clone, Copy, Debug)]
pub struct GaitPreset {
    /// Peak foot lift during swing.
    pub step_height: f32,
    /// Forward/backward swing amplitude for the arms.
    pub arm_swing_amplitude: f32,
    /// Maximum shoulder twist angle in radians.
    pub shoulder_twist_max: f32,
    /// Vertical head bob amplitude.
    pub head_bob_amplitude: f32,
    /// Downward pelvis offset (metres). `0.0` for Walk/Sprint, positive
    /// for Crouch.
    pub pelvis_crouch_offset: f32,
    /// Forward torso lean in radians. `0.0` for upright gaits, positive
    /// for crouched gaits.
    pub torso_pitch: f32,
    /// Fraction of the capture point the foot plants at, in `(0, 1)`.
    /// `1.0` puts the foot at the stopping foothold (body arrests over
    /// it). Values below 1 preserve the divergent component of the LIP
    /// pendulum, so the body passes over the planted foot and continues.
    /// Lower = longer, more committed strides; higher = shorter, more
    /// controlled strides. Step trigger and gait cadence derive from the
    /// same quantity (`GaitTiming`): the planted foot travels
    /// symmetrically from `+gain·v/ω` ahead to `-gain·v/ω` behind the
    /// hip per step, so hip travel per cycle is `4·gain·v/ω`.
    pub stride_gain: f32,
}

/// Bundle of gait presets carried on `CharacterRigConfig`.
#[derive(Clone, Copy, Debug)]
pub struct GaitPresets {
    pub walk: GaitPreset,
    pub sprint: GaitPreset,
    pub crouch_walk: GaitPreset,
    pub crouch_idle: GaitPreset,
}

/// Configuration for a character rig.
#[derive(Clone, Debug)]
pub struct CharacterRigConfig {
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

    // === Gait presets ===
    /// Per-gait parameter bundle (Walk, Sprint, Crouch walking, Crouch idle).
    pub gait_presets: GaitPresets,

    // === Foot placer ===
    /// Procedural foot-placement tuning (capture-point stepping).
    pub foot_placer: FootPlacerConfig,
}

impl CharacterRigConfig {
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

impl Default for CharacterRigConfig {
    fn default() -> Self {
        Self {
            // Lower body geometry
            upper_leg_length: 0.25,
            lower_leg_length: 0.25,
            hip_width: 0.12,
            pelvis_radius: 0.05,

            // Upper body geometry
            torso_height: 0.25,
            shoulder_width: 0.18,
            upper_arm_length: 0.18,
            lower_arm_length: 0.14,
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
            standing_height_ratio: 0.85,

            // Upper body animation
            arm_swing_amplitude: 0.3,
            shoulder_twist_max: 0.15,
            head_tilt_factor: 0.01,
            head_bob_amplitude: 0.02,

            // Probe parameters. Foot probes aim from the hip at the
            // swing's landing target; the length must cover targets a
            // full stride ahead and below the feet (downhill landings),
            // not just the standing hip→ground distance.
            probe_length_factor: 1.8,

            // Colors - metallic blue
            body_colour: Colour::new(0.2, 0.4, 0.85, 1.0),
            head_colour: Colour::new(0.14, 0.22, 0.5, 1.0),
            leg_colour: Colour::new(0.18, 0.38, 0.8, 1.0),
            foot_colour: Colour::new(0.14, 0.22, 0.5, 1.0),
            hip_colour: Colour::new(0.18, 0.38, 0.8, 1.0),
            mesh_segments: 8,

            gait_presets: GaitPresets {
                walk: GaitPreset {
                    step_height: 0.15,
                    arm_swing_amplitude: 0.3,
                    shoulder_twist_max: 0.15,
                    head_bob_amplitude: 0.02,
                    pelvis_crouch_offset: 0.0,
                    torso_pitch: 0.08,
                    stride_gain: 0.4,
                },
                sprint: GaitPreset {
                    step_height: 0.15 * 1.2,
                    arm_swing_amplitude: 0.3 * 1.2,
                    shoulder_twist_max: 0.15 * 1.3,
                    head_bob_amplitude: 0.02 * 1.4,
                    pelvis_crouch_offset: 0.0,
                    torso_pitch: 0.20,
                    stride_gain: 0.3,
                },
                crouch_walk: GaitPreset {
                    step_height: 0.15 * 0.4,
                    arm_swing_amplitude: 0.3 * 0.3,
                    shoulder_twist_max: 0.15 * 0.4,
                    head_bob_amplitude: 0.02 * 0.3,
                    pelvis_crouch_offset: 0.25,
                    torso_pitch: 0.30,
                    stride_gain: 0.7,
                },
                crouch_idle: GaitPreset {
                    step_height: 0.15 * 0.4,
                    arm_swing_amplitude: 0.3 * 0.3,
                    shoulder_twist_max: 0.15 * 0.4,
                    head_bob_amplitude: 0.02 * 0.3,
                    pelvis_crouch_offset: 0.25,
                    torso_pitch: 0.30,
                    stride_gain: 0.7,
                },
            },

            foot_placer: FootPlacerConfig::default(),
        }
    }
}

impl GaitPresets {
    /// Return the preset for the given gait state. `Idle` falls back to
    /// `walk` since none of its movement params are sampled anyway.
    pub fn for_gait(&self, gait: super::humanoid::pose_state::Gait) -> GaitPreset {
        use super::humanoid::pose_state::Gait;
        match gait {
            Gait::Idle => self.walk,
            Gait::Walk => self.walk,
            Gait::Sprint => self.sprint,
            Gait::Crouch { walking: true } => self.crouch_walk,
            Gait::Crouch { walking: false } => self.crouch_idle,
        }
    }
}
