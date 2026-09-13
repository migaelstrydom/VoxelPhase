//! What a peeper is made of.

use crate::animation::rig::{DroopConfig, FootShape};
use crate::animation::FootPlacerConfig;
use crate::rendering::colour::Colour;

/// A one-eyed stalker on stilts.
///
/// ```text
///        \  (eye)  /      ← one big eye, with a lid that droops when bored
///         \___|___/       ← feelers off the crown
///             |           ← neck, which pitches back and then spears forward
///           (haunch)
///            /   \
///           |     |       ← long legs, knees breaking backward like a bird's
///          ==     ==
/// ```
///
/// Every proportion here serves the same read: a creature that is almost
/// all leg and eye, so a player can tell at a glance that it is nothing
/// like the boulder that also wants them dead.
#[derive(Clone, Copy, Debug)]
pub struct PeeperRigConfig {
    /// Hip to knee.
    pub upper_leg_length: f32,
    /// Knee to foot.
    pub lower_leg_length: f32,
    /// Lateral hip separation. Narrow, so the legs cross close under the
    /// body and the walk reads as a wader's.
    pub hip_width: f32,
    pub leg_radius: f32,
    pub knee_radius: f32,

    /// The stub where the legs meet, below the neck.
    pub haunch_height: f32,
    pub haunch_radius: f32,

    /// Haunch to eye centre.
    pub neck_length: f32,
    pub neck_radius: f32,

    pub eye_radius: f32,
    /// Iris size, as a fraction of the eye.
    pub iris_fraction: f32,
    /// Pupil size, as a fraction of the iris.
    pub pupil_fraction: f32,

    /// How far down the eye the lid reaches when it is wide open, in
    /// radians from the crown. Small: what is left is a brow.
    pub lid_open_angle: f32,
    /// How far it reaches when the eye is shut. Past a right angle, or the
    /// lid stops level with the pupil instead of covering it.
    pub lid_shut_angle: f32,
    /// Lid thickness, as a fraction of the eye radius it wraps.
    pub lid_thickness: f32,
    /// How fast the lid chases the alertness it is given, in 1/s. Fast
    /// enough that spotting the player is a snap, not a fade.
    pub lid_rate: f32,

    /// The feelers trailing off the crown.
    pub feeler: DroopConfig,
    /// Where a feeler roots, in eye radii: out to the side, up, and back.
    pub feeler_anchor: (f32, f32, f32),

    /// Neck pitch at rest, in radians from vertical. A slight forward
    /// stoop, which is what makes it read as watching rather than waiting.
    pub rest_pitch: f32,
    /// Neck pitch at full wind-up. Negative: the head rears back over the
    /// haunch, which is the whole warning the player gets.
    pub rear_pitch: f32,
    /// Neck pitch at the moment a peck lands, spearing forward and down.
    pub strike_pitch: f32,

    /// Pelvis height above the foot centres at rest, as a fraction of leg
    /// length. Under 1 because a standing leg keeps some bend.
    pub standing_height_ratio: f32,
    /// Peak height of a swing arc. High for the size: a peeper picks its
    /// feet up.
    pub step_height: f32,
    /// Foot probe reach, as a multiple of leg length.
    pub probe_length_factor: f32,
    /// Fraction of the capture point a step plants at.
    pub stride_gain: f32,
    /// Which way the knees break: `-1` backward, like a bird's hock.
    pub knee_bend: f32,

    pub foot: FootShape,
    pub foot_placer: FootPlacerConfig,

    pub hide_colour: Colour,
    pub sclera_colour: Colour,
    pub iris_colour: Colour,
    pub pupil_colour: Colour,
    pub lid_colour: Colour,
    pub foot_colour: Colour,

    /// Tessellation of every primitive in the rig.
    pub mesh_segments: u32,
}

impl PeeperRigConfig {
    /// Total leg length (upper + lower).
    #[inline]
    pub fn leg_length(&self) -> f32 {
        self.upper_leg_length + self.lower_leg_length
    }

    /// Pelvis height above the foot centres at rest.
    #[inline]
    pub fn standing_height(&self) -> f32 {
        self.leg_length() * self.standing_height_ratio
    }

    /// Foot probe reach in world units.
    #[inline]
    pub fn probe_length(&self) -> f32 {
        self.leg_length() * self.probe_length_factor
    }

    /// Overall standing height, foot centres to the crown of the eye.
    /// What a collider around this creature has to cover.
    #[inline]
    pub fn body_height(&self) -> f32 {
        self.standing_height() + self.haunch_height + self.neck_length + self.eye_radius
    }

    /// How far in front of its own axis the head ends up at the far end
    /// of a peck.
    ///
    /// The def sizes the attack from this rather than from a hand-picked
    /// number, so a longer neck is a longer reach without anyone having to
    /// remember to say so.
    #[inline]
    pub fn strike_extent(&self) -> f32 {
        self.neck_length * self.strike_pitch.sin() + self.eye_radius
    }

    /// The subset of this rig the gait is planned against.
    #[inline]
    pub fn leg_dims(&self) -> crate::animation::LegRigDims {
        crate::animation::LegRigDims {
            hip_width: self.hip_width,
            leg_length: self.leg_length(),
            standing_height: self.standing_height(),
            probe_length: self.probe_length(),
        }
    }
}

impl Default for PeeperRigConfig {
    fn default() -> Self {
        // Long, light feelers rather than ears: the same chain stretched
        // and made springier, so they trail behind the head instead of
        // hanging off it.
        let feeler = DroopConfig {
            segments: 6,
            length: 0.46,
            root_radius: 0.022,
            tip_radius: 0.006,
            splay: 0.5,
            curl: 1.3,
            weight: 0.35,
            damping: 0.05,
            shape_stiffness: 120.0,
            sweep_back: 0.8,
            colour: Colour::new(0.38, 0.30, 0.52, 1.0),
        };

        Self {
            upper_leg_length: 0.52,
            lower_leg_length: 0.5,
            hip_width: 0.07,
            leg_radius: 0.032,
            knee_radius: 0.046,

            haunch_height: 0.16,
            haunch_radius: 0.085,

            neck_length: 0.42,
            neck_radius: 0.035,

            eye_radius: 0.22,
            iris_fraction: 0.5,
            pupil_fraction: 0.5,

            lid_open_angle: 0.62,
            lid_shut_angle: 2.3,
            lid_thickness: 0.05,
            lid_rate: 11.0,

            feeler,
            feeler_anchor: (0.42, 0.72, -0.35),

            rest_pitch: 0.2,
            rear_pitch: -0.55,
            strike_pitch: 1.35,

            standing_height_ratio: 0.86,
            step_height: 0.22,
            probe_length_factor: 1.3,
            stride_gain: 0.6,
            knee_bend: -1.0,

            foot: FootShape {
                radius: 0.045,
                half_length: 0.11,
                ankle_forward_offset: 0.035,
            },
            foot_placer: FootPlacerConfig::default(),

            hide_colour: Colour::new(0.32, 0.26, 0.46, 1.0),
            sclera_colour: Colour::new(0.94, 0.97, 0.90, 1.0),
            iris_colour: Colour::new(0.98, 0.58, 0.12, 1.0),
            pupil_colour: Colour::new(0.06, 0.05, 0.09, 1.0),
            lid_colour: Colour::new(0.26, 0.2, 0.38, 1.0),
            foot_colour: Colour::new(0.2, 0.16, 0.3, 1.0),

            mesh_segments: 14,
        }
    }
}
