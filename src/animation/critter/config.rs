//! What a heart critter is made of.

use crate::animation::rig::{DroopConfig, FootShape};
use crate::animation::{FootPlacerConfig, LegRigDims};
use crate::rendering::colour::Colour;

/// The whole critter: a heart on two legs, with ears.
///
/// ```text
///        ear      ear
///          \     /
///           (heart)     ← torso, with a heart mark front and back
///              |        ← pelvis stub
///            /   \
///           |     |     ← legs
///          ==     ==    ← feet
/// ```
#[derive(Clone, Copy, Debug)]
pub struct CritterRigConfig {
    /// Hip to knee.
    pub upper_leg_length: f32,
    /// Knee to foot.
    pub lower_leg_length: f32,
    /// Lateral hip separation. Narrow, because the legs hang off a stub
    /// rather than off the full width of the torso.
    pub hip_width: f32,
    /// Radius of the leg bones.
    pub leg_radius: f32,
    pub knee_radius: f32,

    /// Distance from the pelvis (where the legs meet) up to the bottom of
    /// the heart.
    ///
    /// Without it the legs hang straight off the widest part of the body
    /// and the stance reads as a waddle. The stub is what gives the
    /// critter a waist to swing from.
    pub pelvis_stub: f32,
    pub pelvis_radius: f32,

    /// Heart torso, lobe to lobe.
    pub torso_width: f32,
    /// Heart torso, cleft to point.
    pub torso_height: f32,
    /// Heart torso, back to front.
    pub torso_depth: f32,

    /// Size of the heart marking as a fraction of the torso's own
    /// silhouette. At 1.0 the marking covers the whole body.
    pub mark_scale: f32,
    /// How far the mark stands off the fur, in metres. Enough to clear
    /// the surface, not enough to read as a separate object.
    pub mark_relief: f32,

    /// The floppy ears hanging off the top of the heart.
    pub ear: DroopConfig,

    /// Pelvis height above the foot centres at rest, as a fraction of leg
    /// length. Under 1 because a standing leg keeps some bend.
    pub standing_height_ratio: f32,
    /// Peak height of a swing arc.
    pub step_height: f32,
    /// Foot probe reach, as a multiple of leg length.
    pub probe_length_factor: f32,
    /// Fraction of the capture point a step plants at. Below 1 so the
    /// body passes over the planted foot instead of stopping on it.
    pub stride_gain: f32,
    /// Which way the knee breaks: `+1` forward like a person, `-1`
    /// backward like a hock. A critter reads as an animal with a hock.
    pub knee_bend: f32,

    pub foot: FootShape,
    pub foot_placer: FootPlacerConfig,

    pub fur_colour: Colour,
    pub mark_colour: Colour,
    pub foot_colour: Colour,

    /// Tessellation of every primitive in the rig.
    pub mesh_segments: u32,
}

impl CritterRigConfig {
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

    /// Where the heart's centre rides above the pelvis.
    #[inline]
    pub fn torso_rise(&self) -> f32 {
        self.pelvis_stub + self.torso_height * 0.5
    }

    /// Overall standing height, foot centres to the top of the heart.
    /// What a collider around this critter has to cover.
    #[inline]
    pub fn body_height(&self) -> f32 {
        self.standing_height() + self.pelvis_stub + self.torso_height
    }

    /// The subset of this rig the gait is planned against.
    ///
    /// A derived view, not a second copy — see
    /// [`crate::animation::LegRigDims`].
    #[inline]
    pub fn leg_dims(&self) -> LegRigDims {
        LegRigDims {
            hip_width: self.hip_width,
            leg_length: self.leg_length(),
            standing_height: self.standing_height(),
            probe_length: self.probe_length(),
        }
    }
}

impl Default for CritterRigConfig {
    fn default() -> Self {
        // A short gait wants short swings and a low trigger: the defaults
        // on `FootPlacerConfig` are metres and seconds sized for a person,
        // and a critter a third the height paces three times as fast.
        let foot_placer = FootPlacerConfig {
            min_step_duration: 0.06,
            max_step_duration: 0.26,
            settle_trigger: 0.02,
            swing_obstacle_clearance: 0.015,
            overstretch_hard_margin: 0.012,
            ..FootPlacerConfig::default()
        };

        Self {
            upper_leg_length: 0.1,
            lower_leg_length: 0.1,
            hip_width: 0.045,
            leg_radius: 0.021,
            knee_radius: 0.025,

            pelvis_stub: 0.045,
            pelvis_radius: 0.03,

            torso_width: 0.26,
            torso_height: 0.24,
            torso_depth: 0.17,

            mark_scale: 0.52,
            mark_relief: 0.004,

            ear: DroopConfig::default(),

            standing_height_ratio: 0.85,
            step_height: 0.05,
            probe_length_factor: 1.4,
            stride_gain: 0.55,
            knee_bend: -1.0,

            foot: FootShape {
                radius: 0.022,
                half_length: 0.042,
                ankle_forward_offset: 0.012,
            },
            foot_placer,

            fur_colour: Colour::new(0.98, 0.82, 0.22, 1.0),
            mark_colour: Colour::new(0.92, 0.18, 0.32, 1.0),
            foot_colour: Colour::new(0.86, 0.62, 0.14, 1.0),

            mesh_segments: 14,
        }
    }
}
