//! What a heart critter is made of.

use crate::animation::rig::FootShape;
use crate::animation::{FootPlacerConfig, LegRigDims};
use crate::rendering::colour::Colour;

/// A floppy ear.
#[derive(Clone, Copy, Debug)]
pub struct EarConfig {
    /// Bones in the chain. Two is a hinge, four is a noodle.
    pub segments: usize,
    /// Root-to-tip length when the ear is held straight.
    pub length: f32,
    /// Radius where the ear meets the head.
    pub root_radius: f32,
    /// Radius at the tip.
    pub tip_radius: f32,
    /// How far the ear leans out to the side at rest, as a fraction of
    /// its length. Zero stands it straight up.
    pub splay: f32,
    /// Total bend from root to tip at rest, in radians, away from the
    /// root direction and downward.
    ///
    /// This, not gravity, is what makes an ear floppy-looking while it is
    /// standing still. A straight rest shape reads as a horn however
    /// loosely it is simulated.
    pub curl: f32,
    /// How hard gravity pulls on the ear, relative to the world's. Below
    /// 1 makes a lighter, springier ear that lags the body more than it
    /// droops.
    pub weight: f32,
    /// Fraction of its velocity the ear sheds each step. Higher is more
    /// sluggish; at zero it swings forever.
    pub damping: f32,
    /// How hard the ear is pulled back toward its rest shape, in 1/s².
    ///
    /// This, not the bones, is what makes an ear an ear: a chain with only
    /// gravity on it hangs straight down and lies flat against the head.
    /// Raise it for a stiff ear that barely moves, lower it for one that
    /// swings about and takes its time coming back.
    pub shape_stiffness: f32,
    /// Backward lean of the rest pose, as a fraction of ear length.
    /// Positive sweeps the ears back off the face.
    pub sweep_back: f32,
    pub colour: Colour,
}

impl Default for EarConfig {
    fn default() -> Self {
        Self {
            segments: 5,
            length: 0.23,
            root_radius: 0.032,
            tip_radius: 0.01,
            splay: 0.14,
            curl: 1.15,
            weight: 0.55,
            damping: 0.08,
            shape_stiffness: 200.0,
            sweep_back: 0.2,
            colour: Colour::new(0.98, 0.85, 0.30, 1.0),
        }
    }
}

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

    pub ear: EarConfig,

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

            ear: EarConfig::default(),

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
