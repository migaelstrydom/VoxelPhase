//! A stilt walker's proportions.

use crate::animation::FootPlacerConfig;

use super::carriage::Carriage;

/// The proportions of a creature on two long legs.
///
/// ```text
///            (head)       ← rides the neck; rears back and spears forward
///              |          ← neck
///           (haunch)
///            /   \
///           |     |       ← legs, knees breaking backward or forward
///          ==     ==
/// ```
///
/// Only the joints and the gait are here. What the creature looks like is
/// its [`Dress`](super::Dress): a model whose parts ride these joints.
#[derive(Clone, Copy, Debug)]
pub struct StiltRigConfig {
    /// Hip to knee.
    pub upper_leg_length: f32,
    /// Knee to sole: the shin, then the ankle under it.
    pub lower_leg_length: f32,
    /// Height of the ankle over the sole, m: where the shin ends and the
    /// foot turns. The shin reaches it along the shin, the foot along the
    /// sole's normal, so the two meet however far the shin leans.
    pub ankle_height: f32,
    /// Lateral hip separation. Narrow, and the legs cross close under the
    /// body like a wader's.
    pub hip_width: f32,
    /// How thick a leg is, for what water and wind see of it.
    pub leg_radius: f32,

    /// The stub where the legs meet, below the neck.
    pub haunch_height: f32,
    /// Its thickness, for what water and wind see of it.
    pub haunch_radius: f32,

    /// Haunch to the middle of the head.
    pub neck_length: f32,
    /// Its thickness, for what water and wind see of it.
    pub neck_radius: f32,

    /// Radius of the ball that holds the head: what the body collides as
    /// across, and how far past the neck a strike reaches.
    pub head_radius: f32,

    /// Neck pitch at rest, in radians from vertical. A slight forward
    /// stoop reads as watching rather than waiting.
    pub rest_pitch: f32,
    /// Neck pitch at full wind-up. Negative: the head rears back over the
    /// haunch, which is the whole warning the player gets.
    pub rear_pitch: f32,
    /// Neck pitch at the moment a strike lands, spearing forward and down.
    pub strike_pitch: f32,

    /// How fast the lids chase the alertness they are given, in 1/s. Fast
    /// enough that spotting the player is a snap, not a fade.
    pub lid_rate: f32,

    /// Pelvis height above the foot centres at rest, as a fraction of leg
    /// length. Under 1 because a standing leg keeps some bend.
    pub standing_height_ratio: f32,
    /// Peak height of a swing arc.
    pub step_height: f32,
    /// Foot probe reach, as a multiple of leg length.
    pub probe_length_factor: f32,
    /// Fraction of the capture point a step plants at.
    pub stride_gain: f32,
    /// Which way the knees break: `-1` backward, like a bird's hock; `1`
    /// forward, like a man's.
    pub knee_bend: f32,

    pub foot_placer: FootPlacerConfig,
    /// How the body rides the legs: its dip, lean, rock and heft.
    pub carriage: Carriage,
}

impl StiltRigConfig {
    /// Total leg length (upper + lower).
    #[inline]
    pub fn leg_length(&self) -> f32 {
        self.upper_leg_length + self.lower_leg_length
    }

    /// Knee to ankle: the shin bone the legs are solved with.
    #[inline]
    pub fn shin_length(&self) -> f32 {
        self.lower_leg_length - self.ankle_height
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

    /// Overall standing height, foot centres to the crown of the head.
    /// What a collider around this creature has to cover.
    #[inline]
    pub fn body_height(&self) -> f32 {
        self.standing_height() + self.haunch_height + self.neck_length + self.head_radius
    }

    /// How far in front of its own axis the head ends up at the far end
    /// of a strike.
    ///
    /// The def sizes the attack from this rather than from a hand-picked
    /// number, so a longer neck is a longer reach without anyone having to
    /// remember to say so.
    #[inline]
    pub fn strike_extent(&self) -> f32 {
        self.neck_length * self.strike_pitch.sin() + self.head_radius
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

impl Default for StiltRigConfig {
    /// The peeper's: almost all leg, with a head the size of a melon.
    fn default() -> Self {
        Self {
            upper_leg_length: 0.52,
            lower_leg_length: 0.5,
            ankle_height: 0.0,
            hip_width: 0.07,
            leg_radius: 0.026,

            haunch_height: 0.16,
            haunch_radius: 0.085,

            neck_length: 0.42,
            neck_radius: 0.035,

            head_radius: 0.22,

            rest_pitch: 0.2,
            rear_pitch: -0.55,
            strike_pitch: 1.35,

            lid_rate: 11.0,

            standing_height_ratio: 0.86,
            step_height: 0.22,
            probe_length_factor: 1.3,
            stride_gain: 0.6,
            knee_bend: -1.0,

            foot_placer: FootPlacerConfig::default(),
            carriage: Carriage::default(),
        }
    }
}
