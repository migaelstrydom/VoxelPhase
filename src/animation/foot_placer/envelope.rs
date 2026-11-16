//! The envelope a leg is drawn in: the foot placer's, for the feet it holds,
//! and a rig's, for the last word on the feet it draws.

use nalgebra::{Point3, Vector3};

use super::capture_point::stance_offset;
use super::placer::{hip_position, max_leg_extension, FootSide, PlacerCtx};

/// How far off straight down a swinging foot is drawn at most, as the
/// tangent of the angle (60 degrees): the cone a stride sweeps. A swing
/// whose path runs outside it (reaching for a landing the body is still
/// short of, or trailing one it has left) is drawn with the knee tucked,
/// not with the leg held out level.
pub(super) const SWING_CONE_TANGENT: f32 = 1.73;

/// The least a swinging foot is drawn under its hip, as a fraction of the
/// leg's stretch: a knee tucked up as far as one goes.
pub(super) const SWING_LEAST_DROP: f32 = 0.3;

/// The least a foot is drawn on its own side of the body's midline, as a
/// fraction of what its steps aim to keep (`FootPlacerConfig::own_side`):
/// a plant the body has passed over is short of what it aimed for, and is
/// still not drawn crossed.
const OWN_SIDE_DRAWN: f32 = 0.4;

/// Where one leg goes: what a foot the legs hold is drawn within.
///
/// ```text
///            hip
///           /   \         no higher than a tucked knee brings it
///          / 60° \        ([`SWING_LEAST_DROP`]), no further off straight
///         /_______\       down than a stride sweeps
///        (  reach  )     ([`SWING_CONE_TANGENT`]), no further than the
///         `-------'      leg stretches, and on its own side of the body
///                        (`FootPlacerConfig::own_side`)
/// ```
///
/// A plant and a swing are places in the world, and the hip a body that
/// may rise, drop, slide sideways or outrun them: where one leaves this
/// envelope the foot is drawn at its edge, knee bent, until the body brings
/// the two back together or the leg lets go. A stride's own plants and
/// swings lie inside it and are drawn where they are. Planted and swinging
/// feet share it, so a foot let go of at its edge starts its swing where it
/// was drawn.
#[derive(Clone, Copy, Debug)]
pub struct LegEnvelope {
    hip: Point3<f32>,
    /// Up the body the leg hangs from: world up for a body standing, the
    /// spine's own axis for a rig that leans.
    up: Vector3<f32>,
    /// The furthest the leg stretches from the hip, in metres.
    reach: f32,
    /// Unit direction from the body's midline out to this leg's side,
    /// square to `up`.
    own_side: Vector3<f32>,
    /// How far the hip lies from the midline, in metres.
    hip_width: f32,
    /// The least the foot is drawn to its own side of the midline, in
    /// metres; `None` for legs that may cross.
    own_side_kept: Option<f32>,
}

impl LegEnvelope {
    /// The envelope of a leg hanging from `hip` on a body whose up is `up`,
    /// stretching `reach` at most, the hip `hip_width` out from the midline
    /// along `own_side`. `keeps_own_side` is the gait's
    /// `FootPlacerConfig::own_side`.
    pub fn new(
        hip: Point3<f32>,
        up: Vector3<f32>,
        own_side: Vector3<f32>,
        hip_width: f32,
        reach: f32,
        keeps_own_side: f32,
    ) -> Self {
        Self {
            hip,
            up,
            reach,
            own_side,
            hip_width,
            own_side_kept: (keeps_own_side > 0.0)
                .then(|| OWN_SIDE_DRAWN * keeps_own_side * hip_width),
        }
    }

    /// The envelope of the placer's own `side` leg, standing upright.
    pub(super) fn of(side: FootSide, ctx: &PlacerCtx<'_>, facing: Vector3<f32>) -> Self {
        let own_side = stance_offset(facing, 1.0, side.side_sign());
        Self::new(
            hip_position(side, ctx, facing),
            Vector3::y(),
            Vector3::new(own_side.x, 0.0, own_side.y),
            ctx.hip_width,
            max_leg_extension(ctx),
            ctx.config.own_side,
        )
    }

    /// The hip the leg hangs from.
    #[cfg(test)]
    pub(super) fn hip(&self) -> Point3<f32> {
        self.hip
    }

    /// The furthest the leg stretches, in metres.
    #[cfg(test)]
    pub(super) fn reach(&self) -> f32 {
        self.reach
    }

    /// Where a foot the legs hold at `held` is drawn.
    pub fn draw(&self, held: Point3<f32>) -> Point3<f32> {
        let from_hip = held - self.hip;
        let mut drop = -from_hip.dot(&self.up);
        let mut across = from_hip + self.up * drop;
        if let Some(kept) = self.own_side_kept {
            let inward = -across.dot(&self.own_side);
            let furthest_in = self.hip_width - kept;
            if inward > furthest_in {
                across += self.own_side * (inward - furthest_in);
            }
        }
        drop = drop.max(SWING_LEAST_DROP * self.reach);
        let (width, widest) = (across.norm(), SWING_CONE_TANGENT * drop);
        if width > widest {
            across *= widest / width;
        }
        let mut from_hip = across - self.up * drop;
        let distance = from_hip.norm();
        if distance > self.reach {
            from_hip *= self.reach / distance;
        }
        self.hip + from_hip
    }
}
