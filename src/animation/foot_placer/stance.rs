//! A planted foot: what its leg says of the stance, and keeping it on its
//! ground.
//!
//! ```text
//!   Strain::of      how far the hip has left the foot, and turned from it:
//!                   a step asked of the clock ahead of its schedule
//!                   (overstretch, turn), or a foot let go of outright
//!   settle_stance   the foot's height kept on the floor its probe finds,
//!                   no lower than the leg goes, and the foot drawn within
//!                   the leg's envelope
//! ```

use nalgebra::{Point3, Vector2, Vector3};

use super::clock::ReleaseKind;
use super::config::FootPlacerConfig;
use super::envelope::{LegEnvelope, SWING_LEAST_DROP};
use super::floor::{floor_height_near, floor_sample};
use super::geometry::angle_diff;
use super::leg::{hip_position, max_leg_extension};
use super::pace::Pace;
use super::placer::{PlacerCtx, PlacerFoot};

/// How far past the stretch that asks it to step a planted foot's hip may get
/// before the leg lets go of it, as a fraction of the leg's length. The
/// stretch release is a request, which the gait's support and stagger rules
/// may hold back for a swing; past this the foot is somewhere the leg cannot
/// be drawn standing, and it lets go whatever the gait says.
const LET_GO_MARGIN: f32 = 0.2;

/// How far off straight down a planted leg may point, as the tangent of the
/// angle (65 degrees), before the leg lets go of a foot the hip is leaving:
/// a sprint's trailing leg at toe-off is about 45 degrees back, a
/// crouch-walk's about 55. Past it the foot is one the body has dropped
/// away from or been carried off, on a ledge it stepped off or a crate that
/// fell.
const LET_GO_TANGENT: f32 = 2.14;

/// The least a planted foot lies from under its hip, across the ground, for
/// its leg's angle to let go of it, as a fraction of the leg's length: a
/// body squashed down onto its feet has them close under it at any angle.
const LET_GO_ACROSS: f32 = 0.4;

/// What a planted foot's leg says of its stance this substep.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Strain {
    /// The step the leg asks the clock for ahead of its schedule.
    pub asks: Option<ReleaseKind>,
    /// Whether the leg is stretched past its hard margin
    /// (`overstretch_hard_margin`): its step does not wait out the stagger
    /// between takeoffs.
    pub hard_overstretch: bool,
    /// Whether the foot is where the leg cannot be drawn standing on it: it
    /// steps whatever the gait's rules say.
    pub let_go: bool,
}

impl Strain {
    /// What `foot`'s leg says of its stance; nothing, of a foot in the air.
    ///
    /// Event releases while moving: a planted foot is asked to step
    /// early when the facing has rotated far enough from its plant
    /// (turn step), or when the hip has slid to the leg's stretch
    /// limit (landing slides, hard accelerations). The kind controls
    /// the support rules the step fires under
    /// (`FootPlacer::select_step`).
    /// Overstretch additionally requires the leg to be *opening* (hip
    /// moving away from the foot): a foot that lands far ahead during
    /// a hard deceleration is momentarily stretched, but the pelvis
    /// is closing on it and the stretch resolves by itself — firing
    /// there would re-lift the foot after one frame of contact. The
    /// trigger is the *actual* stretch, never a prediction: a
    /// predictive lead (latching when the limit would be reached
    /// within the stagger gap) reads 0.3 m into the future at sprint
    /// speeds, and because latches are sticky it re-paced the whole
    /// gait reactively ~0.06 phase ahead of the schedule. Overshoot
    /// while a release is gate-blocked is bounded by the hard-margin
    /// stagger bypass in `FootPlacer::select_step`.
    ///
    /// A hip further still from its foot ([`LET_GO_MARGIN`]), or leaving
    /// one its leg points level at ([`LET_GO_TANGENT`]), lets go of it:
    /// moving or not, whatever the support and stagger rules there would
    /// hold a step back for. Those rules pace a gait; a foot left where
    /// the leg cannot stand on it is not part of one, and held there it
    /// is drawn as a leg pointing at it.
    pub fn of(foot: &PlacerFoot, ctx: &PlacerCtx<'_>, pace: &Pace) -> Self {
        if !foot.is_planted() {
            return Self::default();
        }
        let cfg = ctx.config;
        let hip = hip_position(foot.side, ctx, pace.facing);
        let away = hip - foot.planted_position;
        let distance = away.norm().max(1e-4);
        // Opening, as the stretch release is: a foot planted high ahead
        // of a hip climbing to it points as far off straight down, and
        // is the next step up.
        let opening = away.dot(&ctx.velocity) > 0.0;
        let across = Vector2::new(away.x, away.z).magnitude();
        let levelled = opening
            && across >= LET_GO_ACROSS * ctx.leg_length
            && across > LET_GO_TANGENT * away.y.max(0.0);
        let let_go =
            levelled || distance >= max_leg_extension(ctx) + LET_GO_MARGIN * ctx.leg_length;
        let (mut asks, mut hard_overstretch) = (None, false);
        if pace.moving {
            // Full 3D radial speed: on slopes the hip moves away
            // from a planted foot vertically as much as
            // horizontally, and an xz-only opening gate is blind
            // to that half of the motion.
            let radial_speed = away.dot(&ctx.velocity) / distance;
            if radial_speed > 0.0 && distance >= max_leg_extension(ctx) {
                asks = Some(ReleaseKind::Overstretch);
                hard_overstretch = distance >= max_leg_extension(ctx) + cfg.overstretch_hard_margin;
            } else if yaw_equivalent_error(foot, ctx) >= turn_trigger(cfg, ctx) {
                asks = Some(ReleaseKind::Turn);
            }
        }
        Self {
            asks,
            hard_overstretch,
            let_go,
        }
    }
}

/// Accumulated facing change since this foot's plant, expressed in the
/// same metres-of-error units as the translational trigger.
pub(super) fn yaw_equivalent_error(foot: &PlacerFoot, ctx: &PlacerCtx<'_>) -> f32 {
    let yaw_error = angle_diff(ctx.yaw, foot.planted_yaw).abs();
    yaw_error * ctx.hip_width * ctx.config.k_turn_trigger
}

/// Trigger threshold for turn steps, in the same units.
pub(super) fn turn_trigger(cfg: &FootPlacerConfig, ctx: &PlacerCtx<'_>) -> f32 {
    cfg.turn_step_trigger_angle * ctx.hip_width * cfg.k_turn_trigger
}

/// Keep a planted foot on its ground, and drawn where a leg goes, as the
/// ground and the hip move under and over it.
pub(super) fn settle_stance(foot: &mut PlacerFoot, ctx: &PlacerCtx<'_>, facing: Vector3<f32>) {
    // Planted ankle y tracks the current terrain each substep rather
    // than being frozen at plant time. Without this, a plant that
    // landed mid-recoil (e.g. on the airborne→grounded edge, with
    // physics still settling) leaves foot y stuck above or below
    // the final terrain surface. The contact is extrapolated along
    // its tangent plane to the foot's own xz, so a probe that hit
    // slightly off the anchor still yields the right height.
    // A probe that struck too far from the foot to tell, as one still
    // aimed where the foot was before a replant, leaves it at its height.
    let (x, z) = (foot.planted_position.x, foot.planted_position.z);
    let y = match floor_sample(ctx, foot.side) {
        Some(_) => floor_height_near(ctx, foot.side, x, z),
        None => Some(ctx.foot_y_fallback),
    };
    let hip = hip_position(foot.side, ctx, facing);
    if let Some(y) = y {
        sync_planted_y(foot, y, hip, max_leg_extension(ctx));
    }
    // A plant the hip is leaving is drawn no further out than a leg
    // goes, until the leg lets go of it: the swing it starts then
    // is drawn from the same place.
    if foot.is_planted() {
        foot.position = LegEnvelope::of(foot.side, ctx, facing).draw(foot.planted_position);
    }
}

/// Pull a planted foot's y to the current terrain surface, as far down
/// as the leg goes. No-op while the foot is mid-step — the swing arc
/// owns y during that window.
///
/// The reach limit is what stops a foot from following a floor that is
/// leaving. When a crate is knocked out from under a standing character
/// the probe keeps finding a surface — the crate, on its way down, or
/// the ground it was standing on — and without a limit the planted foot
/// chases it, so the character is drawn with one leg stretched to twice
/// its length reaching after the floor. The leg simply runs out instead,
/// and the foot hangs at the bottom of it.
///
/// Nor does a foot follow a floor up into its own hip: a rock beside the
/// body, a crate shoved under a foot. It stands no higher than a tucked
/// knee brings it ([`SWING_LEAST_DROP`]), sunk into what is higher: a foot
/// in a rock is a foot, and a knee drawn up to the chest is not a stance.
fn sync_planted_y(foot: &mut PlacerFoot, foot_y: f32, hip: Point3<f32>, reach: f32) {
    if !foot.is_planted() {
        return;
    }
    let lowest = lowest_reachable_y(hip, foot.planted_position, reach);
    let highest = hip.y - SWING_LEAST_DROP * reach;
    let y = foot_y.max(lowest).min(highest);
    foot.position.y = y;
    foot.planted_position.y = y;
}

/// The lowest y a foot at this xz can be held at and still be attached
/// to a leg of length `reach`. Where the hip is already further than
/// that horizontally the leg has nothing left to spend on height, so the
/// answer is the hip's own level.
fn lowest_reachable_y(hip: Point3<f32>, foot: Point3<f32>, reach: f32) -> f32 {
    let across = Vector2::new(foot.x - hip.x, foot.z - hip.z).magnitude();
    let drop = (reach * reach - across * across).max(0.0).sqrt();
    hip.y - drop
}
