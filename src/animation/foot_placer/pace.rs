//! The pace of a substep: how the body travels through it, the cadence the
//! gait keeps for that, and where the capture point would have a foot stand.
//!
//! ```text
//!   PlacerCtx ──► Pace::of ──► Pace { facing, velocity, gait_speed,
//!                                │     moving, max_reach, timing }
//!                                ▼
//!   capture point ──► ideal_target ──► where a foot would stand now
//! ```
//!
//! Worked out once at the head of a substep and handed to every stage after
//! it, so that no two of them step to different cadences.

use nalgebra::{Point3, Vector2, Vector3};

use super::capture_point::{stance_offset, turn_in_place_offset};
use super::geometry::facing_from_yaw;
use super::leg::slope_aware_reach_budget;
use super::placer::{PlacerCtx, PlacerFoot};
use super::timing::GaitTiming;

/// How the body travels through one substep, and the cadence the gait keeps
/// for it: worked out once from the substep's inputs, and handed to every
/// stage after.
#[derive(Clone, Copy, Debug)]
pub(super) struct Pace {
    /// The way the body faces: level, unit length.
    pub facing: Vector3<f32>,
    /// The body's velocity over the surface it stands on, without its
    /// vertical part.
    pub velocity: Vector3<f32>,
    /// The speed the gait's clock runs on: the body's own over the ground,
    /// floored while a direction is held so that the first step can fire
    /// before the body has gathered speed.
    pub gait_speed: f32,
    /// Whether the body travels or is asked to. Travelling, feet step on the
    /// gait's clock; at rest, they settle under the body.
    pub moving: bool,
    /// How far from its hip, across the ground, a foot may aim to plant, in
    /// metres: the leg's reach budget, less on a slope.
    pub max_reach: f32,
    /// The cadence at that speed and reach.
    pub timing: GaitTiming,
}

impl Pace {
    /// The pace of the body `ctx` describes.
    pub fn of(ctx: &PlacerCtx<'_>) -> Self {
        let cfg = ctx.config;
        let facing = facing_from_yaw(ctx.yaw);
        let horizontal_velocity = Vector3::new(ctx.velocity.x, 0.0, ctx.velocity.z);
        let speed = horizontal_velocity.magnitude();

        // The gait clock runs on actual horizontal speed, floored while
        // the player holds a direction so the first step can fire before
        // physics velocity catches up with input.
        let has_intent =
            Vector2::new(ctx.intent_direction.x, ctx.intent_direction.z).magnitude_squared() > 1e-4;
        let gait_speed = if has_intent {
            speed.max(cfg.intent_speed_floor)
        } else {
            speed
        };
        let moving = has_intent || speed > cfg.moving_speed_threshold;

        // The reach budget shrinks with terrain slope: the downhill
        // stance endpoint sits `slope · r` below the rest vertical, so
        // plants and cadence must assume the worst endpoint or the
        // planner aims landings the leg cannot reach (seen as straight-
        // leg extensions descending, knees-up scrambling ascending, and
        // overstretch re-pacing the gait into hops).
        let travel = if speed > cfg.moving_speed_threshold {
            Vector2::new(horizontal_velocity.x, horizontal_velocity.z) / speed
        } else {
            Vector2::new(facing.x, facing.z)
        };
        let max_reach = slope_aware_reach_budget(ctx, travel);
        let timing = GaitTiming::derive(
            gait_speed,
            ctx.standing_height,
            ctx.leg_length,
            ctx.stride_gain,
            max_reach,
            cfg,
        );
        Self {
            facing,
            velocity: horizontal_velocity,
            gait_speed,
            moving,
            max_reach,
            timing,
        }
    }
}

/// Compute the ideal xz target for one foot from capture point + turn +
/// stance offsets, clamped to `max_reach` from the pelvis. Y is the
/// caller-supplied foot_y (pelvis-relative).
pub(super) fn ideal_target(
    foot: &PlacerFoot,
    capture_point: Point3<f32>,
    ctx: &PlacerCtx<'_>,
    pace: &Pace,
) -> Point3<f32> {
    let cfg = ctx.config;
    let (facing, max_reach, timing) = (pace.facing, pace.max_reach, &pace.timing);
    let side = foot.side.side_sign();
    let turn = turn_in_place_offset(facing, ctx.yaw_rate, ctx.hip_width, cfg.k_yaw, side);
    let stance = stance_offset(facing, ctx.hip_width, side);

    // `stride` is the velocity/turn-driven displacement from the hip,
    // clamped to `max_reach`. `stance` sits the foot to its own side of
    // the pelvis and is preserved regardless of clamp — otherwise the
    // clamp shrinks lateral foot spacing at speed. `stride_gain` < 1
    // plants short of the full capture point so the body passes over
    // the foot instead of arresting over it (continuous walking).
    //
    // The `2·duty` factor keeps the stance hip-symmetric at every duty
    // factor: a stance lasts `duty` of the cycle, i.e. `4·duty·s` of
    // hip travel (`s = gain·v/ω`), so the foot must plant `2·duty·s`
    // ahead to lift `2·duty·s` behind. Planting at `+s` is only
    // symmetric at duty 0.5 — at running duty (< 0.5) it biased the
    // whole stance forward, reading as legs permanently tilted ahead
    // of the torso.
    //
    // The capture-point part is capped at the *schedule's* plant-ahead
    // (`duty · trigger = 2·duty·s_eff`, with the stride already
    // reach-capped in `GaitTiming::derive`): planting further than the
    // schedule retracts keeps the stance forward-biased and pushes the
    // trailing leg toward the stretch release.
    let plant_gain = ctx.stride_gain * 2.0 * timing.duty_factor;
    let pelvis_xz = Vector2::new(ctx.pelvis.x, ctx.pelvis.z);
    let mut cp_offset = Vector2::new(
        (capture_point.x - ctx.pelvis.x) * plant_gain,
        (capture_point.z - ctx.pelvis.z) * plant_gain,
    );
    let plant_ahead_limit = timing.plant_ahead();
    let cp_sq = cp_offset.x * cp_offset.x + cp_offset.y * cp_offset.y;
    if cp_sq > plant_ahead_limit * plant_ahead_limit {
        cp_offset *= plant_ahead_limit / cp_sq.sqrt();
    }
    let mut stride = cp_offset + turn;
    let stride_sq = stride.x * stride.x + stride.y * stride.y;
    if stride_sq > max_reach * max_reach {
        stride *= max_reach / stride_sq.sqrt();
    }

    // Legs that keep to their own sides land each foot on its side of the
    // body, however the body is carried: sideways, the leading foot steps
    // out and the trailing one closes up to it, never through it.
    let mut offset = stride + stance;
    let own_side = stance_offset(facing, 1.0, side);
    let (aside, least) = (offset.dot(&own_side), cfg.own_side * ctx.hip_width);
    if cfg.own_side > 0.0 && aside < least {
        offset += own_side * (least - aside);
    }

    let target = pelvis_xz + offset;
    Point3::new(target.x, ctx.foot_y_fallback, target.y)
}
