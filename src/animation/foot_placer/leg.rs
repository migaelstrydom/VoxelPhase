//! Where a leg hangs from and how far it goes: the hip, the stance under
//! it, and the two lengths every reach is measured against.
//!
//! ```text
//!   plant_reach        the stretch the gait plans its plants within
//!     ├─ horizontal_reach_budget     across level ground
//!     └─ slope_aware_reach_budget    less, on a slope
//!   max_leg_extension  the stretch at which a leg asks to step: the
//!                      emergency limit, above the planning one
//! ```

use nalgebra::{Point3, Vector2, Vector3};

use super::capture_point::stance_offset;
use super::placer::{FootSide, PlacerCtx};
use super::timing::reach_budget;

/// This foot's hip joint, approximated as the pelvis plus the lateral
/// stance offset at the current facing. Reference point for leg
/// stretch and the overstretch release.
pub(super) fn hip_position(
    side: FootSide,
    ctx: &PlacerCtx<'_>,
    facing: Vector3<f32>,
) -> Point3<f32> {
    let off = stance_offset(facing, ctx.hip_width, side.side_sign());
    Point3::new(ctx.pelvis.x + off.x, ctx.pelvis.y, ctx.pelvis.z + off.y)
}

/// Neutral-stance foot position for a given side, given pelvis and
/// facing. Uses the same lateral convention as `stance_offset` so
/// initial feet sit where the placer wants them by default.
pub(super) fn foot_position_from_stance(
    pelvis: Point3<f32>,
    facing: Vector3<f32>,
    hip_width: f32,
    foot_y: f32,
    side: FootSide,
) -> Point3<f32> {
    let off = stance_offset(facing, hip_width, side.side_sign());
    Point3::new(pelvis.x + off.x, foot_y, pelvis.z + off.y)
}

/// The neutral stance under `side`'s hip: where a swing with nowhere to land
/// gathers.
pub(super) fn under_hip(side: FootSide, ctx: &PlacerCtx<'_>, facing: Vector3<f32>) -> Point3<f32> {
    foot_position_from_stance(ctx.pelvis, facing, ctx.hip_width, ctx.foot_y_fallback, side)
}

/// Hip→foot distance at which the overstretch release fires, in metres.
/// The emergency limit, not the planning one — see `plant_reach`.
pub(super) fn max_leg_extension(ctx: &PlacerCtx<'_>) -> f32 {
    ctx.leg_length * ctx.config.max_leg_stretch_ratio
}

/// Hip→foot distance the planner aims plants within, in metres. Sits
/// below `max_leg_extension` so an ordinary stride leaves the release
/// something to be an emergency about.
fn plant_reach(ctx: &PlacerCtx<'_>) -> f32 {
    ctx.leg_length * ctx.config.plant_reach_ratio
}

/// Horizontal stride budget: how far from the hip a foot may aim to
/// plant. Pythagoras on `plant_reach`, against the *actual* rest
/// vertical from pelvis to foot centre (`pelvis.y − foot_y_fallback`).
pub(super) fn horizontal_reach_budget(ctx: &PlacerCtx<'_>) -> f32 {
    reach_budget(
        plant_reach(ctx),
        (ctx.pelvis.y - ctx.foot_y_fallback).max(0.0),
    )
}

/// Reach budget shrunk for terrain slope along the travel direction.
/// A stance endpoint at horizontal distance `r` on ground dropping `g`
/// per metre sits `v₀ + g·r` below the hip, so the leg constraint is
/// `r² + (v₀ + g·r)² = ext²` — solved for `r` (positive root). One
/// stance endpoint is always the downhill one regardless of travel
/// direction, so the worst |slope| under either foot binds. `g = 0`
/// reduces to `horizontal_reach_budget`.
pub(super) fn slope_aware_reach_budget(ctx: &PlacerCtx<'_>, travel: Vector2<f32>) -> f32 {
    let ext = plant_reach(ctx);
    let v0 = (ctx.pelvis.y - ctx.foot_y_fallback).max(0.0);
    let g = terrain_slope_along(ctx, travel);
    let a = 1.0 + g * g;
    let b = v0 * g;
    let c = v0 * v0 - ext * ext;
    let disc = (b * b - a * c).max(0.0);
    ((disc.sqrt() - b) / a).max(0.0)
}

/// Worst height change per metre of horizontal travel under either
/// foot, from the probe ground normals: `|n·t̂| / n.y` for travel
/// direction `t̂`. Zero when no usable contact (flat assumption).
fn terrain_slope_along(ctx: &PlacerCtx<'_>, travel: Vector2<f32>) -> f32 {
    let mut worst = 0.0f32;
    for normal in [ctx.left_ground_normal, ctx.right_ground_normal] {
        if normal.y > 0.2 {
            let slope = (normal.x * travel.x + normal.z * travel.y) / normal.y;
            worst = worst.max(slope.abs());
        }
    }
    worst
}
