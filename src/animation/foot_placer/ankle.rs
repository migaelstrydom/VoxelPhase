//! How a foot's sole is turned: rolled onto its toe as its step comes due,
//! eased off the ground it leaves and onto the ground it lands on.
//!
//! ```text
//!   update_pre_lift          planted: how near the step's trigger is
//!            │               (PlacerFoot::pre_lift)
//!            ▼
//!   update_foot_orientation  the sole's up-axis, chased each substep
//!                            (PlacerFoot::up)
//! ```
//!
//! Only what is drawn: nothing here moves a foot or decides a step.

use nalgebra::{Point3, Vector3};

use super::geometry::{lerp_unit, planar_distance};
use super::pace::Pace;
use super::placer::{FootPhase, PlacerCtx, PlacerFoot};
use super::stance::{turn_trigger, yaw_equivalent_error};

/// Visual toe-off anticipation for a planted foot: rolls onto the toe
/// as either the translational or turn trigger approaches.
pub(super) fn update_pre_lift(
    foot: &mut PlacerFoot,
    ideal: Point3<f32>,
    ctx: &PlacerCtx<'_>,
    pace: &Pace,
) {
    if !foot.is_planted() {
        return;
    }
    let (timing, moving) = (&pace.timing, pace.moving);

    let err = planar_distance(ideal, foot.planted_position);
    let translation = pre_lift_amount(
        err,
        timing.trigger_threshold,
        ctx.config.prelift_trigger_ratio,
    );
    let turn = if moving {
        pre_lift_amount(
            yaw_equivalent_error(foot, ctx),
            turn_trigger(ctx.config, ctx),
            ctx.config.prelift_trigger_ratio,
        )
    } else {
        0.0
    };
    foot.pre_lift = translation.max(turn);
}

fn pre_lift_amount(trigger: f32, trigger_threshold: f32, trigger_ratio: f32) -> f32 {
    let ratio = trigger_ratio.clamp(0.0, 1.0);
    let start = trigger_threshold * ratio;
    let span = (trigger_threshold - start).max(1e-4);
    let t = ((trigger - start) / span).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Step the foot's rendered up-axis toward a phase-dependent target.
/// The target is:
///
/// - Planted  → `ground_normal` plus visual toe-off pre-lift.
/// - Stepping → a three-segment curve: takeoff-ease → neutral → landing-ease.
///
/// The slerp is a dt-independent exponential chase so the rate config is
/// "how fast the foot hugs the ground" regardless of frame rate.
pub(super) fn update_foot_orientation(
    foot: &mut PlacerFoot,
    ground_normal: Vector3<f32>,
    ctx: &PlacerCtx<'_>,
) {
    let ground = ground_normal.try_normalize(1e-4).unwrap_or_else(Vector3::y);
    let target = match foot.phase {
        FootPhase::Planted => toe_off_up(ground, foot.forward, foot.pre_lift, ctx),
        FootPhase::Stepping(swing) => {
            let u = swing.progress();
            let takeoff = ctx.config.swing_takeoff_fraction.max(1e-4);
            let landing = ctx.config.swing_landing_fraction.max(1e-4);
            let landing_start = (1.0 - landing).clamp(0.0, 1.0);
            if u < takeoff {
                lerp_unit(foot.takeoff_up, Vector3::y(), u / takeoff)
            } else if u >= landing_start {
                lerp_unit(Vector3::y(), ground, (u - landing_start) / landing)
            } else {
                Vector3::y()
            }
        }
    };

    let alpha = 1.0 - (-ctx.config.ankle_slerp_rate * ctx.dt).exp();
    foot.up = lerp_unit(foot.up, target, alpha);
}

fn toe_off_up(
    ground: Vector3<f32>,
    facing: Vector3<f32>,
    pre_lift: f32,
    ctx: &PlacerCtx<'_>,
) -> Vector3<f32> {
    let amount = pre_lift.clamp(0.0, 1.0);
    if amount <= 0.0 {
        return ground;
    }

    let forward_on_ground = (facing - ground * facing.dot(&ground))
        .try_normalize(1e-4)
        .unwrap_or_else(|| Vector3::new(0.0, 0.0, 1.0));
    let pitch = ctx.config.prelift_max_pitch.max(0.0) * amount;
    (ground + forward_on_ground * pitch.tan())
        .try_normalize(1e-4)
        .unwrap_or(ground)
}
