//! A foot's swing, from lift to plant.
//!
//! ```text
//!   start_step ──► Swing ──► advance_stepping, each substep:
//!
//!     time_landing    when it lands: with the body after a flight, paced
//!                     onto a planned step, or in the gait's own time
//!     come_down       if that is now: plant, wait over a flight's
//!                     landing, or hold with nowhere to land
//!     aim_landing     where it lands: the ideal, the plan's step, or back
//!                     under the hip
//!     level_landing   how high: the probe's floor, or the plan's
//!     clear_ground    over what
//!     Swing::advance  and the foot is drawn within its LegEnvelope
//! ```
//!
//! The arc itself is `Swing` (`swing.rs`); this is what steers it. Each
//! stage is handed the swing and what the stages before it decided, and
//! nothing reaches back.

use nalgebra::{Point3, Vector2, Vector3};

use super::envelope::LegEnvelope;
use super::floor::{floor_height_near, floor_sample, plane_height_near, probe_found_geometry};
use super::geometry::{facing_from_yaw, lerp_point, lerp_unit, normalise_facing, planar_distance};
use super::leg::{hip_position, max_leg_extension, under_hip};
use super::pace::Pace;
use super::placer::{FootPhase, FootSide, PlacerCtx, PlacerFoot, ON_STEP};
use super::planned::{planned_landing, PlannedStep};
use super::swing::Swing;
use super::timing::GaitTiming;

/// How much sooner or later than the gait's own swing a foot may land on a
/// planned step, as a fraction of the time that swing would have left, to
/// come down a plant's distance ahead of the hip.
const SWING_GIVE: f32 = 0.3;

/// The speed a swinging foot's landing is diverted at when a plan moves the
/// footfall it swings to, in m/s, over the time the swing has left: about a
/// stride's own.
const DIVERT_SPEED: f32 = 5.0;

/// Begin a swing toward `ideal`, projected forward by the hip travel
/// expected during the swing so the foot lands at `+s` relative to
/// where the hip *will* be at plant time. The landing height starts
/// from the current floor plane extrapolated to the target xz (exact
/// on uniform slopes) and is refined every substep as the landing
/// probe converges.
///
/// A foot the plan has a step for swings onto it instead, over the gait's
/// swing, or as long as the flight that comes before the step
/// ([`planned_landing`]).
pub(super) fn start_step(
    foot: &mut PlacerFoot,
    ideal: Point3<f32>,
    ctx: &PlacerCtx<'_>,
    pace: &Pace,
) -> Option<PlannedStep> {
    let (timing, facing) = (&pace.timing, pace.facing);
    let planned = planned_landing(ctx, foot.side, timing.swing_duration);
    // A swing through a flight leaves the ground with the body, at its
    // speed: from rest, a swing as long as a hop falls a leg's length behind
    // the body before it gets going, and the leg is drawn trailing level.
    let leaves_at = match planned {
        Some((
            PlannedStep {
                landing: Some(_), ..
            },
            _,
        )) => Vector3::new(ctx.velocity.x, 0.0, ctx.velocity.z),
        Some(_) | None => Vector3::zeros(),
    };
    let (to, duration) = match planned {
        Some((step, lands_in)) => (step.at, lands_in.max(ctx.config.min_step_duration)),
        None => {
            let duration = timing.swing_duration;
            let to_x = ideal.x + ctx.velocity.x * duration;
            let to_z = ideal.z + ctx.velocity.z * duration;
            let to_y = floor_height_near(ctx, foot.side, to_x, to_z).unwrap_or(ideal.y);
            (Point3::new(to_x, to_y, to_z), duration)
        }
    };

    foot.takeoff_up = foot.up;
    foot.pre_lift = 0.0;
    foot.landing_shortened = false;
    foot.stands_on = None;
    let mut swing = Swing::new(
        foot.planted_position,
        to,
        foot.forward,
        normalise_facing(facing),
        duration,
        ctx.step_height,
        leaves_at,
    );
    swing.footfall = planned.and_then(|(step, _)| step.id);
    foot.phase = FootPhase::Stepping(swing);
    planned.map(|(step, _)| step)
}

/// Advance a mid-swing foot through one substep, and plant it when its swing
/// completes. In order: when the swing lands ([`time_landing`]); the landing
/// itself, if that is now ([`come_down`]); otherwise where it lands
/// ([`aim_landing`]), how high ([`level_landing`]) and over what
/// ([`clear_ground`]); and then the foot's move along it.
pub(super) fn advance_stepping(
    foot: &mut PlacerFoot,
    ideal: Point3<f32>,
    ctx: &PlacerCtx<'_>,
    pace: &Pace,
    frame_dt: f32,
) {
    let FootPhase::Stepping(mut swing) = foot.phase else {
        // Planted: keep rendered position locked to planted_position so
        // per-frame pelvis motion doesn't drift the foot.
        foot.position = foot.planted_position;
        return;
    };
    let planned = time_landing(&mut swing, foot.side, ctx, pace, frame_dt);
    if swing.remaining() <= ctx.dt {
        come_down(foot, swing, planned, ctx, pace.facing, frame_dt);
        return;
    }
    aim_landing(foot, &mut swing, planned, ideal, ctx, pace.facing, frame_dt);
    level_landing(&mut swing, planned, foot.side, ctx, pace.facing);
    clear_ground(&mut swing, foot.side, ctx);
    swing.advance(ctx.dt, retarget_chase(ctx));
    foot.position = LegEnvelope::of(foot.side, ctx, pace.facing).draw(swing.position());
    foot.forward = swing_forward(swing.from_forward, swing.to_forward, swing.progress());
    foot.pre_lift = 0.0;
    foot.phase = FootPhase::Stepping(swing);
}

/// Set when `swing` lands, and return the planned step it swings to, where
/// the plan has one for `side` that these legs can land
/// ([`planned_landing`]).
///
/// A swing onto a step a flight comes before lands as the body does, however
/// long the flight: a hop's landing foot is in the air for the hop. One onto
/// a step on the ground is paced to it ([`paced_onto`]). One whose step the
/// plan has taken away (or the body has turned from) is the gait's again,
/// and lands in the gait's time.
///
/// Within the swing's last frame (`frame_dt`) its time is not moved: the
/// plan has been told the foot stands there, and has moved the foot's
/// footfall on to its next. Nor is a held swing's, which is due every
/// substep until it finds somewhere to land.
fn time_landing(
    swing: &mut Swing,
    side: FootSide,
    ctx: &PlacerCtx<'_>,
    pace: &Pace,
    frame_dt: f32,
) -> Option<PlannedStep> {
    let steered = swing.remaining() > frame_dt && !swing.held;
    let planned = planned_landing(ctx, side, swing.remaining());
    // A foot swung through a flight comes down with the body, at its speed;
    // a step on the ground comes to rest on its landing.
    swing.arrive_at(match planned {
        Some((
            PlannedStep {
                landing: Some(_), ..
            },
            _,
        )) if !swing.held => Vector3::new(ctx.velocity.x, 0.0, ctx.velocity.z),
        Some(_) | None => Vector3::zeros(),
    });
    if steered {
        match planned {
            Some((step, lands_in)) if step.landing.is_some() => {
                swing.retime(lands_in, ctx.config.min_step_duration);
            }
            Some((step, _)) => {
                if let Some(lands_in) =
                    paced_onto(&step, swing, &pace.timing, side, ctx, pace.facing)
                {
                    swing.retime(lands_in, ctx.config.min_step_duration);
                }
            }
            None if swing.footfall.is_some() && swing.remaining() > pace.timing.swing_duration => {
                swing.retime(pace.timing.swing_duration, ctx.config.min_step_duration);
            }
            None => {}
        }
    }
    planned.map(|(step, _)| step)
}

/// When a `swing` onto the planned `step` lands, in seconds from now: as the
/// hip comes a plant's distance short of it, where the gait itself would
/// set the foot down, and within [`SWING_GIVE`] of the time the gait's own
/// swing would have left. `None` for a body that is not travelling.
///
/// The gait's own steps aim a plant's distance ahead of where the hip will
/// be, and are aimed again as the body speeds up or slows. A planned
/// footfall is a place, so it is the time that gives: a body that gathers
/// speed through the swing is over the footfall sooner than the gait's
/// clock had it, and a foot that came down in the gait's time would land
/// behind the hips, to be lifted again at once.
fn paced_onto(
    step: &PlannedStep,
    swing: &Swing,
    timing: &GaitTiming,
    side: FootSide,
    ctx: &PlacerCtx<'_>,
    facing: Vector3<f32>,
) -> Option<f32> {
    let (ahead, speed) = ahead_along_travel(step, side, ctx, facing)?;
    let comes_over = (ahead - timing.plant_ahead()) / speed;
    let own = (1.0 - swing.progress()) * timing.swing_duration;
    Some(comes_over.clamp(own * (1.0 - SWING_GIVE), own * (1.0 + SWING_GIVE)))
}

/// How far short of the planned `step` the hip of `side` is, along the
/// body's travel over the ground, in metres, and the speed of that travel;
/// `None` for a body that is not travelling.
pub(super) fn ahead_along_travel(
    step: &PlannedStep,
    side: FootSide,
    ctx: &PlacerCtx<'_>,
    facing: Vector3<f32>,
) -> Option<(f32, f32)> {
    let travel = Vector2::new(ctx.velocity.x, ctx.velocity.z);
    let speed = travel.magnitude();
    if speed <= ctx.config.moving_speed_threshold {
        return None;
    }
    let to_step = step.at - hip_position(side, ctx, facing);
    Some((
        Vector2::new(to_step.x, to_step.z).dot(&travel) / speed,
        speed,
    ))
}

/// End a swing that is due this substep: plant the foot where the swing
/// carried it, if a leg can stand there ([`plantable`]); wait over a
/// flight's landing the body is not yet down on; and otherwise hold the
/// swing, gathering under the hip ([`hold_swing`]).
fn come_down(
    foot: &mut PlacerFoot,
    mut swing: Swing,
    planned: Option<PlannedStep>,
    ctx: &PlacerCtx<'_>,
    facing: Vector3<f32>,
    frame_dt: f32,
) {
    swing.advance(ctx.dt, retarget_chase(ctx));
    let landing = floor_sample(ctx, foot.side);
    match (
        plantable(swing.to, landing, ctx, foot.side, facing),
        planned,
    ) {
        (Some(plant_at), _) => foot.plant(plant_at, &swing, &ctx.planned),
        // Over a flight's landing a moment before the body is down on
        // it: the foot waits there, at the end of its leg. Only a
        // flight's: a step on the ground the leg cannot quite reach
        // when its swing ends is shortened to where it can, at once. A
        // foot that waited for the body there would plant frames late,
        // and at a run every stance would lie behind the hips for it.
        (None, Some(step))
            if !swing.held
                && step.landing.is_some()
                && planar_distance(swing.to, step.at) <= ON_STEP =>
        {
            swing.retime(2.0 * frame_dt.max(ctx.dt), ctx.dt);
            foot.position = LegEnvelope::of(foot.side, ctx, facing).draw(swing.position());
            foot.phase = FootPhase::Stepping(swing);
        }
        (None, _) => hold_swing(foot, swing, under_hip(foot.side, ctx, facing), ctx),
    }
}

/// Where a completing swing may put its foot down, if anywhere.
///
/// Its target, which the swing has carried the foot onto, with the
/// height the probe finds there where the probe struck near enough to
/// tell. With no ground under the target at all, nowhere: the swing goes
/// on, gathering under the hip.
///
/// Then the leg has to be able to get there. A foot cannot plant beyond
/// its own bones, and the case that matters is walking off a block: the
/// probe finds ground a metre down, which is real ground and entirely
/// out of reach, and planting on it anchors a foot the body sails away
/// from. That is the straight leg trailing behind a character who has
/// just stepped off an edge. `None` says so, and the swing goes on.
pub(super) fn plantable(
    to: Point3<f32>,
    landing: Option<(Point3<f32>, Vector3<f32>)>,
    ctx: &PlacerCtx<'_>,
    side: FootSide,
    facing: Vector3<f32>,
) -> Option<Point3<f32>> {
    let plant_at = Point3::new(
        to.x,
        plane_height_near(landing?, to.x, to.z).unwrap_or(to.y),
        to.z,
    );
    let hip = hip_position(side, ctx, facing);
    ((plant_at - hip).magnitude() <= max_leg_extension(ctx)).then_some(plant_at)
}

/// Hold a swing that has run out of anywhere to land.
///
/// The swing goes on, due again every substep, carrying the foot after a
/// target that retreats under the hip at the same rate an illegal one does,
/// until it can plant. A leg with nowhere to put its foot gathers under the body,
/// which is both what a falling character's legs do and the pose the
/// airborne sampler is about to blend to.
fn hold_swing(
    foot: &mut PlacerFoot,
    mut swing: Swing,
    under_hip: Point3<f32>,
    ctx: &PlacerCtx<'_>,
) {
    let retreat = 1.0 - (-ctx.config.unsupported_retreat_rate * ctx.dt).exp();
    swing.to = lerp_point(swing.to, under_hip, retreat);
    swing.peak_lift = 0.0;
    swing.held = true;
    swing.footfall = None;
    swing.retime(ctx.dt, ctx.dt);
    foot.position =
        LegEnvelope::of(foot.side, ctx, facing_from_yaw(ctx.yaw)).draw(swing.position());
    foot.phase = FootPhase::Stepping(swing);
    foot.landing_shortened = true;
}

/// Move where `swing` lands, and which way its toes point there.
///
/// Before the retarget cutoff, the landing chases the latest `ideal`
/// (projected by remaining travel), or the plan's step for the foot where it
/// has one, so direction changes mid-swing don't land the foot somewhere
/// stale. Within the swing's last frame (`frame_dt`) it is not moved: the
/// plan has been told the foot stands there, and has moved the foot's
/// footfall on to its next.
///
/// A target the landing probe finds no ground under is illegal, and it
/// takes precedence over both: the step shortens toward the stance under
/// its hip until the probe finds a surface again.
fn aim_landing(
    foot: &mut PlacerFoot,
    swing: &mut Swing,
    planned: Option<PlannedStep>,
    ideal: Point3<f32>,
    ctx: &PlacerCtx<'_>,
    facing: Vector3<f32>,
    frame_dt: f32,
) {
    let chase = retarget_chase(ctx);
    let u = swing.progress();
    // A target with *nothing* under it is the one that has to retreat. A
    // target with a wall or a body under it is a probe that learned nothing
    // about the floor this frame, and a swing that treats the two alike
    // gives up its landing target to a single frame of contact with the
    // thing it is chasing. A planned step is on a foothold the plan found,
    // across a gap the probe may not see over: it is not given up.
    let target_unsupported = planned.is_none() && !probe_found_geometry(ctx, foot.side);
    if target_unsupported || swing.held {
        // The probe aims at the landing target, so an empty probe means
        // there is no ground under where this foot is about to plant —
        // a hole, or a ledge. That is not a legal target,
        // and nothing downstream will notice: grounding comes from the
        // capsule's contacts now, so a foot planted in mid-air simply
        // stays there. The step retreats instead, to the neutral stance
        // under the hip: whatever is holding the body up is under there,
        // and unlike the takeoff position it is somewhere the leg can
        // still reach. Retreating to the takeoff point is right only
        // while the body stays over it — walk off a ledge with both feet
        // mid-swing and it recedes at walking pace, leaving the legs
        // trailing at full stretch until the airborne pose takes over. A
        // held swing, which found nowhere to land, gathers there too.
        let retreat = 1.0 - (-ctx.config.unsupported_retreat_rate * ctx.dt).exp();
        swing.to = lerp_point(swing.to, under_hip(foot.side, ctx, facing), retreat);
        swing.footfall = None;
        foot.landing_shortened = true;
    } else if !foot.landing_shortened
        && u < ctx.config.swing_retarget_until_fraction
        && swing.remaining() > frame_dt
    {
        let remaining = swing.remaining();
        let desired_to = match planned {
            Some(step) => Point3::new(step.at.x, swing.to.y, step.at.z),
            None => Point3::new(
                ideal.x + ctx.velocity.x * remaining,
                swing.to.y,
                ideal.z + ctx.velocity.z * remaining,
            ),
        };
        // A plan that moves a footfall far, late in the swing onto it,
        // moves the landing only as far as a foot diverts in the time it
        // has left: the swing lands short of the footfall, where otherwise
        // it would dart there. Never by taking longer: a swing given more
        // time lands late, and the gait has nothing to bring its stance
        // forward again but the leg running out behind it.
        let divert = desired_to - swing.to;
        let (asked, most) = (divert.norm(), remaining * DIVERT_SPEED);
        let desired_to = if planned.is_some() && asked > most {
            swing.to + divert * (most / asked)
        } else {
            desired_to
        };
        swing.to = lerp_point(swing.to, desired_to, chase);
        swing.footfall = planned.and_then(|step| step.id);
    }
    if u < ctx.config.swing_retarget_until_fraction {
        swing.to_forward = swing_forward(swing.to_forward, normalise_facing(facing), chase);
    }
}

/// Set the height `swing` lands at, and lift its arc over a landing above
/// its takeoff. The height is the probed terrain, or the plan's footfall,
/// for the whole swing: the swing carries the foot onto whatever its target
/// becomes, so a late correction bends its path and a stale height would
/// pop at plant.
fn level_landing(
    swing: &mut Swing,
    planned: Option<PlannedStep>,
    side: FootSide,
    ctx: &PlacerCtx<'_>,
    facing: Vector3<f32>,
) {
    let reach = max_leg_extension(ctx);
    let hip = hip_position(side, ctx, facing);
    // A planned footfall's height is ground the plan sensed there. The
    // probe's floor counts only near where it struck: carried along its
    // plane to a footfall a leap away, it aims the foot half a metre over a
    // slope's shelf.
    let floor_y = match planned {
        Some(step) => Some(step.at.y),
        None => floor_height_near(ctx, side, swing.to.x, swing.to.z),
    };
    if let Some(floor_y) = floor_y {
        // No deeper than a leg below the hip. The probe goes on finding a
        // surface when the one underfoot is falling away — the crate on
        // its way down, or the floor it used to sit on — and a swing
        // that chases it is reaching for a landing the leg cannot make.
        // The bound is the hip's own height rather than the reach left
        // over after the horizontal stride, because at plant time the
        // hip will have travelled out to meet the target; it is a floor
        // under a runaway, not a stride budget.
        swing.to.y = floor_y.max(hip.y - reach);
        // Stepping up: lift the apex enough to clear the higher landing
        // by a full step height, not just the path's midpoint.
        let rise = swing.to.y - swing.from.y;
        if rise > 0.0 {
            swing.peak_lift = swing.peak_lift.max(ctx.step_height + 0.5 * rise);
        }
    }

    // A floor that falls away stops answering probes — the crate on its way
    // down, the ledge that just collapsed — so the bound above needs a partner
    // that does not depend on a hit. The target keeps the height the floor
    // used to be at while the hip drops past it, and a foot above its own hip
    // is not a step at all.
    //
    // Only the ceiling is unconditional. The floor under the target stays
    // inside the `landing` case: without a hit there is no evidence the
    // ground is close, and pulling the target up to meet the leg makes a
    // plant on a steep descent pop.
    swing.to.y = swing.to.y.min(hip.y);
}

/// Lift `swing` over what lies under its path: a riser or bump the probe
/// finds under the foot must push the foot over it, not let the path cut
/// through.
fn clear_ground(swing: &mut Swing, side: FootSide, ctx: &PlacerCtx<'_>) {
    let path = swing.path();
    swing.clear(
        floor_height_near(ctx, side, path.x, path.z).map_or(0.0, |surface_y| {
            surface_y + ctx.config.swing_obstacle_clearance - path.y
        }),
    );
}

/// The share of their gap a swing's target, lift and clearance close in one
/// substep.
fn retarget_chase(ctx: &PlacerCtx<'_>) -> f32 {
    1.0 - (-ctx.config.swing_retarget_rate * ctx.dt).exp()
}

fn swing_forward(from: Vector3<f32>, to: Vector3<f32>, t: f32) -> Vector3<f32> {
    let s = t.clamp(0.0, 1.0);
    let s = s * s * (3.0 - 2.0 * s);
    lerp_unit(from, to, s)
}
