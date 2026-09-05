//! Per-foot `Planted`/`Stepping` FSM driven by the gait clock and the
//! capture point.
//!
//! ```text
//!                 frame inputs (pelvis, velocity, intent, yaw, probes)
//!                                   │
//!                       FootPlacer::tick  ── splits the frame into
//!                                   │       substeps ≤ max_substep_dt,
//!                                   ▼       lerping pelvis/yaw
//!              ┌──────────── substep ─────────────┐
//!              │ GaitTiming::derive (cadence)     │
//!              │ GaitClock::advance (releases)    │
//!              │ ideal targets (capture point)    │
//!              │ fire ≤ 1 step                    │
//!              │ advance swings, y-sync, ankles   │
//!              └──────────────────────────────────┘
//! ```
//!
//! The substep loop is what makes the placer frame-rate independent:
//! plant→lift sequencing that would alias inside a 30 Hz frame resolves
//! identically at any display rate.

use nalgebra::{Point3, Vector2, Vector3};

use super::capture_point::{capture_point_xz, stance_offset, turn_in_place_offset};
use super::clock::{GaitClock, ReleaseKind};
use super::config::FootPlacerConfig;
use super::swing::swing_position;
use super::timing::GaitTiming;

/// Hard cap on substeps per frame, guarding against pathological dt.
const MAX_SUBSTEPS: f32 = 64.0;

/// Which foot a `PlacerFoot` represents.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FootSide {
    Left,
    Right,
}

impl FootSide {
    /// +1 for right, -1 for left. Used to flip lateral offsets.
    #[inline]
    pub fn side_sign(self) -> f32 {
        match self {
            FootSide::Right => 1.0,
            FootSide::Left => -1.0,
        }
    }
}

/// Current phase of a single foot.
#[derive(Clone, Copy, Debug)]
pub enum FootPhase {
    Planted,
    Stepping {
        from: Point3<f32>,
        to: Point3<f32>,
        from_forward: Vector3<f32>,
        to_forward: Vector3<f32>,
        t: f32,
        duration: f32,
        peak_lift: f32,
    },
}

/// State for one foot.
#[derive(Clone, Debug)]
pub struct PlacerFoot {
    pub side: FootSide,
    pub phase: FootPhase,
    /// Current foot position this frame. During `Planted`, equals
    /// `planted_position`. During `Stepping`, traces the swing arc.
    pub position: Point3<f32>,
    /// World-space position where the foot is currently planted.
    pub planted_position: Point3<f32>,
    /// Yaw at the moment of the last plant. Used to trigger steps on
    /// accumulated facing change even when translational motion is zero.
    pub planted_yaw: f32,
    /// Capture-point-based target for this foot this frame — the ideal
    /// xz the foot should be planted at. Exposed for debug overlay.
    pub ideal_xz: Point3<f32>,
    /// Current up-axis of the foot. Slerped each tick toward the target
    /// orientation derived from phase + ground normal. World Y means
    /// "flat, neutral stance".
    pub up: Vector3<f32>,
    /// Current toe direction. Planted feet keep their landing direction
    /// instead of rotating with the hips every frame.
    pub forward: Vector3<f32>,
    /// Up-axis captured at the moment of step-off. Used as the source
    /// pose during the early takeoff-ease portion of the swing — the
    /// foot relaxes from the previous ground contour toward neutral
    /// rather than snapping flat on liftoff.
    pub takeoff_up: Vector3<f32>,
    /// Visual-only toe-off amount in `[0, 1]`. While still planted, this
    /// rolls the foot toward its toe as the step trigger approaches,
    /// giving the back foot anticipation without moving the anchor.
    pub pre_lift: f32,
    /// Seconds this foot has been in its current stance (saturating).
    /// Gates re-releases so a foot landing mid-turn cannot be lifted
    /// again after a single frame of contact.
    pub since_plant: f32,
    /// True once this swing's landing target has been pulled back for
    /// want of ground under it. A shortened step is never re-extended
    /// within the same swing: the target sits at the edge of the ground
    /// the probe can find, and chasing the ideal outward again would
    /// oscillate the foot across that edge every frame.
    pub landing_shortened: bool,
}

impl PlacerFoot {
    fn new(side: FootSide, position: Point3<f32>, facing: Vector3<f32>) -> Self {
        Self {
            side,
            phase: FootPhase::Planted,
            position,
            planted_position: position,
            planted_yaw: 0.0,
            ideal_xz: position,
            up: Vector3::y(),
            forward: normalise_facing(facing),
            takeoff_up: Vector3::y(),
            pre_lift: 0.0,
            since_plant: LONG_AGO,
            landing_shortened: false,
        }
    }

    /// Whether this foot is currently planted.
    #[inline]
    pub fn is_planted(&self) -> bool {
        matches!(self.phase, FootPhase::Planted)
    }

    /// Whether this foot is currently in its swing phase.
    #[inline]
    pub fn is_stepping(&self) -> bool {
        matches!(self.phase, FootPhase::Stepping { .. })
    }

    /// World-space point this foot's ground probe should aim at: the
    /// committed landing target while stepping, the planted anchor
    /// otherwise. Probing the landing (rather than the mid-air foot)
    /// gives the swing a live height estimate of where it will plant.
    pub fn probe_anchor(&self) -> Point3<f32> {
        match self.phase {
            FootPhase::Planted => self.planted_position,
            FootPhase::Stepping { to, .. } => to,
        }
    }
}

/// Inputs to `FootPlacer::tick` — gathered once per frame. The placer
/// substeps internally; `pelvis` and `yaw` are interpolated from the
/// previous frame's values, everything else is held constant across the
/// frame.
#[derive(Clone, Copy)]
pub struct PlacerCtx<'a> {
    pub dt: f32,
    pub pelvis: Point3<f32>,
    /// Pelvis velocity *relative to the surface being stood on*, which is the
    /// only frame a gait means anything in: a character riding a platform at
    /// 3 m/s is standing still, and its feet should behave that way.
    pub velocity: Vector3<f32>,
    /// Velocity of that surface itself, in world space. Planted anchors and
    /// committed landing targets are carried along by it each substep, so a
    /// foot stays on the plank it was put down on rather than on the patch of
    /// world the plank happened to be over.
    pub support_velocity: Vector3<f32>,
    /// Player-requested movement direction before physics velocity has
    /// necessarily caught up. Used for anticipatory start steps.
    pub intent_direction: Vector3<f32>,
    pub yaw: f32,
    pub yaw_rate: f32,
    pub hip_width: f32,
    pub leg_length: f32,
    pub standing_height: f32,
    /// Foot-centre y to use when a per-foot probe hit is not available —
    /// the rest-rig terrain level, typically
    /// `pelvis.y - standing_height - FOOT_HEIGHT`. Capsule is drawn
    /// centred on the foot position so this is the centre, not the sole.
    pub foot_y_fallback: f32,
    pub step_height: f32,
    /// Fraction of the capture point the foot plants at. `1.0` is pure
    /// push-recovery stopping; walking needs a value < 1 so the body
    /// passes over the planted foot and keeps going. Sourced from the
    /// active `GaitPreset`. Also sets the gait cadence: hip travel per
    /// step is `2·gain·v·√(h/g)`.
    pub stride_gain: f32,
    /// Current ground normal under each foot, defaulting to world Y when
    /// no probe has reported a contact. Used as the target up-axis while
    /// the foot is planted and during the landing-ease portion of swing,
    /// and as the tangent plane for extrapolating terrain heights away
    /// from the contact point.
    pub left_ground_normal: Vector3<f32>,
    pub right_ground_normal: Vector3<f32>,
    /// Per-foot terrain contact from the latest probe, aimed at the
    /// foot's `probe_anchor` (landing target while stepping, planted
    /// anchor otherwise). When `Some`, planted feet track the surface
    /// and swings steer their landing height onto it. `None` when no
    /// probe hit is available (e.g. stepping off a ledge) — heights
    /// fall back to `foot_y_fallback`.
    pub left_ground: Option<Point3<f32>>,
    pub right_ground: Option<Point3<f32>>,
    pub config: &'a FootPlacerConfig,
}

/// The procedural foot placer. Owns per-foot phase state and the shared
/// gait clock, and advances them in fixed-bound substeps.
#[derive(Clone, Debug)]
pub struct FootPlacer {
    pub left: PlacerFoot,
    pub right: PlacerFoot,
    /// Whether the placer is currently suspended (airborne). When
    /// suspended, `tick` is a no-op; the placer resumes from whatever
    /// state it was in.
    suspended: bool,
    /// Whether the next `tick` is the first after resuming from suspend.
    /// Used to re-plant feet at neutral stance under the current pelvis
    /// so airborne→grounded doesn't resume from pre-takeoff positions.
    resuming: bool,
    /// Shared phase clock. Schedules releases half a cycle apart so the
    /// feet stay phase-locked.
    clock: GaitClock,
    /// Seconds since the last takeoff (saturating). Gates step firing
    /// so two takeoffs can never happen in (near-)unison — see
    /// `takeoff_stagger_fraction`.
    since_takeoff: f32,
    /// Previous substep's moving flag. The idle→moving edge seeds the
    /// clock so the gait starts with one clean release instead of both
    /// feet releasing together.
    was_moving: bool,
    /// Previous frame's pelvis, for substep input interpolation.
    prev_pelvis: Point3<f32>,
    /// Previous frame's yaw, for substep input interpolation.
    prev_yaw: f32,
    /// Cadence the last substep was planned against. Read-only output:
    /// it is what the gait *intended* — trigger distance, duty factor,
    /// swing duration — as opposed to what the feet were observed doing.
    /// Debug overlays and the offline animation viewer compare the two.
    /// `None` until the first tick, since the timing is derived from
    /// context the placer does not have at construction.
    last_timing: Option<GaitTiming>,
}

/// Saturation value for the placer's event-age timers (`since_takeoff`,
/// `since_plant`) meaning "long ago": never blocks a step.
const LONG_AGO: f32 = 1e6;

impl FootPlacer {
    pub fn new(pelvis: Point3<f32>, facing: Vector3<f32>, hip_width: f32, foot_y: f32) -> Self {
        let left = foot_position_from_stance(pelvis, facing, hip_width, foot_y, FootSide::Left);
        let right = foot_position_from_stance(pelvis, facing, hip_width, foot_y, FootSide::Right);
        Self {
            left: PlacerFoot::new(FootSide::Left, left, facing),
            right: PlacerFoot::new(FootSide::Right, right, facing),
            suspended: false,
            resuming: false,
            clock: GaitClock::new(),
            since_takeoff: LONG_AGO,
            was_moving: false,
            prev_pelvis: pelvis,
            prev_yaw: yaw_from_facing(facing),
            last_timing: None,
        }
    }

    /// Freeze step state. Called while the character is airborne so
    /// feet don't try to step against a body that isn't on the ground.
    /// On the resume edge, the next `tick` re-plants feet at neutral
    /// stance under the current pelvis — resuming from the pre-takeoff
    /// pose would snap visible feet backward when the pelvis has moved.
    pub fn set_suspended(&mut self, suspended: bool) {
        if self.suspended && !suspended {
            self.resuming = true;
        }
        self.suspended = suspended;
    }

    pub fn is_suspended(&self) -> bool {
        self.suspended
    }

    /// Gait phase in `[0, 1)`, for debug output.
    pub fn gait_phase(&self) -> f32 {
        self.clock.phase()
    }

    /// The cadence the most recent substep was planned against, or
    /// `None` before the first tick. Measuring a gait against this is
    /// how a viewer tells "the feet are doing something odd" from "the
    /// feet are doing exactly what they were asked to".
    pub fn timing(&self) -> Option<GaitTiming> {
        self.last_timing
    }

    /// Advance the placer by one frame. Internally splits the frame
    /// into equal substeps no longer than `config.max_substep_dt`, so
    /// plant/lift sequencing cannot alias inside a long display frame.
    pub fn tick(&mut self, ctx: &PlacerCtx<'_>) {
        if self.suspended {
            return;
        }

        if self.resuming {
            self.resuming = false;
            // `prev_pelvis` is frozen while suspended, so it still holds
            // the pelvis at takeoff. A short airborne blip (terrain-seam
            // contact loss, a tiny hop) leaves the planted feet valid —
            // replanting would teleport them and seed a feet-together
            // catch-up storm. Only replant when the body travelled
            // beyond what a normal step can recover.
            if planar_distance(ctx.pelvis, self.prev_pelvis) > horizontal_reach_budget(ctx) {
                self.replant_at_stance(ctx);
            } else {
                self.prev_pelvis = ctx.pelvis;
                self.prev_yaw = ctx.yaw;
            }
        }

        let substeps = (ctx.dt / ctx.config.max_substep_dt.max(1e-4))
            .ceil()
            .clamp(1.0, MAX_SUBSTEPS);
        let n = substeps as u32;
        let sub_dt = ctx.dt / substeps;
        let yaw_delta = angle_diff(ctx.yaw, self.prev_yaw);

        for i in 1..=n {
            let u = i as f32 / substeps;
            let sub_ctx = PlacerCtx {
                dt: sub_dt,
                pelvis: lerp_point(self.prev_pelvis, ctx.pelvis, u),
                yaw: self.prev_yaw + yaw_delta * u,
                ..*ctx
            };
            self.substep(&sub_ctx);
        }

        self.prev_pelvis = ctx.pelvis;
        self.prev_yaw = ctx.yaw;
    }

    /// One fixed-size simulation step.
    fn substep(&mut self, ctx: &PlacerCtx<'_>) {
        let cfg = ctx.config;
        let facing = facing_from_yaw(ctx.yaw);
        // Before anything reads a foot: everything the placer holds in world
        // space belongs to the surface, not to the world.
        carry_feet(&mut self.left, ctx);
        carry_feet(&mut self.right, ctx);
        self.since_takeoff = (self.since_takeoff + ctx.dt).min(LONG_AGO);
        for foot in [&mut self.left, &mut self.right] {
            if foot.is_planted() {
                foot.since_plant = (foot.since_plant + ctx.dt).min(LONG_AGO);
            }
        }
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
        self.last_timing = Some(timing);
        // Ideal targets for both feet from the capture point, clamped to
        // the horizontal reach budget the leg can actually cover.
        let cp = capture_point_xz(ctx.pelvis, horizontal_velocity, ctx.standing_height);
        let left_ideal = ideal_target(&self.left, cp, ctx, facing, max_reach, &timing);
        let right_ideal = ideal_target(&self.right, cp, ctx, facing, max_reach, &timing);
        self.left.ideal_xz = left_ideal;
        self.right.ideal_xz = right_ideal;

        // Gait start: seed the clock so the foot further from its ideal
        // releases now and the other follows half a cycle later. Without
        // this, both feet hit their triggers near-simultaneously under a
        // hard acceleration and the takeoff split starts at a fraction
        // of the stagger gap instead of π — the seed of the in-phase
        // "forward hopping" seen in game. Only seeds when both feet are
        // planted: a live swing means a rhythm already exists.
        if moving && !self.was_moving && self.left.is_planted() && self.right.is_planted() {
            let left_err = planar_distance(left_ideal, self.left.planted_position);
            let right_err = planar_distance(right_ideal, self.right.planted_position);
            let lead = if left_err >= right_err {
                FootSide::Left
            } else {
                FootSide::Right
            };
            self.clock.seed_to_release(lead, &timing);
        }
        self.was_moving = moving;

        self.clock.advance(ctx.dt, gait_speed, &timing, moving);

        // A release that latches while its foot is already mid-swing
        // means the foot stepped ahead of schedule (turn or overstretch
        // trigger) and its window exit arrived during the swing. Acting
        // on it at landing would re-lift the foot after one substep of
        // stance, so drop it; the next regular window exit re-arms the
        // step.
        for foot in [&self.left, &self.right] {
            if foot.is_stepping() && self.clock.pending(foot.side).is_some() {
                self.clock.consume(foot.side);
            }
        }

        let min_takeoff_gap = timing.swing_duration * cfg.takeoff_stagger_fraction;
        let min_stance = timing.swing_duration * cfg.min_stance_fraction;

        // Event releases while moving: a planted foot is asked to step
        // early when the facing has rotated far enough from its plant
        // (turn step), or when the hip has slid to the leg's stretch
        // limit (landing slides, hard accelerations). The kind controls
        // the support rules the step fires under — see `select_step`.
        // Overstretch additionally requires the leg to be *opening* (hip
        // moving away from the foot): a foot that lands far ahead during
        // a hard deceleration is momentarily stretched, but the pelvis
        // is closing on it and the stretch resolves by itself — firing
        // there would re-lift the foot after one frame of contact. The
        // trigger is the *actual* stretch, never a prediction: a
        // predictive lead (latching when the limit would be reached
        // within the stagger gap) reads 0.3 m into the future at sprint
        // speeds, and because latches are sticky it re-paced the whole
        // gait reactively ~0.06 phase ahead of the schedule. Overshoot
        // while a release is gate-blocked is bounded by the hard-margin
        // stagger bypass below.
        let mut hard_overstretch = [false; 2];
        if moving {
            for (slot, foot) in [&self.left, &self.right].into_iter().enumerate() {
                if !foot.is_planted() {
                    continue;
                }
                let hip = hip_position(foot.side, ctx, facing);
                let away = hip - foot.planted_position;
                let distance = away.norm().max(1e-4);
                // Full 3D radial speed: on slopes the hip moves away
                // from a planted foot vertically as much as
                // horizontally, and an xz-only opening gate is blind
                // to that half of the motion.
                let radial_speed = away.dot(&ctx.velocity) / distance;
                if radial_speed > 0.0 && distance >= max_leg_extension(ctx) {
                    self.clock
                        .request_release(foot.side, ReleaseKind::Overstretch);
                    hard_overstretch[slot] =
                        distance >= max_leg_extension(ctx) + cfg.overstretch_hard_margin;
                } else if yaw_equivalent_error(foot, ctx) >= turn_trigger(cfg, ctx) {
                    self.clock.request_release(foot.side, ReleaseKind::Turn);
                }
            }
        }

        // Fire at most one step per substep, and never within a stagger
        // interval of the previous takeoff: simultaneous releases must
        // resolve as visibly staggered steps, not a double-flight hop.
        if let Some(side) = self.select_step(
            &timing,
            moving,
            min_takeoff_gap,
            min_stance,
            hard_overstretch,
            left_ideal,
            right_ideal,
        ) {
            let (foot, ideal) = match side {
                FootSide::Left => (&mut self.left, left_ideal),
                FootSide::Right => (&mut self.right, right_ideal),
            };
            let kind = self.clock.pending(side);
            start_step(foot, ideal, &timing, ctx, facing);
            self.clock.consume(side);
            // Reactive fires ahead of schedule leave the clock with a
            // phase debt, paid down at a bounded catch-up rate (idle
            // settle steps have no pending kind and no schedule to owe).
            if let Some(kind) = kind {
                self.clock.note_fire(side, kind, &timing);
            }
            self.since_takeoff = 0.0;
        }

        update_pre_lift(&mut self.left, left_ideal, &timing, ctx, moving);
        update_pre_lift(&mut self.right, right_ideal, &timing, ctx, moving);

        advance_stepping(&mut self.left, left_ideal, ctx, facing);
        advance_stepping(&mut self.right, right_ideal, ctx, facing);

        // Planted ankle y tracks the current terrain each substep rather
        // than being frozen at plant time. Without this, a plant that
        // landed mid-recoil (e.g. on the airborne→grounded edge, with
        // physics still settling) leaves foot y stuck above or below
        // the final terrain surface. The contact is extrapolated along
        // its tangent plane to the foot's own xz, so a probe that hit
        // slightly off the anchor still yields the right height.
        for foot in [&mut self.left, &mut self.right] {
            let y = floor_sample(ctx, foot.side)
                .map(|(contact, normal)| {
                    floor_height_at(
                        contact,
                        normal,
                        foot.planted_position.x,
                        foot.planted_position.z,
                    )
                })
                .unwrap_or(ctx.foot_y_fallback);
            sync_planted_y(foot, y);
        }

        // Ease each foot's up-axis toward its phase-dependent target.
        // Planted feet pull toward the current ground normal; swinging
        // feet ease off the takeoff pose, pass through neutral in the
        // middle, and ease into the landing-probe normal near plant.
        update_foot_orientation(&mut self.left, ctx.left_ground_normal, ctx);
        update_foot_orientation(&mut self.right, ctx.right_ground_normal, ctx);
    }

    /// Pick the foot (if any) that should start a step this substep.
    ///
    /// While moving, a foot steps when the gait clock has released it
    /// and the previous takeoff is at least a stagger interval old —
    /// near-simultaneous takeoffs would phase-lock the feet into a
    /// two-footed hop. While idle, a foot steps when its planted error
    /// exceeds the settle trigger. A continuous-support gait requires
    /// the other foot to be planted, and *turn* releases require it
    /// regardless of gait — a repositioning step with both feet off
    /// the ground reads as a flail. Every release except overstretch
    /// also waits for `min_stance` of contact: a foot that lands
    /// mid-turn immediately re-accumulates yaw error, and without the
    /// stance gate it would re-lift after a single frame. Ties go to
    /// the larger error.
    fn select_step(
        &self,
        timing: &GaitTiming,
        moving: bool,
        min_takeoff_gap: f32,
        min_stance: f32,
        hard_overstretch: [bool; 2],
        left_ideal: Point3<f32>,
        right_ideal: Point3<f32>,
    ) -> Option<FootSide> {
        let left_err = planar_distance(left_ideal, self.left.planted_position);
        let right_err = planar_distance(right_ideal, self.right.planted_position);

        let eligible = |foot: &PlacerFoot, other: &PlacerFoot, err: f32, hard: bool| {
            if !foot.is_planted() {
                return false;
            }
            if timing.continuous_support && !other.is_planted() {
                return false;
            }
            if moving {
                let staggered = self.since_takeoff >= min_takeoff_gap;
                match self.clock.pending(foot.side) {
                    None => false,
                    // A hard-overstretched leg may not wait out the
                    // stagger — the stretch grows with every blocked
                    // substep at sprint speeds. The two-footed-hop
                    // attractor this could in principle re-open (gotcha
                    // 11) is prevented at the source: resume replants
                    // are displacement-gated and split along travel, so
                    // feet no longer reach the stretch limit in unison.
                    Some(ReleaseKind::Overstretch) => staggered || hard,
                    Some(ReleaseKind::Turn) => {
                        staggered && foot.since_plant >= min_stance && other.is_planted()
                    }
                    // No stance-age gate: the schedule is phase-true (a
                    // window exit comes a fixed half-cycle after the
                    // other foot's), so stance ≥ duty·cycle holds by
                    // construction. Gating scheduled fires on wall-clock
                    // stance age made the gate fight the schedule and,
                    // with the old takeoff resync, locked the feet into
                    // stable off-antiphase splits.
                    Some(ReleaseKind::Scheduled) => staggered,
                }
            } else {
                err >= timing.trigger_threshold && foot.since_plant >= min_stance
            }
        };

        let left_go = eligible(&self.left, &self.right, left_err, hard_overstretch[0]);
        let right_go = eligible(&self.right, &self.left, right_err, hard_overstretch[1]);
        match (left_go, right_go) {
            (true, true) => Some(if left_err >= right_err {
                FootSide::Left
            } else {
                FootSide::Right
            }),
            (true, false) => Some(FootSide::Left),
            (false, true) => Some(FootSide::Right),
            (false, false) => None,
        }
    }

    /// Snap both feet to stance under the current pelvis. Used on the
    /// resume-from-suspend edge (airborne→grounded) after real jumps.
    /// Leaves both feet Planted and marks the current yaw as their
    /// plant reference.
    ///
    /// When landing with horizontal speed the stance is *split* along
    /// the travel direction — left forward, right back, each by the
    /// gait's plant-ahead distance. Replanting both feet at the same
    /// forward position made them hit the stretch release in unison and
    /// phase-locked the recovery into a two-footed hop; a split stance
    /// is also simply how a moving landing looks. The clock is seeded to
    /// release the *back* (right) foot immediately, the front one half a
    /// cycle later.
    fn replant_at_stance(&mut self, ctx: &PlacerCtx<'_>) {
        let facing = facing_from_yaw(ctx.yaw);
        let speed = Vector2::new(ctx.velocity.x, ctx.velocity.z).magnitude();
        let timing = GaitTiming::derive(
            speed,
            ctx.standing_height,
            ctx.leg_length,
            ctx.stride_gain,
            horizontal_reach_budget(ctx),
            ctx.config,
        );
        let split = plant_ahead_distance(&timing);
        let forward = Vector2::new(facing.x, facing.z) * split;

        for (foot, side) in [
            (&mut self.left, FootSide::Left),
            (&mut self.right, FootSide::Right),
        ] {
            let mut position = foot_position_from_stance(
                ctx.pelvis,
                facing,
                ctx.hip_width,
                ctx.foot_y_fallback,
                side,
            );
            let along = match side {
                FootSide::Left => forward,
                FootSide::Right => -forward,
            };
            position.x += along.x;
            position.z += along.y;
            foot.phase = FootPhase::Planted;
            foot.position = position;
            foot.planted_position = position;
            foot.planted_yaw = ctx.yaw;
            foot.up = Vector3::y();
            foot.forward = normalise_facing(facing);
            foot.takeoff_up = Vector3::y();
            foot.pre_lift = 0.0;
            // Old: feet must be immediately eligible for the post-landing
            // catch-up step, not held to the fresh-plant stance gate.
            foot.since_plant = LONG_AGO;
        }
        // Seed rather than plain-reset: the back (right) foot releases
        // immediately and the front foot exactly half a cycle later. A
        // plain reset left the first release a quarter-cycle out, and
        // under a hard post-landing acceleration both feet hit the
        // stretch limit before that — reactive pacing from a
        // near-symmetric stance, i.e. a hop seed.
        self.clock.seed_to_release(FootSide::Right, &timing);
        self.since_takeoff = LONG_AGO;
        self.prev_pelvis = ctx.pelvis;
        self.prev_yaw = ctx.yaw;
    }
}

/// Neutral-stance foot position for a given side, given pelvis and
/// facing. Uses the same lateral convention as `stance_offset` so
/// initial feet sit where the placer wants them by default.
fn foot_position_from_stance(
    pelvis: Point3<f32>,
    facing: Vector3<f32>,
    hip_width: f32,
    foot_y: f32,
    side: FootSide,
) -> Point3<f32> {
    let off = stance_offset(facing, hip_width, side.side_sign());
    Point3::new(pelvis.x + off.x, foot_y, pelvis.z + off.y)
}

/// Compute the ideal xz target for one foot from capture point + turn +
/// stance offsets, clamped to `max_reach` from the pelvis. Y is the
/// caller-supplied foot_y (pelvis-relative).
fn ideal_target(
    foot: &PlacerFoot,
    capture_point: Point3<f32>,
    ctx: &PlacerCtx<'_>,
    facing: Vector3<f32>,
    max_reach: f32,
    timing: &GaitTiming,
) -> Point3<f32> {
    let cfg = ctx.config;
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
    let plant_ahead_limit = plant_ahead_distance(timing);
    let cp_sq = cp_offset.x * cp_offset.x + cp_offset.y * cp_offset.y;
    if cp_sq > plant_ahead_limit * plant_ahead_limit {
        cp_offset *= plant_ahead_limit / cp_sq.sqrt();
    }
    let mut stride = cp_offset + turn;
    let stride_sq = stride.x * stride.x + stride.y * stride.y;
    if stride_sq > max_reach * max_reach {
        stride *= max_reach / stride_sq.sqrt();
    }

    let target = pelvis_xz + stride + stance;
    Point3::new(target.x, ctx.foot_y_fallback, target.y)
}

/// Begin a swing toward `ideal`, projected forward by the hip travel
/// expected during the swing so the foot lands at `+s` relative to
/// where the hip *will* be at plant time. The landing height starts
/// from the current floor plane extrapolated to the target xz (exact
/// on uniform slopes) and is refined every substep as the landing
/// probe converges.
fn start_step(
    foot: &mut PlacerFoot,
    ideal: Point3<f32>,
    timing: &GaitTiming,
    ctx: &PlacerCtx<'_>,
    facing: Vector3<f32>,
) {
    let duration = timing.swing_duration;
    let to_x = ideal.x + ctx.velocity.x * duration;
    let to_z = ideal.z + ctx.velocity.z * duration;
    let to_y = floor_sample(ctx, foot.side)
        .map(|(contact, normal)| floor_height_at(contact, normal, to_x, to_z))
        .unwrap_or(ideal.y);
    let to = Point3::new(to_x, to_y, to_z);

    foot.takeoff_up = foot.up;
    foot.pre_lift = 0.0;
    foot.landing_shortened = false;
    foot.phase = FootPhase::Stepping {
        from: foot.planted_position,
        to,
        from_forward: foot.forward,
        to_forward: normalise_facing(facing),
        t: 0.0,
        duration,
        peak_lift: ctx.step_height,
    };
}

/// Advance a mid-swing foot along its arc; plant it when the swing
/// completes. Before the retarget cutoff, the landing target chases the
/// latest ideal (projected by remaining travel) so direction changes
/// mid-swing don't land the foot somewhere stale. The landing *height*
/// chases the probed terrain for the whole swing — a vertical
/// correction cannot skate, but a stale height pops at plant.
///
/// A target the landing probe finds no ground under is illegal, and it
/// takes precedence over both: the step shortens toward its takeoff
/// point until the probe finds a surface again.
fn advance_stepping(
    foot: &mut PlacerFoot,
    ideal: Point3<f32>,
    ctx: &PlacerCtx<'_>,
    facing: Vector3<f32>,
) {
    let FootPhase::Stepping {
        from,
        mut to,
        from_forward,
        mut to_forward,
        t,
        duration,
        mut peak_lift,
    } = foot.phase
    else {
        // Planted: keep rendered position locked to planted_position so
        // per-frame pelvis motion doesn't drift the foot.
        foot.position = foot.planted_position;
        return;
    };

    let landing = floor_sample(ctx, foot.side);
    let under_hip = foot_position_from_stance(
        ctx.pelvis,
        facing,
        ctx.hip_width,
        ctx.foot_y_fallback,
        foot.side,
    );
    let new_t = t + ctx.dt;
    if new_t >= duration {
        // A target the probe still finds no ground under is refused
        // outright: the retreat below is a smooth approach to the
        // takeoff position, and this is the guarantee it converges on.
        let plant_at = if landing.is_some() { to } else { from };
        foot.phase = FootPhase::Planted;
        foot.position = plant_at;
        foot.planted_position = plant_at;
        foot.forward = to_forward;
        // The turn-trigger reference must be the orientation the foot
        // actually landed with, not the yaw at takeoff — during a fast
        // turn the body sweeps a large angle mid-swing, and a stale
        // reference would re-release the foot on the next substep.
        foot.planted_yaw = yaw_from_facing(to_forward);
        foot.pre_lift = 0.0;
        foot.since_plant = 0.0;
        return;
    }

    let u = new_t / duration;
    let alpha = 1.0 - (-ctx.config.swing_retarget_rate * ctx.dt).exp();
    if landing.is_none() {
        // The probe aims at the landing target, so no floor sample means
        // there is no ground under where this foot is about to plant —
        // a hole, a ledge, or a wall face. That is not a legal target,
        // and nothing downstream will notice: grounding comes from the
        // capsule's contacts now, so a foot planted in mid-air simply
        // stays there. The step retreats instead, to the neutral stance
        // under the hip: whatever is holding the body up is under there,
        // and unlike the takeoff position it is somewhere the leg can
        // still reach. Retreating to the takeoff point is right only
        // while the body stays over it — walk off a ledge with both feet
        // mid-swing and it recedes at walking pace, leaving the legs
        // trailing at full stretch until the airborne pose takes over.
        let retreat = 1.0 - (-ctx.config.unsupported_retreat_rate * ctx.dt).exp();
        to = lerp_point(to, under_hip, retreat);
        foot.landing_shortened = true;
    } else if !foot.landing_shortened && u < ctx.config.swing_retarget_until_fraction {
        let remaining = duration - new_t;
        let desired_to = Point3::new(
            ideal.x + ctx.velocity.x * remaining,
            to.y,
            ideal.z + ctx.velocity.z * remaining,
        );
        to = lerp_point(to, desired_to, alpha);
    }
    if u < ctx.config.swing_retarget_until_fraction {
        to_forward = swing_forward(to_forward, normalise_facing(facing), alpha);
    }

    if let Some((contact, normal)) = landing {
        // The swinging foot's probe aims at the landing target, so its
        // contact (extrapolated along the floor plane to the target xz)
        // is the best live estimate of the plant height. Chase it so
        // the arc arrives at the surface instead of teleporting there
        // at plant.
        let landing_y = floor_height_at(contact, normal, to.x, to.z);
        to.y += (landing_y - to.y) * alpha;
        // Stepping up: lift the apex enough to clear the higher landing
        // by a full step height, not just the lerp baseline's midpoint.
        let rise = to.y - from.y;
        if rise > 0.0 {
            peak_lift = peak_lift.max(ctx.step_height + 0.5 * rise);
        }
    }

    foot.phase = FootPhase::Stepping {
        from,
        to,
        from_forward,
        to_forward,
        t: new_t,
        duration,
        peak_lift,
    };
    foot.position = swing_position(from, to, u, peak_lift);
    // Obstacle clearance: a riser or bump between the endpoints must
    // push the foot over it, not let the arc cut through. Faded at the
    // endpoints so takeoff and plant stay exactly on the surface.
    if let Some((contact, normal)) = landing {
        let surface_y = floor_height_at(contact, normal, foot.position.x, foot.position.z);
        let bell = (std::f32::consts::PI * u).sin();
        let min_y = surface_y + ctx.config.swing_obstacle_clearance * bell;
        if foot.position.y < min_y {
            foot.position.y = min_y;
        }
    }
    foot.forward = swing_forward(from_forward, to_forward, u);
    foot.pre_lift = 0.0;
}

/// Move everything this foot holds in world space along with the surface
/// under it.
///
/// A planted anchor is a promise about a *place on the ground*, and so is a
/// committed landing target. On static terrain the two are the same thing and
/// this is a no-op; on a moving platform, a lift, or a crate falling out from
/// under the character they are not, and without this the foot is left behind
/// by the floor it is standing on — the leg stretches until the overstretch
/// release fires, which reads as feet stuck to the world.
fn carry_feet(foot: &mut PlacerFoot, ctx: &PlacerCtx<'_>) {
    let carry = ctx.support_velocity * ctx.dt;
    if carry == Vector3::zeros() {
        return;
    }
    foot.planted_position += carry;
    foot.position += carry;
    foot.ideal_xz += carry;
    if let FootPhase::Stepping { from, to, .. } = &mut foot.phase {
        *from += carry;
        *to += carry;
    }
}

/// Terrain contact + normal for the given foot, when the probe has hit
/// floor-like geometry (upward-facing normal). Wall hits are not
/// landing surfaces and must not steer foot heights.
fn floor_sample(ctx: &PlacerCtx<'_>, side: FootSide) -> Option<(Point3<f32>, Vector3<f32>)> {
    let (contact, normal) = match side {
        FootSide::Left => (ctx.left_ground, ctx.left_ground_normal),
        FootSide::Right => (ctx.right_ground, ctx.right_ground_normal),
    };
    contact.filter(|_| normal.y > 0.6).map(|c| (c, normal))
}

/// Height of the floor plane through `contact` with the given normal,
/// evaluated at `(x, z)`. Exact for uniform slopes; for flat treads
/// (normal = +Y) it degenerates to `contact.y`.
fn floor_height_at(contact: Point3<f32>, normal: Vector3<f32>, x: f32, z: f32) -> f32 {
    contact.y - (normal.x * (x - contact.x) + normal.z * (z - contact.z)) / normal.y.max(0.6)
}

/// Pull a planted foot's y to the current terrain surface. No-op while
/// the foot is mid-step — the swing arc owns y during that window.
fn sync_planted_y(foot: &mut PlacerFoot, foot_y: f32) {
    if foot.is_planted() {
        foot.position.y = foot_y;
        foot.planted_position.y = foot_y;
    }
}

/// Distance ahead of the hip a foot plants at — the half-span of a
/// hip-symmetric stance. `duty · trigger = 2·duty·s_eff` with `s_eff`
/// the (reach-capped) stride from `GaitTiming::derive`; the stance
/// covers `duty` of the cycle, i.e. `2 · duty · trigger` of hip travel.
fn plant_ahead_distance(timing: &GaitTiming) -> f32 {
    timing.duty_factor * timing.trigger_threshold
}

/// Hip→foot distance at which the overstretch release fires, in metres.
/// The emergency limit, not the planning one — see `plant_reach`.
fn max_leg_extension(ctx: &PlacerCtx<'_>) -> f32 {
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
fn horizontal_reach_budget(ctx: &PlacerCtx<'_>) -> f32 {
    let ext = plant_reach(ctx);
    let vertical = (ctx.pelvis.y - ctx.foot_y_fallback).max(0.0);
    (ext * ext - vertical * vertical).max(0.0).sqrt()
}

/// Reach budget shrunk for terrain slope along the travel direction.
/// A stance endpoint at horizontal distance `r` on ground dropping `g`
/// per metre sits `v₀ + g·r` below the hip, so the leg constraint is
/// `r² + (v₀ + g·r)² = ext²` — solved for `r` (positive root). One
/// stance endpoint is always the downhill one regardless of travel
/// direction, so the worst |slope| under either foot binds. `g = 0`
/// reduces to `horizontal_reach_budget`.
fn slope_aware_reach_budget(ctx: &PlacerCtx<'_>, travel: Vector2<f32>) -> f32 {
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

/// This foot's hip joint, approximated as the pelvis plus the lateral
/// stance offset at the current facing. Reference point for leg
/// stretch and the overstretch release.
fn hip_position(side: FootSide, ctx: &PlacerCtx<'_>, facing: Vector3<f32>) -> Point3<f32> {
    let off = stance_offset(facing, ctx.hip_width, side.side_sign());
    Point3::new(ctx.pelvis.x + off.x, ctx.pelvis.y, ctx.pelvis.z + off.y)
}

/// Accumulated facing change since this foot's plant, expressed in the
/// same metres-of-error units as the translational trigger.
fn yaw_equivalent_error(foot: &PlacerFoot, ctx: &PlacerCtx<'_>) -> f32 {
    let yaw_error = angle_diff(ctx.yaw, foot.planted_yaw).abs();
    yaw_error * ctx.hip_width * ctx.config.k_turn_trigger
}

/// Trigger threshold for turn steps, in the same units.
fn turn_trigger(cfg: &FootPlacerConfig, ctx: &PlacerCtx<'_>) -> f32 {
    cfg.turn_step_trigger_angle * ctx.hip_width * cfg.k_turn_trigger
}

/// Visual toe-off anticipation for a planted foot: rolls onto the toe
/// as either the translational or turn trigger approaches.
fn update_pre_lift(
    foot: &mut PlacerFoot,
    ideal: Point3<f32>,
    timing: &GaitTiming,
    ctx: &PlacerCtx<'_>,
    moving: bool,
) {
    if !foot.is_planted() {
        return;
    }

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

/// Step the foot's rendered up-axis toward a phase-dependent target.
/// The target is:
///
/// - Planted  → `ground_normal` plus visual toe-off pre-lift.
/// - Stepping → a three-segment curve: takeoff-ease → neutral → landing-ease.
///
/// The slerp is a dt-independent exponential chase so the rate config is
/// "how fast the foot hugs the ground" regardless of frame rate.
fn update_foot_orientation(
    foot: &mut PlacerFoot,
    ground_normal: Vector3<f32>,
    ctx: &PlacerCtx<'_>,
) {
    let ground = ground_normal.try_normalize(1e-4).unwrap_or_else(Vector3::y);
    let target = match foot.phase {
        FootPhase::Planted => toe_off_up(ground, foot.forward, foot.pre_lift, ctx),
        FootPhase::Stepping { t, duration, .. } => {
            let u = (t / duration.max(1e-4)).clamp(0.0, 1.0);
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

fn swing_forward(from: Vector3<f32>, to: Vector3<f32>, t: f32) -> Vector3<f32> {
    let s = t.clamp(0.0, 1.0);
    let s = s * s * (3.0 - 2.0 * s);
    lerp_unit(from, to, s)
}

fn pre_lift_amount(trigger: f32, trigger_threshold: f32, trigger_ratio: f32) -> f32 {
    let ratio = trigger_ratio.clamp(0.0, 1.0);
    let start = trigger_threshold * ratio;
    let span = (trigger_threshold - start).max(1e-4);
    let t = ((trigger - start) / span).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
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

#[inline]
pub(crate) fn planar_distance(a: Point3<f32>, b: Point3<f32>) -> f32 {
    let dx = a.x - b.x;
    let dz = a.z - b.z;
    (dx * dx + dz * dz).sqrt()
}

#[inline]
fn lerp_point(a: Point3<f32>, b: Point3<f32>, t: f32) -> Point3<f32> {
    a + (b - a) * t.clamp(0.0, 1.0)
}

#[inline]
pub(crate) fn facing_from_yaw(yaw: f32) -> Vector3<f32> {
    Vector3::new(yaw.sin(), 0.0, yaw.cos())
}

#[inline]
fn yaw_from_facing(facing: Vector3<f32>) -> f32 {
    facing.x.atan2(facing.z)
}

#[inline]
fn normalise_facing(facing: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(facing.x, 0.0, facing.z)
        .try_normalize(1e-4)
        .unwrap_or_else(|| Vector3::new(0.0, 0.0, 1.0))
}

/// Linearly interpolate two unit vectors and renormalise. Good enough for
/// the small angles between world-Y and typical ground normals; avoids
/// pulling in a full quaternion slerp for what is visually a tilt.
#[inline]
fn lerp_unit(a: Vector3<f32>, b: Vector3<f32>, t: f32) -> Vector3<f32> {
    let v = a * (1.0 - t) + b * t;
    v.try_normalize(1e-4).unwrap_or(Vector3::y())
}

/// Shortest signed angle from `a` to `b`, in `(-PI, PI]`.
#[inline]
pub(crate) fn angle_diff(b: f32, a: f32) -> f32 {
    let two_pi = std::f32::consts::TAU;
    let mut d = (b - a) % two_pi;
    if d > std::f32::consts::PI {
        d -= two_pi;
    } else if d < -std::f32::consts::PI {
        d += two_pi;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    const HIP_WIDTH: f32 = 0.12;
    const LEG_LENGTH: f32 = 0.5;
    const STANDING_HEIGHT: f32 = 0.425;

    /// A character standing on something that is itself moving: the body has
    /// the surface's velocity, so its velocity *relative to the surface* — the
    /// only one the placer is given — is zero.
    fn riding(
        pelvis: Point3<f32>,
        support_velocity: Vector3<f32>,
        config: &FootPlacerConfig,
    ) -> PlacerCtx<'_> {
        PlacerCtx {
            dt: 1.0 / 60.0,
            pelvis,
            velocity: Vector3::zeros(),
            support_velocity,
            intent_direction: Vector3::zeros(),
            yaw: 0.0,
            yaw_rate: 0.0,
            hip_width: HIP_WIDTH,
            leg_length: LEG_LENGTH,
            standing_height: STANDING_HEIGHT,
            foot_y_fallback: pelvis.y - STANDING_HEIGHT,
            step_height: 0.15,
            stride_gain: 0.4,
            left_ground_normal: Vector3::y(),
            right_ground_normal: Vector3::y(),
            left_ground: None,
            right_ground: None,
            config,
        }
    }

    /// The sticky-feet case on a moving platform: the anchors are promises
    /// about a place on the floor, and the floor is going somewhere.
    #[test]
    fn a_planted_foot_travels_with_the_surface_under_it() {
        let config = FootPlacerConfig::default();
        let platform = Vector3::new(3.0, 0.0, 0.0);
        let mut pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), HIP_WIDTH, 0.0);
        let start = placer.left.planted_position;

        let dt = 1.0 / 60.0;
        let frames = 60;
        for _ in 0..frames {
            // Carried along with the platform, exactly as friction carries the
            // capsule in game.
            pelvis += platform * dt;
            placer.tick(&riding(pelvis, platform, &config));
        }

        let travelled = platform * (frames as f32 * dt);
        let drift = placer.left.planted_position - (start + travelled);
        assert!(
            drift.magnitude() < 1e-3,
            "the foot slid {:.0} mm across a platform it was standing still on",
            drift.magnitude() * 1000.0
        );
        assert!(
            placer.left.is_planted() && placer.right.is_planted(),
            "standing still on a moving floor is standing still: nothing should step"
        );
    }

    /// The same run with the carrying left out is the bug itself: the anchor
    /// holds a world position and the leg is dragged off it.
    #[test]
    fn without_the_carry_the_body_walks_away_from_its_own_feet() {
        let config = FootPlacerConfig::default();
        let platform = Vector3::new(3.0, 0.0, 0.0);
        let mut pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), HIP_WIDTH, 0.0);
        let start = placer.left.planted_position;

        let dt = 1.0 / 60.0;
        for _ in 0..30 {
            pelvis += platform * dt;
            placer.tick(&riding(pelvis, Vector3::zeros(), &config));
        }

        let drift = (placer.left.planted_position - start).magnitude();
        assert!(
            drift < 1.0,
            "sanity: the anchor cannot have travelled further than the platform"
        );
        assert!(
            (pelvis.x - placer.left.planted_position.x).abs() > 0.2,
            "with no carry the hip must run away from the anchor — that is the defect"
        );
    }
}
