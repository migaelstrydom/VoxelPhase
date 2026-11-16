//! Per-foot `Planted`/`Stepping` FSM driven by the gait clock and the
//! capture point.
//!
//! ```text
//!                 frame inputs (pelvis, velocity, intent, yaw, probes)
//!                                   │
//!                       FootPlacer::tick  ── splits the frame into
//!                                   │       substeps ≤ max_substep_dt,
//!                                   ▼       lerping pelvis/yaw
//!   ┌──────────────────────── substep ─────────────────────────┐
//!   │ carry_with_body  carry and swivel with the body          │
//!   │ Pace::of         travel, reach and cadence     pace.rs   ├─► Pace
//!   │ aim              ideal targets (capture point) pace.rs   ├─► ideals
//!   │ run_clock        seed, advance, drop latches   clock.rs  │
//!   │ ask_releases     overstretch, turn, let go     stance.rs ├─► Strain
//!   │ select_step      which foot lifts, if any                │
//!   │ fire             lift it into its swing      stepping.rs │
//!   │ advance_feet     toe-off                        ankle.rs │
//!   │                  swings                      stepping.rs │
//!   │                  stances kept on their ground  stance.rs │
//!   │                  soles                          ankle.rs │
//!   └──────────────────────────────────────────────────────────┘
//! ```
//!
//! A stage is handed what the ones before it worked out (the `Pace`, the
//! ideal targets, each leg's `Strain`); nothing else passes between them
//! but the feet and the clock. Every foot a stage draws, it draws within
//! the `LegEnvelope`.
//!
//! The substep loop is what makes the placer frame-rate independent:
//! plant→lift sequencing that would alias inside a 30 Hz frame resolves
//! identically at any display rate.
//!
//! The feet are places in the world only while the legs have ground to
//! answer to (`LegMode::Step`). Off, they are wherever a pose draws them
//! ([`FootPlacer::hold_feet`]); and on, a plant or a swing the body has
//! left behind is let go of or drawn at the edge of where a leg goes
//! (`LegEnvelope`), never as a leg pointing at it.

use nalgebra::{Point3, Vector2, Vector3};

use super::ankle::{update_foot_orientation, update_pre_lift};
use super::capture_point::capture_point_xz;
use super::clock::{GaitClock, ReleaseKind};
use super::config::FootPlacerConfig;
use super::geometry::{
    angle_diff, facing_from_yaw, lerp_point, lerp_unit, normalise_facing, planar_distance,
    yaw_from_facing,
};
use super::leg::{
    foot_position_from_stance, hip_position, horizontal_reach_budget, max_leg_extension,
};
use super::mode::LegMode;
use super::pace::{ideal_target, Pace};
use super::planned::{PlannedSteps, StepId};
use super::stance::{settle_stance, Strain};
use super::stepping::{advance_stepping, ahead_along_travel, start_step};
use super::swing::{turned, Swing};
use super::timing::GaitTiming;

/// Hard cap on substeps per frame, guarding against pathological dt.
const MAX_SUBSTEPS: f32 = 64.0;

/// How near its planned step a foot must plant to stand on it, in metres:
/// a swing steered onto a step lands on it; one the plan moved too late
/// for it to follow, or that found nowhere to land there, does not.
pub(super) const ON_STEP: f32 = 0.1;

/// The rate of turn at which a spinning body begins to take its feet round
/// with it, in rad/s: about the fastest turn feet step round, a tight circle
/// at a walk.
pub(super) const SWIVEL_FROM: f32 = 4.0;

/// The rate of turn at which it takes them round entirely, in rad/s. An
/// about-face on the stick spins the body at 20 and more: feet left where
/// they stood are on each other's side of the body half way through it, and
/// take a second of crossed-over steps to come back.
const SWIVEL_FULL: f32 = 8.0;

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

/// One of something for each foot.
#[derive(Clone, Copy, Debug, Default)]
struct BySide<T> {
    left: T,
    right: T,
}

impl<T: Copy> BySide<T> {
    /// The one for `side`.
    fn of(&self, side: FootSide) -> T {
        match side {
            FootSide::Left => self.left,
            FootSide::Right => self.right,
        }
    }
}

/// Current phase of a single foot.
#[derive(Clone, Copy, Debug)]
pub enum FootPhase {
    Planted,
    /// In the air on its swing.
    Stepping(Swing),
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
    /// instead of rotating with the hips every frame; suspended feet,
    /// having no landing to keep, chase the body's heading.
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
    /// The planned step this foot stands on, if it landed on one or a plan
    /// has since named where it stands.
    pub stands_on: Option<StepId>,
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
            stands_on: None,
        }
    }

    /// Plant the foot at `at` from its swing, toes as the swing turned them:
    /// on the planned step the swing steered onto, if it lands on it.
    pub(super) fn plant(&mut self, at: Point3<f32>, swing: &Swing, planned: &PlannedSteps) {
        self.phase = FootPhase::Planted;
        self.position = at;
        self.planted_position = at;
        self.forward = swing.to_forward;
        // The turn-trigger reference must be the orientation the foot
        // actually landed with, not the yaw at takeoff — during a fast turn
        // the body sweeps a large angle mid-swing, and a stale reference
        // would re-release the foot on the next substep.
        self.planted_yaw = yaw_from_facing(swing.to_forward);
        self.pre_lift = 0.0;
        self.since_plant = 0.0;
        self.stands_on = planned
            .of(self.side)
            .filter(|step| step.id.is_some() && step.id == swing.footfall)
            .filter(|step| planar_distance(step.at, at) <= ON_STEP)
            .and_then(|step| step.id);
    }

    /// Whether this foot is currently planted.
    #[inline]
    pub fn is_planted(&self) -> bool {
        matches!(self.phase, FootPhase::Planted)
    }

    /// Whether this foot is currently in its swing phase.
    #[inline]
    pub fn is_stepping(&self) -> bool {
        matches!(self.phase, FootPhase::Stepping(_))
    }

    /// World-space point this foot's ground probe should aim at: the
    /// committed landing target while stepping, the planted anchor
    /// otherwise. Probing the landing (rather than the mid-air foot)
    /// gives the swing a live height estimate of where it will plant.
    pub fn probe_anchor(&self) -> Point3<f32> {
        match self.phase {
            FootPhase::Planted => self.planted_position,
            FootPhase::Stepping(swing) => swing.to,
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
    /// Where a plan of the way ahead lands each foot next: a swing lands
    /// there, when the plan says, instead of where the capture point puts it.
    pub planned: PlannedSteps,
    pub config: &'a FootPlacerConfig,
}

/// The procedural foot placer. Owns per-foot phase state and the shared
/// gait clock, and advances them in fixed-bound substeps.
#[derive(Clone, Debug)]
pub struct FootPlacer {
    pub left: PlacerFoot,
    pub right: PlacerFoot,
    /// What the legs do, as last set ([`Self::set_mode`]).
    mode: LegMode,
    /// The mode left since the last tick, which the next tick enters from.
    entered_from: Option<LegMode>,
    /// Whether, off, the feet are where a pose last drew them
    /// ([`Self::hold_feet`]) rather than where the gait left them.
    held: bool,
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
    /// The length of the frame being ticked. A swing due to land within it
    /// has been reported landed (`traversal::Legs`), and keeps its landing.
    frame_dt: f32,
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
            mode: LegMode::Step,
            entered_from: None,
            held: false,
            clock: GaitClock::new(),
            since_takeoff: LONG_AGO,
            was_moving: false,
            prev_pelvis: pelvis,
            prev_yaw: yaw_from_facing(facing),
            last_timing: None,
            frame_dt: 0.0,
        }
    }

    /// While the legs are off, stand the feet where the pose has drawn them
    /// (a faller's under its hips, a hanger's against a wall, a climber's on
    /// the top). A rig whose pose draws the legs through a flight calls it
    /// every frame of one, so that the legs come back on with every foot
    /// where it was last seen: a landing moves no foot. Ignored while the
    /// legs are on: then the feet are the placer's own.
    pub fn hold_feet(&mut self, left: Point3<f32>, right: Point3<f32>, yaw: f32) {
        if self.mode != LegMode::Off {
            return;
        }
        for (foot, at) in [(&mut self.left, left), (&mut self.right, right)] {
            foot.phase = FootPhase::Planted;
            foot.position = at;
            foot.planted_position = at;
            foot.planted_yaw = yaw;
            foot.pre_lift = 0.0;
            foot.since_plant = LONG_AGO;
        }
        self.held = true;
    }

    /// Set what the legs do from the next tick on. A change of mode is
    /// entered there ([`Self::enter`]), where the frame's context is.
    pub fn set_mode(&mut self, mode: LegMode) {
        if self.mode != mode && self.entered_from.is_none() {
            self.entered_from = Some(self.mode);
        }
        self.mode = mode;
    }

    /// What the legs do.
    pub fn mode(&self) -> LegMode {
        self.mode
    }

    /// Whether the legs are off the gait.
    pub fn is_suspended(&self) -> bool {
        match self.mode {
            LegMode::Step => false,
            LegMode::Off => true,
        }
    }

    /// The planted foot the gait clock has released, which lifts next: on
    /// landing both feet are down at once and the back one is released
    /// straight away.
    pub fn released(&self) -> Option<FootSide> {
        [&self.left, &self.right]
            .into_iter()
            .find(|foot| foot.is_planted() && self.clock.pending(foot.side).is_some())
            .map(|foot| foot.side)
    }

    /// Take `id` as the planned step the planted `side` stands on, where it
    /// landed on none: the plan has named where the foot stands.
    pub fn name_plant(&mut self, side: FootSide, id: StepId) {
        let foot = match side {
            FootSide::Left => &mut self.left,
            FootSide::Right => &mut self.right,
        };
        if foot.is_planted() && foot.stands_on.is_none() {
            foot.stands_on = Some(id);
        }
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
        if let Some(from) = self.entered_from.take() {
            self.enter(from, ctx);
        }
        match self.mode {
            LegMode::Off => {
                self.relax_feet_toward_body(ctx);
                return;
            }
            LegMode::Step => {}
        }

        self.frame_dt = ctx.dt;
        let substeps = (ctx.dt / ctx.config.max_substep_dt.max(1e-4))
            .ceil()
            .clamp(1.0, MAX_SUBSTEPS);
        let n = substeps as u32;
        let sub_dt = ctx.dt / substeps;
        let yaw_delta = angle_diff(ctx.yaw, self.prev_yaw);

        // A body spinning faster than feet step round takes them round with
        // it: the stance it stood in, about its own axis.
        let spin = (yaw_delta / ctx.dt.max(1e-6)).abs();
        let share = ((spin - SWIVEL_FROM) / (SWIVEL_FULL - SWIVEL_FROM)).clamp(0.0, 1.0);
        let swivel = yaw_delta / substeps * (share * share * (3.0 - 2.0 * share));

        for i in 1..=n {
            let u = i as f32 / substeps;
            let sub_ctx = PlacerCtx {
                dt: sub_dt,
                pelvis: lerp_point(self.prev_pelvis, ctx.pelvis, u),
                yaw: self.prev_yaw + yaw_delta * u,
                planned: ctx.planned.later((i - 1) as f32 * sub_dt),
                ..*ctx
            };
            self.substep(&sub_ctx, swivel);
        }

        self.prev_pelvis = ctx.pelvis;
        self.prev_yaw = ctx.yaw;
    }

    /// One fixed-size simulation step, through which the body's spin takes
    /// the feet round by `swivel` radians: the stages of the diagram at the
    /// head of this file, each handed what the ones before it worked out.
    fn substep(&mut self, ctx: &PlacerCtx<'_>, swivel: f32) {
        self.carry_with_body(ctx, swivel);
        let pace = Pace::of(ctx);
        self.last_timing = Some(pace.timing);
        let ideals = self.aim(ctx, &pace);
        self.run_clock(ctx, &pace, ideals);
        let strain = self.ask_releases(ctx, &pace);
        // Fire at most one step per substep, and never within a stagger
        // interval of the previous takeoff: simultaneous releases must
        // resolve as visibly staggered steps, not a double-flight hop.
        if let Some(side) = self.select_step(ctx, &pace, strain, ideals) {
            self.fire(side, ideals.of(side), ctx, &pace);
        }
        self.advance_feet(ctx, &pace, ideals);
    }

    /// Take everything the feet hold in the world along with the surface
    /// they stand on, and round with a body that spins, and age the gait's
    /// timers by the substep.
    fn carry_with_body(&mut self, ctx: &PlacerCtx<'_>, swivel: f32) {
        // Before anything reads a foot: everything the placer holds in world
        // space belongs to the surface, not to the world.
        for foot in [&mut self.left, &mut self.right] {
            carry_feet(foot, ctx);
            swivel_feet(foot, ctx.pelvis, swivel);
        }
        self.since_takeoff = (self.since_takeoff + ctx.dt).min(LONG_AGO);
        for foot in [&mut self.left, &mut self.right] {
            if foot.is_planted() {
                foot.since_plant = (foot.since_plant + ctx.dt).min(LONG_AGO);
            }
        }
    }

    /// Where each foot would stand now: the capture point's targets, clamped
    /// to the horizontal reach budget the leg can actually cover. Each is
    /// kept on its foot for the debug overlay.
    fn aim(&mut self, ctx: &PlacerCtx<'_>, pace: &Pace) -> BySide<Point3<f32>> {
        let cp = capture_point_xz(ctx.pelvis, pace.velocity, ctx.standing_height);
        let ideals = BySide {
            left: ideal_target(&self.left, cp, ctx, pace),
            right: ideal_target(&self.right, cp, ctx, pace),
        };
        self.left.ideal_xz = ideals.left;
        self.right.ideal_xz = ideals.right;
        ideals
    }

    /// Run the gait's clock through the substep: seeded where a gait
    /// starts, advanced by the hip's travel, and cleared of releases it
    /// latched for feet already in the air.
    fn run_clock(&mut self, ctx: &PlacerCtx<'_>, pace: &Pace, ideals: BySide<Point3<f32>>) {
        // Gait start: seed the clock so the foot further from its ideal
        // releases now and the other follows half a cycle later. Without
        // this, both feet hit their triggers near-simultaneously under a
        // hard acceleration and the takeoff split starts at a fraction
        // of the stagger gap instead of π — the seed of the in-phase
        // "forward hopping" seen in game. Only seeds when both feet are
        // planted: a live swing means a rhythm already exists.
        if pace.moving && !self.was_moving && self.left.is_planted() && self.right.is_planted() {
            let left_err = planar_distance(ideals.left, self.left.planted_position);
            let right_err = planar_distance(ideals.right, self.right.planted_position);
            let lead = if left_err >= right_err {
                FootSide::Left
            } else {
                FootSide::Right
            };
            self.clock.seed_to_release(lead, &pace.timing);
        }
        self.was_moving = pace.moving;

        self.clock
            .advance(ctx.dt, pace.gait_speed, &pace.timing, pace.moving);

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
    }

    /// Ask the clock for the steps the planted feet's legs call for ahead of
    /// its schedule, and return what each leg says of its stance
    /// ([`Strain`]).
    fn ask_releases(&mut self, ctx: &PlacerCtx<'_>, pace: &Pace) -> BySide<Strain> {
        let strain = BySide {
            left: Strain::of(&self.left, ctx, pace),
            right: Strain::of(&self.right, ctx, pace),
        };
        for side in [FootSide::Left, FootSide::Right] {
            if let Some(kind) = strain.of(side).asks {
                self.clock.request_release(side, kind);
            }
        }
        strain
    }

    /// Lift `side`'s foot into its swing, towards `ideal` or the step the
    /// plan has for it, and settle the takeoff with the clock.
    fn fire(&mut self, side: FootSide, ideal: Point3<f32>, ctx: &PlacerCtx<'_>, pace: &Pace) {
        let foot = match side {
            FootSide::Left => &mut self.left,
            FootSide::Right => &mut self.right,
        };
        let kind = self.clock.pending(side);
        let planned = start_step(foot, ideal, ctx, pace);
        self.clock.consume(side);
        // Reactive fires ahead of schedule leave the clock with a
        // phase debt, paid down at a bounded catch-up rate (idle
        // settle steps have no pending kind and no schedule to owe).
        if let Some(kind) = kind {
            self.clock.note_fire(side, kind, &pace.timing);
        }
        // A footfall the plan chose, stepped onto on the gait's own
        // clock, is where that clock is pulled to. A flight's landing
        // is not: the plan times that one, and the gait starts again
        // from it.
        let stepped_onto = planned.filter(|step| pace.moving && step.landing.is_none());
        if let Some((ahead, _)) =
            stepped_onto.and_then(|step| ahead_along_travel(&step, side, ctx, pace.facing))
        {
            self.clock.follow(side, ahead, &pace.timing);
        }
        self.since_takeoff = 0.0;
    }

    /// Move each foot through the substep: a planted one's toe-off, a
    /// swinging one's swing, then each stance settled on its ground and
    /// each sole turned to it.
    fn advance_feet(&mut self, ctx: &PlacerCtx<'_>, pace: &Pace, ideals: BySide<Point3<f32>>) {
        let frame_dt = self.frame_dt;
        let mut feet = [
            (&mut self.left, ideals.left),
            (&mut self.right, ideals.right),
        ];
        for (foot, ideal) in &mut feet {
            update_pre_lift(foot, *ideal, ctx, pace);
        }
        for (foot, ideal) in &mut feet {
            advance_stepping(foot, *ideal, ctx, pace, frame_dt);
        }
        for (foot, _) in &mut feet {
            settle_stance(foot, ctx, pace.facing);
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
    ///
    /// A foot the leg has let go of ([`Strain::let_go`]) steps whatever those
    /// rules say.
    fn select_step(
        &self,
        ctx: &PlacerCtx<'_>,
        pace: &Pace,
        strain: BySide<Strain>,
        ideals: BySide<Point3<f32>>,
    ) -> Option<FootSide> {
        let timing = &pace.timing;
        let min_takeoff_gap = timing.swing_duration * ctx.config.takeoff_stagger_fraction;
        let min_stance = timing.swing_duration * ctx.config.min_stance_fraction;
        let left_err = planar_distance(ideals.left, self.left.planted_position);
        let right_err = planar_distance(ideals.right, self.right.planted_position);

        let eligible = |foot: &PlacerFoot, other: &PlacerFoot, err: f32, strain: Strain| {
            if !foot.is_planted() {
                return false;
            }
            if strain.let_go {
                return true;
            }
            if timing.continuous_support && !other.is_planted() {
                return false;
            }
            if pace.moving {
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
                    Some(ReleaseKind::Overstretch) => staggered || strain.hard_overstretch,
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

        let left_go = eligible(&self.left, &self.right, left_err, strain.left);
        let right_go = eligible(&self.right, &self.left, right_err, strain.right);
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

    /// Relax the drawn foot axes toward the body while suspended.
    ///
    /// A planted foot deliberately keeps the direction it landed in, but
    /// a suspended one has no landing to keep: the pose layer hangs its
    /// position off the hips, so axes frozen at takeoff read as feet that
    /// refuse to follow the body through a mid-air turn — and a jump
    /// taken mid-step strands whatever part of that step's turn had not
    /// happened yet. Chasing the body's own heading covers both: the toes
    /// track a turn in progress and finish one the takeoff interrupted.
    /// Heading is chased in yaw rather than across the vectors, because
    /// an about-face in mid-air passes through the antipodal case a
    /// vector lerp has no answer for.
    ///
    /// Only the drawn axes move here. Where the feet are is a pose's to say
    /// while the legs are off ([`Self::hold_feet`]); left unsaid (a
    /// creature's legs, off for a blip of its grounding), phase, anchors
    /// and plant references stay as the gait left them. `takeoff_up`
    /// follows the sole so a swing resumed after a short airborne blip
    /// eases on from what was last drawn.
    fn relax_feet_toward_body(&mut self, ctx: &PlacerCtx<'_>) {
        self.prev_yaw = ctx.yaw;
        let alpha = 1.0 - (-ctx.config.ankle_slerp_rate * ctx.dt).exp();
        for foot in [&mut self.left, &mut self.right] {
            let toe = yaw_from_facing(foot.forward);
            foot.forward = facing_from_yaw(toe + angle_diff(ctx.yaw, toe) * alpha);
            foot.up = lerp_unit(foot.up, Vector3::y(), alpha);
            foot.takeoff_up = foot.up;
        }
    }

    /// Take the legs from mode `from` into the mode now set: the one place
    /// a change of mode keeps or drops what the feet carry across frames.
    ///
    /// Going off drops every footfall the feet stand on or swing to: a plan
    /// named them, and is gone. The feet themselves stay as the gait left
    /// them until a pose holds them where it draws them
    /// ([`Self::hold_feet`]). Coming back on lands them ([`Self::land`]).
    fn enter(&mut self, from: LegMode, ctx: &PlacerCtx<'_>) {
        match (from, self.mode) {
            (LegMode::Step, LegMode::Step) | (LegMode::Off, LegMode::Off) => {}
            (LegMode::Off, LegMode::Step) => self.land(ctx),
            (LegMode::Step, LegMode::Off) => {
                self.held = false;
                for foot in [&mut self.left, &mut self.right] {
                    foot.stands_on = None;
                    if let FootPhase::Stepping(swing) = &mut foot.phase {
                        swing.footfall = None;
                    }
                }
            }
        }
    }

    /// Come back onto the ground.
    ///
    /// Feet a pose held through the flight stand where it last drew them,
    /// which is where the body has just brought them down: no foot moves,
    /// and the gait takes up from there, stepping off the rear one first.
    ///
    /// Feet left as the gait had them (a creature's, whose grounding
    /// flickers) are judged by where they are: each still within its leg's
    /// stretch of its hip, planted or held where its swing was, is kept, and
    /// the others plant at stance, so a blip moves no foot still under the
    /// body.
    fn land(&mut self, ctx: &PlacerCtx<'_>) {
        let kept = self.land_under_body(ctx);
        if std::mem::take(&mut self.held) && kept {
            self.release_rear_foot(ctx);
        }
    }

    /// Come down under the body: keep each foot still within its leg's
    /// stretch of its hip, and plant the others at stance. Where the foot is
    /// is judged, not its swing's landing, which at speed lies a swing's
    /// travel ahead. Returns whether a foot was kept.
    fn land_under_body(&mut self, ctx: &PlacerCtx<'_>) -> bool {
        let facing = facing_from_yaw(ctx.yaw);
        let reach = max_leg_extension(ctx);
        let reaches = |foot: &PlacerFoot| {
            (foot.position - hip_position(foot.side, ctx, facing)).norm() <= reach
        };
        match (reaches(&self.left), reaches(&self.right)) {
            (true, true) => {}
            (false, false) => {
                self.replant_at_stance(ctx);
                return false;
            }
            (left, _) => {
                let side = if left {
                    FootSide::Right
                } else {
                    FootSide::Left
                };
                let position = foot_position_from_stance(
                    ctx.pelvis,
                    facing,
                    ctx.hip_width,
                    ctx.foot_y_fallback,
                    side,
                );
                let foot = match side {
                    FootSide::Left => &mut self.left,
                    FootSide::Right => &mut self.right,
                };
                plant_at_stance(foot, position, ctx.yaw, facing);
            }
        }
        self.prev_pelvis = ctx.pelvis;
        self.prev_yaw = ctx.yaw;
        true
    }

    /// Both feet down from a landing and the body travelling: release the
    /// rear foot now and the front one half a cycle later. Landed side by
    /// side they otherwise reach the stretch release together, and the gait
    /// recovers as a two-footed hop.
    fn release_rear_foot(&mut self, ctx: &PlacerCtx<'_>) {
        let travel = Vector2::new(ctx.velocity.x, ctx.velocity.z);
        let speed = travel.magnitude();
        if speed <= ctx.config.moving_speed_threshold {
            return;
        }
        let along = |foot: &PlacerFoot| {
            let from_pelvis = foot.planted_position - ctx.pelvis;
            Vector2::new(from_pelvis.x, from_pelvis.z).dot(&travel)
        };
        let rear = if along(&self.left) <= along(&self.right) {
            FootSide::Left
        } else {
            FootSide::Right
        };
        let timing = GaitTiming::derive(
            speed,
            ctx.standing_height,
            ctx.leg_length,
            ctx.stride_gain,
            horizontal_reach_budget(ctx),
            ctx.config,
        );
        self.clock.seed_to_release(rear, &timing);
        self.since_takeoff = LONG_AGO;
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
    /// release the *back* foot immediately, the front one half a cycle
    /// later. The front foot is the left.
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
        let split = timing.plant_ahead();
        let forward = Vector2::new(facing.x, facing.z) * split;
        let front = FootSide::Left;

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
            let along = if side == front { forward } else { -forward };
            position.x += along.x;
            position.z += along.y;
            plant_at_stance(foot, position, ctx.yaw, facing);
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
    if let FootPhase::Stepping(swing) = &mut foot.phase {
        swing.carry(carry);
    }
}

/// Turn everything this foot holds in world space by `angle` radians about
/// the vertical through `pivot`, the pelvis of a body spinning on the spot.
///
/// Feet are placed in the world so that a body walking over them does not
/// drag them. A body that spins in a frame or two is not walking over
/// them: left where they stood, they end up behind it, crossed over, and
/// the legs are drawn twisted until the gait has stepped them round. Turned
/// with the body they keep the stance it stood in, sliding round on the
/// ground, which is what a pivot is.
fn swivel_feet(foot: &mut PlacerFoot, pivot: Point3<f32>, angle: f32) {
    if angle == 0.0 {
        return;
    }
    foot.planted_position = pivot + turned(foot.planted_position - pivot, angle);
    foot.position = pivot + turned(foot.position - pivot, angle);
    foot.forward = turned(foot.forward, angle);
    foot.planted_yaw += angle;
    if let FootPhase::Stepping(swing) = &mut foot.phase {
        swing.swivel(pivot, angle);
    }
}

/// Plant `foot` at `position` at stance, facing the body, on no planned
/// step. It is eligible at once for the catch-up step a landing takes, not
/// held to the fresh-plant stance gate.
fn plant_at_stance(foot: &mut PlacerFoot, position: Point3<f32>, yaw: f32, facing: Vector3<f32>) {
    foot.stands_on = None;
    foot.phase = FootPhase::Planted;
    foot.position = position;
    foot.planted_position = position;
    foot.planted_yaw = yaw;
    foot.up = Vector3::y();
    foot.forward = normalise_facing(facing);
    foot.takeoff_up = Vector3::y();
    foot.pre_lift = 0.0;
    foot.since_plant = LONG_AGO;
}

#[cfg(test)]
mod tests {
    use super::super::sim::{simulate_planned, takeoffs, GaitParams, Input, RigDims};
    use super::*;
    use crate::animation::foot_placer::capture_point::stance_offset;
    use crate::animation::foot_placer::envelope::{
        LegEnvelope, SWING_CONE_TANGENT, SWING_LEAST_DROP,
    };
    use crate::animation::foot_placer::floor::floor_sample;
    use crate::animation::foot_placer::planned::{planned_landing, PlannedStep};
    use crate::animation::foot_placer::stepping::plantable;

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
            planned: PlannedSteps::default(),
            config,
        }
    }

    /// Suspend the placer and hold it there for `frames`, driving yaw
    /// from `yaw`. Returns nothing: the feet are read off the placer.
    fn hang(
        placer: &mut FootPlacer,
        config: &FootPlacerConfig,
        frames: u32,
        yaw: impl Fn(u32) -> f32,
    ) {
        let pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        for i in 1..=frames {
            let mut ctx = riding(pelvis, Vector3::zeros(), config);
            ctx.yaw = yaw(i);
            placer.tick(&ctx);
        }
    }

    /// How far a foot's toe points off the body's heading, in degrees.
    /// A swing `elapsed` seconds on from leaving `from` at rest.
    fn swung(
        from: Point3<f32>,
        to: Point3<f32>,
        from_forward: Vector3<f32>,
        to_forward: Vector3<f32>,
        duration: f32,
        peak_lift: f32,
        elapsed: f32,
    ) -> Swing {
        let mut swing = Swing::new(
            from,
            to,
            from_forward,
            to_forward,
            duration,
            peak_lift,
            Vector3::zeros(),
        );
        for _ in 0..10 {
            swing.advance(elapsed / 10.0, 1.0);
        }
        swing
    }

    /// How far a swinging foot's own swing carries it in `dt`, left alone.
    fn carried_on(foot: &PlacerFoot, dt: f32) -> f32 {
        let FootPhase::Stepping(mut swing) = foot.phase else {
            return 0.0;
        };
        let before = swing.position();
        swing.advance(dt, 1.0);
        planar_distance(swing.position(), before)
    }

    /// Where a swinging foot is drawn.
    fn drawn(foot: &PlacerFoot) -> Point3<f32> {
        match foot.phase {
            FootPhase::Stepping(swing) => swing.position(),
            FootPhase::Planted => foot.planted_position,
        }
    }

    fn toe_offset_degrees(foot: &PlacerFoot, yaw: f32) -> f32 {
        angle_diff(yaw_from_facing(foot.forward), yaw).to_degrees()
    }

    /// Turning in mid-air: the feet hang off the hips, so their toes have
    /// to come round with them. Frozen takeoff axes read as boots nailed
    /// to a compass bearing while the body spins.
    #[test]
    fn suspended_feet_turn_their_toes_with_the_body() {
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), HIP_WIDTH, 0.0);
        placer.set_mode(LegMode::Off);

        let turn = std::f32::consts::FRAC_PI_2;
        let frames = 30;
        hang(&mut placer, &config, frames, |i| {
            turn * i as f32 / frames as f32
        });

        // Mid-turn the toes trail the body by the chase's own lag, but
        // they must be most of the way round, not still at takeoff.
        for foot in [&placer.left, &placer.right] {
            let off = toe_offset_degrees(foot, turn);
            assert!(
                off.abs() < 20.0,
                "{:?} toe trails a turning body by {:.1} degrees",
                foot.side,
                off
            );
        }

        // Held heading: the lag is a lag, not an offset the feet keep.
        hang(&mut placer, &config, 30, |_| turn);
        for foot in [&placer.left, &placer.right] {
            let off = toe_offset_degrees(foot, turn);
            assert!(
                off.abs() < 1.0,
                "{:?} toe settled {:.1} degrees off a body that stopped turning",
                foot.side,
                off
            );
        }
    }

    /// Jumping out of a step: the swing that was rotating the foot is
    /// frozen by the suspend, so the half-turned toe has nothing left to
    /// finish it. It must still come round to the body.
    #[test]
    fn a_jump_taken_mid_step_still_finishes_the_turn() {
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), HIP_WIDTH, 0.0);

        // Caught halfway through a quarter-turn step.
        let half_turned = facing_from_yaw(std::f32::consts::FRAC_PI_4);
        placer.left.forward = half_turned;
        placer.right.forward = half_turned;
        placer.set_mode(LegMode::Off);

        let yaw = std::f32::consts::FRAC_PI_2;
        hang(&mut placer, &config, 30, |_| yaw);

        for foot in [&placer.left, &placer.right] {
            let off = toe_offset_degrees(foot, yaw);
            assert!(
                off.abs() < 1.0,
                "{:?} toe stayed {:.1} degrees short: the jump kept the step's turn",
                foot.side,
                off
            );
        }
    }

    /// One walk on flat ground, reported as `(takeoff side, frame)` pairs
    /// and the length in frames of each foot's swings.
    ///
    /// `obstruct` stands in for a probe that struck something upright on
    /// its way to the ground — that foot's contact comes back with a wall
    /// normal instead of the floor's.
    fn walk_flat(
        dims: RigDims,
        gait: GaitParams,
        speed: f32,
        config: &FootPlacerConfig,
        frames: usize,
        obstruct: impl Fn(usize, FootSide) -> bool,
    ) -> (Vec<(FootSide, usize)>, Vec<usize>) {
        const DT: f32 = 1.0 / 60.0;
        let velocity = Vector3::new(0.0, 0.0, speed);
        let mut pelvis = Point3::new(0.0, dims.standing_height, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), dims.hip_width, 0.0);

        let mut takeoffs = Vec::new();
        let mut swings = Vec::new();
        let mut swing_frames = [0usize; 2];

        for f in 0..frames {
            let was_stepping = [placer.left.is_stepping(), placer.right.is_stepping()];
            pelvis += velocity * DT;

            let sample = |foot: &PlacerFoot| {
                let anchor = foot.probe_anchor();
                if obstruct(f, foot.side) {
                    // A capsule's flank: a contact well above the floor,
                    // with a normal pointing sideways out of it.
                    (
                        Some(Point3::new(anchor.x, 0.3, anchor.z)),
                        Vector3::new(1.0, 0.0, 0.0),
                    )
                } else {
                    (Some(Point3::new(anchor.x, 0.0, anchor.z)), Vector3::y())
                }
            };
            let (left_ground, left_ground_normal) = sample(&placer.left);
            let (right_ground, right_ground_normal) = sample(&placer.right);

            placer.tick(&PlacerCtx {
                dt: DT,
                pelvis,
                velocity,
                support_velocity: Vector3::zeros(),
                intent_direction: velocity,
                yaw: 0.0,
                yaw_rate: 0.0,
                hip_width: dims.hip_width,
                leg_length: dims.leg_length,
                standing_height: dims.standing_height,
                foot_y_fallback: pelvis.y - dims.standing_height,
                step_height: gait.step_height,
                stride_gain: gait.stride_gain,
                left_ground_normal,
                right_ground_normal,
                left_ground,
                right_ground,
                planned: PlannedSteps::default(),
                config,
            });

            for (slot, foot) in [&placer.left, &placer.right].into_iter().enumerate() {
                if foot.is_stepping() {
                    if !was_stepping[slot] {
                        takeoffs.push((foot.side, f));
                        swing_frames[slot] = 0;
                    }
                    swing_frames[slot] += 1;
                } else if was_stepping[slot] {
                    swings.push(swing_frames[slot]);
                }
            }
        }
        (takeoffs, swings)
    }

    /// A foot with a planned step lands on it, not where the capture point
    /// would put it: walking over footfalls laid a natural step apart but
    /// 0.1 m along from where the capture point lands, every plant after
    /// the first steps lands on one of its own side's.
    #[test]
    fn a_planned_step_is_landed_on() {
        const DT: f32 = 1.0 / 60.0;
        const SPEED: f32 = 2.0;
        const SHIFT: f32 = 0.1;
        let config = FootPlacerConfig::default();
        let dims = RigDims::player();
        let gait = GaitParams::walk();
        let step = GaitTiming::level_step(
            SPEED,
            dims.standing_height,
            dims.leg_length,
            gait.stride_gain,
            &config,
        );
        // Each side's footfalls a stride apart, the sides a step apart.
        let footfall = |side: FootSide, n: i32| {
            let offset = if side == FootSide::Left { 0.0 } else { 1.0 };
            let across = stance_offset(Vector3::z(), dims.hip_width, side.side_sign());
            Point3::new(across.x, 0.0, (2.0 * n as f32 + offset) * step + SHIFT)
        };
        // Each foot's next footfall: the first of its side's ahead of where
        // it last planted.
        let next = |foot: &PlacerFoot| {
            let n = (1..)
                .find(|&n| footfall(foot.side, n).z > foot.planted_position.z + 0.05)
                .unwrap();
            let at = footfall(foot.side, n);
            PlannedStep {
                id: None,
                at,
                landing: None,
            }
        };
        let velocity = Vector3::new(0.0, 0.0, SPEED);
        let mut pelvis = Point3::new(0.0, dims.standing_height, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), dims.hip_width, 0.0);
        let mut plants = Vec::new();
        for f in 0..240 {
            let was_planted = [placer.left.is_planted(), placer.right.is_planted()];
            pelvis += velocity * DT;
            let ground = |foot: &PlacerFoot| {
                let anchor = foot.probe_anchor();
                Some(Point3::new(anchor.x, 0.0, anchor.z))
            };
            placer.tick(&PlacerCtx {
                dt: DT,
                pelvis,
                velocity,
                support_velocity: Vector3::zeros(),
                intent_direction: velocity,
                yaw: 0.0,
                yaw_rate: 0.0,
                hip_width: dims.hip_width,
                leg_length: dims.leg_length,
                standing_height: dims.standing_height,
                foot_y_fallback: pelvis.y - dims.standing_height,
                step_height: gait.step_height,
                stride_gain: gait.stride_gain,
                left_ground_normal: Vector3::y(),
                right_ground_normal: Vector3::y(),
                left_ground: ground(&placer.left),
                right_ground: ground(&placer.right),
                planned: PlannedSteps {
                    left: Some(next(&placer.left)),
                    right: Some(next(&placer.right)),
                },
                config: &config,
            });
            for (slot, foot) in [&placer.left, &placer.right].into_iter().enumerate() {
                if f > 60 && foot.is_planted() && !was_planted[slot] {
                    plants.push((foot.side, foot.planted_position));
                }
            }
        }
        assert!(plants.len() > 6, "{plants:?}");
        for (side, at) in plants {
            let offset = if side == FootSide::Left { 0.0 } else { 1.0 };
            let n = (0.5 * ((at.z - SHIFT) / step - offset)).round();
            let miss = planar_distance(at, footfall(side, n as i32));
            assert!(miss < 0.01, "{side:?} planted at {at:?}, {miss:.3} m off");
        }
    }

    /// The ground everywhere at height zero, as a foot's probe finds it.
    fn level_ground(foot: &PlacerFoot) -> Option<Point3<f32>> {
        let anchor = foot.probe_anchor();
        Some(Point3::new(anchor.x, 0.0, anchor.z))
    }

    /// The cadence of the player's walk gait at `speed` on level ground.
    fn level_timing(speed: f32, config: &FootPlacerConfig) -> GaitTiming {
        let dims = RigDims::player();
        GaitTiming::derive(
            speed,
            dims.standing_height,
            dims.leg_length,
            GaitParams::walk().stride_gain,
            GaitTiming::level_reach(dims.standing_height, dims.leg_length, config),
            config,
        )
    }

    /// The pace of the body `ctx` describes, stepping to the cadence `timing`
    /// and facing along `facing`.
    fn paced(ctx: &PlacerCtx<'_>, timing: &GaitTiming, facing: Vector3<f32>) -> Pace {
        Pace {
            timing: *timing,
            facing,
            ..Pace::of(ctx)
        }
    }

    /// A hop stridden through is the landing foot's swing: over a 0.4 s hop
    /// with the plan landing the right foot 1.2 m on, the right foot is in
    /// the air for the hop and plants on the landing as the body comes down
    /// on it, not before, without ever giving it up; the left, which pushed
    /// off, lifts and swings on to its own footfall a step further.
    #[test]
    fn a_hop_stridden_through_is_the_landing_foots_swing() {
        const DT: f32 = 1.0 / 60.0;
        const SPEED: f32 = 3.0;
        const FLIGHT: f32 = 0.4;
        let config = FootPlacerConfig::default();
        let dims = RigDims::player();
        let gait = GaitParams::walk();
        // The body rises off the ground and comes down on the landing.
        let rise = |t: f32| 0.3 * (1.0 - (2.0 * t.min(FLIGHT) / FLIGHT - 1.0).powi(2));
        let pelvis_at = |t: f32| Point3::new(0.0, dims.standing_height + rise(t), SPEED * t);
        let climb = |t: f32| (rise(t + 0.5 * DT) - rise(t - 0.5 * DT)) / DT;
        let mut placer = FootPlacer::new(pelvis_at(0.0), Vector3::z(), dims.hip_width, 0.0);
        placer.right.phase = FootPhase::Stepping(swung(
            placer.right.planted_position,
            Point3::new(-0.12, 0.0, 0.6),
            Vector3::z(),
            Vector3::z(),
            0.2,
            gait.step_height,
            0.0,
        ));
        let landing = Point3::new(-0.12, 0.0, SPEED * FLIGHT);
        let after = Point3::new(0.12, 0.0, SPEED * FLIGHT + 0.6);

        let (mut landed_at, mut left_swung_to, mut t) = (None, None, 0.0);
        while t < FLIGHT + 0.2 {
            // The plan lands the right foot, and has no more for it once it
            // has.
            let planned = PlannedSteps {
                left: Some(PlannedStep {
                    id: None,
                    at: after,
                    landing: Some(FLIGHT + 0.2 - t),
                }),
                right: landed_at.is_none().then_some(PlannedStep {
                    id: None,
                    at: landing,
                    landing: Some(FLIGHT - t),
                }),
            };
            placer.tick(&PlacerCtx {
                dt: DT,
                pelvis: pelvis_at(t),
                velocity: Vector3::new(0.0, climb(t), SPEED),
                support_velocity: Vector3::zeros(),
                intent_direction: Vector3::z(),
                yaw: 0.0,
                yaw_rate: 0.0,
                hip_width: dims.hip_width,
                leg_length: dims.leg_length,
                standing_height: dims.standing_height,
                foot_y_fallback: pelvis_at(t).y - dims.standing_height,
                step_height: gait.step_height,
                stride_gain: gait.stride_gain,
                left_ground_normal: Vector3::y(),
                right_ground_normal: Vector3::y(),
                left_ground: level_ground(&placer.left),
                right_ground: level_ground(&placer.right),
                planned,
                config: &config,
            });
            match placer.right.phase {
                FootPhase::Planted if landed_at.is_none() => landed_at = Some(t),
                FootPhase::Stepping(swing) => {
                    assert!(
                        !swing.held,
                        "the landing foot gave up its landing at {t:.2} s"
                    )
                }
                FootPhase::Planted => {}
            }
            if let FootPhase::Stepping(swing) = placer.left.phase {
                left_swung_to = Some(swing.to);
            }
            t += DT;
        }

        let landed_at = landed_at.expect("the landing foot came down");
        assert!(
            landed_at > FLIGHT - 0.1,
            "planted at {landed_at:.2} s, before the body was down"
        );
        assert!(
            planar_distance(placer.right.planted_position, landing) < 0.01,
            "landed at {:?}",
            placer.right.planted_position
        );
        let to = left_swung_to.expect("the push-off foot lifted");
        assert!(planar_distance(to, after) < 0.01, "trailing to {to:?}");
    }

    /// A planned step is the swing's to land only where the body is coming
    /// to it: not one behind a body travelling on, off to its side, a flight
    /// too long ahead, or a storey above or below.
    #[test]
    fn a_step_the_body_is_not_coming_to_is_not_swung_to() {
        const SWING: f32 = 0.2;
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, RigDims::player().standing_height, 0.0);
        let lands = |at: Point3<f32>, landing: Option<f32>, velocity: Vector3<f32>| {
            let planned = PlannedSteps {
                left: None,
                right: Some(PlannedStep {
                    id: None,
                    at,
                    landing,
                }),
            };
            let ctx = flight_ctx(&config, pelvis, velocity, planned);
            planned_landing(&ctx, FootSide::Right, SWING).map(|(_, lands_in)| lands_in)
        };
        let running = Vector3::new(0.0, 0.0, 3.0);
        let at = |x: f32, y: f32, z: f32| Point3::new(x, y, z);
        let ahead = at(-0.12, 0.0, 0.6);

        // A step ahead is landed in the swing's own time; one a flight comes
        // before, as the body comes down on it.
        assert_eq!(lands(ahead, None, running), Some(SWING));
        let hop = Point3::new(-0.12, 0.0, 1.5);
        assert_eq!(lands(hop, Some(0.45), running), Some(0.45));
        // A stride further on than the leg lands it is still swung to: the
        // swing ends where the leg reaches.
        assert_eq!(lands(at(-0.12, 0.0, 1.2), None, running), Some(SWING));

        assert_eq!(lands(at(-0.12, 0.0, -1.0), None, running), None, "behind");
        assert_eq!(lands(at(0.12, 0.0, 0.6), None, running), None, "crossed");
        assert_eq!(lands(at(-1.2, 0.0, 0.6), None, running), None, "aside");
        assert_eq!(
            lands(at(-0.12, 0.0, 4.0), Some(1.3), running),
            None,
            "far on"
        );
        assert_eq!(
            lands(at(-0.12, 0.0, 3.0), Some(0.2), running),
            None,
            "short"
        );
        assert_eq!(lands(at(-0.12, -3.0, 0.6), None, running), None, "below");
        assert_eq!(lands(at(-0.12, 1.5, 0.6), None, running), None, "above");
    }

    /// Footfalls a plan lays a little closer together than the gait's clock
    /// would keep their stance under the hips, because the clock follows
    /// them. A clock that ran on regardless would lift each foot later
    /// against its footfall than the last, until every stance ended at the
    /// end of the leg.
    #[test]
    fn a_stance_on_planned_footfalls_stays_under_the_hips() {
        const DT: f32 = 1.0 / 60.0;
        const SPEED: f32 = 4.0;
        /// How near the hips come to a footfall before the plan names the
        /// next one for that foot, in metres.
        const NAMED_FROM: f32 = 0.1;
        let config = FootPlacerConfig::default();
        let dims = RigDims::player();
        let gait = GaitParams::walk();
        let level = |of: fn(f32, f32, f32, f32, &FootPlacerConfig) -> f32| {
            of(
                SPEED,
                dims.standing_height,
                dims.leg_length,
                gait.stride_gain,
                &config,
            )
        };
        let step = 0.94 * level(GaitTiming::level_step);
        let plant_ahead = level(GaitTiming::level_plant_ahead);
        // Each foot's footfalls lie two steps apart along +z, the left's a
        // step on from the right's.
        let footfall = |side: FootSide, pelvis_z: f32| {
            let first = match side {
                FootSide::Right => 0.0,
                FootSide::Left => step,
            };
            let nth = ((pelvis_z + NAMED_FROM - first) / (2.0 * step)).ceil();
            Some(PlannedStep {
                id: None,
                at: Point3::new(-0.12 * side.side_sign(), 0.0, first + nth * 2.0 * step),
                landing: None,
            })
        };
        let frames = simulate_planned(
            dims,
            config.clone(),
            600,
            |_| DT,
            gait,
            |_, _| Some(0.0),
            |_| Input::moving(Vector3::new(0.0, 0.0, SPEED), 0.0),
            |_| false,
            |_, pelvis, _| PlannedSteps {
                left: footfall(FootSide::Left, pelvis.z),
                right: footfall(FootSide::Right, pelvis.z),
            },
        );

        let mut lifts = 0;
        for (side, at) in takeoffs(&frames).into_iter().filter(|(_, at)| *at > 300) {
            let last_stood = &frames[at - 1];
            let foot = match side {
                FootSide::Left => &last_stood.left,
                FootSide::Right => &last_stood.right,
            };
            let on = footfall(side, foot.planted_position.z - 2.0 * NAMED_FROM)
                .expect("a footfall")
                .at;
            assert!(
                planar_distance(foot.planted_position, on) < 0.03,
                "frame {at}: stood {:?}, off its footfall {on:?}",
                foot.planted_position
            );
            // The frame before it lifts, the foot is up to a frame's
            // travel short of where it lifts: a plant's distance behind
            // its hip.
            let trail = last_stood.pelvis.z - foot.planted_position.z;
            assert!(
                trail > plant_ahead - 2.0 * SPEED * DT && trail < plant_ahead + 0.03,
                "frame {at}: lifted {trail:.2} m behind its hip, for {plant_ahead:.2}"
            );
            lifts += 1;
        }
        assert!(lifts >= 10, "{lifts} lifts");
    }

    /// A body gathering speed is over a planned footfall sooner than the
    /// gait's clock had it when the foot lifted. The swing is paced to the
    /// footfall, so the foot comes down on it ahead of its hip; in the
    /// gait's own time it would be too late for the footfall, and the legs
    /// would give the plan up until the body was at speed.
    #[test]
    fn a_foot_lands_on_a_planned_footfall_ahead_of_a_hip_gathering_speed() {
        const DT: f32 = 1.0 / 60.0;
        const SPEED: f32 = 8.0;
        const GATHERS: f32 = 16.0;
        const NAMED_FROM: f32 = 0.1;
        let config = FootPlacerConfig::default();
        let dims = RigDims::player();
        let gait = GaitParams::walk();
        // The plan lays its footfalls for the speed asked for, from the
        // start, as the game's does.
        let step = GaitTiming::level_step(
            SPEED,
            dims.standing_height,
            dims.leg_length,
            gait.stride_gain,
            &config,
        );
        let footfall = |side: FootSide, pelvis_z: f32| {
            let first = match side {
                FootSide::Right => 0.0,
                FootSide::Left => step,
            };
            let nth = ((pelvis_z + NAMED_FROM - first) / (2.0 * step)).ceil();
            Point3::new(-0.12 * side.side_sign(), 0.0, first + nth * 2.0 * step)
        };
        let planned = |side: FootSide, pelvis_z: f32| {
            Some(PlannedStep {
                id: None,
                at: footfall(side, pelvis_z),
                landing: None,
            })
        };
        let frames = simulate_planned(
            dims,
            config.clone(),
            150,
            |_| DT,
            gait,
            |_, _| Some(0.0),
            |f| {
                let speed = (GATHERS * DT * f as f32).min(SPEED);
                Input::moving(Vector3::new(0.0, 0.0, speed), 0.0)
            },
            |_| false,
            |_, pelvis, _| PlannedSteps {
                left: planned(FootSide::Left, pelvis.z),
                right: planned(FootSide::Right, pelvis.z),
            },
        );

        let mut on_footfalls = 0;
        let mut while_gathering = 0;
        for pair in frames.windows(2) {
            for (before, now) in [
                (&pair[0].left, &pair[1].left),
                (&pair[0].right, &pair[1].right),
            ] {
                if !before.is_stepping() || !now.is_planted() {
                    continue;
                }
                let on = footfall(now.side, now.planted_position.z - 2.0 * NAMED_FROM);
                if planar_distance(now.planted_position, on) > 0.03 {
                    continue;
                }
                on_footfalls += 1;
                while_gathering += usize::from(pair[1].velocity.z < SPEED);
                let lead = now.planted_position.z - pair[1].pelvis.z;
                assert!(
                    lead > 0.1,
                    "{:.2} s: landed {lead:.2} m ahead of its hip",
                    pair[1].time
                );
            }
        }
        assert!(on_footfalls >= 8, "{on_footfalls} plants on footfalls");
        assert!(
            while_gathering >= 2,
            "{while_gathering} plants on footfalls while gathering speed"
        );
    }

    /// Legs that go off keep no footfall, and coming back on far below they
    /// stand under the body rather than reaching back up to where it left.
    #[test]
    fn legs_gone_off_keep_no_footfall_and_land_under_the_body() {
        const DT: f32 = 1.0 / 60.0;
        let config = FootPlacerConfig::default();
        let dims = RigDims::player();
        let pelvis_at = |t: f32| Point3::new(0.0, dims.standing_height - 5.0 * t * t, 0.5 * t);
        let mut placer = FootPlacer::new(pelvis_at(0.0), Vector3::z(), dims.hip_width, 0.0);
        let mut swing = swung(
            placer.right.planted_position,
            Point3::new(-0.12, 0.0, 0.3),
            Vector3::z(),
            Vector3::z(),
            0.2,
            0.15,
            0.0,
        );
        swing.footfall = Some(StepId(4));
        placer.right.phase = FootPhase::Stepping(swing);
        placer.left.stands_on = Some(StepId(3));
        let ctx = |t: f32| PlacerCtx {
            foot_y_fallback: pelvis_at(t).y - dims.standing_height,
            ..flight_ctx(
                &config,
                pelvis_at(t),
                Vector3::new(0.0, -10.0 * t, 0.5),
                PlannedSteps::default(),
            )
        };

        placer.set_mode(LegMode::Off);
        let mut t = 0.0;
        while t < 2.0 {
            placer.tick(&ctx(t));
            t += DT;
        }
        assert_eq!(placer.left.stands_on, None);
        let FootPhase::Stepping(swing) = placer.right.phase else {
            panic!("a swing left as the gait had it is kept, frozen");
        };
        assert_eq!(swing.footfall, None, "the swing keeps the plan's footfall");

        placer.set_mode(LegMode::Step);
        placer.tick(&ctx(t));
        let pelvis = pelvis_at(t);
        for foot in [&placer.left, &placer.right] {
            assert!(
                (pelvis.y - foot.position.y).abs() < dims.leg_length,
                "{:?} foot left {:.1} m up",
                foot.side,
                foot.position.y - pelvis.y
            );
        }
    }

    /// Feet a pose holds through a flight land where it last drew them: no
    /// foot moves across the ground as the legs come back on, and the rear
    /// one is the first to step.
    #[test]
    fn feet_a_pose_held_through_a_flight_land_where_it_drew_them() {
        const DT: f32 = 1.0 / 60.0;
        const SPEED: f32 = 4.0;
        let config = FootPlacerConfig::default();
        let dims = RigDims::player();
        // Falling 1 m and running on along +z.
        let pelvis_at = |t: f32| {
            let fallen = (5.0 * t * t).min(1.0);
            Point3::new(0.0, dims.standing_height + 1.0 - fallen, SPEED * t)
        };
        // The pose hangs the feet under the hips, the left a little ahead.
        let hung = |pelvis: Point3<f32>| {
            let drop = Vector3::y() * (0.9 * dims.standing_height);
            (
                pelvis + Vector3::new(0.12, 0.0, 0.1) - drop,
                pelvis + Vector3::new(-0.12, 0.0, -0.1) - drop,
            )
        };
        let ctx = |t: f32| PlacerCtx {
            foot_y_fallback: pelvis_at(t).y - dims.standing_height,
            ..flight_ctx(
                &config,
                pelvis_at(t),
                Vector3::new(0.0, 0.0, SPEED),
                PlannedSteps::default(),
            )
        };
        let mut placer = FootPlacer::new(pelvis_at(0.0), Vector3::z(), dims.hip_width, 1.0);

        placer.set_mode(LegMode::Off);
        let (mut t, mut drawn) = (0.0, hung(pelvis_at(0.0)));
        while 5.0 * t * t < 1.0 {
            placer.tick(&ctx(t));
            drawn = hung(pelvis_at(t));
            placer.hold_feet(drawn.0, drawn.1, 0.0);
            t += DT;
        }

        placer.set_mode(LegMode::Step);
        let mut landing = ctx(t);
        landing.left_ground = level_ground(&placer.left);
        landing.right_ground = level_ground(&placer.right);
        placer.tick(&landing);
        for (foot, drawn) in [(&placer.left, drawn.0), (&placer.right, drawn.1)] {
            let moved = planar_distance(foot.position, drawn);
            let stepped = if foot.is_planted() {
                0.0
            } else {
                carried_on(foot, DT) + 0.02
            };
            assert!(
                moved <= stepped + 1e-4,
                "{:?} foot moved {moved:.2} m across the ground on landing",
                foot.side
            );
        }
        // Both were down; the rear one, the right, is the one the gait
        // releases.
        assert!(placer.left.is_planted(), "the front foot holds the landing");
        assert!(
            placer.right.is_stepping() || placer.released() == Some(FootSide::Right),
            "the rear foot steps first"
        );
    }

    /// The envelope draws a stride's own feet where they are, and one held
    /// outside it at its edge: no higher than a tucked knee, no further off
    /// straight down than a stride sweeps, no further than the leg
    /// stretches and, for legs that keep to their sides, not across the
    /// body.
    #[test]
    fn a_foot_is_drawn_where_a_leg_goes() {
        let config = FootPlacerConfig {
            own_side: 0.4,
            ..FootPlacerConfig::default()
        };
        let pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let ctx = riding(pelvis, Vector3::zeros(), &config);
        // Facing +z, the right leg hangs at -x.
        let leg = LegEnvelope::of(FootSide::Right, &ctx, Vector3::z());
        let hip = leg.hip();
        let drawn = |from_hip: Vector3<f32>| leg.draw(hip + from_hip) - hip;

        for stride in [
            Vector3::new(0.0, -STANDING_HEIGHT, 0.0),
            Vector3::new(0.0, -STANDING_HEIGHT, 0.3),
            Vector3::new(0.0, -0.38, -0.4),
            Vector3::new(0.0, -0.28, 0.1),
        ] {
            assert!((drawn(stride) - stride).norm() < 1e-6, "{stride:?} moved");
        }

        let level = drawn(Vector3::new(0.0, -0.1, -0.5));
        assert!(
            level.y <= -SWING_LEAST_DROP * leg.reach() + 1e-6,
            "{level:?}"
        );
        assert!(
            level.z.abs() <= SWING_CONE_TANGENT * -level.y + 1e-6,
            "{level:?}"
        );
        let over_hip = drawn(Vector3::new(0.0, 0.3, 0.1));
        assert!(over_hip.y <= -SWING_LEAST_DROP * leg.reach() + 1e-6);
        let far = drawn(Vector3::new(0.0, -2.0, 0.5));
        assert!(far.norm() <= leg.reach() + 1e-6);

        // 0.2 m to the body's left of the right hip is across the midline.
        let crossed = drawn(Vector3::new(0.2, -STANDING_HEIGHT, 0.0));
        assert!(
            crossed.x < HIP_WIDTH,
            "drawn {:.2} m past the midline",
            crossed.x - HIP_WIDTH
        );
        let free = FootPlacerConfig::default();
        let ctx = riding(pelvis, Vector3::zeros(), &free);
        let crossing = LegEnvelope::of(FootSide::Right, &ctx, Vector3::z());
        let across = Vector3::new(0.2, -STANDING_HEIGHT, 0.0);
        assert!((crossing.draw(hip + across) - (hip + across)).norm() < 1e-6);
    }

    /// A planted foot the hip has been carried out of reach of is let go of
    /// at once, standing or not, though the gait would hold it for the other
    /// foot's swing: it steps back under the body.
    #[test]
    fn a_foot_left_out_of_the_legs_reach_is_let_go_of() {
        let config = FootPlacerConfig::default();
        let dims = RigDims::player();
        let start = Point3::new(0.0, dims.standing_height, 0.0);
        let mut placer = FootPlacer::new(start, Vector3::z(), dims.hip_width, 0.0);
        // The right foot mid-swing: a walk's continuous support would keep
        // the left down until it lands.
        placer.right.phase = FootPhase::Stepping(swung(
            placer.right.planted_position,
            Point3::new(-0.12, 0.0, 0.1),
            Vector3::z(),
            Vector3::z(),
            0.4,
            0.15,
            0.0,
        ));
        // The body is somewhere else, standing still.
        let moved = start + Vector3::new(1.5, 0.0, 0.0);
        let mut ctx = flight_ctx(&config, moved, Vector3::zeros(), PlannedSteps::default());
        ctx.intent_direction = Vector3::zeros();
        ctx.left_ground = level_ground(&placer.left);
        ctx.right_ground = level_ground(&placer.right);
        placer.tick(&ctx);
        assert!(placer.left.is_stepping(), "the foot left behind is let go");

        for _ in 0..90 {
            ctx.left_ground = level_ground(&placer.left);
            ctx.right_ground = level_ground(&placer.right);
            placer.tick(&ctx);
        }
        for foot in [&placer.left, &placer.right] {
            let hip = hip_position(foot.side, &ctx, Vector3::z());
            assert!(foot.is_planted(), "{:?} foot came to rest", foot.side);
            assert!(
                planar_distance(foot.position, hip) < 0.15,
                "{:?} foot stands {:.2} m from under its hip",
                foot.side,
                planar_distance(foot.position, hip)
            );
        }
    }

    /// A context for the tests below: the player's legs, the pelvis at
    /// `pelvis` moving at `velocity`, with `planned` steps.
    fn flight_ctx<'a>(
        config: &'a FootPlacerConfig,
        pelvis: Point3<f32>,
        velocity: Vector3<f32>,
        planned: PlannedSteps,
    ) -> PlacerCtx<'a> {
        let dims = RigDims::player();
        let gait = GaitParams::walk();
        PlacerCtx {
            dt: 1.0 / 60.0,
            pelvis,
            velocity,
            support_velocity: Vector3::zeros(),
            intent_direction: Vector3::z(),
            yaw: 0.0,
            yaw_rate: 0.0,
            hip_width: dims.hip_width,
            leg_length: dims.leg_length,
            standing_height: dims.standing_height,
            foot_y_fallback: 0.0,
            step_height: gait.step_height,
            stride_gain: gait.stride_gain,
            left_ground_normal: Vector3::y(),
            right_ground_normal: Vector3::y(),
            left_ground: None,
            right_ground: None,
            planned,
            config,
        }
    }

    /// A swing whose planned step lies a hop away is given the hop's time
    /// from where the foot is: retiming it moves no foot.
    #[test]
    fn a_swing_retimed_for_a_hop_carries_on_from_where_the_foot_is() {
        const SPEED: f32 = 3.0;
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, RigDims::player().standing_height, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), 0.12, 0.0);
        let (from, to) = (Point3::new(-0.12, 0.0, -0.3), Point3::new(-0.12, 0.0, 0.5));
        placer.right.phase =
            FootPhase::Stepping(swung(from, to, Vector3::z(), Vector3::z(), 0.2, 0.05, 0.1));
        placer.right.position = drawn(&placer.right);
        let before = placer.right.position;
        // Half way through its swing, the foot's footfall becomes a hop's
        // landing 1.5 m on.
        let planned = PlannedSteps {
            left: None,
            right: Some(PlannedStep {
                id: None,
                at: Point3::new(-0.12, 0.0, 1.5),
                landing: Some(0.45),
            }),
        };
        let ctx = flight_ctx(&config, pelvis, Vector3::new(0.0, 0.0, SPEED), planned);
        let carried = carried_on(&placer.right, ctx.dt);
        let timing = level_timing(SPEED, &config);
        advance_stepping(
            &mut placer.right,
            pelvis,
            &ctx,
            &paced(&ctx, &timing, Vector3::z()),
            ctx.dt,
        );
        let FootPhase::Stepping(swing) = placer.right.phase else {
            panic!("the swing ended");
        };
        assert!(
            swing.remaining() > 0.3,
            "the swing lands in {:.2} s, before the body is over its step",
            swing.remaining()
        );
        let moved = planar_distance(placer.right.position, before);
        assert!(
            moved <= carried + 0.01,
            "the foot jumped {moved:.2} m, where its swing carries it {carried:.2} m"
        );
    }

    #[test]
    fn a_swing_onto_a_planned_footfall_aims_at_the_plans_height() {
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, RigDims::player().standing_height, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), 0.12, 0.0);
        let (from, to) = (Point3::new(-0.12, 0.0, -0.3), Point3::new(-0.12, 0.1, 0.5));
        placer.right.phase =
            FootPhase::Stepping(swung(from, to, Vector3::z(), Vector3::z(), 0.3, 0.05, 0.05));
        placer.right.position = drawn(&placer.right);
        let planned = PlannedSteps {
            left: None,
            right: Some(PlannedStep {
                id: None,
                at: Point3::new(-0.12, 0.0, 0.5),
                landing: None,
            }),
        };
        let mut ctx = flight_ctx(&config, pelvis, Vector3::new(0.0, 0.0, 1.0), planned);
        // The probe's floor, carried out along its plane, lies half a metre
        // above the footfall the plan sensed.
        ctx.right_ground = Some(Point3::new(-0.12, 0.5, 0.5));
        advance_stepping(
            &mut placer.right,
            pelvis,
            &ctx,
            &paced(&ctx, &level_timing(1.0, &config), Vector3::z()),
            ctx.dt,
        );
        let FootPhase::Stepping(Swing { to: aimed, .. }) = placer.right.phase else {
            panic!("the swing ended");
        };
        assert!(
            aimed.y < to.y,
            "the swing aims {:.2} m up, not at the plan's 0.0",
            aimed.y
        );
    }

    #[test]
    fn a_swing_leaves_its_takeoff_however_high_the_probe_finds_the_floor() {
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, RigDims::player().standing_height, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), 0.12, 0.0);
        let (from, to) = (Point3::new(-0.12, 0.0, -0.3), Point3::new(-0.12, 0.0, 0.5));
        placer.right.phase =
            FootPhase::Stepping(swung(from, to, Vector3::z(), Vector3::z(), 0.3, 0.05, 0.0));
        placer.right.position = from;
        // Lifting off a slope onto the shelf above it: the probe aims at the
        // shelf, half a metre over the slope the foot leaves.
        let mut ctx = flight_ctx(
            &config,
            pelvis,
            Vector3::new(0.0, 0.0, 1.0),
            PlannedSteps::default(),
        );
        ctx.right_ground = Some(Point3::new(-0.12, 0.5, 0.5));
        advance_stepping(
            &mut placer.right,
            pelvis,
            &ctx,
            &paced(&ctx, &level_timing(1.0, &config), Vector3::z()),
            ctx.dt,
        );
        let rise = placer.right.position.y - from.y;
        assert!(
            rise < 0.15,
            "the foot jumped {rise:.2} m up off its takeoff"
        );
    }

    /// Probes are cast against the physics world as well as the terrain,
    /// so a creature walking up to the player puts a capsule between its
    /// landing target and the ground. The probe comes back with that
    /// capsule's flank — a contact that is not floor — and for that frame
    /// the placer has no floor under the target.
    ///
    /// Reading a wall normal as "there is no ground here" cost the swing
    /// its target for the rest of its length: the step froze where it was
    /// while the body ran on, and by the end of the swing the foot was
    /// further away than the leg is long, so it could not plant at all.
    /// That is the foot seen dragging behind a peeper that has just
    /// cornered onto the player.
    #[test]
    fn a_probe_that_strikes_a_body_does_not_cost_the_swing_its_target() {
        let config = FootPlacerConfig::default();
        const FRAMES: usize = 240;
        // A peeper at a chase run. Long legs and a fast gait are what make
        // the frozen target unreachable: the body covers more ground in one
        // swing than a leg that has stopped aiming can follow.
        let dims = RigDims::peeper();
        let gait = GaitParams::peeper();
        const SPEED: f32 = 4.0;

        let (clean_takeoffs, clean_swings) =
            walk_flat(dims, gait, SPEED, &config, FRAMES, |_, _| false);
        // The frame each swing begins on is exactly when the target jumps
        // forward onto whatever the creature is chasing.
        let blips: Vec<(usize, FootSide)> = clean_takeoffs
            .iter()
            .map(|&(side, frame)| (frame, side))
            .collect();
        let (takeoffs, swings) = walk_flat(dims, gait, SPEED, &config, FRAMES, |f, side| {
            blips.contains(&(f, side))
        });

        let sides: Vec<FootSide> = takeoffs.iter().map(|&(side, _)| side).collect();
        for pair in sides.windows(2) {
            assert_ne!(
                pair[0], pair[1],
                "the same foot stepped twice: {sides:?} — the other one was still dragging"
            );
        }
        assert_eq!(
            takeoffs.len(),
            clean_takeoffs.len(),
            "a blinded probe turned {} steps into {}",
            clean_takeoffs.len(),
            takeoffs.len()
        );
        let longest = swings.iter().copied().max().unwrap_or(0);
        let clean_longest = clean_swings.iter().copied().max().unwrap_or(0);
        assert!(
            longest <= clean_longest,
            "a swing ran {longest} frames against {clean_longest} on the same walk \
             with nothing in the way"
        );
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

    /// A floor that leaves. The crate the character is standing on is knocked
    /// away, the probe goes on finding a surface — the crate on its way down,
    /// or the ground it used to sit on — and the planted foot chased it: one
    /// leg drawn at twice its length, reaching after the floor, which is the
    /// sticky foot the collapsing-box case complains about.
    #[test]
    fn a_planted_foot_does_not_follow_a_floor_out_of_reach() {
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let mut placer = FootPlacer::new(pelvis, Vector3::z(), HIP_WIDTH, 0.0);

        let mut floor = 0.0;
        for _ in 0..30 {
            floor -= 0.1;
            let mut ctx = riding(pelvis, Vector3::zeros(), &config);
            ctx.left_ground = Some(Point3::new(0.0, floor, 0.0));
            ctx.right_ground = Some(Point3::new(0.0, floor, 0.0));
            ctx.foot_y_fallback = floor;
            placer.tick(&ctx);
        }

        let ctx = riding(pelvis, Vector3::zeros(), &config);
        for foot in [&placer.left, &placer.right] {
            let hip = hip_position(foot.side, &ctx, Vector3::z());
            let reach = (foot.planted_position - hip).magnitude();
            assert!(
                reach <= max_leg_extension(&ctx) + 1e-3,
                "the foot chased the floor {:.0} mm down to {:.0}% of the leg",
                (0.0 - foot.planted_position.y) * 1000.0,
                reach / LEG_LENGTH * 100.0
            );
        }
    }

    /// The height a completing swing plants at is the ground the probe found,
    /// not the arc's half-finished chase toward it. Walking off a block the
    /// chase has a frame or two to cover most of a metre, and planting where
    /// it got to leaves a foot standing in mid-air.
    #[test]
    fn a_plant_takes_the_grounds_height_and_not_the_arcs() {
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let ctx = riding(pelvis, Vector3::zeros(), &config);
        let to = Point3::new(0.2, 0.18, HIP_WIDTH);
        let ground = Point3::new(0.2, 0.0, HIP_WIDTH);

        let plant = plantable(
            to,
            Some((ground, Vector3::y())),
            &ctx,
            FootSide::Right,
            Vector3::z(),
        )
        .expect("ground under the target and within reach");

        assert!(
            (plant.y - ground.y).abs() < 1e-6,
            "planted at {:.3} m, mid-arc, rather than on the floor at {:.3} m",
            plant.y,
            ground.y
        );
    }

    /// Climbing a V channel, the body is wedged in the crease and each foot
    /// stands on one of its walls: the Water Park's ramp climbs 43° with
    /// sides that rise about 50° across it, a face near 57° from level.
    /// Physics holds the body up there, so it is floor; were it a wall, the
    /// feet would find nowhere to land all the way up and never step.
    #[test]
    fn a_face_the_body_stands_on_in_a_crease_is_floor() {
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let mut ctx = riding(pelvis, Vector3::zeros(), &config);
        ctx.left_ground = Some(Point3::new(0.0, 0.0, HIP_WIDTH));
        ctx.left_ground_normal = Vector3::new(-1.2, 1.0, -0.93).normalize();
        ctx.right_ground = Some(Point3::new(0.0, 0.3, -HIP_WIDTH));
        ctx.right_ground_normal = Vector3::new(1.0, 0.0, 0.0);

        assert!(
            floor_sample(&ctx, FootSide::Left).is_some(),
            "a face {:.0}° from level, which the body stands on, was taken for a wall",
            ctx.left_ground_normal.y.acos().to_degrees()
        );
        assert!(
            floor_sample(&ctx, FootSide::Right).is_none(),
            "an upright face was taken for floor"
        );
    }

    /// And a leg cannot plant beyond its own bones. Walking off a block the
    /// probe finds the ground a metre down — real ground, entirely out of
    /// reach — and anchoring a foot there leaves a straight leg trailing
    /// behind a character who has already stepped off the edge.
    #[test]
    fn a_swing_refuses_a_landing_the_leg_cannot_reach() {
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let ctx = riding(pelvis, Vector3::zeros(), &config);
        let to = Point3::new(0.3, 0.0, HIP_WIDTH);
        let below = Point3::new(0.3, -1.0, HIP_WIDTH);

        assert!(
            plantable(
                to,
                Some((below, Vector3::y())),
                &ctx,
                FootSide::Right,
                Vector3::z(),
            )
            .is_none(),
            "a foot planted a metre below the hip is a leg the rig cannot draw"
        );
    }

    /// A swing with nowhere to land keeps swinging, and what it swings toward
    /// is the stance under the hip: a falling character's legs gather under
    /// him, and that is also the pose the airborne sampler is about to blend
    /// to, so nothing has to be unwound when it does.
    #[test]
    fn a_swing_with_nowhere_to_land_gathers_under_the_hip() {
        let config = FootPlacerConfig::default();
        let pelvis = Point3::new(0.0, STANDING_HEIGHT, 0.0);
        let ctx = riding(pelvis, Vector3::zeros(), &config);
        let facing = Vector3::z();
        let under_hip = foot_position_from_stance(
            pelvis,
            facing,
            HIP_WIDTH,
            ctx.foot_y_fallback,
            FootSide::Right,
        );
        let mut foot = PlacerFoot::new(FootSide::Right, Point3::new(-0.6, 0.0, HIP_WIDTH), facing);
        let far = Point3::new(0.6, 0.0, HIP_WIDTH);
        foot.phase = FootPhase::Stepping(swung(
            Point3::new(-0.6, 0.0, HIP_WIDTH),
            far,
            facing,
            facing,
            0.2,
            0.0,
            0.2,
        ));

        let before = (far - under_hip).magnitude();
        advance_stepping(
            &mut foot,
            far,
            &ctx,
            &paced(&ctx, &level_timing(1.0, &config), facing),
            ctx.dt,
        );

        assert!(!foot.is_planted(), "there was nowhere to plant");
        let after = (foot.position - under_hip).magnitude();
        assert!(
            after < before,
            "the foot must close on the stance under the hip, not hold its target"
        );
    }
}
