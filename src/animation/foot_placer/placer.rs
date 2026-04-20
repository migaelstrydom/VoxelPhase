//! Per-foot `Planted`/`Stepping` FSM driven by planted error against
//! the capture point.

use nalgebra::{Point3, Vector2, Vector3};

use super::capture_point::{capture_point_xz, stance_offset, turn_in_place_offset, GRAVITY};
use super::config::FootPlacerConfig;
use super::swing::swing_position;

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
    /// Up-axis captured at the moment of step-off. Used as the source
    /// pose during the early takeoff-ease portion of the swing — the
    /// foot relaxes from the previous ground contour toward neutral
    /// rather than snapping flat on liftoff.
    pub takeoff_up: Vector3<f32>,
}

impl PlacerFoot {
    fn new(side: FootSide, position: Point3<f32>) -> Self {
        Self {
            side,
            phase: FootPhase::Planted,
            position,
            planted_position: position,
            planted_yaw: 0.0,
            ideal_xz: position,
            up: Vector3::y(),
            takeoff_up: Vector3::y(),
        }
    }

    /// Whether this foot is currently planted.
    #[inline]
    pub fn is_planted(&self) -> bool {
        matches!(self.phase, FootPhase::Planted)
    }
}

/// Inputs to `FootPlacer::tick` — gathered once per frame.
pub struct PlacerCtx<'a> {
    pub dt: f32,
    pub pelvis: Point3<f32>,
    pub velocity: Vector3<f32>,
    pub facing: Vector3<f32>,
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
    /// active `GaitPreset`.
    pub stride_gain: f32,
    /// Current ground normal under each foot, defaulting to world Y when
    /// no probe has reported a contact. Used as the target up-axis while
    /// the foot is planted and during the landing-ease portion of swing.
    pub left_ground_normal: Vector3<f32>,
    pub right_ground_normal: Vector3<f32>,
    /// Per-foot terrain-surface y from the latest probe. When `Some`, a
    /// planted foot tracks the terrain directly beneath it. Falls back to
    /// `foot_y_fallback` when no probe hit is available (e.g. stepping
    /// off a ledge).
    pub left_ground_y: Option<f32>,
    pub right_ground_y: Option<f32>,
    pub config: &'a FootPlacerConfig,
}

/// The procedural foot placer. Owns per-foot phase state and advances
/// it each frame based on capture-point stepping.
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
    /// Side that most recently planted (or stepped off, if mid-step).
    /// Trigger evaluation prefers the OTHER side first to enforce
    /// alternation at high speeds — otherwise the side evaluated first
    /// re-fires every frame when its post-plant error still exceeds the
    /// trigger, starving the other foot.
    last_planted_side: FootSide,
}

impl FootPlacer {
    pub fn new(pelvis: Point3<f32>, facing: Vector3<f32>, hip_width: f32, foot_y: f32) -> Self {
        let left = foot_position_from_stance(pelvis, facing, hip_width, foot_y, FootSide::Left);
        let right = foot_position_from_stance(pelvis, facing, hip_width, foot_y, FootSide::Right);
        Self {
            left: PlacerFoot::new(FootSide::Left, left),
            right: PlacerFoot::new(FootSide::Right, right),
            suspended: false,
            resuming: false,
            last_planted_side: FootSide::Right,
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

    /// Advance the placer by one frame.
    pub fn tick(&mut self, ctx: &PlacerCtx<'_>) {
        if self.suspended {
            return;
        }

        if self.resuming {
            self.replant_at_stance(ctx);
            self.resuming = false;
        }

        let horizontal_velocity = Vector3::new(ctx.velocity.x, 0.0, ctx.velocity.z);
        let cp = capture_point_xz(ctx.pelvis, horizontal_velocity, ctx.standing_height);
        let max_reach = ctx.leg_length * ctx.config.max_stride_reach_ratio;

        let speed = horizontal_velocity.magnitude();

        // Symmetric-plant trigger from LIP: the foot enters at `+s`
        // ahead of the hip and, by energy conservation, reaches `-s`
        // behind the hip at the mirrored pendulum phase. Trigger when
        // the foot has drifted `2s` past its plant. `s = gain·v/ω`.
        //
        // `settle_trigger` is an always-on floor. At normal walking
        // speed the symmetric term dominates and the floor is invisible;
        // as the body decelerates it takes over, preventing the trigger
        // from collapsing to zero and firing a cascade of micro-steps
        // against decelerating-pelvis drift. At rest it becomes the
        // sole trigger, pulling any off-centre foot to neutral stance.
        let h = ctx.standing_height.max(0.01);
        let omega_inv = (h / GRAVITY).sqrt();
        let stride_scalar = ctx.stride_gain * speed * omega_inv;
        let symmetric_trigger = 2.0 * stride_scalar;
        let trigger_threshold = symmetric_trigger.max(ctx.config.settle_trigger);

        // Compute ideal targets for both feet first — we need both to
        // evaluate step triggers with the "other foot must be planted"
        // invariant.
        let left_ideal = ideal_target(
            &self.left,
            cp,
            ctx.pelvis,
            ctx.facing,
            ctx.yaw_rate,
            ctx.hip_width,
            ctx.foot_y_fallback,
            max_reach,
            ctx.stride_gain,
            ctx.config,
        );
        let right_ideal = ideal_target(
            &self.right,
            cp,
            ctx.pelvis,
            ctx.facing,
            ctx.yaw_rate,
            ctx.hip_width,
            ctx.foot_y_fallback,
            max_reach,
            ctx.stride_gain,
            ctx.config,
        );
        self.left.ideal_xz = left_ideal;
        self.right.ideal_xz = right_ideal;

        // Evaluate triggers BEFORE advancing. Prefer the side that did
        // NOT most recently plant so that at high speed both feet
        // alternate instead of the same foot re-firing every frame.
        match self.last_planted_side {
            FootSide::Left => {
                let other_planted = self.left.is_planted();
                try_trigger_step(
                    &mut self.right,
                    right_ideal,
                    ctx,
                    other_planted,
                    trigger_threshold,
                );
                let other_planted = self.right.is_planted();
                try_trigger_step(
                    &mut self.left,
                    left_ideal,
                    ctx,
                    other_planted,
                    trigger_threshold,
                );
            }
            FootSide::Right => {
                let other_planted = self.right.is_planted();
                try_trigger_step(
                    &mut self.left,
                    left_ideal,
                    ctx,
                    other_planted,
                    trigger_threshold,
                );
                let other_planted = self.left.is_planted();
                try_trigger_step(
                    &mut self.right,
                    right_ideal,
                    ctx,
                    other_planted,
                    trigger_threshold,
                );
            }
        }

        // Advance any currently stepping foot. A foot that completes its
        // step this frame becomes `last_planted_side`.
        if advance_stepping(&mut self.left, ctx.dt) {
            self.last_planted_side = FootSide::Left;
        }
        if advance_stepping(&mut self.right, ctx.dt) {
            self.last_planted_side = FootSide::Right;
        }

        // Planted ankle y tracks the current pelvis each frame rather
        // than being frozen at plant time. Without this, a plant that
        // landed mid-recoil (e.g. on the airborne→grounded edge, with
        // physics still settling) leaves foot y stuck above or below
        // the final terrain surface.
        sync_planted_y(
            &mut self.left,
            ctx.left_ground_y.unwrap_or(ctx.foot_y_fallback),
        );
        sync_planted_y(
            &mut self.right,
            ctx.right_ground_y.unwrap_or(ctx.foot_y_fallback),
        );

        // Ease each foot's up-axis toward its phase-dependent target.
        // Planted feet pull toward the current ground normal; swinging
        // feet ease off the takeoff pose, pass through neutral in the
        // middle, and ease into the landing-probe normal near plant.
        update_foot_orientation(&mut self.left, ctx.left_ground_normal, ctx);
        update_foot_orientation(&mut self.right, ctx.right_ground_normal, ctx);
    }

    /// Snap both feet to neutral stance under the current pelvis. Used
    /// on the resume-from-suspend edge (airborne→grounded). Leaves both
    /// feet Planted and marks the current yaw as their plant reference.
    fn replant_at_stance(&mut self, ctx: &PlacerCtx<'_>) {
        let left = foot_position_from_stance(
            ctx.pelvis,
            ctx.facing,
            ctx.hip_width,
            ctx.foot_y_fallback,
            FootSide::Left,
        );
        let right = foot_position_from_stance(
            ctx.pelvis,
            ctx.facing,
            ctx.hip_width,
            ctx.foot_y_fallback,
            FootSide::Right,
        );
        self.left.phase = FootPhase::Planted;
        self.left.position = left;
        self.left.planted_position = left;
        self.left.planted_yaw = ctx.yaw;
        self.right.phase = FootPhase::Planted;
        self.right.position = right;
        self.right.planted_position = right;
        self.right.planted_yaw = ctx.yaw;
        self.left.up = Vector3::y();
        self.left.takeoff_up = Vector3::y();
        self.right.up = Vector3::y();
        self.right.takeoff_up = Vector3::y();
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
    pelvis: Point3<f32>,
    facing: Vector3<f32>,
    yaw_rate: f32,
    hip_width: f32,
    foot_y: f32,
    max_reach: f32,
    stride_gain: f32,
    cfg: &FootPlacerConfig,
) -> Point3<f32> {
    let side = foot.side.side_sign();
    let turn = turn_in_place_offset(facing, yaw_rate, hip_width, cfg.k_yaw, side);
    let stance = stance_offset(facing, hip_width, side);

    // `stride` is the velocity/turn-driven displacement from the hip,
    // clamped to `max_reach`. `stance` sits the foot to its own side of
    // the pelvis and is preserved regardless of clamp — otherwise the
    // clamp shrinks lateral foot spacing at speed. `stride_gain` < 1
    // plants short of the full capture point so the body passes over
    // the foot instead of arresting over it (continuous walking).
    let pelvis_xz = Vector2::new(pelvis.x, pelvis.z);
    let cp_offset = Vector2::new(
        (capture_point.x - pelvis.x) * stride_gain,
        (capture_point.z - pelvis.z) * stride_gain,
    );
    let mut stride = cp_offset + turn;
    let stride_sq = stride.x * stride.x + stride.y * stride.y;
    if stride_sq > max_reach * max_reach {
        stride *= max_reach / stride_sq.sqrt();
    }

    let target = pelvis_xz + stride + stance;
    Point3::new(target.x, foot_y, target.y)
}

/// Returns true if the foot transitioned Stepping→Planted this call.
fn advance_stepping(foot: &mut PlacerFoot, dt: f32) -> bool {
    if let FootPhase::Stepping {
        from,
        to,
        t,
        duration,
        peak_lift,
    } = foot.phase
    {
        let new_t = t + dt;
        if new_t >= duration {
            foot.phase = FootPhase::Planted;
            foot.position = to;
            foot.planted_position = to;
            true
        } else {
            foot.phase = FootPhase::Stepping {
                from,
                to,
                t: new_t,
                duration,
                peak_lift,
            };
            foot.position = swing_position(from, to, new_t / duration, peak_lift);
            false
        }
    } else {
        // Planted: keep rendered position locked to planted_position so
        // per-frame pelvis motion doesn't drift the foot.
        foot.position = foot.planted_position;
        false
    }
}

/// Pull a planted foot's y to the current pelvis-relative ankle y. No-op
/// while the foot is mid-step — the swing arc owns y during that window.
fn sync_planted_y(foot: &mut PlacerFoot, foot_y: f32) {
    if foot.is_planted() {
        foot.position.y = foot_y;
        foot.planted_position.y = foot_y;
    }
}

fn try_trigger_step(
    foot: &mut PlacerFoot,
    ideal: Point3<f32>,
    ctx: &PlacerCtx<'_>,
    other_planted: bool,
    trigger_threshold: f32,
) {
    if !foot.is_planted() || !other_planted {
        return;
    }

    let dx = ideal.x - foot.planted_position.x;
    let dz = ideal.z - foot.planted_position.z;
    let planted_error = (dx * dx + dz * dz).sqrt();

    let yaw_error = angle_diff(ctx.yaw, foot.planted_yaw).abs();
    let yaw_equiv = yaw_error * ctx.hip_width * ctx.config.k_turn_trigger;

    let trigger = planted_error.max(yaw_equiv);
    if trigger < trigger_threshold {
        return;
    }

    // Swing duration is the trigger distance divided by body speed —
    // i.e. about as long as it takes the hip to cover one trigger's
    // worth of ground. Clamped to keep feet neither teleporting at
    // sprint nor hovering during slow decelerations.
    let speed = (ctx.velocity.x * ctx.velocity.x + ctx.velocity.z * ctx.velocity.z).sqrt();
    let nominal = trigger_threshold / speed.max(0.01);
    let duration = nominal.clamp(ctx.config.min_step_duration, ctx.config.max_step_duration);

    // Forward-project the target by how far the hip will actually
    // travel during the swing. `advance_stepping` completes on the
    // tick where cumulative `t >= duration`, which takes
    // `ceil(duration/dt)` ticks. The trigger itself consumes one tick,
    // so the hip receives `(ceil(duration/dt) - 1)` position updates
    // between trigger and plant — not `duration/dt` updates. Using
    // `v·duration` naively overshoots by ~`v·dt` and leaves the cycle
    // tilted forward.
    let swing_ticks = if ctx.dt > 0.0 {
        (duration / ctx.dt).ceil().max(1.0)
    } else {
        1.0
    };
    let travel = (swing_ticks - 1.0) * ctx.dt;
    let to = Point3::new(
        ideal.x + ctx.velocity.x * travel,
        ideal.y,
        ideal.z + ctx.velocity.z * travel,
    );

    foot.takeoff_up = foot.up;
    foot.phase = FootPhase::Stepping {
        from: foot.planted_position,
        to,
        t: 0.0,
        duration,
        peak_lift: ctx.step_height,
    };
    foot.planted_yaw = ctx.yaw;
}

/// Step the foot's rendered up-axis toward a phase-dependent target.
/// The target is:
///
/// - Planted  → `ground_normal` (fallback world Y if no probe hit).
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
        FootPhase::Planted => ground,
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
fn angle_diff(b: f32, a: f32) -> f32 {
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
mod trace {
    use super::*;

    fn phase_label(p: &FootPhase) -> &'static str {
        match p {
            FootPhase::Planted => "PLANTED",
            FootPhase::Stepping { .. } => "STEP   ",
        }
    }

    fn run(label: &str, frames: usize, step_input: impl Fn(usize) -> (Vector3<f32>, f32)) {
        let cfg = FootPlacerConfig::default();
        let hip_width = 0.12;
        let leg_length = 0.5;
        let standing_height = 0.425;
        let foot_y = 0.0;

        let mut pelvis = Point3::new(0.0, standing_height, 0.0);
        let init_facing = Vector3::new(0.0, 0.0, 1.0);
        let mut placer = FootPlacer::new(pelvis, init_facing, hip_width, foot_y);
        let dt = 1.0 / 60.0;
        let mut last_yaw = 0.0f32;

        println!("\n=== {label} ===");
        println!(
            "frame  t     pelvisXZ          velXZ           yaw    L[phase pos.xz plantedXZ idealXZ err]                     R[...]"
        );

        for f in 0..frames {
            let (velocity, yaw) = step_input(f);
            let yaw_rate = (yaw - last_yaw) / dt;
            last_yaw = yaw;

            let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());
            pelvis.x += velocity.x * dt;
            pelvis.z += velocity.z * dt;

            let ctx = PlacerCtx {
                dt,
                pelvis,
                velocity,
                facing,
                yaw,
                yaw_rate,
                hip_width,
                leg_length,
                standing_height,
                foot_y_fallback: foot_y,
                step_height: 0.15,
                stride_gain: 0.5,
                left_ground_normal: Vector3::y(),
                right_ground_normal: Vector3::y(),
                left_ground_y: None,
                right_ground_y: None,
                config: &cfg,
            };
            placer.tick(&ctx);

            let l = &placer.left;
            let r = &placer.right;
            let l_err = ((l.ideal_xz.x - l.planted_position.x).powi(2)
                + (l.ideal_xz.z - l.planted_position.z).powi(2))
            .sqrt();
            let r_err = ((r.ideal_xz.x - r.planted_position.x).powi(2)
                + (r.ideal_xz.z - r.planted_position.z).powi(2))
            .sqrt();

            println!(
                "{f:>4}  {:>5.2}  ({:>5.2},{:>5.2})  ({:>4.2},{:>4.2})  {:>5.2}  L[{} ({:>5.2},{:>5.2}) p=({:>5.2},{:>5.2}) i=({:>5.2},{:>5.2}) e={:>4.2}]  R[{} ({:>5.2},{:>5.2}) p=({:>5.2},{:>5.2}) i=({:>5.2},{:>5.2}) e={:>4.2}]",
                f as f32 * dt,
                pelvis.x, pelvis.z,
                velocity.x, velocity.z,
                yaw,
                phase_label(&l.phase), l.position.x, l.position.z, l.planted_position.x, l.planted_position.z, l.ideal_xz.x, l.ideal_xz.z, l_err,
                phase_label(&r.phase), r.position.x, r.position.z, r.planted_position.x, r.planted_position.z, r.ideal_xz.x, r.ideal_xz.z, r_err,
            );
        }
    }

    #[test]
    #[ignore]
    fn trace_standing_still() {
        run("standing still (120 frames = 2s)", 120, |_| {
            (Vector3::zeros(), 0.0)
        });
    }

    #[test]
    #[ignore]
    fn trace_walk_forward() {
        // Stand still for 30 frames, then walk at 1 m/s +z for 150 frames.
        run("walk forward at 1 m/s (3s)", 180, |f| {
            let v = if f < 30 {
                Vector3::zeros()
            } else {
                Vector3::new(0.0, 0.0, 1.0)
            };
            (v, 0.0)
        });
    }

    #[test]
    #[ignore]
    fn trace_sprint() {
        run("sprint at 3 m/s (2s)", 120, |f| {
            let v = if f < 20 {
                Vector3::zeros()
            } else {
                Vector3::new(0.0, 0.0, 3.0)
            };
            (v, 0.0)
        });
    }

    #[test]
    #[ignore]
    fn trace_landing_slide() {
        // Start sliding at 4 m/s, decelerate to 0 over 1s.
        run("landing slide 4→0 m/s (1.5s)", 90, |f| {
            let t = f as f32 / 60.0;
            let v = if t < 1.0 {
                Vector3::new(0.0, 0.0, 4.0 * (1.0 - t))
            } else {
                Vector3::zeros()
            };
            (v, 0.0)
        });
    }

    #[test]
    #[ignore]
    fn trace_walk_to_idle() {
        // Walk for 1s, then stop abruptly.
        run("walk-to-idle transition (2s)", 120, |f| {
            let v = if f < 60 {
                Vector3::new(0.0, 0.0, 1.0)
            } else {
                Vector3::zeros()
            };
            (v, 0.0)
        });
    }
}
