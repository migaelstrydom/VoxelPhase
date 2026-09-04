//! Tuning parameters for the foot placer.

/// Parameters that shape when a step fires, where it aims, and how long
/// it takes. Lives on `CharacterRigConfig`.
#[derive(Clone, Copy, Debug)]
pub struct FootPlacerConfig {
    /// Upper bound on the placer's internal simulation step. Each render
    /// frame is split into equal substeps no longer than this, so the
    /// step state machine resolves plant/lift sequencing identically at
    /// 30 Hz and 240 Hz. The cost per substep is trivial (vector math).
    pub max_substep_dt: f32,
    /// Speed floor applied to the gait clock while movement intent is
    /// held. Lets the first step fire before physics velocity has caught
    /// up with input, without distorting steady-state cadence.
    pub intent_speed_floor: f32,
    /// Horizontal speed above which the placer considers the body
    /// "moving" even without input intent (landing slides, pushes).
    /// Below this, idle settling rules apply. Filters out physics
    /// jitter that would otherwise creep the gait clock forward.
    pub moving_speed_threshold: f32,
    /// Sideways lunge applied to `ideal_xz` while actively turning,
    /// proportional to instantaneous `yaw_rate`. This is a *landing*
    /// offset: when a step fires during a spin, the foot lands further
    /// out to the side than its neutral stance. High values look like
    /// an exaggerated sidestep; low values keep turn-steps close to the
    /// natural stance position. Units: dimensionless multiplier on
    /// `yaw_rate · hip_width`. Does not by itself trigger steps.
    pub k_yaw: f32,
    /// How aggressively accumulated yaw-since-plant contributes to the
    /// step *trigger*. Higher = steps fire earlier into a turn (feet
    /// reshuffle sooner, less stretched stance). Purely a trigger term;
    /// does not change where the foot lands. Units: metres of
    /// trigger-equivalent per (radian · hip_width).
    pub k_turn_trigger: f32,
    /// Accumulated yaw since plant that can trigger a turn step regardless
    /// of run-speed stride length. Keeps fast 180-degree turns from
    /// waiting for the translational stride threshold before releasing a
    /// foot.
    pub turn_step_trigger_angle: f32,
    /// Floor on a swing's duration. Prevents teleport-stepping at high
    /// speeds; should cover at least a few display frames at 30 Hz.
    pub min_step_duration: f32,
    /// Ceiling on a swing's duration. Also the duration of idle settle
    /// steps, where the cycle-derived swing time diverges.
    pub max_step_duration: f32,
    /// Maximum hip→foot distance as a fraction of leg length. Values
    /// slightly above 1.0 allow for heel/toe extension. Drives two
    /// guards: the horizontal stride budget for ideal targets (via
    /// Pythagoras with `standing_height`), and the overstretch release
    /// that forces a planted foot to step when the hip has slid too far
    /// from it (landing slides, uncommanded pushes).
    pub max_leg_stretch_ratio: f32,
    /// Minimum time between any two takeoffs while moving, as a
    /// fraction of the current swing duration. Without this, two feet
    /// that hit their release conditions together (e.g. both planted at
    /// the same spot after a landing replant) take off in phase and
    /// the gait degenerates into a stable two-footed hop. Staggered
    /// takeoffs let the scheduled windows pull the feet back to
    /// antiphase within a cycle.
    pub takeoff_stagger_fraction: f32,
    /// Always-on trigger floor. The speed-proportional symmetric trigger
    /// scales with `v`, so at rest it collapses to zero and any
    /// pelvis-drift would fire a cascade of micro-steps. This floor
    /// keeps the placer stable at low speeds and serves as the sole
    /// trigger at rest, pulling off-centre feet into neutral stance.
    pub settle_trigger: f32,
    /// Exponential rate (per second) at which the foot's up-axis chases
    /// its target. Larger = snappier ground-hugging; smaller = lazier.
    /// Used as `1 - exp(-rate * dt)` so behaviour is dt-independent.
    pub ankle_slerp_rate: f32,
    /// Fraction of swing spent easing OFF the takeoff ground orientation.
    /// Target blends takeoff_up → neutral Y across `[0, fraction)`.
    pub swing_takeoff_fraction: f32,
    /// Fraction of swing (from the end) spent easing INTO the landing
    /// ground orientation. Target blends neutral Y → landing normal
    /// across `[1 - fraction, 1]`.
    pub swing_landing_fraction: f32,
    /// Fraction of the step trigger at which a planted foot starts
    /// visually rolling onto its toe. `1.0` means no pre-lift before the
    /// actual step fires; lower values give more anticipation.
    pub prelift_trigger_ratio: f32,
    /// Maximum toe-off pitch in radians while pre-lifting. This only
    /// changes the rendered foot up-axis; the planted anchor remains
    /// fixed until the step commits.
    pub prelift_max_pitch: f32,
    /// Fraction of swing progress after which the landing target stops
    /// retargeting. Late swing needs a stable contact point so the foot
    /// does not skate just before plant. Applies to the horizontal
    /// target only — the vertical landing height chases the probed
    /// terrain for the entire swing, since a vertical correction cannot
    /// skate but a stale height pops at plant.
    pub swing_retarget_until_fraction: f32,
    /// Minimum height the mid-swing foot keeps above the terrain sample
    /// under it, faded to zero at both swing endpoints. This is the
    /// cheap stand-in for swing-arc obstacle avoidance: a riser or bump
    /// between the endpoints pushes the foot up over it instead of
    /// letting the arc clip through.
    pub swing_obstacle_clearance: f32,
    /// Minimum stance age before a planted foot may be released again,
    /// as a fraction of the current swing duration. Scheduled and turn
    /// releases wait for it; overstretch ignores it (delaying that
    /// would stretch the leg without bound). Prevents one-frame stances
    /// when a turn trigger re-arms the moment a foot lands mid-turn.
    pub min_stance_fraction: f32,
    /// Stretch past `max_leg_stretch_ratio · leg_length` (in metres) at
    /// which an opening leg may step even inside the takeoff stagger
    /// window. The stagger normally delays overstretch releases to keep
    /// takeoffs visibly distinct, but at sprint speeds a full stagger
    /// wait can add tens of centimetres of stretch — past this margin
    /// the leg fires immediately and a briefly-tight takeoff pair is
    /// the lesser evil.
    pub overstretch_hard_margin: f32,
    /// Exponential rate (per second) at which an in-flight foot target
    /// chases the latest stance target before the retarget cutoff.
    pub swing_retarget_rate: f32,
    /// Exponential rate (per second) at which a landing target with no
    /// ground under it retreats toward the foot's takeoff position.
    /// Used as `1 - exp(-rate * dt)`, so it is dt-independent. It matches
    /// `swing_retarget_rate`: an illegal target must be corrected at
    /// least as fast as a stale one, and even the shortest swing has to
    /// arrive close to the takeoff position before the plant refusal
    /// snaps it the rest of the way.
    pub unsupported_retreat_rate: f32,
    /// Duty factor at very low speed. Values above `0.5` mean the stance
    /// windows overlap, so at least one foot can remain planted.
    pub slow_duty_factor: f32,
    /// Duty factor at high speed. Values below `0.5` introduce a flight
    /// window between stance phases.
    pub fast_duty_factor: f32,
    /// Froude number where duty factor starts moving from slow to fast.
    pub duty_froude_start: f32,
    /// Froude number where duty factor reaches `fast_duty_factor`.
    pub duty_froude_end: f32,
}

impl Default for FootPlacerConfig {
    fn default() -> Self {
        Self {
            max_substep_dt: 1.0 / 240.0,
            intent_speed_floor: 0.5,
            moving_speed_threshold: 0.15,
            k_yaw: 0.01,
            k_turn_trigger: 0.5,
            turn_step_trigger_angle: 0.65,
            min_step_duration: 0.12,
            max_step_duration: 0.4,
            max_leg_stretch_ratio: 1.15,
            takeoff_stagger_fraction: 0.4,
            settle_trigger: 0.05,
            ankle_slerp_rate: 18.0,
            swing_takeoff_fraction: 0.15,
            swing_landing_fraction: 0.15,
            prelift_trigger_ratio: 0.7,
            prelift_max_pitch: 0.55,
            swing_retarget_until_fraction: 0.9,
            swing_obstacle_clearance: 0.03,
            min_stance_fraction: 0.6,
            overstretch_hard_margin: 0.03,
            swing_retarget_rate: 30.0,
            unsupported_retreat_rate: 30.0,
            slow_duty_factor: 0.62,
            fast_duty_factor: 0.38,
            duty_froude_start: 0.25,
            duty_froude_end: 0.75,
        }
    }
}
