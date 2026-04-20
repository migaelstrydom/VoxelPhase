//! Tuning parameters for the foot placer.

/// Parameters that shape when a step fires, where it aims, and how long
/// it takes. Lives on `CharacterRigConfig`.
#[derive(Clone, Copy, Debug)]
pub struct FootPlacerConfig {
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
    /// Floor on a swing's duration. Prevents teleport-stepping at high
    /// speeds.
    pub min_step_duration: f32,
    /// Ceiling on a swing's duration. Prevents feet remaining lifted if
    /// speed drops mid-step.
    pub max_step_duration: f32,
    /// Clamp on `ideal_xz` offset from hip as a fraction of leg length.
    /// Pathology guard against terminal-velocity falls etc.
    pub max_stride_reach_ratio: f32,
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
}

impl Default for FootPlacerConfig {
    fn default() -> Self {
        Self {
            k_yaw: 0.01,
            k_turn_trigger: 0.5,
            min_step_duration: 0.08,
            max_step_duration: 0.1,
            max_stride_reach_ratio: 1.5,
            settle_trigger: 0.05,
            ankle_slerp_rate: 18.0,
            swing_takeoff_fraction: 0.15,
            swing_landing_fraction: 0.15,
        }
    }
}
