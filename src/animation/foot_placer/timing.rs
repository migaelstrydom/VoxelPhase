//! Derived gait timing.
//!
//! Everything cadence-related — trigger distance, cycle distance, duty
//! factor, swing duration — is derived here from a single speed, so the
//! phase clock, step trigger, and swing length cannot drift apart. The
//! core identity is the LIP symmetric plant:
//!
//! ```text
//! s        = stride_gain · v · √(h/g)     (plant-ahead distance)
//! step     = max(2·s, settle_trigger)    (hip travel per step)
//! cycle    = 2 · step                    (two steps per gait cycle)
//! ```
//!
//! Swing duration is the swing share of the cycle, `(1 − duty) ·
//! cycle_time`, clamped — so a swing always occupies its phase window
//! instead of snapping through it.

use super::capture_point::GRAVITY;
use super::config::FootPlacerConfig;

/// Fraction of the horizontal reach budget the scheduled stance may
/// use. The plant-ahead must satisfy `2·duty·s ≤ safety·reach`, or the
/// leg hits its stretch limit *before* the scheduled window exit and the
/// overstretch release — not the clock — paces the gait (seen in game as
/// a persistent antiphase wobble at top speed). Short legs at high speed
/// therefore take faster, shorter steps, which is also what real
/// biomechanics does. Below 1.0 so the stretch release keeps headroom to
/// act as an emergency valve only.
const STRIDE_REACH_SAFETY: f32 = 0.9;

/// All cadence quantities for one substep, derived from the gait speed.
#[derive(Clone, Copy, Debug)]
pub struct GaitTiming {
    /// Planted error at which a step fires; also the hip travel per step.
    pub trigger_threshold: f32,
    /// Hip travel per full gait cycle (two steps).
    pub cycle_distance: f32,
    /// Fraction of the cycle each foot spends planted.
    pub duty_factor: f32,
    /// Seconds a swing takes, clamped to the configured range.
    pub swing_duration: f32,
    /// Whether at least one foot must stay planted (duty ≥ 0.5). Below
    /// 0.5 the gait has a flight phase and both feet may swing.
    pub continuous_support: bool,
}

impl GaitTiming {
    /// Derive timing from the gait speed (the speed driving the phase
    /// clock — actual horizontal speed, floored by intent).
    /// `reach_budget` is the horizontal distance a foot may sit from its
    /// hip at full leg extension; it caps the stride (see
    /// `STRIDE_REACH_SAFETY`).
    pub fn derive(
        gait_speed: f32,
        standing_height: f32,
        leg_length: f32,
        stride_gain: f32,
        reach_budget: f32,
        cfg: &FootPlacerConfig,
    ) -> Self {
        let h = standing_height.max(0.01);
        let omega_inv = (h / GRAVITY).sqrt();

        let duty_factor = duty_factor_for_speed(gait_speed, leg_length, cfg);

        let max_stride = STRIDE_REACH_SAFETY * reach_budget / (2.0 * duty_factor.max(0.05));
        let stride_scalar = (stride_gain * gait_speed * omega_inv).min(max_stride);
        let trigger_threshold = (2.0 * stride_scalar).max(cfg.settle_trigger);
        let cycle_distance = 2.0 * trigger_threshold;

        let cycle_time = cycle_distance / gait_speed.max(1e-3);
        let swing_duration =
            ((1.0 - duty_factor) * cycle_time).clamp(cfg.min_step_duration, cfg.max_step_duration);

        Self {
            trigger_threshold,
            cycle_distance,
            duty_factor,
            swing_duration,
            continuous_support: duty_factor >= 0.5,
        }
    }
}

/// Smoothstep from `slow_duty_factor` to `fast_duty_factor` across the
/// configured Froude-number band. Froude = v²/(g·leg) is the standard
/// dimensionless walk/run discriminator.
fn duty_factor_for_speed(speed: f32, leg_length: f32, cfg: &FootPlacerConfig) -> f32 {
    let froude = speed * speed / (GRAVITY * leg_length.max(0.01));
    let start = cfg.duty_froude_start;
    let end = cfg.duty_froude_end.max(start + 1e-4);
    let t = ((froude - start) / (end - start)).clamp(0.0, 1.0);
    let t = t * t * (3.0 - 2.0 * t);
    cfg.slow_duty_factor * (1.0 - t) + cfg.fast_duty_factor * t
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> FootPlacerConfig {
        FootPlacerConfig::default()
    }

    #[test]
    fn symmetric_trigger_dominates_at_speed() {
        let cfg = cfg();
        let t = GaitTiming::derive(3.0, 0.425, 0.5, 0.4, 10.0, &cfg);
        let omega_inv = (0.425f32 / GRAVITY).sqrt();
        let expected = 2.0 * 0.4 * 3.0 * omega_inv;
        assert!((t.trigger_threshold - expected).abs() < 1e-5);
        assert!((t.cycle_distance - 2.0 * expected).abs() < 1e-5);
    }

    #[test]
    fn settle_floor_dominates_at_rest() {
        let cfg = cfg();
        let t = GaitTiming::derive(0.0, 0.425, 0.5, 0.4, 10.0, &cfg);
        assert_eq!(t.trigger_threshold, cfg.settle_trigger);
    }

    #[test]
    fn slow_speed_uses_slow_duty_factor() {
        let cfg = cfg();
        let t = GaitTiming::derive(0.2, 0.425, 0.5, 0.4, 10.0, &cfg);
        assert!((t.duty_factor - cfg.slow_duty_factor).abs() < 1e-5);
        assert!(t.continuous_support);
    }

    #[test]
    fn fast_speed_uses_fast_duty_factor() {
        let cfg = cfg();
        let t = GaitTiming::derive(5.0, 0.425, 0.5, 0.4, 10.0, &cfg);
        assert!((t.duty_factor - cfg.fast_duty_factor).abs() < 1e-5);
        assert!(!t.continuous_support);
    }

    #[test]
    fn swing_duration_clamped_to_max_at_rest() {
        let cfg = cfg();
        let t = GaitTiming::derive(0.0, 0.425, 0.5, 0.4, 10.0, &cfg);
        assert_eq!(t.swing_duration, cfg.max_step_duration);
    }

    #[test]
    fn swing_duration_clamped_to_min_at_extreme_speed() {
        let cfg = cfg();
        let t = GaitTiming::derive(100.0, 0.425, 0.5, 0.01, 10.0, &cfg);
        assert_eq!(t.swing_duration, cfg.min_step_duration);
    }

    #[test]
    fn stride_capped_by_reach_budget_raises_cadence() {
        let cfg = cfg();
        let reach = 0.29;
        let capped = GaitTiming::derive(5.0, 0.425, 0.5, 0.4, reach, &cfg);
        let free = GaitTiming::derive(5.0, 0.425, 0.5, 0.4, 10.0, &cfg);
        assert!(capped.trigger_threshold < free.trigger_threshold);
        // Scheduled plant-ahead (duty · trigger) must fit the budget
        // with the safety headroom.
        let plant_ahead = capped.duty_factor * capped.trigger_threshold;
        assert!(plant_ahead <= STRIDE_REACH_SAFETY * reach + 1e-5);
    }

    #[test]
    fn swing_duration_is_swing_share_of_cycle_in_band() {
        let cfg = cfg();
        let t = GaitTiming::derive(2.0, 0.425, 0.5, 0.4, 10.0, &cfg);
        let cycle_time = t.cycle_distance / 2.0;
        let expected = (1.0 - t.duty_factor) * cycle_time;
        assert!((t.swing_duration - expected).abs() < 1e-5);
    }
}
