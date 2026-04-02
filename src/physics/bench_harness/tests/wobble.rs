//! Tests for thin-block angular oscillation (wobble).
//!
//! The `JengaCrossWobbleScenario` places a thin box perpendicular across another
//! thin box's edge — a configuration that arises frequently when a Jenga tower
//! collapses. The solver must settle both blocks without sustained angular
//! oscillation.

use crate::physics::bench_harness::framework::{run_scenario, BenchRunConfig};
use crate::physics::bench_harness::scenarios::JengaCrossWobbleScenario;

use super::write_exports;

/// The top block in a Jenga cross configuration must settle to near-zero
/// angular velocity within a few seconds. A persistent wobble (angular speed
/// oscillating above the threshold) indicates the solver is injecting energy
/// through contact iteration order bias, warm-start cross-contamination, or
/// stale contacts across substeps.
#[test]
fn jenga_cross_settles_without_wobble() {
    let scenario = JengaCrossWobbleScenario::new();
    let cfg = BenchRunConfig {
        duration: 6.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "jenga_cross_wobble");

    // The block should not fall through or fly away.
    let final_y = run.samples.last().expect("no samples").y;
    assert!(
        final_y > 0.0 && final_y < 2.0,
        "Top block Y should stay near resting height, got {final_y:.4}"
    );

    // In the tail window (last 2 seconds), angular speed must be negligible.
    // A wobbling block typically shows angular speeds of 0.1–1.0 rad/s;
    // a settled block is below 0.01 with occasional minor contact bumps up
    // to ~0.06.
    let tail_seconds = 2.0;
    let (tail_max_linear, tail_max_angular) = run.tail_max_speeds(tail_seconds);

    assert!(
        tail_max_angular < 0.08,
        "Top block should settle (angular speed < 0.08 rad/s in last {tail_seconds}s), \
         but peak was {tail_max_angular:.4} rad/s"
    );

    assert!(
        tail_max_linear < 0.05,
        "Top block should settle (linear speed < 0.05 m/s in last {tail_seconds}s), \
         but peak was {tail_max_linear:.4} m/s"
    );
}
