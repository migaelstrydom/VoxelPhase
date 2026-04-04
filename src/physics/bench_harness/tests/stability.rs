//! Structural stability regression tests.
//!
//! Each test builds a multi-body structure on flat terrain and verifies that
//! it settles to rest within tight tolerance. These are regression guards
//! against solver changes that break stacking stability.

use crate::physics::bench_harness::framework::{run_scenario, BenchRunConfig};
use crate::physics::bench_harness::scenarios::{
    HoneycombWallScenario, JengaTowerScenario, TempleScenario, VoussoirArchScenario,
};

use super::write_exports;

// ── Honeycomb wall ───────────────────────────────────────────────────

#[test]
fn honeycomb_wall_settles() {
    let scenario = HoneycombWallScenario::new();
    let cfg = BenchRunConfig {
        duration: 8.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "honeycomb_wall");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    let final_y = run.samples.last().unwrap().y;
    assert!(
        final_y > 0.0 && final_y < 3.0,
        "tracked cell should stay at rest height, got y={final_y:.4}"
    );

    let (tail_linear, tail_angular) = run.tail_max_speeds(2.0);
    assert!(
        tail_linear < 0.02,
        "honeycomb wall should settle (linear < 0.02 m/s), got {tail_linear:.4}"
    );
    assert!(
        tail_angular < 0.05,
        "honeycomb wall should settle (angular < 0.05 rad/s), got {tail_angular:.4}"
    );
}

// ── Voussoir arch ────────────────────────────────────────────────────

#[test]
fn voussoir_arch_holds_together() {
    let scenario = VoussoirArchScenario::new();
    let cfg = BenchRunConfig {
        duration: 15.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "voussoir_arch");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    // The keystone should stay near the top of the arch.
    // inner_r=10, outer_r=15, center_y=0.5 → keystone centroid ≈ y=12.5+0.5=13
    let final_y = run.samples.last().unwrap().y;
    assert!(
        final_y > 10.0,
        "keystone should stay near arch apex, got y={final_y:.4}"
    );

    let (tail_linear, tail_angular) = run.tail_max_speeds(3.0);
    assert!(
        tail_linear < 0.30,
        "arch keystone should settle (linear < 0.30 m/s), got {tail_linear:.4}"
    );
    assert!(
        tail_angular < 0.30,
        "arch keystone should settle (angular < 0.30 rad/s), got {tail_angular:.4}"
    );
}

// ── Jenga tower ──────────────────────────────────────────────────────

#[test]
fn jenga_tower_settles() {
    let scenario = JengaTowerScenario::new(7);
    let cfg = BenchRunConfig {
        duration: 8.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "jenga_tower_7");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    // Top-center block of a 7-layer tower: y ≈ block_half_height + 6 * block_height
    // half_height = 0.15, block_height = 0.3 → y ≈ 0.15 + 6*0.3 = 1.95
    let final_y = run.samples.last().unwrap().y;
    assert!(
        final_y > 1.5 && final_y < 2.5,
        "top block should stay near tower top, got y={final_y:.4}"
    );

    let (tail_linear, tail_angular) = run.tail_max_speeds(2.0);
    assert!(
        tail_linear < 0.02,
        "jenga tower should settle (linear < 0.02 m/s), got {tail_linear:.4}"
    );
    assert!(
        tail_angular < 0.05,
        "jenga tower should settle (angular < 0.05 rad/s), got {tail_angular:.4}"
    );
}

// ── Temple ───────────────────────────────────────────────────────────

#[test]
fn temple_stands_stable() {
    let scenario = TempleScenario::new();
    let cfg = BenchRunConfig {
        duration: 10.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "temple");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    // Tracked body is a front column. Column base is at stylobate_top ≈ 1.0,
    // column height = 8.0, centroid ≈ y=5.0.
    let final_y = run.samples.last().unwrap().y;
    assert!(
        final_y > 3.0 && final_y < 7.0,
        "tracked column should stay upright, got y={final_y:.4}"
    );

    let (tail_linear, tail_angular) = run.tail_max_speeds(2.0);
    assert!(
        tail_linear < 0.02,
        "temple should settle (linear < 0.02 m/s), got {tail_linear:.4}"
    );
    assert!(
        tail_angular < 0.05,
        "temple should settle (angular < 0.05 rad/s), got {tail_angular:.4}"
    );
}
