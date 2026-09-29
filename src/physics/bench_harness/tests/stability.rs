//! Structural stability regression tests.
//!
//! Each test builds a multi-body structure on flat terrain and verifies that
//! it settles to rest within tight tolerance. These are regression guards
//! against solver changes that break stacking stability.

use crate::debug::DebugLines;
use crate::physics::bench_harness::framework::{
    run_scenario, BenchRunConfig, PhysicsBenchScenario,
};
use crate::physics::bench_harness::scenarios::{
    BoxGridScenario, HoneycombWallScenario, JengaTowerScenario, TempleScenario,
    VoussoirArchScenario,
};
use crate::physics::{RigidBodyHandle, SequentialStepper, Stepper};

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

// ── Every block holds still ──────────────────────────────────────────

/// Once a structure has settled, none of its blocks creeps. The tests above
/// follow one body each, with tolerances loose enough to pass a structure in
/// slow collapse: with warm starts at 60 %, the arch sagged 9 cm in three
/// seconds and the jenga tower crept 1.4 cm while each passed. Sleep hides
/// that in the game until something wakes the structure.
///
/// Not the honeycomb wall, whose top cell rolls off as it settles.
#[test]
fn every_block_of_a_settled_structure_holds_still() {
    const LIMIT: f32 = 1.0e-3;
    let structures: [(&str, &dyn PhysicsBenchScenario); 4] = [
        ("voussoir arch", &VoussoirArchScenario::new()),
        ("jenga tower", &JengaTowerScenario::new(12)),
        ("box grid", &BoxGridScenario::new(5)),
        ("temple", &TempleScenario::new()),
    ];
    let failures: Vec<String> = structures
        .into_iter()
        .filter_map(|(name, scenario)| {
            let creep = creep_once_settled(scenario, 3.0, 3.0);
            (creep > LIMIT).then(|| format!("{name}: a block crept {creep:.4} m"))
        })
        .collect();
    assert!(failures.is_empty(), "\n{}", failures.join("\n"));
}

/// How far the block that moved most moved over `watch` seconds, after
/// `settle` seconds to come to rest. Every block starts awake, as a blast
/// or a footstep would leave it, and the scenario's world keeps it awake.
fn creep_once_settled(scenario: &dyn PhysicsBenchScenario, settle: f32, watch: f32) -> f32 {
    const FRAME_DT: f32 = 1.0 / 60.0;
    let mut world = scenario.build_world();
    scenario.setup(&mut world);
    let blocks: Vec<RigidBodyHandle> = world
        .bodies()
        .iter()
        .filter(|(_, body)| body.is_dynamic())
        .map(|(index, _)| RigidBodyHandle(index))
        .collect();
    for &block in &blocks {
        world.wake_body(block);
    }

    let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
    let mut debug = DebugLines::default();
    let mut run_for = |world: &mut _, seconds: f32| {
        for _ in 0..(seconds / FRAME_DT).round() as u32 {
            stepper.step(world, FRAME_DT, scenario.geometry(), &[], &[], &mut debug);
        }
    };
    run_for(&mut world, settle);
    let settled: Vec<_> = blocks
        .iter()
        .map(|&b| world.body(b).unwrap().position())
        .collect();
    run_for(&mut world, watch);
    blocks
        .iter()
        .zip(&settled)
        .map(|(&b, at)| (world.body(b).unwrap().position() - at).norm())
        .fold(0.0, f32::max)
}
