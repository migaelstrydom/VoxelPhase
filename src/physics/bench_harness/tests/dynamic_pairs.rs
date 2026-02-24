use super::super::framework::{run_scenario, BenchRunConfig};
use super::super::scenarios::*;
use super::write_exports;

// ── Dynamic pairs: sphere-sphere ───────────────────────────────────

#[test]
fn sphere_sphere_collision_transfers_momentum() {
    let scenario = SphereSphereCollisionScenario::new(0.8);
    let cfg = BenchRunConfig {
        duration: 3.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sphere_sphere_collision_r0_8");

    let last = run.samples.last().unwrap();
    assert!(
        last.linear_speed > 0.1 || run.samples.iter().any(|s| s.linear_speed > 1.0),
        "target sphere should gain speed from collision"
    );
}

// ── Dynamic pairs: sphere-OBB ──────────────────────────────────────

#[test]
fn sphere_obb_collision_transfers_momentum() {
    let scenario = SphereObbCollisionScenario::new(0.8);
    let cfg = BenchRunConfig {
        duration: 3.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sphere_obb_collision_r0_8");

    // The box target should have gained lateral velocity from the sphere hit.
    assert!(
        run.samples.iter().any(|s| s.linear_speed > 0.5),
        "target box should gain speed from sphere impact"
    );
}

// ── Dynamic pairs: OBB-OBB ─────────────────────────────────────────

#[test]
fn obb_obb_collision_transfers_momentum() {
    let scenario = ObbObbCollisionScenario::new(0.8);
    let cfg = BenchRunConfig {
        duration: 3.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "obb_obb_collision_r0_8");

    assert!(
        run.samples.iter().any(|s| s.linear_speed > 0.5),
        "target box should gain speed from box-box impact"
    );
}
