use super::super::framework::{run_scenario, BenchRunConfig};
use super::super::scenarios::{
    GrazingSphereWallCcdScenario, GrenadeSpeedWallCcdScenario, HighSpeedSphereCcdScenario,
    SphereThroughDynamicSlabScenario,
};
use super::assertions::*;
use super::write_exports;

#[test]
fn high_speed_sphere_does_not_tunnel() {
    let scenario = HighSpeedSphereCcdScenario::new();
    let cfg = BenchRunConfig {
        duration: 6.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "high_speed_sphere_ccd");

    assert_above_floor(&run, -0.1);

    let last = run.samples.last().unwrap();
    assert!(
        last.y > 0.2,
        "sphere should rest above ground: y={}",
        last.y
    );
}

/// A sphere that has narrowphase floor contacts at frame start must still be
/// swept by CCD later in the same frame, or it tunnels through the wall.
#[test]
fn grazing_sphere_does_not_tunnel_through_wall() {
    let scenario = GrazingSphereWallCcdScenario::new();
    let cfg = BenchRunConfig {
        duration: 0.5,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "grazing_sphere_wall_ccd");

    let min_x = run.samples.iter().map(|s| s.x).fold(f32::MAX, f32::min);
    assert!(
        min_x > -scenario.radius,
        "sphere tunnelled through the wall at x=0: min_x={min_x}"
    );

    assert_above_floor(&run, -0.1);
}

/// The floor the sphere is sliding along must not be treated as a CCD hit:
/// clamping to a t=0 graze would teleport it back to its substep-start
/// position and freeze it in place.
#[test]
fn grazing_sphere_is_not_frozen_by_floor_contact() {
    let scenario = GrazingSphereWallCcdScenario::new();
    let cfg = BenchRunConfig {
        duration: 0.5,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);

    let min_x = run.samples.iter().map(|s| s.x).fold(f32::MAX, f32::min);
    assert!(
        min_x < scenario.radius + 0.15,
        "sphere should reach the wall rather than stall on the floor: min_x={min_x}"
    );
}

/// A grenade-speed sphere is too slow for the per-substep CCD gate but fast
/// enough to cross the wall within one 8-substep frame. Only the frame-level
/// gate catches it.
#[test]
fn grenade_speed_sphere_does_not_tunnel_through_wall() {
    let scenario = GrenadeSpeedWallCcdScenario::new();
    let cfg = BenchRunConfig {
        duration: 1.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "grenade_speed_wall_ccd");

    let min_x = run.samples.iter().map(|s| s.x).fold(f32::MAX, f32::min);
    assert!(
        min_x > -scenario.radius,
        "grenade tunnelled through the wall at x=0: min_x={min_x}"
    );
}

/// The pendulum bug: CCD only sweeps against static geometry, so a thin
/// *dynamic* obstacle is invisible to it. At grenade speed the once-per-frame
/// narrowphase cannot close the gap either, and the sphere passes through.
#[test]
fn grenade_speed_sphere_does_not_tunnel_through_dynamic_slab() {
    let scenario = SphereThroughDynamicSlabScenario::new();
    let cfg = BenchRunConfig {
        duration: 0.5,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sphere_through_dynamic_slab");

    let min_x = run.samples.iter().map(|s| s.x).fold(f32::MAX, f32::min);
    assert!(
        min_x > scenario.far_face_x(),
        "sphere tunnelled through the dynamic slab: min_x={min_x}"
    );
}

/// Passing through and stopping short are both failures, and the min_x bound
/// alone cannot tell them apart. The sphere must actually be turned around.
#[test]
fn grenade_speed_sphere_rebounds_off_dynamic_slab() {
    let scenario = SphereThroughDynamicSlabScenario::new();
    let cfg = BenchRunConfig {
        duration: 0.5,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);

    let last = run.samples.last().unwrap();
    assert!(
        last.x > scenario.radius,
        "sphere should end up back on the near side of the slab: x={}",
        last.x
    );
}
