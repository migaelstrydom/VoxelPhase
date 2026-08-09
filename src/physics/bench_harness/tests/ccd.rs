use super::super::framework::{run_scenario, BenchRunConfig};
use super::super::scenarios::{
    GrazingSphereWallCcdScenario, GrenadeSpeedWallCcdScenario, HighSpeedSphereCcdScenario,
    SpeculativeBandApproachScenario, SphereIntoDynamicCornerScenario,
    SphereThroughDynamicSlabScenario, SphereThroughTwoSlabsScenario,
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

/// A candidate can find more than one impact in a substep, but it can only be
/// clamped to one of them. It must be the earliest, or the sphere is placed
/// past the obstacle it should have stopped at.
#[test]
fn sphere_stops_at_the_nearer_of_two_slabs() {
    let scenario = SphereThroughTwoSlabsScenario::new();
    let cfg = BenchRunConfig {
        duration: 0.5,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sphere_through_two_slabs");

    let min_x = run.samples.iter().map(|s| s.x).fold(f32::MAX, f32::min);
    assert!(
        min_x > scenario.near_slab_far_face_x(),
        "sphere passed the near slab: min_x={min_x}"
    );
}

/// An oblique impact on a dynamic obstacle. The sphere approaches two crossing
/// slabs along their bisector, so it drives into neither face head-on, and must
/// still be stopped by the one it reaches first.
#[test]
fn sphere_does_not_tunnel_into_a_dynamic_corner() {
    let scenario = SphereIntoDynamicCornerScenario::new();
    let cfg = BenchRunConfig {
        duration: 0.5,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sphere_into_dynamic_corner");

    let min_x = run.samples.iter().map(|s| s.x).fold(f32::MAX, f32::min);
    let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
    assert!(
        min_x > scenario.far_face(),
        "sphere passed through the wall: min_x={min_x}"
    );
    assert!(
        min_y > scenario.far_face(),
        "sphere passed through the floor: min_y={min_y}"
    );
}

/// The speculative band, isolated: too fast for the discrete margin, too slow
/// for either CCD gate. The pair must still be stopped without overlapping.
#[ignore = "red: speculative contacts are gated, sized and paired on a substep, but generated once per frame — see the doc on assert_band_pair_separates"]
#[test]
fn spheres_closing_in_the_speculative_band_do_not_interpenetrate() {
    assert_band_pair_separates(SpeculativeBandApproachScenario::spheres(), "spec_spheres");
}

/// The same band, with boxes. Nothing about the gap is sphere-specific.
#[ignore = "red: speculative contacts are gated, sized and paired on a substep, but generated once per frame — see the doc on assert_band_pair_separates"]
#[test]
fn boxes_closing_in_the_speculative_band_do_not_interpenetrate() {
    assert_band_pair_separates(SpeculativeBandApproachScenario::boxes(), "spec_boxes");
}

/// Both band tests are red, for three compounding reasons found by walking the
/// pair through the pipeline. None is a CCD defect; all three are in the
/// discrete narrowphase's handling of the band beneath CCD.
///
/// 1. The narrowphase broadphase bounds each collider where it *is*. A pair
///    0.2 m apart and closing at 0.4 m per frame is never paired at all, so the
///    speculative branch is not reached. Speculative contacts can therefore
///    only fire for pairs already nearly touching — precisely when they are not
///    needed. Bounding over the frame's travel fixes this.
///
/// 2. The prediction horizon is one substep while generation is once per frame,
///    so it looks 1/8th of the way ahead it must cover.
///
/// 3. The band's ceiling is `ccd_threshold` in substep units while CCD's frame
///    gate is `ccd_frame_coverage` in frame units. The two are not comparable,
///    so the claim that speculative contacts cover the band below CCD could
///    never be checked. For a 0.2 m sphere at 60 Hz there is a real gap at
///    around 12 m/s where neither mechanism fires.
///
/// Fixing all three makes the pair meet — and then stop 0.6 m apart instead of
/// 0.4 m, because a zero-depth contact tells the solver to arrest approach
/// *now* rather than on arrival. A correct speculative contact has to carry its
/// separation so the solver permits approach up to `gap / dt`. That is a change
/// to what the solver reads out of a contact, which is why this is left red
/// rather than half-fixed: bodies halting in mid-air are worse than bodies
/// briefly overlapping, and the second assertion below is what catches it.
fn assert_band_pair_separates(scenario: SpeculativeBandApproachScenario, stem: &str) {
    let cfg = BenchRunConfig {
        duration: 0.5,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, stem);

    // The pair meets when the tracked body reaches half the separation. It must
    // get there — stopping short is as wrong as overshooting, and a test that
    // only bounds one side is satisfied by bodies halting in mid-air.
    let contact_x = scenario.min_separation() * 0.5;
    let min_x = run.samples.iter().map(|s| s.x).fold(f32::MAX, f32::min);
    assert!(
        min_x > contact_x - 0.05,
        "pair interpenetrated: tracked body reached x={min_x}, contact at {contact_x}"
    );
    assert!(
        min_x < contact_x + 0.05,
        "pair stopped short of contact: tracked body stalled at x={min_x}, contact at {contact_x}"
    );
}
