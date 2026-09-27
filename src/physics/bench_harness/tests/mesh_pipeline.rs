use super::super::framework::{run_scenario, BenchRunConfig, PhysicsBenchScenario};
use super::super::scenarios::*;
use super::assertions::*;
use super::write_exports;

// ── Mesh pipeline: sphere vs flat terrain ──────────────────────────

#[test]
fn flat_sphere_rest_settles_on_ground() {
    let scenario = FlatSphereRestScenario::new(0.0);
    let cfg = BenchRunConfig::default();
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "flat_sphere_rest_r0_0");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    assert_settled(&run, 1.0, 0.02, f32::MAX);
    assert_final_y_near(&run, 0.5, 0.05);
    assert_resting_contacts(&run, 1);
}

#[test]
fn flat_sphere_rest_bouncy_sphere_reaches_higher_peak() {
    let cfg = BenchRunConfig {
        duration: 4.0,
        ..BenchRunConfig::default()
    };

    let run_inelastic = run_scenario(&FlatSphereRestScenario::new(0.0), cfg);
    let run_bouncy = run_scenario(&FlatSphereRestScenario::new(0.8), cfg);

    write_exports(&run_inelastic, "flat_sphere_rest_bounce_r0_0");
    write_exports(&run_bouncy, "flat_sphere_rest_bounce_r0_8");

    let peak_inelastic = run_inelastic
        .samples
        .iter()
        .filter(|s| s.sim_time >= 1.5)
        .map(|s| s.y)
        .fold(0.0f32, f32::max);
    let peak_bouncy = run_bouncy
        .samples
        .iter()
        .filter(|s| s.sim_time >= 1.5)
        .map(|s| s.y)
        .fold(0.0f32, f32::max);

    assert!(
        peak_bouncy > peak_inelastic + 0.1,
        "bouncy sphere peak {peak_bouncy:.3} should exceed inelastic {peak_inelastic:.3}"
    );
}

/// A ball landing in the speculative band bounces as high as its restitution
/// says: `e² × h` for the first bounce, within what one substep of travel
/// either side of the ground can cost.
#[test]
fn bouncy_ball_landing_in_the_speculative_band_rebounds_to_its_restitution() {
    let scenario = BouncyBallDropScenario::new(0.8);
    let cfg = BenchRunConfig {
        duration: 2.5,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "bouncy_ball_drop");

    // The underside's height, sample by sample: falling to the landing, then
    // rising to the first bounce's apex.
    let heights: Vec<f32> = run.samples.iter().map(|s| s.y - scenario.radius).collect();
    let landing = heights
        .windows(2)
        .position(|w| w[1] > w[0])
        .expect("the ball never bounced");
    let lowest = heights[landing];
    let apex = heights[landing..]
        .windows(2)
        .position(|w| w[1] < w[0])
        .map(|i| landing + i)
        .expect("the first bounce never peaked");
    let rebound = heights[apex];

    let expected = scenario.expected_first_rebound();
    eprintln!(
        "bouncy ball: first rebound {rebound:.3} m, expected {expected:.3} m, lowest {lowest:.4} m"
    );
    assert!(
        (rebound - expected).abs() < 0.05 * expected,
        "first rebound rose to {rebound:.3} m, restitution predicts {expected:.3} m"
    );
    assert!(
        lowest > -0.02,
        "the ball sank {:.3} m into the ground on landing",
        -lowest
    );
}

// ── Mesh pipeline: sphere sliding over internal edges ────────────

#[test]
fn sphere_slide_no_jitter_over_seam() {
    let scenario = SphereSlideScenario::new();
    let cfg = BenchRunConfig {
        duration: 4.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sphere_slide");

    assert!(!run.samples.is_empty());

    // The sphere should stay on the surface the entire time. With
    // radius 0.5 on a y=0 plane, center should be near 0.5.
    assert_y_in_range(&run, 0.4, 0.7);

    // The sphere should move in +X (it started with positive X velocity).
    let last = run.samples.last().unwrap();
    assert!(
        last.x > -3.0,
        "sphere should have moved in +X: x={}",
        last.x
    );

    // On flat terrain the sphere should only ever have 1 contact point.
    assert_max_contacts_after_settle(&run, 10, 1);

    // Check for vertical speed spikes as the sphere crosses the mesh seam.
    assert_no_vertical_jitter(&run, cfg.fixed_dt, 50, 0.5);
}

// ── Mesh pipeline: box vs flat terrain ─────────────────────────────

#[test]
fn flat_box_rest_zero_restitution_settles_in_tail_window() {
    let scenario = FlatBoxRestScenario::new(0.0);
    let cfg = BenchRunConfig::default();
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "flat_box_rest_r0_0");

    assert_eq!(scenario.name(), "flat_box_rest");
    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    assert_settled(&run, 1.0, 0.02, 0.05);

    let last = run.samples.last().expect("last sample exists");
    assert!(
        last.y > 0.45,
        "final y should remain above ground: {}",
        last.y
    );
    assert_resting_contacts(&run, 1);
    assert_no_deep_penetration(&run, 0.02);
}

#[test]
fn flat_box_rest_restitution_sweep_exports_and_effective_restitution_order() {
    let cfg = BenchRunConfig::default();
    let values = [0.0f32, 0.2, 0.8];

    for restitution in values {
        let scenario = FlatBoxRestScenario::new(restitution);
        let run = run_scenario(&scenario, cfg);
        let stem = format!("flat_box_rest_r{:.1}", restitution).replace('.', "_");
        write_exports(&run, &stem);
        let _ = run
            .samples
            .last()
            .expect("sweep run should produce samples");
    }
}

// ── Mesh pipeline: ramp terrain ────────────────────────────────────

#[test]
fn sphere_on_ramp_rolls_downhill() {
    let scenario = SphereOnRampScenario::new();
    let cfg = BenchRunConfig {
        duration: 4.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sphere_on_ramp");

    assert!(!run.samples.is_empty());
    assert_gained_speed(&run, 1.0);
    assert_above_floor(&run, -0.5);
}

#[test]
fn box_on_ramp_slides_downhill() {
    let scenario = BoxOnRampScenario::new();
    let cfg = BenchRunConfig {
        duration: 4.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "box_on_ramp");

    assert!(!run.samples.is_empty());
    assert_gained_speed(&run, 0.5);
    assert_above_floor(&run, -0.5);
}

// ── Mesh pipeline: step terrain ────────────────────────────────────

#[test]
fn box_on_step_settles_on_lower_level() {
    let scenario = BoxOnStepScenario::new();
    let cfg = BenchRunConfig::default();
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "box_on_step");

    assert!(!run.samples.is_empty());

    assert_settled(&run, 1.0, 0.05, f32::MAX);
    assert_final_y_in_range(&run, 0.2, 1.0);
}

#[test]
fn heavy_sphere_on_platform_near_edge_settles_without_rotational_jitter() {
    let scenario = HeavySphereOnPlatformScenario::new();
    let cfg = BenchRunConfig {
        duration: 10.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "heavy_sphere_on_platform");

    assert!(!run.samples.is_empty());

    let tail_start = (cfg.duration - 3.0).max(0.0);
    let tail_max_angular_speed = run
        .samples
        .iter()
        .filter(|s| s.sim_time >= tail_start)
        .map(|s| s.angular_speed)
        .fold(0.0f32, f32::max);
    eprintln!("heavy_sphere_on_platform tail_max_angular_speed={tail_max_angular_speed:.6}");
    assert!(
        tail_max_angular_speed < 0.005,
        "platform rotational jitter detected: tail_max_angular_speed={tail_max_angular_speed:.6}"
    );
}

// ── Mesh pipeline: bowl (multi-face concave) ────────────────────────

#[test]
fn sphere_in_bowl_settles_without_falling_through() {
    let scenario = SphereInBowlScenario::new();
    let cfg = BenchRunConfig {
        duration: 6.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sphere_in_bowl");

    assert!(!run.samples.is_empty());

    // The sphere should settle inside the bowl. With half_size=2,
    // depth=2, radius=0.5, the equilibrium center is at y ≈ -1.29.
    // It must NOT fall through the apex (y < -2.0).
    assert_above_floor(&run, -2.0);
    assert_final_y_in_range(&run, -1.8, 0.0);
    assert_stayed_centered_x(&run, 0.3);
    assert_tail_min_contacts(&run, 1.0, 2);
    assert_settled(&run, 1.0, 0.02, f32::MAX);
}
