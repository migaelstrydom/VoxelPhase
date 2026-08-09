//! Static rigid bodies are ordinary colliders of infinite mass.
//!
//! They are not the static *geometry* that `StaticGeometry` supplies, so they
//! only work if the body-vs-body narrowphase includes them.

use super::super::framework::{run_scenario, BenchRunConfig};
use super::super::scenarios::{BoxOnStaticPlatformScenario, SphereOnStaticBodyScenario};
use super::write_exports;

/// With no terrain in the world, a static body is the only thing that can hold
/// the sphere up. If static bodies are invisible to the narrowphase, it falls
/// forever.
#[test]
fn sphere_rests_on_static_body_without_terrain() {
    let scenario = SphereOnStaticBodyScenario::new(0.2);
    let cfg = BenchRunConfig {
        duration: 3.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sphere_on_static_body");

    let radius = 0.4;
    let expected_rest = scenario.platform_top + radius;

    let last = run.samples.last().unwrap();
    assert!(
        (last.y - expected_rest).abs() < 0.05,
        "sphere should rest on top of the static platform at y={expected_rest}: y={}",
        last.y
    );

    let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
    assert!(
        min_y > scenario.platform_top - 0.05,
        "sphere sank into or through the static platform: min_y={min_y}"
    );
}

/// The sphere must actually come to rest, not hover or jitter on the contact.
#[test]
fn sphere_on_static_body_settles() {
    let scenario = SphereOnStaticBodyScenario::new(0.2);
    let cfg = BenchRunConfig {
        duration: 3.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);

    let tail = &run.samples[run.samples.len() * 3 / 4..];
    let max_speed = tail
        .iter()
        .map(|s| s.linear_speed)
        .fold(0.0f32, |acc, v| acc.max(v));
    assert!(
        max_speed < 0.05,
        "sphere should be at rest on the static body: max tail speed={max_speed}"
    );
}

/// Shock propagation must treat a static body as ground. If it is instead
/// modelled as a stackable node, the box above it is ordered as though its
/// support were itself supported, and the contact sinks or jitters.
#[test]
fn box_settles_on_static_platform() {
    let scenario = BoxOnStaticPlatformScenario::new(0.0);
    let cfg = BenchRunConfig {
        duration: 3.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "box_on_static_platform");

    let expected_rest = 1.0 + 0.4;
    let last = run.samples.last().unwrap();
    assert!(
        (last.y - expected_rest).abs() < 0.05,
        "box should rest on the static platform at y={expected_rest}: y={}",
        last.y
    );

    let tail = &run.samples[run.samples.len() * 3 / 4..];
    let max_speed = tail
        .iter()
        .map(|s| s.linear_speed)
        .fold(0.0f32, |acc, v| acc.max(v));
    assert!(
        max_speed < 0.05,
        "box should be at rest on the static platform: max tail speed={max_speed}"
    );
}
