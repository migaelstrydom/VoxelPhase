use super::super::framework::{run_scenario, BenchRunConfig};
use super::super::scenarios::HighSpeedSphereCcdScenario;
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
