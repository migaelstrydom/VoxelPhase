//! The scenarios as tests: each runs headlessly and checks what the water
//! did.

use super::driver::{run as run_scenario, Run, RunConfig};
use super::scenarios::{catalogue, find};

/// Scenarios run on the field are long: run these with
/// `cargo test --release --lib water_viewer::tests -- --ignored`.
fn run(name: &str) -> Run {
    let scenario = find(name).unwrap_or_else(|| panic!("no scenario {name}"));
    run_scenario(&scenario, RunConfig::default(), &[], |_, _, _| {}).unwrap()
}

fn probe(run: &Run, name: &str) -> Vec<Option<f32>> {
    let index = run.probe_names.iter().position(|p| *p == name).unwrap();
    run.samples.iter().map(|s| s.probes[index]).collect()
}

/// The case the network got wrong: a grenade in a running river's bed. The
/// river above the crater keeps its level while the crater fills, and the
/// river below comes back to the level it ran at before.
#[test]
#[ignore]
fn a_crater_in_a_river_leaves_the_river_above_it_alone() {
    let run = run("river_blast");
    for name in ["upper", "above"] {
        let levels = probe(&run, name);
        let before = levels[0].unwrap();
        for (sample, level) in run.samples.iter().zip(&levels) {
            let level = level.unwrap_or_else(|| panic!("{name} dry at {}", sample.time));
            assert!(
                (level - before).abs() < 0.03,
                "{name} moved from {before} to {level} at {} s",
                sample.time
            );
        }
    }
    for name in ["below", "lower"] {
        let levels = probe(&run, name);
        let (before, after) = (levels[0].unwrap(), levels.last().unwrap().unwrap());
        assert!(
            (after - before).abs() < 0.02,
            "{name} ran at {before} before and {after} after"
        );
    }
}

/// Every scenario: the ledger balances, no cell goes below empty, and
/// nothing becomes infinite or NaN.
#[test]
#[ignore]
fn every_scenario_conserves_water_and_stays_finite() {
    for scenario in catalogue() {
        let run = run_scenario(&scenario, RunConfig::default(), &[], |_, _, _| {}).unwrap();
        for s in &run.samples {
            assert!(s.finite, "{}: not finite at {} s", run.scenario, s.time);
            assert!(
                s.shallowest > -1.0e-9,
                "{}: a cell at {} m at {} s",
                run.scenario,
                s.shallowest,
                s.time
            );
            let scale = 1.0 + s.volume + s.sunk.abs();
            assert!(
                s.ledger_error.abs() < 1.0e-9 * scale,
                "{}: ledger off by {} m³ at {} s",
                run.scenario,
                s.ledger_error,
                s.time
            );
        }
    }
}

/// Water nothing disturbs does not move, and once it has been seen to be
/// calm, a few seconds after the level opens, it costs nothing.
#[test]
#[ignore]
fn still_water_sleeps_where_it_stands() {
    let run = run("island_pool");
    for s in &run.samples {
        assert_eq!(s.probes, run.samples[0].probes, "moved at {} s", s.time);
        if s.time > 3.0 {
            assert_eq!(s.awake, 0, "awake chunks at {} s", s.time);
        }
    }
}

/// Water a blast set moving comes to rest, and sleeps.
#[test]
#[ignore]
fn disturbed_water_comes_to_rest() {
    for name in ["sea_wall", "spill_merge"] {
        let run = run(name);
        let last = run.samples.last().unwrap();
        assert_eq!(
            last.awake, 0,
            "{name}: {} chunks awake at the end",
            last.awake
        );
    }
}

/// A pit opening into a closed tunnel fills it to its roof and no further,
/// then goes on rising above it.
#[test]
#[ignore]
fn a_full_cave_takes_no_more_water() {
    const ROOF: f32 = 3.0;
    let run = run("cave_pool");
    for (s, level) in run.samples.iter().zip(probe(&run, "tunnel")) {
        if let Some(level) = level {
            assert!(level < ROOF + 0.05, "tunnel at {level} at {} s", s.time);
        }
    }
    let pit = probe(&run, "pit").last().copied().flatten().unwrap();
    assert!(pit > ROOF + 1.0, "the pit stopped at {pit}");
}

/// A pool fills only the hollow its seed lies in, from wherever in it the
/// seed is: not the valley below a lake authored above its rim, and all of
/// a pond seeded on its slope.
#[test]
#[ignore]
fn a_pool_fills_its_own_hollow() {
    let breached = run("river");
    assert!(
        breached.samples[0].volume < 100.0,
        "a breached lake opened with {} m³",
        breached.samples[0].volume
    );
    let pond = run("island_pool");
    assert_eq!(probe(&pond, "pond")[0], Some(-1.5));
    assert_eq!(probe(&pond, "pond_open")[0], Some(-1.5));
}
