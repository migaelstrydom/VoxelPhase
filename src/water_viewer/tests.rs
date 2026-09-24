//! The scenarios as tests: each runs headlessly and checks what the water did.

use super::driver::{run, RunConfig};
use super::scenarios::find;

fn probe(run: &super::driver::Run, name: &str, sample: usize) -> Option<f32> {
    let index = run.probe_names.iter().position(|p| *p == name)?;
    run.samples[sample].probes[index]
}

#[test]
fn an_island_pool_and_the_pond_under_it_are_two_bodies() {
    let scenario = find("island_pool").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let last = recorded.samples.len() - 1;
    assert_eq!(recorded.samples[last].basins, 2);
    assert_eq!(probe(&recorded, "island", last), Some(7.5));
    assert!((probe(&recorded, "pond", last).unwrap() + 1.5).abs() < 1e-3);
    assert!(recorded.samples[last].ledger_error.abs() < 1e-6);
}

#[test]
fn a_breached_dam_drains_to_its_notch() {
    let scenario = find("breach").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let first = &recorded.samples[0];
    let last = recorded.samples.last().unwrap();
    assert_eq!(
        first.discarded, 0.0,
        "the pond was authored below its spill"
    );
    assert!(last.discarded > 1.0, "the breach released water");
    let before = probe(&recorded, "pond", 0).unwrap();
    let after = probe(&recorded, "pond", recorded.samples.len() - 1).unwrap();
    assert!(after < before - 0.2, "{before} -> {after}");
    assert!(last.ledger_error.abs() < 1e-6);
}

#[test]
fn a_crater_in_a_lake_floor_lowers_the_level_and_moves_no_water() {
    let scenario = find("crater_lake").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let first = &recorded.samples[0];
    let last = recorded.samples.last().unwrap();
    assert_eq!(last.basins, 1);
    assert_eq!(last.discarded, 0.0);
    assert!((last.volume - first.volume).abs() < 1e-6);
    let before = probe(&recorded, "pond", 0).unwrap();
    let after = probe(&recorded, "pond", recorded.samples.len() - 1).unwrap();
    assert!(after < before, "the crater's volume came out of the level");
    assert!(before - after < 0.2, "{before} -> {after}");
    // The crater is part of the lake now, at its level.
    let crater = probe(&recorded, "crater", recorded.samples.len() - 1).unwrap();
    assert_eq!(crater, after);
}
