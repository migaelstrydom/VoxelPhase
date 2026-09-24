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
fn a_breached_dam_drains_over_a_weir_off_the_map() {
    let scenario = find("breach").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let first = &recorded.samples[0];
    let last = recorded.samples.last().unwrap();
    assert_eq!(first.sunk, 0.0, "the pond was authored below its spill");
    assert!(
        last.sunk > 0.5 * first.volume,
        "the breach released most of the pond: {} of {}",
        last.sunk,
        first.volume
    );
    assert!(probe(&recorded, "pond", recorded.samples.len() - 1).is_none());
    assert_eq!(last.links, 0, "the weir closed once the flow ran out");
    assert!(recorded.samples.iter().all(|s| s.discarded == 0.0));
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

#[test]
fn a_pond_spills_into_a_dry_pit_until_the_two_merge() {
    let scenario = find("spill_merge").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let first = &recorded.samples[0];
    let last = recorded.samples.last().unwrap();
    assert_eq!(last.basins, 1, "the two halves merged");
    assert!(
        (last.volume - first.volume).abs() < 1e-6,
        "nothing left the trench"
    );
    let (west, east) = (
        probe(&recorded, "west", recorded.samples.len() - 1).unwrap(),
        probe(&recorded, "east", recorded.samples.len() - 1).unwrap(),
    );
    assert_eq!(west, east);
    assert!(last.ledger_error.abs() < 1e-6);
}

#[test]
fn a_lake_drained_below_its_divider_splits() {
    let scenario = find("drain_split").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let last = recorded.samples.last().unwrap();
    assert_eq!(last.basins, 2);
    let n = recorded.samples.len() - 1;
    let (west, east) = (
        probe(&recorded, "west", n).unwrap(),
        probe(&recorded, "east", n).unwrap(),
    );
    assert!(
        east > west + 0.5,
        "the east half kept its water: {east} vs {west}"
    );
    assert!(last.sunk > 0.0);
    assert!(last.ledger_error.abs() < 1e-6);
}

#[test]
fn an_island_pool_holed_through_pours_onto_the_pond_below() {
    let scenario = find("island_hole").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let first = &recorded.samples[0];
    let last = recorded.samples.last().unwrap();
    let n = recorded.samples.len() - 1;
    assert_eq!(first.basins, 2);
    assert_eq!(last.basins, 1, "the island pool is gone into the pond");
    assert!((last.volume - first.volume).abs() < 1e-6, "no water lost");
    let before = probe(&recorded, "pond_open", 0).unwrap();
    let after = probe(&recorded, "pond_open", n).unwrap();
    assert!(after > before, "the pond rose: {before} -> {after}");
    assert!(last.ledger_error.abs() < 1e-6);
}

#[test]
fn a_lake_drains_down_a_channel_whose_front_advances() {
    let scenario = find("river").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let first = &recorded.samples[0];
    let last = recorded.samples.last().unwrap();
    // The front reaches the lower probe some seconds after the upper one.
    let wet_from = |name: &str| {
        recorded
            .samples
            .iter()
            .enumerate()
            .find(|(i, _)| probe(&recorded, name, *i).is_some())
            .map(|(_, s)| s.time)
    };
    let (upper, lower) = (wet_from("upper").unwrap(), wet_from("lower").unwrap());
    assert!(
        upper < lower,
        "the front ran down the channel: {upper} then {lower}"
    );
    assert!(!recorded.reaches.is_empty(), "the outlet laid a channel");
    assert!(
        last.sunk > 0.5 * first.volume,
        "{} of {}",
        last.sunk,
        first.volume
    );
    assert!(recorded.samples.iter().all(|s| s.discarded == 0.0));
    assert!(last.ledger_error.abs() < 1e-6);
}
