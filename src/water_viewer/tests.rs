//! The scenarios as tests: each runs headlessly and checks what the water did.

use std::path::Path;

use nalgebra::Point3;

use crate::level::{load_level, Settle};
use crate::level_check::build_terrain;
use crate::water::WaterWorld;

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
    // Its trough is flat at its lip, 6 m: at most a film is left there.
    let trough = probe(&recorded, "trough", n);
    assert!(
        trough.is_none_or(|level| level < 6.01),
        "the island pool is gone into the pond: {trough:?}"
    );
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

#[test]
fn a_spring_runs_down_a_staircase_falling_tread_to_tread() {
    let scenario = find("staircase").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let wet_from = |name: &str| {
        (0..recorded.samples.len())
            .find(|i| probe(&recorded, name, *i).is_some())
            .map(|i| recorded.samples[i].time)
    };
    let (top, middle, bottom) = (
        wet_from("top").unwrap(),
        wet_from("middle").unwrap(),
        wet_from("bottom").unwrap(),
    );
    assert!(top < middle && middle < bottom, "{top} {middle} {bottom}");
    // Every riser is a fall: one channel of reaches, one per tread.
    assert!(
        recorded.reaches.len() >= 7,
        "{} reaches",
        recorded.reaches.len()
    );
    // At rest, what the spring gives leaves the bottom.
    let n = recorded.samples.len();
    let (a, b) = (&recorded.samples[n - 11], &recorded.samples[n - 1]);
    let rate = (b.sunk - a.sunk) / (b.time - a.time) as f64;
    assert!((rate - 1.0).abs() < 0.01, "{rate} m³/s leaves");
    assert!(b.ledger_error.abs() < 1e-6);
}

#[test]
fn a_staircase_opened_steady_is_already_running() {
    let scenario = find("staircase").unwrap();
    let config = RunConfig {
        settle: Some(Settle::Steady),
        ..RunConfig::default()
    };
    let recorded = run(&scenario, config).unwrap();
    let first = &recorded.samples[0];
    let last = recorded.samples.last().unwrap();
    for name in ["top", "middle", "bottom"] {
        assert!(
            probe(&recorded, name, 0).is_some(),
            "{name} dry at the start"
        );
    }
    assert!(
        (last.volume - first.volume).abs() < 1e-3 * first.volume,
        "{} -> {}",
        first.volume,
        last.volume
    );
    let rate = last.sunk / last.time as f64;
    assert!((rate - 1.0).abs() < 0.01, "{rate} m³/s leaves");
    assert!(last.ledger_error.abs() < 1e-6);
}

#[test]
fn a_source_fills_spills_and_merges_before_the_level_opens() {
    let scenario = find("spring_pools").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let first = &recorded.samples[0];
    let last = recorded.samples.last().unwrap();
    assert_eq!(first.basins, 1, "the two halves opened merged");
    assert_eq!(probe(&recorded, "west", 0), probe(&recorded, "east", 0));
    assert!((last.volume - first.volume).abs() < 1e-6 * first.volume);
    let rate = last.sunk / last.time as f64;
    assert!((rate - 0.5).abs() < 1e-3, "{rate} m³/s leaves");
    assert!(last.ledger_error.abs() < 1e-6);
}

#[test]
fn a_breach_lays_a_channel_that_retires_once_the_weir_closes() {
    let scenario = find("breach").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    assert!(
        recorded.samples.iter().any(|s| s.reaches > 0),
        "a channel advanced from the breach"
    );
    let last = recorded.samples.last().unwrap();
    assert_eq!(last.reaches, 0, "the channel retired");
    assert_eq!(last.links, 0);
    assert!(last.ledger_error.abs() < 1e-6);
}

#[test]
fn an_island_pool_pours_through_its_hole_along_a_fall() {
    let scenario = find("island_hole").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    assert!(
        recorded.samples.iter().any(|s| s.falls > 0),
        "the orifice carried a fall onto the pond"
    );
}

#[test]
fn an_island_pool_breached_at_its_edge_spills_along_a_fall() {
    let scenario = find("island_ditch").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let last = recorded.samples.last().unwrap();
    assert!(
        probe(&recorded, "island", recorded.samples.len() - 1).unwrap() < 7.3,
        "the pool drains"
    );
    assert!(last.falls > 0, "over the cliff face along a fall");
}

#[test]
fn a_river_diverted_into_a_crater_fills_it_and_runs_on() {
    let scenario = find("river_diversion").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let n = recorded.samples.len() - 1;
    let blast = recorded
        .samples
        .iter()
        .position(|s| s.time >= 12.0)
        .unwrap();
    let crater = (blast..=n)
        .find(|i| probe(&recorded, "crater", *i).is_some())
        .expect("the crater filled");
    assert!(
        probe(&recorded, "crater", n).is_some(),
        "and kept its water"
    );
    assert!(
        probe(&recorded, "lower", n).is_some(),
        "the river runs on below the crater"
    );
    let (at_blast, last) = (&recorded.samples[blast], &recorded.samples[n]);
    assert!(
        last.sunk > at_blast.sunk,
        "water left the map after the crater"
    );
    assert!(recorded.samples[crater].basins >= 2);
    assert!(recorded.samples.iter().all(|s| s.discarded == 0.0));
    assert!(last.ledger_error.abs() < 1e-6);
}

#[test]
fn a_spring_keeps_flowing_when_the_rock_around_it_is_blown_away() {
    let scenario = find("spring_rock").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let n = recorded.samples.len();
    let (a, b) = (&recorded.samples[n - 601], &recorded.samples[n - 1]);
    let rate = (b.sunk - a.sunk) / (b.time - a.time) as f64;
    assert!((rate - 1.0).abs() < 0.01, "{rate} m³/s leaves");
    assert!(b.falls > 0, "its fall was traced again");
    assert!(b.ledger_error.abs() < 1e-6);
}

#[test]
fn a_crater_merged_into_a_lake_keeps_its_water_when_the_lake_drains() {
    let scenario = find("crater_drain").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    let at = |t: f32| recorded.samples.iter().position(|s| s.time >= t).unwrap();
    // Merged: one body, one level.
    let merged = at(4.0);
    assert_eq!(recorded.samples[merged].basins, 1);
    assert_eq!(
        probe(&recorded, "crater", merged),
        probe(&recorded, "lake", merged)
    );
    // Drained: the lake is gone below the crater's rim, the crater is not.
    let n = recorded.samples.len() - 1;
    let crater = probe(&recorded, "crater", n).expect("the crater kept its water");
    assert!(probe(&recorded, "lake", n).is_none_or(|lake| lake < crater - 0.1));
    let last = &recorded.samples[n];
    assert!(recorded.samples.iter().all(|s| s.discarded == 0.0));
    assert!(last.ledger_error.abs() < 1e-6);
}

/// thin_ice's lake is over the valve's size (§9.2): a blast at its shore
/// freezes it for the blast's frame, and it re-floods on the next.
#[test]
fn the_valve_defers_a_large_lake_s_reflood_by_a_frame() {
    use crate::level::load_level;
    use crate::level_check::build_terrain;
    use crate::terrain::BlastConfig;
    use crate::water::topology::VALVE_SPANS;
    use crate::water::WaterWorld;
    use nalgebra::Point3;

    let level = load_level(std::path::Path::new("levels/thin_ice.level.ron")).unwrap();
    let mut terrain = build_terrain(&level);
    let (mut water, _) = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
    let (lake, _) = water.basins().max_by_key(|(_, b)| b.region.len()).unwrap();
    assert!(
        water
            .network()
            .store(lake)
            .unwrap()
            .as_basin()
            .unwrap()
            .region
            .len()
            > VALVE_SPANS
    );
    let volume = water.volume();

    terrain.detonate(Point3::new(45.2, 3.0, 60.2), &BlastConfig::default());
    terrain.update();
    water.on_terrain_update(&terrain);
    water.step(1.0 / 60.0);
    let frozen = |water: &WaterWorld| water.basins().any(|(_, b)| b.frozen);
    assert!(frozen(&water), "the re-flood waits a frame");
    assert_eq!(water.volume(), volume, "a frozen lake holds still");

    terrain.update();
    water.on_terrain_update(&terrain);
    water.step(1.0 / 60.0);
    assert!(!frozen(&water), "and runs on the next");
    assert!(water.balance().is_balanced());
}

#[test]
fn a_breached_sea_wall_floods_the_lowland_until_it_joins_the_sea() {
    let scenario = find("sea_wall").unwrap();
    let recorded = run(&scenario, RunConfig::default()).unwrap();
    assert_eq!(probe(&recorded, "sea", 0), Some(0.0), "the sea stands at 0");
    assert!(
        probe(&recorded, "lowland", 0).is_none(),
        "the lowland starts dry"
    );
    // It floods as a basin of its own, filled from the sea over a weir...
    let flooding = recorded
        .samples
        .iter()
        .position(|s| s.basins > 0)
        .expect("the breach made a lowland basin");
    assert!(recorded.samples[flooding..].iter().any(|s| s.volume > 1.0));
    // ...and at sea level joins it.
    let last = recorded.samples.last().unwrap();
    let n = recorded.samples.len() - 1;
    assert_eq!(last.basins, 0, "the lowland is the sea's now");
    assert_eq!(probe(&recorded, "lowland", n), Some(0.0));
    assert!(last.volume.abs() < 1e-6, "its water is the ocean's");
    assert!(last.ledger_error.abs() < 1e-6);
}

#[test]
fn the_island_sea_demo_opens_with_its_pools_above_the_sea() {
    let level = load_level(Path::new("levels/island_sea.level.ron")).unwrap();
    let terrain = build_terrain(&level);
    let (water, errors) = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
    assert!(errors.is_empty(), "{errors:?}");
    let query = water.query();
    // The floating pool and the sea beneath it are two bodies in one column.
    assert_eq!(query.level_at(Point3::new(0.0, 11.0, -26.0)), Some(11.5));
    assert_eq!(query.level_at(Point3::new(0.0, -1.0, -26.0)), Some(0.0));
    assert_eq!(query.level_at(Point3::new(-4.0, 0.0, 12.0)), Some(1.0));
    assert!(water.sources().iter().all(|s| !s.buried));
    assert!(water.balance().is_balanced());
}
