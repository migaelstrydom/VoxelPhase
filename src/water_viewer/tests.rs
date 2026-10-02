//! The scenarios as tests: each runs headlessly and checks what the water did.

use std::path::Path;

use nalgebra::Point3;

use crate::level::{load_level, Settle, WaterBody};
use crate::level_check::build_terrain;
use crate::terrain::BlastConfig;
use crate::water::ids::StoreId;
use crate::water::network::{FallPath, Store, STEP_EPSILON};
use crate::water::solver::Account;
use crate::water::topology::TopologyEdit;
use crate::water::WaterWorld;

use super::driver::{run, run_with_captures, RunConfig};
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
fn the_sea_pouring_back_over_a_lowland_s_weir_falls_along_its_far_side() {
    let scenario = find("sea_wall").unwrap();
    // The weir is laid lowland → sea; the sea runs back over it. The breach
    // goes off at 2 s and the sheet runs for about a second before the
    // lowland fills up to it.
    let mut poured = Vec::new();
    run_with_captures(
        &scenario,
        RunConfig::default(),
        &[2.5, 3.0, 4.0, 8.0, 16.0],
        |_, _, water| {
            poured.push(water.network().links().any(|(id, l)| {
                let heights = water.link_interface(id);
                l.back.as_ref().is_some_and(|b| b.fall.is_some())
                    && heights.is_some_and(|h| h.back && h.step() > STEP_EPSILON)
            }));
        },
    )
    .unwrap();
    assert!(
        poured.iter().any(|p| *p),
        "no sheet on the sea's side: {poured:?}"
    );
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

#[test]
fn the_water_park_opens_at_rest_with_its_river_running() {
    let level = load_level(Path::new("levels/water_park.level.ron")).unwrap();
    let terrain = build_terrain(&level);
    let (water, errors) = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
    assert!(errors.is_empty(), "{errors:?}");
    assert!(water.steady_report().is_some_and(|r| r.converged));
    let query = water.query();
    // The undercroft's pond stands over its cavern, which is dry.
    assert_eq!(query.level_at(Point3::new(-64.0, 7.0, -6.0)), Some(8.5));
    assert_eq!(query.level_at(Point3::new(-64.0, 0.0, -6.0)), None);
    // The sea wall holds the lowland dry.
    assert_eq!(query.level_at(Point3::new(-50.0, -1.5, -43.0)), None);
    // The spring's 2.5 m³/s runs down the river into the catch lake.
    let river = water
        .network()
        .stores()
        .filter_map(|(_, s)| s.as_reach())
        .filter(|r| r.inflow > 2.4)
        .count();
    assert!(river >= 7, "{river} reaches carry the river");
    // It leaves the head lake at the lake's level...
    let head = query
        .sample(Point3::new(64.0, 6.0, 64.0))
        .expect("the head lake")
        .body;
    let first = water
        .network()
        .links()
        .find(|(_, l)| {
            l.up == head
                && water
                    .network()
                    .store(l.down)
                    .is_some_and(|s| s.as_reach().is_some())
        })
        .map(|(_, l)| l.down)
        .expect("the river leaves the head lake");
    let top = water.network().store(first).unwrap().as_reach().unwrap();
    let start = top.surface_at(0.0, water.reach_ends(first));
    let level = water.level(head).unwrap();
    assert!(
        (start - level).abs() < 0.01,
        "starts at {start}, the lake is at {level}"
    );
    // ...and ends over the cliff top at the catch lake's shore, 0.9 m above
    // the water, falling from its own surface to the lake's.
    let lake = query
        .sample(Point3::new(11.0, 2.0, 19.0))
        .expect("the catch lake")
        .body;
    let (id, link, reach) = water
        .network()
        .links()
        .filter(|(_, l)| l.down == lake && l.fall.is_some())
        .find_map(|(id, l)| Some((id, l, water.network().store(l.up)?.as_reach()?)))
        .expect("the river falls into the catch lake");
    let heights = water.link_interface(id).unwrap();
    let end = reach.surface_at(reach.length, water.reach_ends(link.up));
    assert!(
        (heights.upper - end).abs() < 0.01,
        "falls from {}, ends at {end}",
        heights.upper
    );
    assert!(
        heights.free() && heights.upper - heights.lower > 1.0,
        "{heights:?}"
    );
    let surface = water.level(lake).unwrap();
    assert!(
        (heights.lower - surface).abs() < 1e-4,
        "lands at {}, the lake is at {surface}",
        heights.lower
    );
    assert!(water.balance().is_balanced());
}

#[test]
fn the_water_park_opens_at_rest_whatever_level_its_catch_lake_is_authored_at() {
    let level = load_level(Path::new("levels/water_park.level.ron")).unwrap();
    let terrain = build_terrain(&level);
    for authored in [1.0, 3.3] {
        let mut config = level.water.clone().unwrap();
        for body in config.bodies.iter_mut() {
            if let WaterBody::Pool {
                seed: (11.0, 19.0),
                surface_level,
            } = body
            {
                *surface_level = authored;
            }
        }
        let (water, _) = WaterWorld::from_config(&config, &terrain);
        let report = water.steady_report().unwrap();
        assert!(report.converged, "authored at {authored}: {report:?}");
        assert!(report.sweeps < 60, "authored at {authored}: {report:?}");
        let lake = water.query().sample(Point3::new(11.0, 2.0, 19.0)).unwrap();
        println!(
            "authored at {authored}: {} sweeps, lake at {:.3}",
            report.sweeps, lake.surface
        );
        assert!(water.balance().is_balanced());
    }
}

#[test]
fn a_lake_moves_a_channel_s_shoreline_without_pouring_the_channel_into_it() {
    let scenario = find("shoreline").unwrap();
    let mut logs = Vec::new();
    let recorded = run_with_captures(
        &scenario,
        RunConfig::default(),
        &[179.9, 239.9],
        |_, _, water| {
            let lake = water
                .query()
                .sample(Point3::new(4.0, 6.2, 0.0))
                .map(|s| s.body);
            let bed = water
                .query()
                .sample(Point3::new(4.0, 6.2, 0.0))
                .and_then(|s| water.network().store(s.body)?.as_reach().map(|_| s.body));
            logs.push((water.topology_log().to_vec(), lake, bed));
        },
    )
    .unwrap();
    assert!(recorded.samples.iter().all(|s| s.ledger_error.abs() < 1e-6));
    let (before, lake, _) = &logs[0];
    assert!(lake.is_some(), "the lake");
    // Rising, the lake takes the channel's cells, and the network is laid
    // again each time: the water on the cells it has taken is poured into
    // it, and the rest back into the channel. None leaves the stores.
    let relays = before
        .iter()
        .filter(|e| matches!(e, TopologyEdit::Cleared(_)))
        .count();
    assert!(relays >= 2, "laid again {relays} times");
    for edit in before {
        if let TopologyEdit::Poured { into, .. } = edit {
            assert!(matches!(into, Account::Store(_)), "{edit:?}");
        }
    }
    // Before the blast its level climbs smoothly: no river's storage is
    // dumped into it at once. The channel's flat shelf is drowned whole in
    // one rise, and the 2 m³ running over it joins the lake together, 1.4 cm
    // of it; the whole river's 15 m³ would be several times that.
    let rising: Vec<f32> = recorded
        .samples
        .iter()
        .filter(|s| s.time > 20.0 && s.time < 179.0)
        .filter_map(|s| s.probes[0])
        .collect();
    let jump = rising
        .windows(2)
        .map(|w| (w[1] - w[0]).abs())
        .fold(0.0f32, f32::max);
    assert!(jump < 0.02, "the lake jumped {jump} m");
    // Drained, the lake leaves the channel's bed dry, and the channel runs
    // on down it.
    assert!(logs[0].2.is_none(), "the lake stood over the bed");
    let (_, _, bed) = &logs[1];
    assert!(bed.is_some(), "no channel runs over the drained bed");
    // It is laid again as the falling lake bares each cell of the bed, not
    // on every tick: a fall landing on the dry bank above the water names
    // the water's edge as its shore, not the bank it landed on.
    let mut relaid: Vec<f32> = recorded
        .events
        .iter()
        .filter(|e| e.time > 180.0 && e.text.starts_with("Cleared"))
        .map(|e| e.time)
        .collect();
    relaid.dedup();
    assert!(
        relaid.len() < 60,
        "laid again {} times draining",
        relaid.len()
    );
}

#[test]
fn every_level_s_water_comes_to_rest_when_it_opens() {
    for entry in std::fs::read_dir("levels").unwrap() {
        let path = entry.unwrap().path();
        if !path.to_string_lossy().ends_with(".level.ron") {
            continue;
        }
        let level = load_level(&path).unwrap();
        let Some(config) = level.water.as_ref() else {
            continue;
        };
        let terrain = build_terrain(&level);
        let (water, _) = WaterWorld::from_config(config, &terrain);
        if let Some(report) = water.steady_report() {
            assert!(report.converged, "{}: {report:?}", path.display());
        }
        assert!(water.balance().is_balanced(), "{}", path.display());
    }
}

#[test]
fn a_weir_between_two_basins_draws_a_sheet_only_while_there_is_a_step() {
    let scenario = find("spill_merge").unwrap();
    let times: Vec<f32> = (1..36).map(|i| i as f32 * 5.0).collect();
    // At each capture: the drawn step across the pond's weir into the pit,
    // if the two are still apart.
    let mut steps: Vec<Option<f32>> = Vec::new();
    run_with_captures(&scenario, RunConfig::default(), &times, |_, _, water| {
        let weir = water
            .network()
            .links()
            .find(|(_, l)| l.law.reversible() && l.fall.is_some());
        steps.push(weir.and_then(|(id, l)| {
            let heights = water.link_interface(id)?;
            let (_, arc) = l.side(if heights.back { -1.0 } else { 1.0 });
            Some(if arc.is_some() { heights.step() } else { 0.0 })
        }));
    })
    .unwrap();
    let first = steps
        .iter()
        .position(|s| s.is_some())
        .expect("the weir was laid");
    // While the pit fills from dry, water falls into it...
    assert!(steps[first].is_some_and(|s| s > 0.3), "{steps:?}");
    // ...and the step shrinks as the pit rises, to nothing by the time the
    // two are one lake.
    let drawn: Vec<f32> = steps.iter().flatten().copied().collect();
    assert!(drawn.windows(2).all(|w| w[1] <= w[0] + 0.02), "{steps:?}");
    assert!(
        steps.last().unwrap().is_none_or(|s| s <= STEP_EPSILON),
        "{steps:?}"
    );
}

#[test]
fn a_blast_under_the_water_a_fall_lands_in_leaves_the_river_above_it() {
    let level = load_level(Path::new("levels/water_park.level.ron")).unwrap();
    let mut terrain = build_terrain(&level);
    let (mut water, _) = WaterWorld::recording(level.water.as_ref().unwrap(), &terrain);
    let lake = water
        .query()
        .sample(Point3::new(11.0, 2.0, 19.0))
        .unwrap()
        .body;
    let (link, reach) = water
        .network()
        .links()
        .find(|(_, l)| l.down == lake && l.fall.is_some())
        .map(|(id, l)| (id, l.up))
        .expect("the river falls into the catch lake");
    let landing = water
        .network()
        .link(link)
        .unwrap()
        .fall
        .as_ref()
        .unwrap()
        .landing()
        .unwrap();
    let _ = reach;
    // The river falling into the lake, and the links along it, by where
    // they stand: every reach above the lake's fall link.
    let river = |water: &WaterWorld| -> Vec<String> {
        let network = water.network();
        let lake = water
            .query()
            .sample(Point3::new(11.0, 2.0, 19.0))
            .unwrap()
            .body;
        let mut above: Vec<StoreId> = network
            .links()
            .filter(|(_, l)| l.down == lake && l.fall.is_some())
            .map(|(_, l)| l.up)
            .collect();
        let mut cursor = 0;
        while cursor < above.len() {
            let id = above[cursor];
            cursor += 1;
            above.extend(
                network
                    .links()
                    .filter(|(_, l)| l.down == id)
                    .map(|(_, l)| l.up)
                    .filter(|up| network.store(*up).is_some_and(|s| s.as_reach().is_some())),
            );
        }
        let keys: Vec<String> = above
            .iter()
            .filter_map(|id| network.store(*id)?.as_reach())
            .map(|r| format!("reach {:?}", r.cells.first().map(|c| c.column)))
            .collect();
        fingerprint(water)
            .into_iter()
            .filter(|l| keys.iter().any(|k| l.starts_with(&format!("{k}:"))))
            .collect()
    };
    let before = river(&water);
    // A crater in the lake bed just past where the fall comes down: its
    // columns reach under the arc, but only where the arc runs through the
    // lake.
    let fall = water.network().link(link).unwrap().fall.clone().unwrap();
    let (centre, radius) = (Point3::new(landing.x, landing.y, landing.z - 1.0), 0.8);
    let surface = water.level(lake).unwrap();
    let under = |p: &Point3<f32>| (p.z - centre.z).abs() <= radius + 0.5;
    assert!(fall.points.iter().any(under), "the crater misses the arc");
    assert!(fall
        .points
        .iter()
        .filter(|p| under(p))
        .all(|p| p.y < surface));
    terrain.detonate(centre, &BlastConfig::fixed_radius(radius));
    terrain.update();
    water.on_terrain_update(&terrain);
    // Laid again, the river stands as it did, water and all, and falls
    // along the same arc into the lake.
    assert_eq!(river(&water), before, "the blast reached the river");
    let lake = water
        .query()
        .sample(Point3::new(11.0, 2.0, 19.0))
        .unwrap()
        .body;
    let after = water
        .network()
        .links()
        .find(|(_, l)| l.down == lake && l.fall.is_some())
        .map(|(_, l)| l)
        .expect("the river still falls into the lake");
    assert!(water
        .network()
        .store(after.up)
        .unwrap()
        .as_reach()
        .is_some());
    // Under the water an edit is not in the fall's air: traced again, the
    // arc comes down where it did, but for the throw of the flow it carries
    // now rather than the flow it was first traced at, as the level opened.
    let again = after.fall.as_ref().and_then(FallPath::landing).unwrap();
    assert!((again - landing).norm() < 0.5, "{landing:?} -> {again:?}");
    for _ in 0..60 {
        water.step(1.0 / 60.0);
    }
    assert!(water.balance().is_balanced());
}

#[test]
fn a_river_ending_a_little_over_the_sea_falls_into_it() {
    let scenario = find("low_mouth").unwrap();
    let mut seen = None;
    run_with_captures(&scenario, RunConfig::default(), &[5.0], |_, _, water| {
        let sea = water.ocean().map(|(id, _)| id).expect("the sea");
        seen = water
            .network()
            .links()
            .find(|(_, l)| l.down == sea && l.fall.is_some())
            .and_then(|(id, l)| {
                let reach = water.network().store(l.up)?.as_reach()?;
                let end = reach.surface_at(reach.length, water.reach_ends(l.up));
                Some((water.link_interface(id)?, end))
            });
    })
    .unwrap();
    let (heights, end) = seen.expect("the river falls into the sea");
    // The river's end stands at its brink over the 0.3 m lip, and the sheet
    // runs from there down to the sea.
    assert!((heights.lip - 0.3).abs() < 0.15, "{heights:?}");
    assert!(heights.free() && heights.step() > 0.05, "{heights:?}");
    assert_eq!(heights.lower, 0.0);
    assert!(
        (heights.upper - end).abs() < 0.01,
        "sheet at {}, river at {end}",
        heights.upper
    );
}

#[test]
fn a_channel_laid_again_as_water_rises_over_it_lets_out_what_it_did_before() {
    let scenario = find("shoreline").unwrap();
    let dt = 1.0 / 60.0;
    let times: Vec<f32> = (0..6000).map(|i| 60.0 + i as f32 * dt).collect();
    // Each tick: how many times the lake has been laid again, and what the
    // channel lets out into it.
    let mut ticks: Vec<(usize, f64, f32)> = Vec::new();
    run_with_captures(&scenario, RunConfig::default(), &times, |t, _, water| {
        let network = water.network();
        let lake = water
            .query()
            .sample(Point3::new(4.0, 6.2, 0.0))
            .map(|s| s.body)
            .filter(|b| network.store(*b).and_then(Store::as_basin).is_some());
        let relays = water
            .topology_log()
            .iter()
            .filter(|e| matches!(e, TopologyEdit::Cleared(id) if Some(*id) == lake))
            .count();
        let into = network
            .links()
            .filter(|(_, l)| Some(l.down) == lake)
            .filter_map(|(_, l)| network.store(l.up)?.as_reach())
            .map(|r| r.outflow)
            .sum();
        ticks.push((relays, into, t));
    })
    .unwrap();
    let mut checked = 0;
    for w in ticks.windows(2) {
        if w[1].0 == w[0].0 {
            continue;
        }
        let (before, after) = (w[0].1, w[1].1);
        assert!(
            (after - before).abs() < 0.05 * before.max(0.1),
            "outflow {before} -> {after} across laying the channel again at {} s",
            w[1].2
        );
        checked += 1;
    }
    assert!(checked >= 2, "laid again {checked} times in the window");
}

#[test]
fn a_breached_dam_pours_down_its_spillway_as_a_river() {
    let level = load_level(Path::new("levels/water_park.level.ron")).unwrap();
    let mut terrain = build_terrain(&level);
    let (mut water, _) = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
    terrain.detonate(Point3::new(-48.3, 7.5, 40.0), &BlastConfig::default());
    terrain.update();
    water.on_terrain_update(&terrain);
    for _ in 0..(60 * 20) {
        water.step(1.0 / 60.0);
    }
    let query = water.query();
    let reservoir = query.level_at(Point3::new(-62.0, 5.0, 40.0)).unwrap();
    assert!(reservoir < 8.3, "the reservoir stands at {reservoir}");
    // Its water runs as a river down the spillway, from the breach to the
    // pit, wetted along its length...
    let spillway: Vec<_> = water
        .network()
        .stores()
        .filter_map(|(_, s)| s.as_reach())
        .filter(|r| {
            r.centreline
                .points
                .iter()
                .all(|p| p.x > -50.0 && p.x < -25.0 && (p.z - 40.0).abs() < 6.0)
        })
        .collect();
    let length: f32 = spillway.iter().map(|r| r.length).sum();
    assert!(length > 12.0, "{} m of river down the spillway", length);
    assert!(spillway
        .iter()
        .all(|r| r.inflow > 1.0 && r.front >= r.length));
    // ...into the pit, filling.
    let pit = query.level_at(Point3::new(-29.0, -1.5, 40.0));
    assert!(pit.is_some_and(|l| l > -1.5), "the pit stands at {pit:?}");
    assert!(water.balance().is_balanced());
}

#[test]
fn a_lake_rising_past_its_cap_keeps_the_river_leaving_it() {
    // From 140 s the lake spills through the notch and goes on rising, which
    // re-floods it at its cap every few centimetres until the blast at 180 s.
    let scenario = find("shoreline").unwrap();
    let dt = 1.0 / 60.0;
    let times: Vec<f32> = (0..2100).map(|i| 145.0 + i as f32 * dt).collect();
    let refloods_seen = |water: &WaterWorld| {
        water
            .topology_log()
            .iter()
            .filter(|e| matches!(e, TopologyEdit::Reregion { .. }))
            .count()
    };
    // The ticks that re-flood it. Measuring every surface on every tick costs
    // far more than the water does, so a first run only finds them.
    let mut counts = Vec::new();
    run_with_captures(&scenario, RunConfig::default(), &times, |_, _, water| {
        counts.push(refloods_seen(water));
    })
    .unwrap();
    let refloods: Vec<usize> = (1..counts.len())
        .filter(|&i| counts[i] != counts[i - 1])
        .collect();
    assert!(refloods.len() >= 2, "{} re-floods", refloods.len());
    // Each re-flood lays the network again, and the water, the weir's over
    // the notch and the river's below it, stands where it did the tick
    // before.
    let around: Vec<f32> = refloods
        .iter()
        .flat_map(|&i| [times[i - 1], times[i]])
        .collect();
    let mut seen = Vec::new();
    run_with_captures(&scenario, RunConfig::default(), &around, |t, _, water| {
        seen.push((t, refloods_seen(water), surfaces(water)));
    })
    .unwrap();
    let mut moved = Vec::new();
    for pair in seen.chunks(2) {
        let [(_, before, previous), (t, after, now)] = pair else {
            panic!("a re-flood was not seen with the tick before it");
        };
        assert_ne!(before, after, "no re-flood between the ticks at {t:.2} s");
        for ((at, a), (_, b)) in previous.iter().zip(now) {
            let off = match (a, b) {
                (Some(a), Some(b)) => (a - b).abs() > 0.02,
                (None, None) => false,
                _ => true,
            };
            if off {
                moved.push(format!("{t:.2} s at {at:?}: {a:?} -> {b:?}"));
            }
        }
    }
    assert!(moved.is_empty(), "{:#?}", &moved[..moved.len().min(8)]);
}

#[test]
fn a_tributary_joins_a_river_partway_down_it() {
    let scenario = find("confluence").unwrap();
    let mut logs = Vec::new();
    let mut joined = None;
    let recorded = run_with_captures(&scenario, RunConfig::default(), &[59.9], |_, _, water| {
        let network = water.network();
        let is_reach = |id: StoreId| network.store(id).is_some_and(|s| s.as_reach().is_some());
        // The reach both channels run into: fed by two reaches.
        joined = network.stores().find_map(|(id, s)| {
            let reach = s.as_reach()?;
            let feeders = network
                .links()
                .filter(|(_, l)| l.down == id && is_reach(l.up))
                .count();
            (feeders == 2).then_some(reach.inflow)
        });
        logs.push(water.topology_log().to_vec());
    })
    .unwrap();
    // Both springs' 1.5 m³/s run on down the river below the junction.
    let inflow = joined.expect("a reach fed by both channels");
    assert!((inflow - 1.5).abs() < 0.05, "{inflow}");
    // And reach the pit, every drop accounted for.
    let pit = recorded.samples.last().unwrap().probes[0];
    assert!(pit.is_some_and(|l| l > 3.2), "{pit:?}");
    assert!(recorded.samples.iter().all(|s| s.ledger_error.abs() < 1e-6));
    // Nothing was laid twice.
    let removed = logs[0]
        .iter()
        .filter(|e| matches!(e, TopologyEdit::RemoveStore { .. }))
        .count();
    assert_eq!(removed, 0, "{:?}", logs[0]);
}

/// The network described by where its stores stand, not by their ids: two
/// networks laid over the same ground holding the same water describe alike.
fn fingerprint(water: &WaterWorld) -> Vec<String> {
    let network = water.network();
    let graph = water.geometry().graph();
    let key = |id: StoreId| match network.store(id) {
        Some(Store::Basin(b)) => {
            let at = b
                .region
                .iter()
                .map(|r| (r.span.column, r.span.ordinal))
                .min();
            format!("basin {at:?}")
        }
        Some(Store::Reach(r)) => format!("reach {:?}", r.cells.first().map(|c| c.column)),
        Some(Store::Ocean(_)) => "ocean".to_string(),
        Some(Store::Sink) => "sink".to_string(),
        Some(Store::Reservoir) => format!("reservoir {}", id.0),
        None => "gone".to_string(),
    };
    let mut lines: Vec<String> = network
        .stores()
        .filter_map(|(id, s)| match s {
            Store::Basin(b) => Some(format!(
                "{}: level {:.4} volume {:.4} spans {} outflows {}",
                key(id),
                b.level(),
                b.volume,
                b.region.len(),
                b.outflows.len()
            )),
            Store::Reach(r) => Some(format!(
                "{}: cells {} to {:?} storage {:.4} {:?} front {:.2} tail {:.2} design {:.3} floor {:.3}",
                key(id),
                r.cells.len(),
                r.cells.last().map(|c| c.column),
                r.storage,
                r.state,
                r.front,
                r.tail,
                r.rating.design(),
                r.cells.first().map_or(0.0, |c| graph.span(*c).floor_c),
            )),
            _ => None,
        })
        .collect();
    lines.extend(network.links().map(|(_, l)| {
        format!(
            "link {} -> {} open {} fall {}",
            key(l.up),
            key(l.down),
            l.open,
            l.fall.is_some()
        )
    }));
    lines.sort();
    lines
}

/// The still or running surface over every span's floor, where water
/// stands: what the player sees, without swell or ripples.
fn surfaces(water: &WaterWorld) -> Vec<(Point3<f32>, Option<f32>)> {
    let graph = water.geometry().graph();
    let (min, max) = graph.column_bounds();
    let query = water.query();
    let mut out = Vec::new();
    for k in min.k..=max.k {
        for i in min.i..=max.i {
            let column = crate::water::geometry::Column::new(i, k);
            let (x, z) = column.centre();
            for span in graph.refs(column) {
                let at = Point3::new(x, graph.span(span).floor_min + 0.01, z);
                out.push((at, query.level_at(at)));
            }
        }
    }
    out
}

/// Scenarios where what the settle keeps inside a hysteresis band differs
/// from what a network laid afresh starts from, so that laid again their
/// water stands a little differently; laid twice, it must not:
///
/// - river: a draining lake's outflow is linked once its flow passes twice
///   `Q_RETIRE` and kept until it drops below `Q_RETIRE`; laid again in
///   between, it is not linked;
/// - shoreline: a channel is laid down to the first span water stands over,
///   and laid again only once the water stands `DROWN_MARGIN` over a cell's
///   highest floor; in between, laid again it is a cell shorter, and cut
///   into reaches differently;
/// - river_diversion, river_blast: a fall's arc is traced at the flow when
///   it was laid; laid again at the flow now, it lands a cell over.
const SETTLE_HISTORY: [&str; 4] = ["river", "shoreline", "river_diversion", "river_blast"];

#[test]
fn laying_the_network_again_over_unchanged_ground_leaves_the_water_where_it_was() {
    let mut failures = Vec::new();
    for scenario in super::scenarios::catalogue() {
        let times = [0.3 * scenario.duration, 0.9 * scenario.duration];
        run_with_captures(&scenario, RunConfig::default(), &times, |t, _, water| {
            let volume = water.volume();
            let seen = surfaces(water);
            water.rebuild();
            let once = fingerprint(water);
            let moved: Vec<String> = seen
                .iter()
                .zip(surfaces(water))
                .filter(|((_, a), (_, b))| match (a, b) {
                    (Some(a), Some(b)) => (a - b).abs() > 0.02,
                    (None, None) => false,
                    _ => true,
                })
                .map(|((at, a), (_, b))| {
                    format!("({:.1}, {:.1}, {:.1}) {a:?} -> {b:?}", at.x, at.y, at.z)
                })
                .collect();
            let history = SETTLE_HISTORY.contains(&scenario.name);
            if (!moved.is_empty() && !history) || (water.volume() - volume).abs() > 1e-9 {
                failures.push(format!(
                    "{} at {t:.1} s: volume {volume:.6} -> {:.6}; {} points moved, e.g. {:#?}",
                    scenario.name,
                    water.volume(),
                    moved.len(),
                    &moved[..moved.len().min(6)]
                ));
            }
            water.rebuild();
            let twice = fingerprint(water);
            if once != twice {
                let gone: Vec<_> = once.iter().filter(|l| !twice.contains(l)).collect();
                let new: Vec<_> = twice.iter().filter(|l| !once.contains(l)).collect();
                failures.push(format!(
                    "{} at {t:.1} s, laid twice:\n  once: {gone:#?}\n  twice: {new:#?}",
                    scenario.name
                ));
            }
        })
        .unwrap();
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_crater_in_a_river_fills_while_the_river_below_it_runs_on() {
    let scenario = find("river_blast").unwrap();
    // Water in reaches above the crater, and below it, and the most any
    // reach below it lets out, at each capture.
    let mut seen = Vec::new();
    let recorded = run_with_captures(
        &scenario,
        RunConfig::default(),
        &[9.9, 10.05, 15.0, 20.0],
        |_, _, water| {
            let (mut above, mut below, mut running) = (0.0, 0.0, 0.0f64);
            for (_, store) in water.network().stores() {
                let Some(reach) = store.as_reach() else {
                    continue;
                };
                let x = reach.centreline.points[0].x;
                if x < -9.0 {
                    above += reach.storage;
                } else if x > -3.0 {
                    below += reach.storage;
                    running = running.max(reach.outflow);
                }
            }
            seen.push((above, below, running));
        },
    )
    .unwrap();
    let (above, below, _) = seen[0];
    // Laid again over the new ground, the river above the crater holds the
    // water it held, but for what stood over the crater's columns, now the
    // crater's; the river below it holds its own. None is laid again empty,
    // and none is poured into the crater from afar.
    let (after_above, after_below, _) = seen[1];
    assert!(
        after_above > 0.8 * above && after_above <= above + 1e-9,
        "{above} -> {after_above}"
    );
    assert!(after_below > 0.9 * below, "{below} -> {after_below}");
    // While the crater fills, the river below it drains on: water still
    // runs down it seconds later.
    for &(_, _, running) in &seen[1..] {
        assert!(running > 0.3, "the river below stopped: {running}");
    }
    // Fed by the river above, still running at the blast, the crater is
    // near its lip, 7.9 m, within 20 s: the 50 m³ it takes at 1 m³/s, less
    // what stood over it.
    let crater = recorded
        .samples
        .iter()
        .find(|s| s.time >= 30.0)
        .and_then(|s| s.probes[1]);
    assert!(crater.is_some_and(|l| l > 7.5), "{crater:?}");
    assert!(recorded.samples.iter().all(|s| s.ledger_error.abs() < 1e-6));
}

#[test]
fn a_river_falls_from_where_its_water_runs_as_wide_as_it_runs() {
    let level = load_level(Path::new("levels/water_park.level.ron")).unwrap();
    let terrain = build_terrain(&level);
    let (water, _) = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
    let graph = water.geometry().graph();
    let mut checked = 0;
    for (id, link) in water.network().links() {
        let Some(reach) = water.network().store(link.up).and_then(Store::as_reach) else {
            continue;
        };
        if link.fall.is_none() {
            continue;
        }
        // The water the river is drawn with over its last cell: the ground
        // across it that lies under the surface it runs at, within the
        // width its strip is drawn to either side of its line.
        let surface = reach.surface_at(reach.length, water.reach_ends(link.up));
        let direction = link.lip.direction;
        let across = nalgebra::Vector2::new(-direction.y, direction.x);
        let inside = link.lip.at - nalgebra::Vector3::new(direction.x, 0.0, direction.y) * 0.25;
        let end = reach.centreline.points.last().unwrap();
        let line = (end.x - inside.x) * across.x + (end.z - inside.z) * across.y;
        let drawn = reach.rating.at(reach.rating.design()).top_width * 0.5 + 0.5;
        let wet: Vec<f32> = (-24..=24)
            .map(|i| i as f32 * 0.25)
            .filter(|s| (s - line).abs() <= drawn)
            .filter(|s| {
                let p = inside + nalgebra::Vector3::new(across.x, 0.0, across.y) * *s;
                crate::water::network::floor_near(graph, p.x, p.z, inside.y)
                    .is_some_and(|(_, floor)| floor < surface)
            })
            .collect();
        // The run of it the lip stands in.
        let Some(&at) = wet.iter().min_by(|a, b| a.abs().total_cmp(&b.abs())) else {
            panic!("link {id:?}: no water over the lip");
        };
        let (mut lo, mut hi) = (at, at);
        while wet.contains(&(lo - 0.25)) {
            lo -= 0.25;
        }
        while wet.contains(&(hi + 0.25)) {
            hi += 0.25;
        }
        // The river's line runs to the lip the sheet leaves from, and the
        // lip stands in its water, with water either side: laid from the
        // walked cell, the fall into the catch lake left from the edge of
        // its water, a metre off its middle.
        assert!(
            line.abs() <= 0.25,
            "link {id:?}: the river ends {line} m off the lip"
        );
        assert!(
            lo < 0.0 && hi > 0.0,
            "link {id:?}: the lip stands at the edge of {lo}..{hi}"
        );
        // And the sheet covers the water it leaves over, as wide as it runs.
        let q = water.link_discharge(id).unwrap();
        let half = 0.5 * water.fall_width(id, q, false).unwrap();
        let (top, bottom) = (hi + 0.125, lo - 0.125);
        let overlap = (half.min(top) - (-half).max(bottom)).max(0.0);
        let union = half.max(top) - (-half).min(bottom);
        assert!(
            overlap / union >= 0.5,
            "link {id:?}: a sheet ±{half} over water {lo}..{hi}"
        );
        checked += 1;
    }
    assert!(checked >= 3, "{checked} falls from rivers");
}
