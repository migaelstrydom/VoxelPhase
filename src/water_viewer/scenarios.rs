//! The catalogue of things worth watching water do.
//!
//! Each scenario is the smallest terrain that shows one behaviour, so a report
//! reads as that behaviour and nothing else. The acceptance scenarios of the
//! hydrology design (§9.3) live here as they come online.

use nalgebra::Point3;

use super::scenario::{Action, Beat, Probe, Scenario};

/// Every scenario, in the order they are worth reading.
pub fn catalogue() -> Vec<Scenario> {
    vec![breach(), island_pool(), crater_lake(), staircase()]
}

pub fn find(name: &str) -> Option<Scenario> {
    catalogue().into_iter().find(|s| s.name == name)
}

/// Scenarios matching a name, or every scenario for "all".
pub fn select(name: &str) -> Vec<Scenario> {
    if name == "all" {
        return catalogue();
    }
    if let Some(exact) = find(name) {
        return vec![exact];
    }
    catalogue()
        .into_iter()
        .filter(|s| s.name.contains(name))
        .collect()
}

/// A pond behind a dam on a hillside. The dam is blown out at 2 s.
///
/// The ground falls 12 m across the 64 m map along +x. A bowl centred at
/// x = −10 holds the pond; a 3 m wall across its downhill rim at x = 4 is the
/// dam, and lifts the lowest rim to about −2.7. The pond stands at −2.8. The
/// charge cuts a 3 m crater through the dam's middle, down past the pond's
/// surface and out to the hillside below it.
fn breach() -> Scenario {
    Scenario {
        name: "breach",
        description: "a pond's dam is blown out; it drains down the hill",
        level: BREACH_LEVEL,
        duration: 60.0,
        beats: vec![Beat {
            at: 2.0,
            action: Action::Blast {
                centre: Point3::new(4.0, -1.5, 0.0),
                radius: 3.0,
            },
        }],
        probes: vec![
            Probe {
                name: "pond",
                at: Point3::new(-10.0, -2.0, 0.0),
            },
            Probe {
                name: "notch",
                at: Point3::new(4.0, -3.0, 0.0),
            },
            Probe {
                name: "hill",
                at: Point3::new(16.0, -5.0, 0.0),
            },
        ],
        camera: (Point3::new(18.0, 8.0, 18.0), Point3::new(-4.0, -3.0, 0.0)),
    }
}

const BREACH_LEVEL: &str = r#"
Level(
    name: "water_viewer: breach",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Ramp(from: (-32.0, 0.0), to: (32.0, 0.0), start_height: 4.0, end_height: -8.0, width: 400.0),
                Crater(center: (-10.0, 0.0), radius: 14.0, depth: 5.0),
                Wall(from: (4.0, -10.0), to: (4.0, 10.0), height: 3.0, thickness: 3.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (20.0, 0.0, 20.0),
    water: Some((
        bodies: [Pool(seed: (-10.0, 0.0), surface_level: -2.8)],
    )),
)
"#;

/// A pool in a trough on top of a floating island, over a pond on the ground.
///
/// Two bodies of water share every column under the island. A heightfield
/// holds one per column, so this is the scenario that shows why spans exist.
fn island_pool() -> Scenario {
    Scenario {
        name: "island_pool",
        description: "a pool on a floating island above a pond; two waters per column",
        level: ISLAND_POOL_LEVEL,
        duration: 20.0,
        beats: Vec::new(),
        probes: vec![
            Probe {
                name: "island",
                at: Point3::new(0.0, 7.5, 0.0),
            },
            Probe {
                name: "pond",
                at: Point3::new(0.0, -2.0, 0.0),
            },
            Probe {
                name: "pond_open",
                at: Point3::new(10.0, -2.0, 0.0),
            },
        ],
        camera: (Point3::new(22.0, 14.0, 22.0), Point3::new(0.0, 2.0, 0.0)),
    }
}

const ISLAND_POOL_LEVEL: &str = r#"
Level(
    name: "water_viewer: island_pool",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [
                Crater(center: (0.0, 0.0), radius: 20.0, depth: 6.0),
            ],
            volumes: [
                Island(center: (0.0, 6.0, 0.0), half_extents: (8.0, 2.0, 8.0), edge_noise: 0.0),
                Tunnel(center: (0.0, 0.0), direction: (1.0, 0.0), length: 6.0, radius: 2.5, depth: 8.5),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (20.0, 0.0, 20.0),
    water: Some((
        ocean_level: None,
        bodies: [
            Pool(seed: (12.0, 0.0), surface_level: -1.5),
            Pool(seed: (0.0, 0.0), surface_level: 7.5),
        ],
    )),
)
"#;

/// A stepped channel: eight 7 m treads, each 1.25 m below the last, cut 4 m
/// wide into banks at 12 m. Water run down it falls from tread to tread and
/// leaves the map at its low end.
fn staircase() -> Scenario {
    Scenario {
        name: "staircase",
        description: "a stepped channel of eight treads, open at its low end",
        level: STAIRCASE_LEVEL,
        duration: 30.0,
        beats: Vec::new(),
        probes: vec![
            Probe {
                name: "top",
                at: Point3::new(-24.0, 10.5, 0.0),
            },
            Probe {
                name: "middle",
                at: Point3::new(3.0, 5.5, 0.0),
            },
            Probe {
                name: "bottom",
                at: Point3::new(24.0, 1.8, 0.0),
            },
        ],
        camera: (Point3::new(0.0, 20.0, 24.0), Point3::new(0.0, 5.0, 0.0)),
    }
}

const STAIRCASE_LEVEL: &str = r#"
Level(
    name: "water_viewer: staircase",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [
                Plateau(min: (-40.0, -40.0), max: (40.0, 40.0), height: 12.0),
                Plateau(min: (-28.0, -2.0), max: (-21.0, 2.0), height: 10.0),
                Plateau(min: (-21.0, -2.0), max: (-14.0, 2.0), height: 8.75),
                Plateau(min: (-14.0, -2.0), max: (-7.0, 2.0), height: 7.5),
                Plateau(min: (-7.0, -2.0), max: (0.0, 2.0), height: 6.25),
                Plateau(min: (0.0, -2.0), max: (7.0, 2.0), height: 5.0),
                Plateau(min: (7.0, -2.0), max: (14.0, 2.0), height: 3.75),
                Plateau(min: (14.0, -2.0), max: (21.0, 2.0), height: 2.5),
                Plateau(min: (21.0, -2.0), max: (40.0, 2.0), height: 1.25),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 13.0, 10.0),
    water: Some((
        ocean_level: None,
        bodies: [],
    )),
)
"#;

/// A pond in a bowl on flat ground, with a crater blown in its floor at 2 s.
///
/// The crater is under water and inside the pond's region: it merges at once,
/// and the level drops by the crater's volume over the pond's area. Nothing
/// flows anywhere (§9.3, scenario 6).
fn crater_lake() -> Scenario {
    Scenario {
        name: "crater_lake",
        description: "a crater is blown in a pond's floor; the level drops, nothing flows",
        level: CRATER_LAKE_LEVEL,
        duration: 10.0,
        beats: vec![Beat {
            at: 2.0,
            action: Action::Blast {
                centre: Point3::new(3.0, -3.5, 0.0),
                radius: 2.0,
            },
        }],
        probes: vec![
            Probe {
                name: "pond",
                at: Point3::new(-4.0, -2.0, 0.0),
            },
            Probe {
                name: "crater",
                at: Point3::new(3.0, -4.0, 0.0),
            },
        ],
        camera: (Point3::new(16.0, 10.0, 16.0), Point3::new(0.0, -3.0, 0.0)),
    }
}

const CRATER_LAKE_LEVEL: &str = r#"
Level(
    name: "water_viewer: crater_lake",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Crater(center: (0.0, 0.0), radius: 14.0, depth: 5.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (20.0, 1.0, 20.0),
    water: Some((
        bodies: [Pool(seed: (0.0, 0.0), surface_level: -1.5)],
    )),
)
"#;
