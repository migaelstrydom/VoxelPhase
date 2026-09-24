//! The catalogue of things worth watching water do.
//!
//! Each scenario is the smallest terrain that shows one behaviour, so a report
//! reads as that behaviour and nothing else. The acceptance scenarios of the
//! hydrology design (§9.3) live here as they come online.

use nalgebra::Point3;

use super::scenario::{Action, Beat, Probe, Scenario};

/// Every scenario, in the order they are worth reading.
pub fn catalogue() -> Vec<Scenario> {
    vec![
        breach(),
        spill_merge(),
        drain_split(),
        river(),
        island_pool(),
        island_hole(),
        crater_lake(),
        staircase(),
        spring_pools(),
    ]
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

/// A pond held against a dam in a walled channel on a hillside. The dam is
/// blown out at 2 s.
///
/// The ground falls 12 m across the 64 m map along +x. Two walls 12 m apart
/// make a channel down the hill, and a 3 m wall across it at x = 4 is the
/// dam. The pond stands against it at −1.0, reaching back up the channel to
/// x ≈ −5. The charge cuts a 2.5 m crater through the dam's middle, down past
/// the pond's surface and out to the hillside below.
fn breach() -> Scenario {
    Scenario {
        name: "breach",
        description: "a pond's dam is blown out; it drains down the hill",
        level: BREACH_LEVEL,
        duration: 120.0,
        beats: vec![Beat {
            at: 2.0,
            action: Action::Blast {
                centre: Point3::new(4.0, -1.5, 0.0),
                radius: 2.5,
            },
        }],
        probes: vec![
            Probe {
                name: "pond",
                at: Point3::new(0.0, -1.5, 0.0),
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
        camera: (Point3::new(18.0, 8.0, 16.0), Point3::new(0.0, -2.0, 0.0)),
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
                Wall(from: (-20.0, -6.0), to: (6.0, -6.0), height: 3.0, thickness: 3.0),
                Wall(from: (-20.0, 6.0), to: (6.0, 6.0), height: 3.0, thickness: 3.0),
                Wall(from: (4.0, -7.0), to: (4.0, 7.0), height: 3.0, thickness: 3.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (20.0, 0.0, 20.0),
    water: Some((
        bodies: [Pool(seed: (1.0, 0.0), surface_level: -1.0)],
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
/// wide into banks at 12 m. A spring in the head wall runs 1 m³/s down it:
/// the water falls from tread to tread and leaves the map at its low end.
fn staircase() -> Scenario {
    Scenario {
        name: "staircase",
        description: "a spring runs down a stepped channel, falling tread to tread",
        level: STAIRCASE_LEVEL,
        duration: 150.0,
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
        camera: (Point3::new(-31.0, 16.0, 7.0), Point3::new(-17.0, 8.0, 0.0)),
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
        bodies: [
            Spring(position: (-28.3, 10.9, 0.0), direction: (1.5, 0.0, 0.0), discharge: 1.0),
        ],
        settle: AsAuthored,
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

/// A trench split by a wall, water in the west half, the east half dry. A
/// notch is blown in the wall at 2 s, below the pond's surface and below
/// where the two halves would stand together: the pond spills into the dry
/// half until the levels meet, then the two merge (§7.2, Fill–Spill–Merge).
fn spill_merge() -> Scenario {
    Scenario {
        name: "spill_merge",
        description: "a notch lets a pond spill into a dry pit until the two merge",
        level: SPILL_MERGE_LEVEL,
        duration: 180.0,
        beats: vec![Beat {
            at: 2.0,
            action: Action::Blast {
                centre: Point3::new(0.0, -0.5, 0.0),
                radius: 2.0,
            },
        }],
        probes: vec![
            Probe {
                name: "west",
                at: Point3::new(-10.0, -2.0, 0.0),
            },
            Probe {
                name: "east",
                at: Point3::new(10.0, -2.0, 0.0),
            },
        ],
        camera: (Point3::new(0.0, 12.0, 22.0), Point3::new(0.0, -2.0, 0.0)),
    }
}

const SPILL_MERGE_LEVEL: &str = r#"
Level(
    name: "water_viewer: spill_merge",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Plateau(min: (-20.0, -4.0), max: (20.0, 4.0), height: -3.0),
                Wall(from: (0.0, -5.0), to: (0.0, 5.0), height: 3.5, thickness: 2.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 1.0, 12.0),
    water: Some((
        bodies: [Pool(seed: (-10.0, 0.0), surface_level: -1.5)],
    )),
)
"#;

/// The same trench, its divider lower, one lake over both halves. At 2 s the
/// trench's west end is blown open to a channel running off the map: the lake
/// drains, and once it falls below the divider the east half keeps its water
/// and the west half drains on alone (§8.1, Split).
fn drain_split() -> Scenario {
    Scenario {
        name: "drain_split",
        description: "a lake over a divider drains below it and splits in two",
        level: DRAIN_SPLIT_LEVEL,
        duration: 120.0,
        beats: vec![Beat {
            at: 2.0,
            action: Action::Blast {
                centre: Point3::new(-20.0, -1.0, 0.0),
                radius: 2.5,
            },
        }],
        probes: vec![
            Probe {
                name: "west",
                at: Point3::new(-10.0, -2.0, 0.0),
            },
            Probe {
                name: "east",
                at: Point3::new(10.0, -2.0, 0.0),
            },
        ],
        camera: (Point3::new(0.0, 12.0, 22.0), Point3::new(-4.0, -2.0, 0.0)),
    }
}

const DRAIN_SPLIT_LEVEL: &str = r#"
Level(
    name: "water_viewer: drain_split",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Plateau(min: (-40.0, -4.0), max: (20.0, 4.0), height: -3.0),
                Wall(from: (0.0, -5.0), to: (0.0, 5.0), height: 2.0, thickness: 2.0),
                Wall(from: (-20.0, -5.0), to: (-20.0, 5.0), height: 4.0, thickness: 2.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 1.0, 12.0),
    water: Some((
        bodies: [Pool(seed: (-10.0, 0.0), surface_level: -0.5)],
    )),
)
"#;

/// The island pool again, its floor blown through at 2 s: the pool drains
/// through an orifice onto the pond below, and the pond's region is
/// unchanged (§9.3, scenario 2).
fn island_hole() -> Scenario {
    Scenario {
        name: "island_hole",
        description: "an island pool's floor is blown through; it pours onto the pond below",
        level: ISLAND_POOL_LEVEL,
        duration: 60.0,
        beats: vec![Beat {
            at: 2.0,
            action: Action::Blast {
                centre: Point3::new(1.0, 5.2, 0.0),
                radius: 2.2,
            },
        }],
        ..island_pool()
    }
}

/// A lake at the head of a carved, meandering channel down a hillside,
/// standing above the channel's lip: it drains down the channel, whose front
/// advances reach by reach to the bottom of the map. The channel is the
/// sloped carved bed stage 4b re-runs the routing criteria on.
fn river() -> Scenario {
    Scenario {
        name: "river",
        description: "a lake drains down a meandering carved channel; the front advances",
        level: RIVER_LEVEL,
        duration: 90.0,
        beats: Vec::new(),
        probes: vec![
            Probe {
                name: "lake",
                at: Point3::new(-26.0, 4.0, 0.0),
            },
            Probe {
                name: "upper",
                at: Point3::new(-12.2, 1.5, 4.2),
            },
            Probe {
                name: "lower",
                at: Point3::new(7.8, -3.0, 0.8),
            },
        ],
        camera: (Point3::new(0.0, 26.0, 34.0), Point3::new(0.0, -1.0, 0.0)),
    }
}

const RIVER_LEVEL: &str = r#"
Level(
    name: "water_viewer: river",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Ramp(from: (-32.0, 0.0), to: (32.0, 0.0), start_height: 7.0, end_height: -7.0, width: 400.0),
                Crater(center: (-26.0, 0.0), radius: 7.0, depth: 3.0),
                Ramp(from: (-22.0, 0.0), to: (-15.0, 6.0), start_height: 3.3, end_height: 1.2, width: 7.0),
                Ramp(from: (-15.0, 6.0), to: (0.0, -4.0), start_height: 1.2, end_height: -2.0, width: 7.0),
                Ramp(from: (0.0, -4.0), to: (15.0, 5.0), start_height: -2.0, end_height: -5.3, width: 7.0),
                Ramp(from: (15.0, 5.0), to: (32.0, 0.0), start_height: -5.3, end_height: -9.0, width: 7.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 6.0, 20.0),
    water: Some((
        bodies: [Pool(seed: (-26.0, 0.0), surface_level: 4.4)],
    )),
)
"#;

/// A sky source over the west half of a divided trench, with a notch at the
/// east end running off the map. Opened steady, the west half has filled and
/// spilled, the two halves have merged into one lake over the divider, and
/// the lake passes the source's discharge out through the notch: the level
/// opens at rest, with nothing left to fill (§13, `settle: Steady`).
fn spring_pools() -> Scenario {
    Scenario {
        name: "spring_pools",
        description: "a sky source fills, spills and merges two pools; opened at rest",
        level: SPRING_POOLS_LEVEL,
        duration: 30.0,
        beats: Vec::new(),
        probes: vec![
            Probe {
                name: "west",
                at: Point3::new(-10.0, -2.0, 0.0),
            },
            Probe {
                name: "east",
                at: Point3::new(10.0, -2.0, 0.0),
            },
        ],
        camera: (Point3::new(0.0, 14.0, 24.0), Point3::new(4.0, -2.0, 0.0)),
    }
}

const SPRING_POOLS_LEVEL: &str = r#"
Level(
    name: "water_viewer: spring_pools",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Plateau(min: (-20.0, -4.0), max: (20.0, 4.0), height: -3.0),
                Wall(from: (0.0, -5.0), to: (0.0, 5.0), height: 1.5, thickness: 2.0),
                Ramp(from: (19.0, 0.0), to: (32.0, 0.0), start_height: -1.0, end_height: -4.0, width: 3.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 1.0, 12.0),
    water: Some((
        bodies: [SkySource(position: (-10.0, 6.0, 0.0), discharge: 0.5)],
    )),
)
"#;
