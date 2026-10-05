//! The catalogue of things worth watching water do.
//!
//! Each scenario is the smallest terrain that shows one behaviour, so a report
//! reads as that behaviour and nothing else. The acceptance scenarios of the
//! hydrology design (§9.3) live here as they come online.

use nalgebra::{Point3, UnitQuaternion, Vector3};

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
        island_ditch(),
        crater_lake(),
        staircase(),
        spring_pools(),
        crater_drain(),
        river_diversion(),
        spring_rock(),
        sea_wall(),
        shoreline(),
        low_mouth(),
        confluence(),
        river_blast(),
        river_dam(),
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

/// The staircase, opened running.
const STAIRCASE_STEADY_LEVEL: &str = r#"
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
        settle: Steady,
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
        probes: island_pool()
            .probes
            .into_iter()
            .chain([Probe {
                name: "trough",
                at: Point3::new(-2.0, 7.5, 0.0),
            }])
            .collect(),
        ..island_pool()
    }
}

/// The island pool again, a ditch blown from its end out through the
/// island's edge: the pool spills over the cliff face straight into the pond
/// below, along a fall.
fn island_ditch() -> Scenario {
    Scenario {
        name: "island_ditch",
        description: "a ditch is blown from an island pool through the island's edge; it spills over the cliff",
        level: ISLAND_POOL_LEVEL,
        duration: 12.0,
        beats: [3.5, 5.0, 6.5, 8.0]
            .into_iter()
            .enumerate()
            .map(|(i, x)| Beat {
                at: 1.0 + i as f32,
                action: Action::Blast {
                    centre: Point3::new(x, 8.0, 0.0),
                    radius: 1.5,
                },
            })
            .collect(),
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

/// A lake in a bowl, held by a dam across a channel cut down to the bowl's
/// floor. A crater is blown in the lake floor at 2 s and merges into the
/// lake. The dam is blown at 6 s: the lake drains below the crater's rim,
/// and the crater splits off and keeps its water (§9.3, scenario 6).
fn crater_drain() -> Scenario {
    Scenario {
        name: "crater_drain",
        description: "a crater merges into a lake; the lake drains and the crater keeps its water",
        level: CRATER_DRAIN_LEVEL,
        duration: 120.0,
        beats: vec![
            Beat {
                at: 2.0,
                action: Action::Blast {
                    centre: Point3::new(-6.0, -3.2, 0.0),
                    radius: 2.0,
                },
            },
            Beat {
                at: 6.0,
                action: Action::Blast {
                    centre: Point3::new(10.0, -3.0, 0.0),
                    radius: 3.0,
                },
            },
        ],
        probes: vec![
            Probe {
                name: "lake",
                at: Point3::new(-2.0, -4.5, 5.0),
            },
            Probe {
                name: "crater",
                at: Point3::new(-6.0, -4.5, 0.0),
            },
        ],
        camera: (Point3::new(14.0, 12.0, 20.0), Point3::new(0.0, -4.0, 0.0)),
    }
}

const CRATER_DRAIN_LEVEL: &str = r#"
Level(
    name: "water_viewer: crater_drain",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Crater(center: (0.0, 0.0), radius: 14.0, depth: 5.0),
                Ramp(from: (0.0, 0.0), to: (32.0, 0.0), start_height: -5.2, end_height: -12.0, width: 4.0),
                Wall(from: (10.0, -4.0), to: (10.0, 4.0), height: 8.0, thickness: 2.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (20.0, 1.0, 20.0),
    water: Some((
        bodies: [Pool(seed: (0.0, 0.0), surface_level: -2.5)],
    )),
)
"#;

/// The river scenario with a crater blown into its bed at 12 s, while it
/// runs. The channel is laid again: it now ends in the crater, which fills
/// as a new basin and spills, and the river runs on below it (§9.3,
/// scenario 3).
fn river_diversion() -> Scenario {
    let mut probes = river().probes;
    probes.push(Probe {
        name: "crater",
        at: Point3::new(-7.0, -0.5, 1.0),
    });
    Scenario {
        name: "river_diversion",
        description:
            "a crater is blown in a running river's bed; it fills, spills, and the river runs on",
        beats: vec![Beat {
            at: 12.0,
            action: Action::Blast {
                centre: Point3::new(-7.0, 0.6, 1.0),
                radius: 2.0,
            },
        }],
        probes,
        ..river()
    }
}

/// The staircase, opened running, with the rock around its spring blown
/// away at 5 s. The spring is anchored in space: it keeps flowing from the
/// same point, and its fall is traced again over the new ground (§9.3,
/// scenario 5).
fn spring_rock() -> Scenario {
    Scenario {
        name: "spring_rock",
        description: "the rock around a spring is blown away; it keeps flowing, its fall re-traced",
        level: STAIRCASE_STEADY_LEVEL,
        duration: 200.0,
        beats: vec![Beat {
            at: 5.0,
            action: Action::Blast {
                centre: Point3::new(-28.5, 10.5, 0.0),
                radius: 2.5,
            },
        }],
        ..staircase()
    }
}

/// A coast: the sea to the south at 0 m, and behind a strip of land a dry
/// lowland 2 m below it. The strip is blown through at 2 s: the sea floods
/// the lowland over a weir, and once it stands at sea level the lowland
/// joins the sea (§9.3, scenario 4).
fn sea_wall() -> Scenario {
    Scenario {
        name: "sea_wall",
        description: "a sea wall is breached; the lowland floods and joins the sea",
        level: SEA_WALL_LEVEL,
        duration: 120.0,
        beats: vec![Beat {
            at: 2.0,
            action: Action::Blast {
                centre: Point3::new(0.0, -0.5, -8.0),
                radius: 3.5,
            },
        }],
        probes: vec![
            Probe {
                name: "lowland",
                at: Point3::new(0.0, -1.5, 0.0),
            },
            Probe {
                name: "sea",
                at: Point3::new(0.0, -1.0, -20.0),
            },
        ],
        camera: (Point3::new(18.0, 12.0, 16.0), Point3::new(0.0, -1.0, -6.0)),
    }
}

const SEA_WALL_LEVEL: &str = r#"
Level(
    name: "water_viewer: sea_wall",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 2.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Plateau(min: (-40.0, -40.0), max: (40.0, -11.0), height: -6.0),
                Plateau(min: (-10.0, -5.0), max: (10.0, 5.0), height: -2.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 3.0, 12.0),
    water: Some((
        ocean: Some((level: 0.0, open_edges: [South])),
        bodies: [],
    )),
)
"#;

/// A spring runs down a channel into a pit that spills through a notch in
/// its east wall at 7.3 m. The channel's last 10 m widen into a flat-bottomed
/// fan, with a level shelf where it begins. The channel runs down to that line; below it the
/// bed is the pit's. The lake that fills it stands 0.4 m over the notch to
/// spill the spring's 1 m³/s, drowning the channel's last metres (§8.2,
/// Drowned). At 180 s the notch is blown deeper; the lake drains and the
/// channel is carried on down the bed it leaves dry (Exposed).
fn shoreline() -> Scenario {
    Scenario {
        name: "shoreline",
        description: "a lake climbs a river's channel, then drains down it again",
        level: SHORELINE_LEVEL,
        duration: 240.0,
        beats: vec![Beat {
            at: 180.0,
            action: Action::Blast {
                centre: Point3::new(9.5, 6.8, 0.0),
                radius: 2.0,
            },
        }],
        probes: vec![Probe {
            name: "lake",
            at: Point3::new(4.0, 6.2, 0.0),
        }],
        camera: (Point3::new(-6.0, 16.0, 16.0), Point3::new(0.0, 6.0, 0.0)),
    }
}

const SHORELINE_LEVEL: &str = r#"
Level(
    name: "water_viewer: shoreline",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [
                Plateau(min: (-40.0, -40.0), max: (11.0, 40.0), height: 12.0),
                // The channel, falling 3 m over 26 m to the pit's floor.
                Ramp(from: (-24.0, 0.0), to: (2.0, 0.0), start_height: 9.0, end_height: 6.0, width: 4.0, flat_width: 2.0),
                // Its last 10 m widen on the same slope, so a metre of it
                // holds less water there than above.
                Ramp(from: (-8.0, 0.0), to: (2.0, 0.0), start_height: 7.1538, end_height: 6.0, width: 9.0, flat_width: 6.0),
                // The pit, and a notch at 7.3 m through its 3 m east wall.
                Plateau(min: (0.0, -5.0), max: (8.0, 5.0), height: 6.0),
                Plateau(min: (8.0, -1.0), max: (11.0, 1.0), height: 7.3),
                // Low ground beyond the wall.
                Plateau(min: (11.0, -40.0), max: (40.0, 40.0), height: 2.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 13.0, 10.0),
    water: Some((
        bodies: [
            Spring(position: (-24.5, 9.6, 0.0), direction: (1.5, 0.0, 0.0), discharge: 1.0),
        ],
        settle: AsAuthored,
    )),
)
"#;

/// A spring runs down a channel whose bed ends 0.3 m over the sea, at the
/// top of a cliff. A drop that small is still a fall (§7.9): the river's
/// end stands at its brink and a short sheet reaches the sea.
fn low_mouth() -> Scenario {
    Scenario {
        name: "low_mouth",
        description: "a river ends 0.3 m over the sea and falls into it",
        level: LOW_MOUTH_LEVEL,
        duration: 10.0,
        beats: Vec::new(),
        probes: vec![Probe {
            name: "sea",
            at: Point3::new(6.0, -1.0, 0.0),
        }],
        camera: (Point3::new(6.0, 3.0, 6.0), Point3::new(-1.0, 0.0, 0.0)),
    }
}

const LOW_MOUTH_LEVEL: &str = r#"
Level(
    name: "water_viewer: low_mouth",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 2.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [
                Ramp(from: (-20.0, 0.0), to: (0.0, 0.0), start_height: 1.5, end_height: 0.3, width: 4.0, flat_width: 2.0),
                Plateau(min: (0.0, -40.0), max: (40.0, 40.0), height: -3.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 5.0, 10.0),
    water: Some((
        ocean: Some((level: 0.0, open_edges: [East])),
        bodies: [
            Spring(position: (-20.5, 1.9, 0.0), direction: (1.5, 0.0, 0.0), discharge: 1.0),
        ],
        settle: Steady,
    )),
)
"#;

/// Two springs run down two channels that meet: the tributary's walk ends on
/// the main river's reach partway down it, and the two run on together over
/// a 3 m fall into a pit.
fn confluence() -> Scenario {
    Scenario {
        name: "confluence",
        description: "a tributary joins a river partway down; both fall into a pit",
        level: CONFLUENCE_LEVEL,
        duration: 60.0,
        beats: Vec::new(),
        probes: vec![Probe {
            name: "pit",
            at: Point3::new(14.0, 3.2, 0.0),
        }],
        camera: (Point3::new(-10.0, 22.0, 18.0), Point3::new(-2.0, 6.0, -2.0)),
    }
}

const CONFLUENCE_LEVEL: &str = r#"
Level(
    name: "water_viewer: confluence",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [
                Plateau(min: (-40.0, -40.0), max: (40.0, 40.0), height: 12.0),
                // The tributary, falling from 11 m to the river's bed where
                // the two meet at x = -2.
                Ramp(from: (-12.0, -16.0), to: (-2.0, 0.0), start_height: 11.0, end_height: 7.25, width: 3.0, flat_width: 1.5),
                // The river, falling from 10 m to 6 m at the pit's edge.
                Ramp(from: (-24.0, 0.0), to: (8.0, 0.0), start_height: 10.0, end_height: 6.0, width: 4.0, flat_width: 2.0),
                // The pit, 3 m below the river's end.
                Plateau(min: (8.0, -6.0), max: (20.0, 6.0), height: 3.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 13.0, 10.0),
    water: Some((
        bodies: [
            Spring(position: (-24.5, 10.6, 0.0), direction: (1.5, 0.0, 0.0), discharge: 1.0),
            Spring(position: (-12.3, 11.6, -16.5), direction: (0.8, 0.0, 1.27), discharge: 0.5),
        ],
        settle: AsAuthored,
    )),
)
"#;

/// A spring keeps a lake full; the lake spills down a river into a pit. A
/// grenade cuts a crater into the river's bed halfway down: the crater fills
/// from the river above it, and once full the river runs on below it again.
fn river_blast() -> Scenario {
    Scenario {
        name: "river_blast",
        description: "a grenade craters a running river; the crater fills and the river runs on",
        level: RIVER_BLAST_LEVEL,
        duration: 120.0,
        beats: vec![Beat {
            at: 10.0,
            action: Action::Blast {
                centre: Point3::new(-6.0, 8.0, 0.0),
                radius: 2.5,
            },
        }],
        probes: vec![
            Probe {
                name: "lake",
                at: Point3::new(-25.0, 9.5, 0.0),
            },
            Probe {
                name: "crater",
                at: Point3::new(-6.0, 6.5, 0.0),
            },
            Probe {
                name: "pit",
                at: Point3::new(14.0, 3.2, 0.0),
            },
        ],
        camera: (Point3::new(-6.0, 22.0, 18.0), Point3::new(-6.0, 7.0, 0.0)),
    }
}

/// Rubble's Phase 4 headline: the river of `river_blast`, and a column on
/// its bank. A grenade at the column's foot brings it down across the
/// channel, where it is deposited back into the terrain, and the river backs
/// up behind it.
fn river_dam() -> Scenario {
    Scenario {
        name: "river_dam",
        description: "a column blasted into a running river is deposited across it and dams it",
        level: RIVER_DAM_LEVEL,
        duration: 120.0,
        beats: vec![Beat {
            at: 10.0,
            action: Action::Rockfall {
                centre: Point3::new(-6.0, 12.5, 5.0),
                radius: 2.0,
                rest: Point3::new(-6.0, 9.5, 0.0),
                turn: UnitQuaternion::from_axis_angle(
                    &Vector3::x_axis(),
                    std::f32::consts::FRAC_PI_2,
                ),
            },
        }],
        probes: vec![
            Probe {
                name: "lake",
                at: Point3::new(-25.0, 9.5, 0.0),
            },
            Probe {
                name: "above",
                at: Point3::new(-11.0, 9.0, 0.0),
            },
            Probe {
                name: "pit",
                at: Point3::new(14.0, 3.2, 0.0),
            },
        ],
        camera: (Point3::new(-6.0, 22.0, 18.0), Point3::new(-6.0, 9.0, 0.0)),
    }
}

const RIVER_DAM_LEVEL: &str = r#"
Level(
    name: "water_viewer: river_dam",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 24.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Plateau(min: (-40.0, -40.0), max: (40.0, 40.0), height: 12.0),
                Plateau(min: (-30.0, -5.0), max: (-20.0, 5.0), height: 7.0),
                Ramp(from: (-20.0, 0.0), to: (8.0, 0.0), start_height: 10.0, end_height: 6.0, width: 4.0, flat_width: 2.0),
                Plateau(min: (8.0, -6.0), max: (40.0, 6.0), height: 3.0),
            ],
            volumes: [
                // Six metres of column on the river's bank.
                Pillar(center: (-6.0, 5.0), height: 18.0, radius: 1.5),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 13.0, 10.0),
    water: Some((
        bodies: [
            Pool(seed: (-25.0, 0.0), surface_level: 9.5),
            Spring(position: (-25.0, 11.0, 4.5), direction: (0.0, 0.0, -1.0), discharge: 1.0),
        ],
        settle: Steady,
    )),
)
"#;

const RIVER_BLAST_LEVEL: &str = r#"
Level(
    name: "water_viewer: river_blast",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Plateau(min: (-40.0, -40.0), max: (40.0, 40.0), height: 12.0),
                // The lake, 3 m deep below the river's head.
                Plateau(min: (-30.0, -5.0), max: (-20.0, 5.0), height: 7.0),
                // The river, falling from 10 m at the lake's lip to 6 m at the pit.
                Ramp(from: (-20.0, 0.0), to: (8.0, 0.0), start_height: 10.0, end_height: 6.0, width: 4.0, flat_width: 2.0),
                // A trench 3 m below the river's end, open off the map's edge.
                Plateau(min: (8.0, -6.0), max: (40.0, 6.0), height: 3.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 13.0, 10.0),
    water: Some((
        bodies: [
            Pool(seed: (-25.0, 0.0), surface_level: 9.5),
            Spring(position: (-25.0, 11.0, 4.5), direction: (0.0, 0.0, -1.0), discharge: 1.0),
        ],
        settle: Steady,
    )),
)
"#;
