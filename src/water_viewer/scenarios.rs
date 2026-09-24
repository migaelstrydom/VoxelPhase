//! The catalogue of things worth watching water do.
//!
//! Each scenario is the smallest terrain that shows one behaviour, so a report
//! reads as that behaviour and nothing else. The acceptance scenarios of the
//! hydrology design (§9.3) live here as they come online.

use nalgebra::Point3;

use super::scenario::{Action, Beat, Probe, Scenario};

/// Every scenario, in the order they are worth reading.
pub fn catalogue() -> Vec<Scenario> {
    vec![breach(), island_pool()]
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

/// A pond in a bowl on a hillside. Its downhill rim is blown out at 2 s.
///
/// The ground falls 8 m across the 64 m map along +x. The bowl's lowest rim
/// is on the downhill side at about −0.75; the pond stands at −1.5. The charge
/// cuts a 2.5 m crater into that rim, well below the water line.
fn breach() -> Scenario {
    Scenario {
        name: "breach",
        description: "a pond's downhill rim is blown out; it drains down the hill",
        level: BREACH_LEVEL,
        duration: 60.0,
        beats: vec![Beat {
            at: 2.0,
            action: Action::Blast {
                centre: Point3::new(-2.5, -0.9, 0.0),
                radius: 2.5,
            },
        }],
        probes: vec![
            Probe {
                name: "pond",
                at: Point3::new(-14.0, -1.0, 0.0),
            },
            Probe {
                name: "notch",
                at: Point3::new(-1.0, -2.0, 0.0),
            },
            Probe {
                name: "hill",
                at: Point3::new(12.0, -3.0, 0.0),
            },
        ],
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
                Ramp(from: (-32.0, 0.0), to: (32.0, 0.0), start_height: 3.0, end_height: -5.0, width: 400.0),
                Crater(center: (-14.0, 0.0), radius: 12.0, depth: 6.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (20.0, 0.0, 20.0),
    water: Some((
        ocean_level: None,
        bodies: [Pool(seed: (-14.0, 0.0), surface_level: -1.5)],
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
