//! Scenarios that hunt for bugs in a water model: shapes the acceptance
//! scenarios leave out, chosen where a model is most likely to break. Falls,
//! fronts over dry ground, supercritical chutes, flow along the lattice's
//! diagonal and through one-column slots, water against a ceiling, blasts
//! in quick succession, films on flat and sloping ground, and a seiche.

use nalgebra::Point3;

use super::scenario::{Action, Beat, Probe, Scenario};
use super::scenarios::{ISLAND_POOL_LEVEL, RIVER_BLAST_LEVEL};

/// Every bug-hunting scenario.
pub fn hunt() -> Vec<Scenario> {
    vec![
        cliff_fall(),
        dam_break(),
        steep_chute(),
        diagonal_channel(),
        slot(),
        cave_pool(),
        grenade_spam(),
        film_plain(),
        ridge_spring(),
        island_waterfall(),
        lake_seiche(),
    ]
}

fn probe(name: &'static str, x: f32, y: f32, z: f32) -> Probe {
    Probe {
        name,
        at: Point3::new(x, y, z),
    }
}

fn blast(at: f32, x: f32, y: f32, z: f32, radius: f32) -> Beat {
    Beat {
        at,
        action: Action::Blast {
            centre: Point3::new(x, y, z),
            radius,
        },
    }
}

/// A channel on a plateau runs to a 7 m cliff and falls into a pool below,
/// which spills off the map's east edge.
fn cliff_fall() -> Scenario {
    Scenario {
        name: "cliff_fall",
        description: "a channel runs off a 7 m cliff into a pool that spills off the map",
        level: CLIFF_FALL_LEVEL,
        duration: 120.0,
        beats: vec![],
        probes: vec![
            probe("channel", -10.0, 7.6, 0.0),
            probe("pool", 8.0, 0.0, 0.0),
            probe("outflow", 24.0, 0.2, 0.0),
        ],
        camera: (Point3::new(14.0, 16.0, 16.0), Point3::new(2.0, 3.0, 0.0)),
    }
}

const CLIFF_FALL_LEVEL: &str = r#"
Level(
    name: "water_viewer: cliff_fall",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [
                Plateau(min: (-40.0, -40.0), max: (0.0, 40.0), height: 8.0),
                Ramp(from: (-28.0, 0.0), to: (0.5, 0.0), start_height: 7.5, end_height: 7.0, width: 3.0, flat_width: 1.5),
                Crater(center: (8.0, 0.0), radius: 6.0, depth: 2.0),
                Ramp(from: (12.0, 0.0), to: (40.0, 0.0), start_height: -0.2, end_height: -1.0, width: 3.0, flat_width: 1.5),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (-20.0, 9.0, 10.0),
    water: Some((
        bodies: [
            Spring(position: (-27.0, 9.0, 0.0), direction: (1.0, 0.0, 0.0), discharge: 0.5),
        ],
        settle: AsAuthored,
    )),
)
"#;

/// A dam holding 5 m of water is blown out; the flood crosses a dry plain
/// and leaves off its far edge.
fn dam_break() -> Scenario {
    Scenario {
        name: "dam_break",
        description: "a dam holding 5 m of water is blown; the flood crosses a dry plain",
        level: DAM_BREAK_LEVEL,
        duration: 90.0,
        beats: vec![blast(2.0, -12.0, 3.0, 0.0, 3.5)],
        probes: vec![
            probe("reservoir", -20.0, 1.0, 0.0),
            probe("near", 0.0, 0.5, 0.0),
            probe("far", 20.0, 0.5, 0.0),
            probe("side", 0.0, 0.5, 20.0),
        ],
        camera: (Point3::new(10.0, 30.0, 28.0), Point3::new(0.0, 0.0, 0.0)),
    }
}

const DAM_BREAK_LEVEL: &str = r#"
Level(
    name: "water_viewer: dam_break",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Sand)],
            features: [
                Wall(from: (-12.0, -11.0), to: (-12.0, 11.0), height: 7.0, thickness: 3.0),
                Wall(from: (-30.0, -11.0), to: (-12.0, -11.0), height: 7.0, thickness: 3.0),
                Wall(from: (-30.0, 11.0), to: (-12.0, 11.0), height: 7.0, thickness: 3.0),
                Wall(from: (-30.0, -11.0), to: (-30.0, 11.0), height: 7.0, thickness: 3.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 2.0, 20.0),
    water: Some((
        bodies: [Pool(seed: (-20.0, 0.0), surface_level: 5.0)],
        settle: AsAuthored,
    )),
)
"#;

/// A spring at the head of a 30 % chute: water much faster than its waves.
fn steep_chute() -> Scenario {
    Scenario {
        name: "steep_chute",
        description: "a spring runs down a 30 % chute into a pit",
        level: STEEP_CHUTE_LEVEL,
        duration: 60.0,
        beats: vec![],
        probes: vec![
            probe("top", -18.0, 11.5, 0.0),
            probe("middle", 0.0, 6.0, 0.0),
            probe("pit", 18.0, -1.0, 0.0),
        ],
        camera: (Point3::new(0.0, 22.0, 22.0), Point3::new(0.0, 5.0, 0.0)),
    }
}

const STEEP_CHUTE_LEVEL: &str = r#"
Level(
    name: "water_viewer: steep_chute",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 20.0, 32.0)),
            base_height: 14.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [
                Ramp(from: (-20.0, 0.0), to: (14.0, 0.0), start_height: 11.0, end_height: 0.8, width: 3.0, flat_width: 1.5),
                Plateau(min: (14.0, -6.0), max: (24.0, 6.0), height: -2.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 15.0, 10.0),
    water: Some((
        bodies: [
            Spring(position: (-19.5, 12.0, 0.0), direction: (1.0, 0.0, 0.0), discharge: 0.5),
        ],
        settle: AsAuthored,
    )),
)
"#;

/// A channel cut along the lattice's diagonal: 4-connected flow has to
/// staircase down it.
fn diagonal_channel() -> Scenario {
    Scenario {
        name: "diagonal_channel",
        description: "a channel at 45° to the lattice runs into a pit",
        level: DIAGONAL_LEVEL,
        duration: 90.0,
        beats: vec![],
        probes: vec![
            probe("upper", -10.0, 5.5, -10.0),
            probe("lower", 8.0, 3.5, 8.0),
            probe("pit", 20.0, -1.0, 20.0),
        ],
        camera: (Point3::new(-6.0, 26.0, 20.0), Point3::new(2.0, 3.0, 2.0)),
    }
}

const DIAGONAL_LEVEL: &str = r#"
Level(
    name: "water_viewer: diagonal_channel",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 8.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Ramp(from: (-22.0, -22.0), to: (16.0, 16.0), start_height: 6.0, end_height: 3.0, width: 4.0, flat_width: 2.0),
                Plateau(min: (16.0, 16.0), max: (26.0, 26.0), height: -2.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 9.0, 10.0),
    water: Some((
        bodies: [
            Spring(position: (-21.0, 7.0, -21.0), direction: (0.7, 0.0, 0.7), discharge: 0.5),
        ],
        settle: AsAuthored,
    )),
)
"#;

/// A slot 0.75 m wide, about one column of the lattice, at a 2 % fall.
fn slot() -> Scenario {
    Scenario {
        name: "slot",
        description: "a spring runs down a slot about one column wide",
        level: SLOT_LEVEL,
        duration: 90.0,
        beats: vec![],
        probes: vec![
            probe("upper", -10.0, 5.6, 0.0),
            probe("lower", 10.0, 5.2, 0.0),
        ],
        camera: (Point3::new(4.0, 14.0, 8.0), Point3::new(0.0, 5.0, 0.0)),
    }
}

const SLOT_LEVEL: &str = r#"
Level(
    name: "water_viewer: slot",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 8.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [
                Ramp(from: (-24.0, 0.0), to: (40.0, 0.0), start_height: 5.6, end_height: 4.3, width: 1.2, flat_width: 0.75),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 9.0, 10.0),
    water: Some((
        bodies: [
            Spring(position: (-23.0, 6.6, 0.0), direction: (1.0, 0.0, 0.0), discharge: 0.1),
        ],
        settle: AsAuthored,
    )),
)
"#;

/// A spring fills a pit that opens into a closed tunnel: the water rises to
/// the tunnel's roof, then goes on rising in the pit above it.
fn cave_pool() -> Scenario {
    Scenario {
        name: "cave_pool",
        description: "a pit fills a closed tunnel to its roof, then rises above it",
        level: CAVE_POOL_LEVEL,
        duration: 150.0,
        beats: vec![],
        probes: vec![
            probe("pit", -10.0, 0.5, 0.0),
            probe("tunnel", 4.0, 0.5, 0.0),
        ],
        camera: (Point3::new(-10.0, 16.0, 12.0), Point3::new(-6.0, 1.0, 0.0)),
    }
}

const CAVE_POOL_LEVEL: &str = r#"
Level(
    name: "water_viewer: cave_pool",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 6.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [
                Plateau(min: (-14.0, -4.0), max: (-8.0, 4.0), height: 0.0),
            ],
            volumes: [
                Tunnel(center: (0.0, 0.0), direction: (1.0, 0.0), length: 18.0, radius: 1.5, depth: 1.5),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 7.0, 10.0),
    water: Some((
        bodies: [SkySource(position: (-11.0, 8.0, 0.0), discharge: 3.0)],
        settle: AsAuthored,
    )),
)
"#;

/// The river of `river_blast`, blasted eight times in fifteen seconds: up
/// and down its bed, through a bank, and into the lake's lip.
fn grenade_spam() -> Scenario {
    Scenario {
        name: "grenade_spam",
        description: "a running river is blasted eight times in fifteen seconds",
        level: RIVER_BLAST_LEVEL,
        duration: 90.0,
        beats: vec![
            blast(5.0, -15.0, 9.3, 0.0, 2.0),
            blast(6.5, -6.0, 8.0, 0.0, 2.5),
            blast(8.0, 2.0, 6.9, 0.0, 2.0),
            blast(9.0, -10.0, 8.6, 2.0, 2.5),
            blast(11.0, -20.0, 10.0, 0.0, 2.5),
            blast(13.0, 6.0, 6.4, 0.0, 2.5),
            blast(15.0, -6.0, 6.0, 0.0, 2.0),
            blast(20.0, -12.0, 9.0, -2.0, 2.0),
        ],
        probes: vec![
            probe("lake", -25.0, 9.5, 0.0),
            probe("above", -10.0, 9.1, 0.0),
            probe("below", 0.0, 7.6, 0.0),
            probe("pit", 14.0, 3.2, 0.0),
        ],
        camera: (Point3::new(-8.0, 34.0, 6.0), Point3::new(-8.0, 7.0, 0.0)),
    }
}

/// A spring onto a wide, nearly flat plain: the sheet's front, and how
/// thin water spreads.
fn film_plain() -> Scenario {
    Scenario {
        name: "film_plain",
        description: "a spring spreads a sheet over a wide, nearly flat plain",
        level: FILM_PLAIN_LEVEL,
        duration: 120.0,
        beats: vec![],
        probes: vec![
            probe("centre", 0.0, 0.5, 0.0),
            probe("ring", 8.0, 0.5, 0.0),
            probe("far", 20.0, 0.5, 0.0),
        ],
        camera: (Point3::new(0.0, 30.0, 26.0), Point3::new(0.0, 0.0, 0.0)),
    }
}

const FILM_PLAIN_LEVEL: &str = r#"
Level(
    name: "water_viewer: film_plain",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Grass)],
            features: [
                Ramp(from: (-32.0, 0.0), to: (32.0, 0.0), start_height: 0.3, end_height: -0.3, width: 80.0, flat_width: 80.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 2.0, 20.0),
    water: Some((
        bodies: [SkySource(position: (0.0, 6.0, 0.0), discharge: 0.2)],
        settle: AsAuthored,
    )),
)
"#;

/// A sky source on the crest of a ridge: water has to split down both
/// sides.
fn ridge_spring() -> Scenario {
    Scenario {
        name: "ridge_spring",
        description: "a sky source falls on a ridge's crest and runs down both sides",
        level: RIDGE_LEVEL,
        duration: 60.0,
        beats: vec![],
        probes: vec![
            probe("west", -12.0, 0.5, 0.0),
            probe("east", 12.0, 0.5, 0.0),
        ],
        camera: (Point3::new(0.0, 20.0, 24.0), Point3::new(0.0, 2.0, 0.0)),
    }
}

const RIDGE_LEVEL: &str = r#"
Level(
    name: "water_viewer: ridge_spring",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Dirt)],
            features: [
                Wall(from: (0.0, -30.0), to: (0.0, 30.0), height: 5.0, thickness: 16.0),
                Crater(center: (-12.0, 0.0), radius: 5.0, depth: 1.5),
                Crater(center: (12.0, 0.0), radius: 5.0, depth: 1.5),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 6.0, 20.0),
    water: Some((
        bodies: [SkySource(position: (0.0, 9.0, 0.0), discharge: 0.4)],
        settle: AsAuthored,
    )),
)
"#;

/// The pool on `island_pool`'s floating island has its rim blown away: it
/// pours over the island's edge onto the pond below.
fn island_waterfall() -> Scenario {
    Scenario {
        name: "island_waterfall",
        description: "an island pool's rim is blown; it pours off the island's edge",
        level: ISLAND_POOL_LEVEL,
        duration: 60.0,
        beats: vec![blast(2.0, 7.5, 7.5, 0.0, 2.5)],
        probes: vec![
            probe("island", 0.0, 7.5, 0.0),
            probe("pond", 12.0, -1.0, 0.0),
        ],
        camera: (Point3::new(20.0, 14.0, 16.0), Point3::new(6.0, 3.0, 0.0)),
    }
}

/// A long trough lake has a deep crater blown in one end's floor: the
/// level drops there and the lake sloshes along its length.
fn lake_seiche() -> Scenario {
    Scenario {
        name: "lake_seiche",
        description: "a crater in one end of a long lake sets it sloshing",
        level: SEICHE_LEVEL,
        duration: 120.0,
        beats: vec![blast(2.0, -18.0, -2.5, 0.0, 3.5)],
        probes: vec![
            probe("west", -18.0, -1.0, 0.0),
            probe("middle", 0.0, -1.0, 0.0),
            probe("east", 18.0, -1.0, 0.0),
        ],
        camera: (Point3::new(0.0, 18.0, 20.0), Point3::new(0.0, -1.0, 0.0)),
    }
}

const SEICHE_LEVEL: &str = r#"
Level(
    name: "water_viewer: lake_seiche",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 2.0,
            material_layers: [(depth: 999.0, material: Sand)],
            features: [
                Plateau(min: (-24.0, -4.0), max: (24.0, 4.0), height: -3.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (0.0, 3.0, 10.0),
    water: Some((
        bodies: [Pool(seed: (0.0, 0.0), surface_level: 0.0)],
        settle: AsAuthored,
    )),
)
"#;
