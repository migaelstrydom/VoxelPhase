//! The scenario catalogue.

use nalgebra::Point3;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use super::driver::Run;
use super::garden;
use super::scenario::{Blast, Scenario};

/// Every scenario, in the order `--list` prints them.
pub fn catalogue() -> Vec<Scenario> {
    vec![
        shelf(),
        rim_cusps(),
        arch_both_legs(),
        arch_one_leg(),
        sky_island(),
        long_bridge(),
    ]
    .into_iter()
    .chain(garden::scenarios())
    .collect()
}

/// The scenario called `name`, if there is one.
pub fn find(name: &str) -> Option<Scenario> {
    catalogue().into_iter().find(|s| s.name == name)
}

/// The largest fragment of a run, in samples.
pub(super) fn largest(run: &Run) -> usize {
    run.fragments().map(|f| f.samples).max().unwrap_or(0)
}

/// A blast under the surface opens a cavity and leaves the top layer as a
/// roof one sample thick: the zero-thickness shelf.
fn shelf() -> Scenario {
    Scenario {
        name: "shelf",
        description: "a blast under the surface; the roof it leaves breaks off inside a lip",
        level: FLAT_LEVEL,
        blasts: vec![Blast::sized(Point3::new(0.0, -2.0, 0.0), 1.9)],
        known_gap: None,
        expect: |run| match largest(run) {
            n if n >= 4 => Ok(()),
            n => Err(format!(
                "expected the roof to fall, largest fragment {n} samples"
            )),
        },
    }
}

/// Grenades thrown across a field until the rims of their craters cross: how
/// the floating strips and shelves were first made. At 1 m voxels, without
/// cutting anything loose, this leaves four samples floating and seven drawn
/// paper-thin by the end.
fn rim_cusps() -> Scenario {
    let mut rng = StdRng::seed_from_u64(1);
    let blasts = (0..60)
        .map(|_| {
            Blast::grenade(Point3::new(
                rng.gen_range(-12.0..12.0),
                rng.gen_range(-3.0..0.5),
                rng.gen_range(-12.0..12.0),
            ))
        })
        .collect();
    Scenario {
        name: "rim_cusps",
        description:
            "60 seeded grenades across a field; nothing they leave may float or be paper-thin",
        level: FIELD_LEVEL,
        blasts,
        known_gap: None,
        expect: |_| Ok(()),
    }
}

/// An arch cut through at both feet is held by nothing, and falls whole.
fn arch_both_legs() -> Scenario {
    Scenario {
        name: "arch_both_legs",
        description: "an arch cut at both feet comes down as one piece",
        level: ARCH_LEVEL,
        blasts: vec![
            Blast::sized(Point3::new(2.0, 0.25, 0.0), 1.5),
            Blast::sized(Point3::new(-2.0, 0.25, 0.0), 1.5),
        ],
        known_gap: None,
        expect: |run| match largest(run) {
            n if n >= 20 => Ok(()),
            n => Err(format!(
                "expected the span to fall, largest fragment {n} samples"
            )),
        },
    }
}

/// The same arch cut at one foot still stands on the other.
fn arch_one_leg() -> Scenario {
    Scenario {
        name: "arch_one_leg",
        description: "an arch cut at one foot stands on the other",
        level: ARCH_LEVEL,
        blasts: vec![Blast::sized(Point3::new(2.0, 0.25, 0.0), 1.5)],
        known_gap: None,
        expect: |run| match largest(run) {
            n if n < 20 => Ok(()),
            n => Err(format!(
                "expected the span to stand, a fragment of {n} samples fell"
            )),
        },
    }
}

/// A floating island stands on nothing, before a blast as after it. A blast
/// at its edge takes a bite and may chip it; the island stays.
fn sky_island() -> Scenario {
    Scenario {
        name: "sky_island",
        description: "a blast at a floating island's edge chips it; the island stays up",
        level: ISLAND_LEVEL,
        blasts: vec![Blast::sized(Point3::new(2.2, 6.0, 0.0), 1.0)],
        known_gap: None,
        expect: |run| match largest(run) {
            n if n < 100 => Ok(()),
            n => Err(format!(
                "expected the island to stay, a fragment of {n} samples fell"
            )),
        },
    }
}

/// Assumption A of the design: a span longer than the search reaches past
/// its edge and is held up by whatever it reaches, cut free or not. The audit
/// sees it left floating; recorded so that changing it is a decision.
fn long_bridge() -> Scenario {
    Scenario {
        name: "long_bridge",
        description: "a span cut at both ends but longer than the search stays up (assumption A)",
        level: BRIDGE_LEVEL,
        blasts: vec![
            Blast::sized(Point3::new(9.0, 2.75, 0.0), 1.5),
            Blast::sized(Point3::new(-9.0, 2.75, 0.0), 1.5),
        ],
        known_gap: Some("a span longer than the search region is left floating"),
        expect: |run| match largest(run) {
            n if n < 50 => Ok(()),
            n => Err(format!(
                "expected the span to stand, a fragment of {n} samples fell"
            )),
        },
    }
}

/// A field at the game's coarsest resolution, where the screenshots were
/// taken: grass over dirt.
const FIELD_LEVEL: &str = r#"
Level(
    name: "rubble_viewer: field",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 1.0,
            bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
            base_height: 0.3,
            material_layers: [(depth: 0.6, material: Grass), (depth: 999.0, material: Dirt)],
            features: [],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (10.0, 1.0, 10.0),
)
"#;

/// Flat ground: grass over dirt, the surface at y = 0.3 so the top layer of
/// samples bears.
const FLAT_LEVEL: &str = r#"
Level(
    name: "rubble_viewer: flat",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-16.0, -8.0, -16.0), max: (16.0, 8.0, 16.0)),
            base_height: 0.3,
            material_layers: [(depth: 0.6, material: Grass), (depth: 999.0, material: Dirt)],
            features: [],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (10.0, 1.0, 10.0),
)
"#;

/// An arch 4 m across on flat ground, 2 m thick. A thinner one is joined to
/// itself only diagonally where it curves, so the audit finds it standing free
/// as authored and no cut can bring it down.
const ARCH_LEVEL: &str = r#"
Level(
    name: "rubble_viewer: arch",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-16.0, -8.0, -16.0), max: (16.0, 8.0, 16.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [],
            volumes: [
                Arch(from: (-2.0, 0.0, 0.0), to: (2.0, 0.0, 0.0), radius: 2.0, thickness: 2.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (10.0, 1.0, 10.0),
)
"#;

/// A floating island over flat ground.
const ISLAND_LEVEL: &str = r#"
Level(
    name: "rubble_viewer: island",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-16.0, -8.0, -16.0), max: (16.0, 12.0, 16.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [],
            volumes: [
                Island(center: (0.0, 6.0, 0.0), half_extents: (2.0, 1.0, 2.0), edge_noise: 0.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (10.0, 1.0, 10.0),
)
"#;

/// A deck 24 m long on two pillars.
const BRIDGE_LEVEL: &str = r#"
Level(
    name: "rubble_viewer: bridge",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-16.0, -8.0, -16.0), max: (16.0, 8.0, 16.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [],
            volumes: [
                Pillar(center: (-11.0, 0.0), height: 3.0, radius: 1.0),
                Pillar(center: (11.0, 0.0), height: 3.0, radius: 1.0),
                Platform(center: (0.0, 3.0, 0.0), half_extents: (12.0, 1.0), thickness: 1.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (10.0, 1.0, 10.0),
)
"#;
