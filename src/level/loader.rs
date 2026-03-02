//! Level file loading and validation.

use std::fmt;
use std::path::Path;

use super::data::Level;

/// Errors that can occur when loading a level file.
#[derive(Debug)]
pub enum LevelError {
    Io(std::io::Error),
    Parse(ron::de::SpannedError),
    Validation(String),
}

impl fmt::Display for LevelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LevelError::Io(e) => write!(f, "Failed to read level file: {}", e),
            LevelError::Parse(e) => write!(f, "Failed to parse level file: {}", e),
            LevelError::Validation(msg) => write!(f, "Level validation error: {}", msg),
        }
    }
}

impl From<std::io::Error> for LevelError {
    fn from(e: std::io::Error) -> Self {
        LevelError::Io(e)
    }
}

impl From<ron::de::SpannedError> for LevelError {
    fn from(e: ron::de::SpannedError) -> Self {
        LevelError::Parse(e)
    }
}

/// Load and validate a level from a RON file.
pub fn load_level(path: &Path) -> Result<Level, LevelError> {
    let contents = std::fs::read_to_string(path)?;
    let level: Level = ron::from_str(&contents)?;
    validate(&level)?;
    Ok(level)
}

/// Validate level data for consistency.
fn validate(level: &Level) -> Result<(), LevelError> {
    if level.world_size <= 0.0 {
        return Err(LevelError::Validation("world_size must be positive".into()));
    }

    if level.voxel_size <= 0.0 {
        return Err(LevelError::Validation("voxel_size must be positive".into()));
    }

    let ratio = level.world_size / level.voxel_size;
    if ratio < 1.0 || ratio.log2().fract().abs() > f32::EPSILON {
        return Err(LevelError::Validation(format!(
            "world_size / voxel_size must be a power of two, got {} / {} = {}",
            level.world_size, level.voxel_size, ratio
        )));
    }

    let depth = level.octree_depth();
    if depth < 2 || depth > 10 {
        return Err(LevelError::Validation(format!(
            "computed octree depth {} is out of range [2, 10]",
            depth
        )));
    }

    let half = level.world_size;
    let (px, py, pz) = level.player_spawn;
    if px.abs() > half || py.abs() > half || pz.abs() > half {
        return Err(LevelError::Validation(format!(
            "player_spawn ({}, {}, {}) is outside world bounds (±{})",
            px, py, pz, half
        )));
    }

    for layer in &level.terrain.material_layers {
        if layer.depth <= 0.0 {
            return Err(LevelError::Validation(
                "material_layer depth must be positive".into(),
            ));
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_minimal_level() {
        let ron = r#"
            Level(
                name: "Test",
                world_size: 64.0,
                voxel_size: 1.0,
                terrain: Terrain(
                    base_height: 0.0,
                    features: [
                        Hill(center: (5.0, 5.0), radius: 10.0, height: 3.0),
                    ],
                ),
                player_spawn: (0.0, 2.0, 0.0),
                objects: [
                    BeachBall(pos: (1.0, 3.0, 0.0)),
                    Crate(pos: (4.0, 1.0, 2.0), size: 0.5),
                ],
            )
        "#;

        let level: Level = ron::from_str(ron).expect("Failed to parse RON");
        validate(&level).expect("Validation failed");

        assert_eq!(level.name, "Test");
        assert_eq!(level.octree_depth(), 6); // log2(64/1) = 6
        assert_eq!(level.objects.len(), 2);
        assert!(level.terrain.volumes.is_empty());
    }

    #[test]
    fn parse_full_level() {
        let ron = r#"
            Level(
                name: "Full Test",
                world_size: 32.0,
                voxel_size: 0.5,
                terrain: Terrain(
                    base_height: -2.0,
                    material_layers: [
                        (depth: 1.0, material: Grass),
                        (depth: 4.0, material: Dirt),
                        (depth: 999.0, material: Rock),
                    ],
                    features: [
                        Plateau(min: (-5.0, -5.0), max: (5.0, 5.0), height: 0.0),
                        Wall(from: (5.0, 0.0), to: (10.0, 0.0), height: 3.0, thickness: 1.0),
                        Ramp(from: (10.0, 0.0), to: (15.0, 0.0), start_height: 0.0, end_height: 5.0, width: 3.0),
                        Crater(center: (0.0, 0.0), radius: 3.0, depth: 2.0),
                        TerrainRoughness(frequency: 0.1, amplitude: 0.5, octaves: 2, seed: 42),
                    ],
                    volumes: [
                        Island(center: (10.0, 12.0, 5.0), half_extents: (3.0, 1.0, 3.0), edge_noise: 0.3),
                        Pillar(center: (5.0, 0.0), height: 8.0, radius: 1.5),
                    ],
                ),
                player_spawn: (0.0, 2.0, 0.0),
                objects: [
                    BeachBall(pos: (1.0, 3.0, 0.0)),
                    Box(
                        pos: (5.0, 1.0, 0.0),
                        half_extents: (0.5, 0.5, 0.5),
                        style: Metal,
                        density: 80.0,
                        restitution: 0.1,
                        friction: 0.8,
                    ),
                    Plank(pos: (8.0, 4.0, 0.0), length: 4.0, width: 1.0),
                    Crate(pos: (3.0, 1.0, 0.0), size: 0.5),
                    HeavyCrate(pos: (6.0, 1.0, 0.0), size: 0.5),
                    Stack(
                        base: (12.0, 0.0, 5.0),
                        items: [
                            Crate(size: 1.0),
                            Crate(size: 1.0),
                            BeachBall,
                        ],
                    ),
                    Tower(
                        base: (-5.0, 0.0, 8.0),
                        box_half_extents: (0.5, 0.5, 0.5),
                        count: 6,
                        density: 40.0,
                    ),
                    BoxWall(
                        base: (15.0, 0.0, 0.0),
                        box_half_extents: (1.0, 0.5, 0.5),
                        columns: 4,
                        rows: 3,
                        density: 50.0,
                        stagger: true,
                    ),
                    House(pos: (0.0, 0.0, 10.0), half_extents: (2.0, 1.5, 2.0)),
                ],
            )
        "#;

        let level: Level = ron::from_str(ron).expect("Failed to parse RON");
        validate(&level).expect("Validation failed");

        assert_eq!(level.name, "Full Test");
        assert_eq!(level.octree_depth(), 6); // log2(32/0.5) = 6
        assert_eq!(level.terrain.features.len(), 5);
        assert_eq!(level.terrain.volumes.len(), 2);
        assert_eq!(level.terrain.material_layers.len(), 3);
        assert_eq!(level.objects.len(), 9);
    }

    #[test]
    fn reject_invalid_world_size() {
        let ron = r#"
            Level(
                name: "Bad",
                world_size: 0.0,
                voxel_size: 1.0,
                terrain: Terrain(base_height: 0.0, features: []),
                player_spawn: (0.0, 0.0, 0.0),
                objects: [],
            )
        "#;
        let level: Level = ron::from_str(ron).expect("parse");
        assert!(validate(&level).is_err());
    }

    #[test]
    fn reject_non_power_of_two_ratio() {
        let ron = r#"
            Level(
                name: "Bad",
                world_size: 30.0,
                voxel_size: 1.0,
                terrain: Terrain(base_height: 0.0, features: []),
                player_spawn: (0.0, 0.0, 0.0),
                objects: [],
            )
        "#;
        let level: Level = ron::from_str(ron).expect("parse");
        assert!(validate(&level).is_err());
    }

    #[test]
    fn reject_player_outside_bounds() {
        let ron = r#"
            Level(
                name: "Bad",
                world_size: 16.0,
                voxel_size: 1.0,
                terrain: Terrain(base_height: 0.0, features: []),
                player_spawn: (100.0, 0.0, 0.0),
                objects: [],
            )
        "#;
        let level: Level = ron::from_str(ron).expect("parse");
        assert!(validate(&level).is_err());
    }

    #[test]
    fn load_test_arena_from_file() {
        let level = load_level(std::path::Path::new("levels/test_arena.level.ron"))
            .expect("Failed to load test_arena");
        assert_eq!(level.name, "Test Arena");
        assert_eq!(level.octree_depth(), 6);
    }

    #[test]
    fn load_grenade_gauntlet_from_file() {
        let level = load_level(std::path::Path::new("levels/grenade_gauntlet.level.ron"))
            .expect("Failed to load grenade_gauntlet");
        assert_eq!(level.name, "Grenade Gauntlet");
        assert_eq!(level.octree_depth(), 6);
        assert!(!level.objects.is_empty());
        assert!(!level.terrain.volumes.is_empty());
    }

    #[test]
    fn defaults_applied_for_box() {
        let ron = r#"
            Level(
                name: "Defaults",
                world_size: 64.0,
                voxel_size: 1.0,
                terrain: Terrain(base_height: 0.0, features: []),
                player_spawn: (0.0, 0.0, 0.0),
                objects: [
                    Box(
                        pos: (0.0, 1.0, 0.0),
                        half_extents: (0.5, 0.5, 0.5),
                    ),
                ],
            )
        "#;

        let level: Level = ron::from_str(ron).expect("parse");
        validate(&level).expect("validate");

        match &level.objects[0] {
            super::super::data::LevelObject::Box {
                density,
                restitution,
                friction,
                ..
            } => {
                assert_eq!(*density, 50.0);
                assert_eq!(*restitution, 0.2);
                assert_eq!(*friction, 0.6);
            }
            _ => panic!("Expected Box"),
        }
    }
}
