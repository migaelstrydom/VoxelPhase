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
    let voxel_size = level.terrain.voxel_size;
    if voxel_size <= 0.0 {
        return Err(LevelError::Validation("voxel_size must be positive".into()));
    }

    let bounds = level.terrain.bounds.to_aabb();
    let size = bounds.size();
    if size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0 {
        return Err(LevelError::Validation(format!(
            "terrain bounds must have positive extent on every axis, got {:?} to {:?}",
            bounds.min, bounds.max
        )));
    }

    // Chunks are allocated on demand, so a generous extent is cheap — but an
    // extent measured in thousands of voxels per axis still means a very long
    // heightfield pass, which is worth flagging as an authoring mistake.
    let max_voxels = (size.x / voxel_size)
        .max(size.y / voxel_size)
        .max(size.z / voxel_size);
    if max_voxels > 4096.0 {
        return Err(LevelError::Validation(format!(
            "terrain bounds span {:.0} voxels on its longest axis at voxel_size {}; \
             limit is 4096",
            max_voxels, voxel_size
        )));
    }

    let (px, py, pz) = level.player_spawn;
    let spawn = nalgebra::Point3::new(px, py, pz);
    if !bounds.contains_point(spawn) {
        return Err(LevelError::Validation(format!(
            "player_spawn ({}, {}, {}) is outside terrain bounds {:?} to {:?}",
            px, py, pz, bounds.min, bounds.max
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
                terrain: Terrain(
                    voxel_size: 1.0,
                    bounds: (min: (-64.0, -32.0, -64.0), max: (64.0, 32.0, 64.0)),
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
        assert_eq!(level.terrain.voxel_size, 1.0);
        assert_eq!(level.objects.len(), 2);
        assert!(level.terrain.volumes.is_empty());
    }

    #[test]
    fn parse_full_level() {
        let ron = r#"
            Level(
                name: "Full Test",
                terrain: Terrain(
                    voxel_size: 0.5,
                    bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
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
        assert_eq!(level.terrain.voxel_size, 0.5);
        assert_eq!(level.terrain.features.len(), 5);
        assert_eq!(level.terrain.volumes.len(), 2);
        assert_eq!(level.terrain.material_layers.len(), 3);
        assert_eq!(level.objects.len(), 9);
    }

    /// A level with an inverted or degenerate extent generates nothing and is
    /// almost certainly an authoring slip.
    #[test]
    fn reject_degenerate_bounds() {
        let ron = r#"
            Level(
                name: "Bad",
                terrain: Terrain(
                    voxel_size: 1.0,
                    bounds: (min: (0.0, 0.0, 0.0), max: (0.0, 16.0, 16.0)),
                    base_height: 0.0,
                    features: [],
                ),
                player_spawn: (0.0, 0.0, 0.0),
                objects: [],
            )
        "#;
        let level: Level = ron::from_str(ron).expect("parse");
        assert!(validate(&level).is_err());
    }

    #[test]
    fn reject_non_positive_voxel_size() {
        let ron = r#"
            Level(
                name: "Bad",
                terrain: Terrain(
                    voxel_size: 0.0,
                    bounds: (min: (-16.0, -16.0, -16.0), max: (16.0, 16.0, 16.0)),
                    base_height: 0.0,
                    features: [],
                ),
                player_spawn: (0.0, 0.0, 0.0),
                objects: [],
            )
        "#;
        let level: Level = ron::from_str(ron).expect("parse");
        assert!(validate(&level).is_err());
    }

    /// An extent that would need millions of voxels per axis is a mistake, not
    /// an ambition — the heightfield pass still walks every column.
    #[test]
    fn reject_absurdly_large_bounds() {
        let ron = r#"
            Level(
                name: "Bad",
                terrain: Terrain(
                    voxel_size: 0.25,
                    bounds: (min: (-4096.0, -16.0, -16.0), max: (4096.0, 16.0, 16.0)),
                    base_height: 0.0,
                    features: [],
                ),
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
                terrain: Terrain(
                    voxel_size: 1.0,
                    bounds: (min: (-16.0, -16.0, -16.0), max: (16.0, 16.0, 16.0)),
                    base_height: 0.0,
                    features: [],
                ),
                player_spawn: (100.0, 0.0, 0.0),
                objects: [],
            )
        "#;
        let level: Level = ron::from_str(ron).expect("parse");
        assert!(validate(&level).is_err());
    }

    #[test]
    fn load_test_arena_from_file() {
        load_level(std::path::Path::new("levels/test_arena.level.ron"))
            .expect("Failed to load test_arena");
    }

    #[test]
    fn test_arena_declares_metre_voxels() {
        let level = load_level(std::path::Path::new("levels/test_arena.level.ron"))
            .expect("Failed to load test_arena");
        assert_eq!(level.terrain.voxel_size, 1.0);
        assert!(!level.objects.is_empty());
        assert!(!level.terrain.volumes.is_empty());
    }

    #[test]
    fn defaults_applied_for_box() {
        let ron = r#"
            Level(
                name: "Defaults",
                terrain: Terrain(
                    voxel_size: 1.0,
                    bounds: (min: (-64.0, -32.0, -64.0), max: (64.0, 32.0, 64.0)),
                    base_height: 0.0,
                    features: [],
                ),
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

    /// Every level shipped in `levels/` must parse and validate. Guards against
    /// a `LevelObject` variant changing shape without its authored uses being
    /// updated.
    ///
    /// Levels are the `*.level.ron` files; the directory also holds other RON
    /// documents about those levels, such as the mesh baselines.
    #[test]
    fn shipped_levels_parse_and_validate() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("levels");
        let mut checked = 0;

        for entry in std::fs::read_dir(&dir).expect("levels directory should exist") {
            let path = entry.expect("readable dir entry").path();
            let is_level = path
                .file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.ends_with(".level.ron"));
            if !is_level {
                continue;
            }

            let level = load_level(&path)
                .unwrap_or_else(|e| panic!("{} failed to load: {:?}", path.display(), e));
            validate(&level)
                .unwrap_or_else(|e| panic!("{} failed validation: {:?}", path.display(), e));
            checked += 1;
        }

        assert!(checked > 0, "no level files found in {}", dir.display());
    }
}
