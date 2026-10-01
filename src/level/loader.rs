//! Level file loading, validation and placement resolution.
//!
//! ```text
//!   file ──parse──▶ Level ──validate──▶ resolve placements ──▶ bake to world
//!                                            │                      │
//!                                     one frame per segment   objects + spawn
//! ```
//!
//! Loading is where a level stops being segment-local: once the placement tree
//! has resolved, object positions and the player spawn are lifted into world
//! coordinates so that everything downstream sees one frame. Terrain stays
//! local — that is what keeps a segment relocatable.

use std::fmt;
use std::path::Path;

use nalgebra::Point3;

use super::data::{Level, LevelObject, ObjectPlacement};
use super::placement::{resolve_placements, PlacementError};

/// Errors that can occur when loading a level file.
#[derive(Debug)]
pub enum LevelError {
    Io(std::io::Error),
    Parse(ron::de::SpannedError),
    Validation(String),
    Placement(PlacementError),
}

impl fmt::Display for LevelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LevelError::Io(e) => write!(f, "Failed to read level file: {}", e),
            LevelError::Parse(e) => write!(f, "Failed to parse level file: {}", e),
            LevelError::Validation(msg) => write!(f, "Level validation error: {}", msg),
            LevelError::Placement(e) => write!(f, "Level placement error: {}", e),
        }
    }
}

impl From<PlacementError> for LevelError {
    fn from(e: PlacementError) -> Self {
        LevelError::Placement(e)
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

/// Load, validate and place a level from a RON file.
pub fn load_level(path: &Path) -> Result<Level, LevelError> {
    let contents = std::fs::read_to_string(path)?;
    parse_level(&contents)
}

/// Parse, validate and place a level from RON text, for levels that live in
/// code rather than on disk (the water harness's synthetic scenarios).
pub fn parse_level(contents: &str) -> Result<Level, LevelError> {
    let mut level: Level = ron::from_str(contents)?;
    validate(&level)?;
    place(&mut level)?;
    Ok(level)
}

/// Resolve the placement tree and lift authored positions into world space.
fn place(level: &mut Level) -> Result<(), LevelError> {
    let frames = resolve_placements(level)?;

    // The player spawn is authored in the root segment's frame — the root is
    // the one segment placed at an explicit world transform, so it is the only
    // frame an author can reason about before anything else is placed.
    let root = root_index(level);
    let (px, py, pz) = level.player_spawn;
    let spawn = frames[root].to_world(Point3::new(px, py, pz));
    level.player_spawn = (spawn.x, spawn.y, spawn.z);

    for (segment, frame) in level.segments.iter_mut().zip(&frames) {
        for object in &mut segment.objects {
            object.place_in(frame);
        }
    }

    level.frames = frames;
    Ok(())
}

/// Index of the root segment. Placement resolution has already guaranteed there
/// is exactly one.
fn root_index(level: &Level) -> usize {
    level
        .placements
        .iter()
        .find_map(|p| match p {
            super::data::Placement::Root { segment, .. } => level.segment_index(segment),
            _ => None,
        })
        .expect("placement resolution guarantees exactly one root")
}

/// Validate level data for consistency, before placement is resolved.
fn validate(level: &Level) -> Result<(), LevelError> {
    if level.segments.is_empty() {
        return Err(LevelError::Validation(
            "a level needs at least one segment".into(),
        ));
    }

    for segment in &level.segments {
        validate_segment(segment)?;
    }

    // The spawn is authored in the root segment's frame, so it is checked
    // against that segment's local extent before anything is transformed.
    let root = level
        .placements
        .iter()
        .find_map(|p| match p {
            super::data::Placement::Root { segment, .. } => level.segment_index(segment),
            _ => None,
        })
        .ok_or(LevelError::Placement(PlacementError::NoRoot))?;

    let bounds = level.segments[root].terrain.bounds.to_aabb();
    let (px, py, pz) = level.player_spawn;
    let spawn = Point3::new(px, py, pz);
    if !bounds.contains_point(spawn) {
        return Err(LevelError::Validation(format!(
            "player_spawn ({}, {}, {}) is outside the root segment '{}' bounds {:?} to {:?} \
             (the spawn is authored in the root segment's local frame)",
            px, py, pz, level.segments[root].name, bounds.min, bounds.max
        )));
    }

    Ok(())
}

/// A drop moves a free object down onto what is below it. Around anything
/// else it has nothing to act on: a terrain-anchored object already takes its
/// height from the ground, and a drop around a drop would only fall twice.
fn validate_drop(segment: &str, object: &LevelObject) -> Result<(), LevelError> {
    let LevelObject::Dropped(inner) = object else {
        return Ok(());
    };
    let kind = inner.describe().kind;
    if inner.is_dropped() {
        return Err(LevelError::Validation(format!(
            "segment '{segment}': Dropped(Dropped(..)) around a {kind}; drop it once"
        )));
    }
    match inner.describe().placement {
        ObjectPlacement::Free(_) => Ok(()),
        _ => Err(LevelError::Validation(format!(
            "segment '{segment}': a {kind} takes its height from the terrain and cannot be \
             Dropped"
        ))),
    }
}

fn validate_segment(segment: &super::data::SegmentDef) -> Result<(), LevelError> {
    let name = &segment.name;
    let voxel_size = segment.terrain.voxel_size;
    if voxel_size <= 0.0 {
        return Err(LevelError::Validation(format!(
            "segment '{name}': voxel_size must be positive"
        )));
    }

    let bounds = segment.terrain.bounds.to_aabb();
    let size = bounds.size();
    if size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0 {
        return Err(LevelError::Validation(format!(
            "segment '{name}': terrain bounds must have positive extent on every axis, \
             got {:?} to {:?}",
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
            "segment '{name}': terrain bounds span {:.0} voxels on its longest axis at \
             voxel_size {}; limit is 4096",
            max_voxels, voxel_size
        )));
    }

    for object in &segment.objects {
        validate_drop(name, object)?;
    }

    for layer in &segment.terrain.material_layers {
        if layer.depth <= 0.0 {
            return Err(LevelError::Validation(format!(
                "segment '{name}': material_layer depth must be positive"
            )));
        }
    }

    let mut seen: Vec<&str> = Vec::new();
    for anchor in &segment.anchors {
        if seen.contains(&anchor.name.as_str()) {
            return Err(LevelError::Validation(format!(
                "segment '{name}' declares two anchors named '{}'",
                anchor.name
            )));
        }
        seen.push(&anchor.name);
        super::placement::local_frame(name, anchor)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A one-segment level body, so tests can vary only what they care about.
    fn one_segment(terrain: &str, spawn: &str, objects: &str) -> String {
        format!(
            r#"Level(
                name: "Test",
                segments: [
                    (
                        name: "main",
                        terrain: {terrain},
                        objects: [{objects}],
                    ),
                ],
                placements: [Root(segment: "main")],
                player_spawn: {spawn},
            )"#
        )
    }

    fn parse(ron: &str) -> Level {
        ron::from_str(ron).expect("Failed to parse RON")
    }

    const FLAT: &str = r#"Terrain(
        voxel_size: 1.0,
        bounds: (min: (-32.0, -32.0, -32.0), max: (32.0, 32.0, 32.0)),
        base_height: 0.0,
        features: [],
    )"#;

    /// A drop is moved through the segment's frame with what it wraps, and
    /// keeps the authored point as where its fall starts.
    #[test]
    fn a_dropped_free_object_parses_as_a_drop() {
        let level = parse_level(&one_segment(
            FLAT,
            "(0.0, 2.0, 0.0)",
            "Dropped(Octahedron(pos: (1.0, 3.0, 2.0), size: 1.5))",
        ))
        .expect("a dropped free object is valid");

        let (_, object) = level.objects().next().unwrap();
        assert!(object.is_dropped());
        let info = object.describe();
        assert_eq!(info.kind, "Octahedron");
        assert_eq!(
            info.placement,
            ObjectPlacement::Dropped(Point3::new(1.0, 3.0, 2.0))
        );
    }

    #[test]
    fn a_terrain_anchored_object_cannot_be_dropped() {
        let error = parse_level(&one_segment(
            FLAT,
            "(0.0, 2.0, 0.0)",
            "Dropped(Rock(pos: (1.0, 2.0)))",
        ))
        .err()
        .expect("a rock takes its height from the terrain");
        assert!(error.to_string().contains("cannot be"), "{error}");
    }

    #[test]
    fn a_drop_cannot_wrap_a_drop() {
        let error = parse_level(&one_segment(
            FLAT,
            "(0.0, 2.0, 0.0)",
            "Dropped(Dropped(BeachBall(pos: (1.0, 3.0, 0.0))))",
        ))
        .err()
        .expect("a drop of a drop is rejected");
        assert!(error.to_string().contains("drop it once"), "{error}");
    }

    #[test]
    fn parse_minimal_level() {
        let level = parse(&one_segment(
            r#"Terrain(
                voxel_size: 1.0,
                bounds: (min: (-64.0, -32.0, -64.0), max: (64.0, 32.0, 64.0)),
                base_height: 0.0,
                features: [Hill(center: (5.0, 5.0), radius: 10.0, height: 3.0)],
            )"#,
            "(0.0, 2.0, 0.0)",
            "BeachBall(pos: (1.0, 3.0, 0.0)), Crate(pos: (4.0, 1.0, 2.0), size: 0.5)",
        ));
        validate(&level).expect("Validation failed");

        assert_eq!(level.name, "Test");
        assert_eq!(level.segments.len(), 1);
        assert_eq!(level.segments[0].terrain.voxel_size, 1.0);
        assert_eq!(level.object_count(), 2);
        assert!(level.segments[0].terrain.volumes.is_empty());
    }

    #[test]
    fn parse_full_level() {
        let ron = r#"
            Level(
                name: "Full Test",
                segments: [
                    (
                        name: "main",
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
                        anchors: [
                            (name: "exit_east", pos: (32.0, 0.0, 0.0)),
                        ],
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
                            Stack(
                                base: (12.0, 0.0, 5.0),
                                items: [Crate(size: 1.0), BeachBall],
                            ),
                        ],
                    ),
                ],
                placements: [Root(segment: "main", origin: (0.0, 0.0, 0.0), yaw: 0.0)],
                player_spawn: (0.0, 2.0, 0.0),
            )
        "#;

        let level = parse(ron);
        validate(&level).expect("Validation failed");

        assert_eq!(level.name, "Full Test");
        let segment = &level.segments[0];
        assert_eq!(segment.terrain.voxel_size, 0.5);
        assert_eq!(segment.terrain.features.len(), 5);
        assert_eq!(segment.terrain.volumes.len(), 2);
        assert_eq!(segment.terrain.material_layers.len(), 3);
        assert_eq!(segment.anchors.len(), 1);
        assert_eq!(level.object_count(), 4);
    }

    /// A level with an inverted or degenerate extent generates nothing and is
    /// almost certainly an authoring slip.
    #[test]
    fn reject_degenerate_bounds() {
        let level = parse(&one_segment(
            r#"Terrain(
                voxel_size: 1.0,
                bounds: (min: (0.0, 0.0, 0.0), max: (0.0, 16.0, 16.0)),
                base_height: 0.0,
                features: [],
            )"#,
            "(0.0, 0.0, 0.0)",
            "",
        ));
        assert!(validate(&level).is_err());
    }

    #[test]
    fn reject_non_positive_voxel_size() {
        let level = parse(&one_segment(
            r#"Terrain(
                voxel_size: 0.0,
                bounds: (min: (-16.0, -16.0, -16.0), max: (16.0, 16.0, 16.0)),
                base_height: 0.0,
                features: [],
            )"#,
            "(0.0, 0.0, 0.0)",
            "",
        ));
        assert!(validate(&level).is_err());
    }

    /// An extent that would need millions of voxels per axis is a mistake, not
    /// an ambition — the heightfield pass still walks every column.
    #[test]
    fn reject_absurdly_large_bounds() {
        let level = parse(&one_segment(
            r#"Terrain(
                voxel_size: 0.25,
                bounds: (min: (-4096.0, -16.0, -16.0), max: (4096.0, 16.0, 16.0)),
                base_height: 0.0,
                features: [],
            )"#,
            "(0.0, 0.0, 0.0)",
            "",
        ));
        assert!(validate(&level).is_err());
    }

    /// The spawn is authored in the root segment's frame, so it is checked
    /// against that segment's local extent.
    #[test]
    fn reject_player_outside_root_segment() {
        let level = parse(&one_segment(
            r#"Terrain(
                voxel_size: 1.0,
                bounds: (min: (-16.0, -16.0, -16.0), max: (16.0, 16.0, 16.0)),
                base_height: 0.0,
                features: [],
            )"#,
            "(100.0, 0.0, 0.0)",
            "",
        ));
        assert!(validate(&level).is_err());
    }

    #[test]
    fn reject_duplicate_anchor_names() {
        let ron = r#"
            Level(
                name: "Dupes",
                segments: [(
                    name: "main",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (-16.0, -16.0, -16.0), max: (16.0, 16.0, 16.0)),
                        base_height: 0.0,
                        features: [],
                    ),
                    anchors: [
                        (name: "gate", pos: (16.0, 0.0, 0.0)),
                        (name: "gate", pos: (-16.0, 0.0, 0.0), yaw: 180.0),
                    ],
                )],
                placements: [Root(segment: "main")],
                player_spawn: (0.0, 0.0, 0.0),
            )
        "#;
        assert!(validate(&parse(ron)).is_err());
    }

    /// An anchor yaw that is not a quarter turn must be rejected at load, and
    /// the message must name the offending anchor.
    #[test]
    fn reject_non_quarter_anchor_yaw() {
        let ron = r#"
            Level(
                name: "Skew",
                segments: [(
                    name: "plaza",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (-16.0, -16.0, -16.0), max: (16.0, 16.0, 16.0)),
                        base_height: 0.0,
                        features: [],
                    ),
                    anchors: [(name: "gate", pos: (16.0, 0.0, 0.0), yaw: 30.0)],
                )],
                placements: [Root(segment: "main")],
                player_spawn: (0.0, 0.0, 0.0),
            )
        "#;
        let err = validate(&parse(ron)).expect_err("30° yaw should be rejected");
        let text = err.to_string();
        assert!(text.contains("plaza.gate"), "unhelpful message: {text}");
        assert!(text.contains("90"), "unhelpful message: {text}");
    }

    /// A non-quarter *segment* yaw is rejected at load too, naming the segment.
    #[test]
    fn reject_non_quarter_segment_yaw() {
        let ron = one_segment(
            r#"Terrain(
                voxel_size: 1.0,
                bounds: (min: (-16.0, -16.0, -16.0), max: (16.0, 16.0, 16.0)),
                base_height: 0.0,
                features: [],
            )"#,
            "(0.0, 0.0, 0.0)",
            "",
        )
        .replace(
            r#"Root(segment: "main")"#,
            r#"Root(segment: "main", yaw: 45.0)"#,
        );

        let mut level = parse(&ron);
        let err = place(&mut level).expect_err("45° yaw should be rejected");
        let text = err.to_string();
        assert!(text.contains("main"), "unhelpful message: {text}");
        assert!(text.contains("90"), "unhelpful message: {text}");
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
        assert_eq!(level.segments[0].terrain.voxel_size, 1.0);
        assert!(level.object_count() > 0);
        assert!(!level.segments[0].terrain.volumes.is_empty());
    }

    #[test]
    fn defaults_applied_for_box() {
        let level = parse(&one_segment(
            r#"Terrain(
                voxel_size: 1.0,
                bounds: (min: (-64.0, -32.0, -64.0), max: (64.0, 32.0, 64.0)),
                base_height: 0.0,
                features: [],
            )"#,
            "(0.0, 0.0, 0.0)",
            "Box(pos: (0.0, 1.0, 0.0), half_extents: (0.5, 0.5, 0.5))",
        ));
        validate(&level).expect("validate");

        match &level.segments[0].objects[0] {
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

    /// Objects and the spawn are authored segment-locally and must come out of
    /// the loader in world space.
    #[test]
    fn loading_lifts_objects_and_spawn_into_world_space() {
        let ron = one_segment(
            r#"Terrain(
                voxel_size: 1.0,
                bounds: (min: (-16.0, -16.0, -16.0), max: (16.0, 16.0, 16.0)),
                base_height: 0.0,
                features: [],
            )"#,
            "(2.0, 1.0, 0.0)",
            "BeachBall(pos: (4.0, 0.0, 0.0))",
        )
        .replace(
            r#"Root(segment: "main")"#,
            r#"Root(segment: "main", origin: (100.0, 0.0, 0.0), yaw: 90.0)"#,
        );

        let mut level = parse(&ron);
        place(&mut level).expect("should place");

        // One quarter turn maps local +X onto world −Z, then the origin shifts.
        let (sx, sy, sz) = level.player_spawn;
        assert!(
            (sx - 100.0).abs() < 1e-4 && (sy - 1.0).abs() < 1e-4 && (sz + 2.0).abs() < 1e-4,
            "spawn landed at ({sx}, {sy}, {sz})"
        );

        match &level.segments[0].objects[0] {
            super::super::data::LevelObject::BeachBall { pos } => {
                assert!(
                    (pos.0 - 100.0).abs() < 1e-4 && (pos.2 + 4.0).abs() < 1e-4,
                    "{pos:?}"
                );
            }
            _ => panic!("Expected BeachBall"),
        }
    }

    /// Every level shipped in `levels/` must parse, validate and place. Guards
    /// against a `LevelObject` variant or a placement clause changing shape
    /// without its authored uses being updated.
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

            load_level(&path)
                .unwrap_or_else(|e| panic!("{} failed to load: {:?}", path.display(), e));
            checked += 1;
        }

        assert!(checked > 0, "no level files found in {}", dir.display());
    }
}
