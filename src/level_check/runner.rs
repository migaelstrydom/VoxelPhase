//! Runs every check over one level and collects the result.
//!
//! ```text
//!   level.ron ──load──▶ Level ──build_segments──▶ TerrainWorld
//!                         │                             │
//!                         └───────────────┬─────────────┘
//!                                         ▼
//!    per-segment stats · totals · mesh integrity · Rule 4 contention ·
//!    connections and reach · spawn and object placement
//!                                         ▼
//!                                       Report
//! ```
//!
//! No Vulkan and no ECS: terrain is built through the headless constructor, so
//! this runs anywhere `cargo test` does.

use std::path::Path;

use crate::character::LocomotionConfig;
use crate::collision::AABB;
use crate::level::{build_segments, Level};
use crate::physics::PhysicsConfig;
use crate::terrain::TerrainWorld;

use super::baseline::{BaselineVerdict, Baselines};
use super::placement;
use super::reach::{JumpEnvelope, OPTIMISM_CAVEAT};
use super::report::{Report, Section};
use super::routes;
use super::segments;
use super::water;

/// Generate and mesh a level's terrain without a graphics device.
///
/// Mirrors `level::create_level_terrain` minus the noise texture, which is
/// purely a rendering concern.
pub fn build_terrain(level: &Level) -> TerrainWorld {
    let segments = build_segments(level).expect("a loaded level has resolved placements");
    TerrainWorld::from_segments_headless(segments)
}

/// Check a level's terrain, placements and reach envelope.
///
/// `level_path` is used only to locate the committed mesh baselines; pass the
/// path the level was loaded from.
pub fn check_level(level: &Level, level_path: &Path, terrain: &TerrainWorld) -> Report {
    let mut report = Report::default();

    report.push_section(level_section(level));
    report.push_section(segments::segment_section(terrain));
    report.push_section(terrain_section(terrain));

    let integrity = mesh_integrity(level_path, terrain, &mut report);
    report.push_section(integrity);
    report.push_section(reach_section());

    let routes = routes::route_section(level, &mut report);
    report.push_section(routes);

    let connections = segments::check_connections(level, &mut report);
    report.push_section(connections);

    if let Some(section) = water::check_water(level, terrain, &mut report) {
        report.push_section(section);
    }

    segments::check_contention(terrain, &mut report);
    placement::check_player_spawn(level, terrain, &mut report);
    placement::check_objects(level, terrain, &mut report);
    placement::check_object_orientation(level, &mut report);

    report
}

fn level_section(level: &Level) -> Section {
    let mut section = Section::new("Level");
    let (x, y, z) = level.player_spawn;
    section
        .row("Name", &level.name)
        .row("Segments", level.segments.len().to_string())
        .row("Objects", level.object_count().to_string())
        .row(
            "Connections",
            format!(
                "{} placement joins + {} assertions",
                level
                    .placements
                    .iter()
                    .filter(|p| !matches!(p, crate::level::Placement::Root { .. }))
                    .count(),
                level.connections.len()
            ),
        )
        .row("Player spawn", format!("({x:.1}, {y:.1}, {z:.1})"))
        .row(
            "Water bodies",
            level
                .water
                .as_ref()
                .map_or(0, |w| w.bodies.len())
                .to_string(),
        );
    section
}

fn terrain_section(terrain: &TerrainWorld) -> Section {
    let total = terrain.chunk_count();
    let solid = terrain.solid_chunk_count();

    let mut section = Section::new("Terrain");
    section
        .row(
            "Finest voxel size",
            format!("{:.3} m", terrain.voxel_size()),
        )
        .row("Derived bounds", format_aabb(terrain.bounds()))
        .row(
            "Chunks",
            format!(
                "{total} total = {solid} with solid voxels + {} seam shell",
                total.saturating_sub(solid)
            ),
        )
        .row("Triangles", terrain.triangle_count().to_string())
        .row("Vertices", terrain.render_vertices().len().to_string())
        .row("Mesh octree leaves", terrain.leaf_count().to_string())
        .note(
            "Shell chunks exist only to own the marching-cubes cells that close their \
             neighbours' minimum faces, so chunk count is not a proxy for content volume.",
        )
        .note(
            "Derived bounds are the union of allocated chunks and can exceed the authored \
             extent by up to one chunk on the negative side, for the same reason.",
        );
    section
}

/// Count open edges and compare against the committed baseline.
///
/// Findings go straight onto `report`; the returned section is the numbers.
fn mesh_integrity(level_path: &Path, terrain: &TerrainWorld, report: &mut Report) -> Section {
    let open = terrain.open_edge_count();
    let triangles = terrain.triangle_count();

    let mut section = Section::new("Mesh integrity");
    section.row(
        "Open edges",
        format!(
            "{open} of {triangles} triangles ({:.3}%)",
            if triangles == 0 {
                0.0
            } else {
                100.0 * open as f32 / triangles as f32
            }
        ),
    );

    match Baselines::for_level(level_path) {
        Err(e) => {
            report.warn("mesh", e);
            section.row("Baseline", "unavailable");
        }
        Ok(baselines) => match baselines.compare(level_path, open) {
            BaselineVerdict::Unbaselined => {
                section.row("Baseline", "none committed");
                report.warn(
                    "mesh",
                    format!(
                        "no open-edge baseline committed for this level; add one to {}",
                        Baselines::path_for(level_path).display()
                    ),
                );
            }
            BaselineVerdict::Within { baseline } => {
                section.row("Baseline", format!("{baseline} — within tolerance"));
            }
            BaselineVerdict::Risen { baseline, limit } => {
                section.row("Baseline", format!("{baseline} — exceeded"));
                report.error(
                    "mesh",
                    format!(
                        "{open} open edges against a baseline of {baseline} (limit {limit}): \
                         new cracks in the terrain mesh"
                    ),
                );
            }
            BaselineVerdict::Stale { baseline } => {
                section.row("Baseline", format!("{baseline} — stale"));
                report.warn(
                    "mesh",
                    format!(
                        "{open} open edges is well below the committed baseline of {baseline}; \
                         re-commit it or it stops catching regressions"
                    ),
                );
            }
        },
    }

    section
}

fn reach_section() -> Section {
    let envelope = JumpEnvelope::derive(
        &LocomotionConfig::default(),
        PhysicsConfig::default().gravity,
    );

    let mut section = Section::new("Player reach");
    for arc in envelope.arcs() {
        section.row(
            arc.name,
            format!(
                "apex {:.2} m · airtime {:.2} s · flat range {:.2} m  (launch {:.1} m/s h, {:.1} m/s v)",
                arc.apex, arc.airtime, arc.flat_range, arc.horizontal_speed, arc.vertical_speed
            ),
        );
    }
    section.note(OPTIMISM_CAVEAT);
    section.note("Derived from LocomotionConfig and PhysicsConfig::gravity at runtime.");
    section
}

fn format_aabb(aabb: &AABB) -> String {
    format!(
        "({:.0}, {:.0}, {:.0}) .. ({:.0}, {:.0}, {:.0})",
        aabb.min.x, aabb.min.y, aabb.min.z, aabb.max.x, aabb.max.y, aabb.max.z
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::level_check::report::Severity;

    /// A small flat level with one object at `object_y`.
    ///
    /// Flat ground at y = 0 over a two-chunk footprint: big enough to mesh
    /// normally, small enough to generate in a test.
    fn synthetic_level(object_y: f32) -> Level {
        let ron = format!(
            r#"
            Level(
                name: "Synthetic",
                segments: [(
                    name: "main",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (-32.0, -32.0, -32.0), max: (32.0, 32.0, 32.0)),
                        base_height: 0.0,
                        features: [],
                    ),
                    objects: [
                        Crate(pos: (4.0, {object_y}, 4.0), size: 0.5),
                    ],
                )],
                placements: [Root(segment: "main")],
                player_spawn: (0.0, 2.0, 0.0),
            )
            "#
        );
        let mut level: Level = ron::from_str(&ron).expect("synthetic level should parse");
        level.frames =
            crate::level::resolve_placements(&level).expect("synthetic placement should resolve");
        level
    }

    /// Path in a directory holding no baseline file, so baselines contribute a
    /// warning rather than an error either way.
    fn unbaselined_path() -> &'static Path {
        Path::new("target/synthetic.level.ron")
    }

    fn findings_of(level: &Level, severity: Severity) -> Vec<String> {
        let terrain = build_terrain(level);
        check_level(level, unbaselined_path(), &terrain)
            .findings
            .iter()
            .filter(|f| f.severity == severity)
            .map(|f| f.message.clone())
            .collect()
    }

    /// An object authored below the surface is an error: it will either be
    /// ejected violently or trapped, and either way it is not what was meant.
    #[test]
    fn an_object_buried_in_terrain_is_an_error() {
        let errors = findings_of(&synthetic_level(-8.0), Severity::Error);
        assert_eq!(
            errors.len(),
            1,
            "expected exactly one error, got {errors:?}"
        );
        assert!(
            errors[0].contains("inside solid terrain"),
            "unexpected error: {}",
            errors[0]
        );
    }

    /// The same object resting just above the surface is clean.
    #[test]
    fn a_valid_object_placement_reports_no_errors() {
        let level = synthetic_level(1.0);
        let terrain = build_terrain(&level);
        let report = check_level(&level, unbaselined_path(), &terrain);

        assert_eq!(
            report.error_count(),
            0,
            "expected no errors, got {:?}",
            report.findings
        );
        // The only warning should be the missing baseline for this throwaway
        // path — a placement warning here would mean the void check is
        // misfiring on ordinary ground.
        let placement_warnings: Vec<_> = report
            .findings
            .iter()
            .filter(|f| f.category != "mesh")
            .collect();
        assert!(
            placement_warnings.is_empty(),
            "unexpected findings: {placement_warnings:?}"
        );
    }

    /// An object authored exactly on the surface is resting, not buried. This
    /// is how levels are actually written, so a check that flags it is useless.
    #[test]
    fn an_object_resting_on_the_surface_is_not_buried() {
        let errors = findings_of(&synthetic_level(0.0), Severity::Error);
        assert!(errors.is_empty(), "surface placement flagged: {errors:?}");
    }

    /// A flat plain with a raised half, and one object straddling the join.
    ///
    /// The step runs along `x = 0`: everything at positive x is 4 m up. An
    /// object authored at the origin therefore has good ground under the point
    /// it declares and a four-metre drop under whatever reaches the other way,
    /// which is precisely the case a single-point check cannot see.
    fn stepped_level(objects: &str) -> Level {
        let ron = format!(
            r#"
            Level(
                name: "Stepped",
                segments: [(
                    name: "main",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (-32.0, -32.0, -32.0), max: (32.0, 32.0, 32.0)),
                        base_height: 0.0,
                        features: [
                            Plateau(min: (0.0, -32.0), max: (32.0, 32.0), height: 4.0),
                        ],
                    ),
                    objects: [{objects}],
                )],
                placements: [Root(segment: "main")],
                player_spawn: (-8.0, 2.0, 0.0),
            )
            "#
        );
        let mut level: Level = ron::from_str(&ron).expect("stepped level should parse");
        level.frames =
            crate::level::resolve_placements(&level).expect("stepped placement should resolve");
        level
    }

    /// The bug this whole footprint model exists for. A domino row is authored
    /// at its *first* block; the rest of it is wherever `direction` leads, and
    /// before footprints that ground was never asked about.
    #[test]
    fn a_row_running_off_a_step_is_reported_even_though_its_first_block_is_fine() {
        let warnings = findings_of(
            &stepped_level(
                "Domino(base: (2.0, 4.0, 0.0), direction: (-1.0, 0.0), count: 12, spacing: 0.7)",
            ),
            Severity::Warning,
        );
        let step = warnings.iter().find(|w| w.contains("steps by"));
        assert!(
            step.is_some(),
            "a row walking off a 4 m step went unreported: {warnings:?}"
        );
        assert!(
            step.unwrap().contains("Domino"),
            "wrong object named: {step:?}"
        );
    }

    /// The same row laid the other way stays on the upper bench, and must be
    /// silent. A check that cannot tell these two apart is just noise.
    #[test]
    fn the_same_row_laid_along_the_bench_is_silent() {
        let warnings = findings_of(
            &stepped_level(
                "Domino(base: (2.0, 4.0, 0.0), direction: (0.0, 1.0), count: 12, spacing: 0.7)",
            ),
            Severity::Warning,
        );
        let placement: Vec<_> = warnings
            .iter()
            .filter(|w| !w.contains("baseline"))
            .collect();
        assert!(placement.is_empty(), "unexpected warnings: {placement:?}");
    }

    /// A bridge is authored to have nothing under its middle. Reporting that
    /// would be reporting that it works, so `Support::Spanning` opts it out.
    #[test]
    fn a_bridge_is_allowed_to_have_nothing_under_it() {
        let warnings = findings_of(
            &stepped_level("PlankBridge(pos: (0.0, 4.5, 0.0), length: 12.0, yaw: 90.0)"),
            Severity::Warning,
        );
        let placement: Vec<_> = warnings
            .iter()
            .filter(|w| !w.contains("baseline"))
            .collect();
        assert!(
            placement.is_empty(),
            "a spanning object was checked as bedded: {placement:?}"
        );
    }

    /// One stray solid sample is more likely a doubled surface on a mesh with
    /// open edges than an object in a bank — measured, on a crate standing in
    /// an open cave. Only a whole side's worth counts.
    #[test]
    fn an_object_hard_against_a_riser_is_reported_as_embedded() {
        let warnings = findings_of(
            &stepped_level("Box(pos: (-1.5, 0.0, 0.0), half_extents: (3.0, 0.5, 3.0))"),
            Severity::Warning,
        );
        assert!(
            warnings
                .iter()
                .any(|w| w.contains("runs into solid terrain")),
            "a box half inside the riser went unreported: {warnings:?}"
        );
    }

    /// Both shipped levels must pass. If this fails, either a level or the
    /// check is wrong — and the report says which.
    #[test]
    fn shipped_levels_pass() {
        for name in ["test_arena.level.ron", "test_empty_terrain.level.ron"] {
            let path = Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("levels")
                .join(name);
            let level = crate::level::load_level(&path).expect("shipped level should load");
            let terrain = build_terrain(&level);
            let report = check_level(&level, &path, &terrain);
            assert!(
                report.passed(),
                "{name} reported errors: {:?}",
                report
                    .findings
                    .iter()
                    .filter(|f| f.severity == Severity::Error)
                    .collect::<Vec<_>>()
            );
        }
    }
}
