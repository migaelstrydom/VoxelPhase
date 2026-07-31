//! Checks that only exist once a level has more than one segment.
//!
//! Three questions, in the order they matter:
//!
//! 1. **Do two segments contend for space?** (Rule 4) Two grids overlapping has
//!    no well-defined meaning — neither is authoritative — and it silently
//!    produces duplicate geometry in `query_region`, which the physics notes
//!    record as a source of phantom forces.
//! 2. **Can the player actually cross the gaps?** Every join and every declared
//!    connection carries a `gap`, measured here against the jump envelope
//!    derived from live player tuning.
//! 3. **Do the asserted connections hold?** A `Connect` derives nothing, so
//!    unlike a join it can be wrong, and nothing else would notice.

use crate::level::{world_anchor, Connection, Level, Placement};
use crate::physics::PhysicsConfig;
use crate::player::PlayerConfig;
use crate::terrain::{outward, Segment, TerrainWorld};
use nalgebra::Point3;

use super::reach::JumpEnvelope;
use super::report::{Report, Section};
use super::routes::{bridge_window, RouteMap};

/// How far a measured anchor separation may drift from its declared gap.
///
/// Anchor frames are exact under quarter-turn placement, so any real
/// disagreement is an authoring error rather than accumulated float noise. The
/// tolerance only absorbs the noise.
const GAP_TOLERANCE: f32 = 0.01;

/// Which move a gap demands of the player.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Crossing {
    /// Bridged by a traversal primitive — the player walks across.
    Walk,
    /// Within the standing jump — the safe default.
    Standing,
    /// Needs the flat long-jump arc.
    LongJump,
    /// Needs a run-up and a sprint jump.
    SprintJump,
    /// Beyond every move in the envelope.
    Unreachable,
}

impl Crossing {
    /// The cheapest move that covers `gap`.
    pub fn for_gap(gap: f32, envelope: &JumpEnvelope) -> Self {
        if gap <= envelope.standing.flat_range {
            Crossing::Standing
        } else if gap <= envelope.long.flat_range {
            Crossing::LongJump
        } else if gap <= envelope.sprint.flat_range {
            Crossing::SprintJump
        } else {
            Crossing::Unreachable
        }
    }

    /// The crossing a gap demands once the level's routes are taken into
    /// account.
    ///
    /// A `Path` or `Platform` running the whole way across turns a jump into a
    /// walk. Without this, bridging a chasm makes the report worse rather than
    /// better, and an author learns to stop reading it.
    pub fn for_gap_over(
        gap: f32,
        from: Point3<f32>,
        to: Point3<f32>,
        envelope: &JumpEnvelope,
        routes: &RouteMap,
    ) -> Self {
        if routes.spans(from, to, bridge_window(envelope)) {
            return Crossing::Walk;
        }
        Crossing::for_gap(gap, envelope)
    }

    pub fn describe(&self) -> &'static str {
        match self {
            Crossing::Walk => "walk — bridged by a traversal primitive",
            Crossing::Standing => "standing jump",
            Crossing::LongJump => "long jump",
            Crossing::SprintJump => "sprint jump with a run-up",
            Crossing::Unreachable => "beyond every jump",
        }
    }
}

/// One anchor-to-anchor relationship, whether it placed anything or not.
struct Link {
    /// `segment.anchor` of the parent side.
    from: String,
    /// `segment.anchor` of the child side.
    to: String,
    /// Declared separation.
    gap: f32,
    /// True for a placement join, false for a `Connect` assertion.
    derives_placement: bool,
}

/// Per-segment statistics: what each authored area actually produced.
pub fn segment_section(terrain: &TerrainWorld) -> Section {
    let mut section = Section::new("Segments");
    for segment in terrain.segments() {
        let frame = segment.frame();
        let origin = frame.origin();
        let solid = segment.solid_chunk_count();
        section.row(
            segment.name(),
            format!(
                "origin ({:.0}, {:.0}, {:.0}) yaw {:.0}° · voxel {:.2} m · chunk {:.0} m · \
                 {} chunks ({solid} solid) · {} triangles · {} anchors",
                origin.x,
                origin.y,
                origin.z,
                frame.yaw_degrees(),
                segment.voxel_size(),
                segment.chunk_extent(),
                segment.chunk_count(),
                segment.triangle_count(),
                segment.anchors().len(),
            ),
        );
    }
    section.note(
        "Origin and yaw are derived from the placement tree, not authored directly \
         (except for the root).",
    );
    section
}

/// Rule 4: no two segments may own solid chunks whose world extents overlap.
///
/// Compared at **solid-chunk** granularity, never at segment bounds. A
/// segment's allocated extent bulges a full chunk past its content on −X, −Y
/// and −Z because of the seam shell, so two islands a jumpable gap apart have
/// overlapping allocated extents while contending for nothing.
pub fn check_contention(terrain: &TerrainWorld, report: &mut Report) {
    let segments = terrain.segments();
    for (i, a) in segments.iter().enumerate() {
        for b in segments.iter().skip(i + 1) {
            if let Some(overlap) = first_contention(a, b) {
                report.error(
                    "segments",
                    format!(
                        "segments '{}' and '{}' both own solid chunks covering \
                         ({:.0}, {:.0}, {:.0})..({:.0}, {:.0}, {:.0}); neither grid is \
                         authoritative there, and the overlap duplicates geometry in \
                         physics queries",
                        a.name(),
                        b.name(),
                        overlap.0.x,
                        overlap.0.y,
                        overlap.0.z,
                        overlap.1.x,
                        overlap.1.y,
                        overlap.1.z,
                    ),
                );
            }
        }
    }
}

/// The first overlapping pair of solid chunks between two segments, as the
/// corners of the overlapping box.
fn first_contention(
    a: &Segment,
    b: &Segment,
) -> Option<(nalgebra::Point3<f32>, nalgebra::Point3<f32>)> {
    let (Some(a_solid), Some(b_solid)) = (a.solid_bounds(), b.solid_bounds()) else {
        return None;
    };
    if !overlaps(&a_solid, &b_solid) {
        return None;
    }

    let b_chunks = b.solid_chunk_bounds();
    for chunk_a in a.solid_chunk_bounds() {
        for chunk_b in &b_chunks {
            if overlaps(&chunk_a, chunk_b) {
                return Some((
                    nalgebra::Point3::new(
                        chunk_a.min.x.max(chunk_b.min.x),
                        chunk_a.min.y.max(chunk_b.min.y),
                        chunk_a.min.z.max(chunk_b.min.z),
                    ),
                    nalgebra::Point3::new(
                        chunk_a.max.x.min(chunk_b.max.x),
                        chunk_a.max.y.min(chunk_b.max.y),
                        chunk_a.max.z.min(chunk_b.max.z),
                    ),
                ));
            }
        }
    }
    None
}

/// Strict AABB overlap. Chunk bounds are half-open, so two chunks that merely
/// share a face are adjacent, not contending.
fn overlaps(a: &crate::collision::AABB, b: &crate::collision::AABB) -> bool {
    a.min.x < b.max.x
        && b.min.x < a.max.x
        && a.min.y < b.max.y
        && b.min.y < a.max.y
        && a.min.z < b.max.z
        && b.min.z < a.max.z
}

/// Report every anchor link, check its gap against the player's reach, and
/// verify that declared connections actually hold.
pub fn check_connections(level: &Level, report: &mut Report) -> Section {
    let envelope = JumpEnvelope::derive(&PlayerConfig::default(), PhysicsConfig::default().gravity);
    let routes = RouteMap::build(level);
    let links = collect_links(level);

    let mut section = Section::new("Connections");
    if links.is_empty() {
        section.row("None", "single-segment level");
        return section;
    }

    for link in &links {
        let kind = if link.derives_placement {
            "join"
        } else {
            "assert"
        };

        let (Ok(from), Ok(to)) = (
            world_anchor(level, &level.frames, &link.from),
            world_anchor(level, &level.frames, &link.to),
        ) else {
            report.error(
                "connections",
                format!("{} ↔ {}: unresolvable anchor", link.from, link.to),
            );
            continue;
        };

        let measured = (to.origin() - from.origin()).norm();
        let crossing =
            Crossing::for_gap_over(link.gap, from.origin(), to.origin(), &envelope, &routes);
        section.row(
            format!("{} ↔ {}", link.from, link.to),
            format!(
                "{kind} · gap {:.1} m · measured {measured:.1} m · {}",
                link.gap,
                crossing.describe()
            ),
        );

        match crossing {
            Crossing::Unreachable => report.error(
                "connections",
                format!(
                    "{} ↔ {} is {:.1} m apart, beyond the {:.1} m sprint-jump range; \
                     the player cannot cross it",
                    link.from,
                    link.to,
                    link.gap,
                    envelope.max_flat_range()
                ),
            ),
            Crossing::Walk | Crossing::Standing => {}
            other => report.warn(
                "connections",
                format!(
                    "{} ↔ {} is {:.1} m apart, past the {:.1} m standing-jump range; \
                     it needs a {}",
                    link.from,
                    link.to,
                    link.gap,
                    envelope.standing.flat_range,
                    other.describe()
                ),
            ),
        }

        // A join derives the transform, so its gap holds by construction. An
        // assertion derives nothing and is the thing that can be wrong.
        if !link.derives_placement {
            if (measured - link.gap).abs() > GAP_TOLERANCE {
                report.error(
                    "connections",
                    format!(
                        "{} ↔ {} asserts a {:.2} m gap but the anchors end up {measured:.2} m \
                         apart",
                        link.from, link.to, link.gap
                    ),
                );
            }
            if (outward(&from) + outward(&to)).norm() > 1e-3 {
                report.warn(
                    "connections",
                    format!(
                        "{} ↔ {} are connected but do not face each other; one of the \
                         anchors points the wrong way",
                        link.from, link.to
                    ),
                );
            }
        }
    }

    section.note(
        "Ranges are the point-mass figures in the Player reach section; author gaps \
         well inside them.",
    );
    section
}

/// Placement joins and declared connections, as one uniform list.
fn collect_links(level: &Level) -> Vec<Link> {
    let joins = level.placements.iter().filter_map(|p| match p {
        Placement::Root { .. } => None,
        Placement::Join {
            segment,
            anchor,
            to,
            gap,
            ..
        } => Some(Link {
            from: to.clone(),
            to: format!("{segment}.{anchor}"),
            gap: *gap,
            derives_placement: true,
        }),
    });

    let asserted = level.connections.iter().map(|c: &Connection| Link {
        from: c.from.clone(),
        to: c.to.clone(),
        gap: c.gap,
        derives_placement: false,
    });

    joins.chain(asserted).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::level_check::report::Severity;
    use crate::level_check::runner::build_terrain;

    /// Two flat 32 m segments in a row, `gap` metres apart.
    fn two_islands(gap: f32) -> Level {
        let ron = format!(
            r#"Level(
                name: "Two islands",
                segments: [
                    (
                        name: "a",
                        terrain: Terrain(
                            voxel_size: 1.0,
                            bounds: (min: (0.0, -8.0, 0.0), max: (32.0, 8.0, 32.0)),
                            base_height: 0.0,
                            features: [],
                        ),
                        anchors: [(name: "east", pos: (32.0, 0.0, 16.0))],
                    ),
                    (
                        name: "b",
                        terrain: Terrain(
                            voxel_size: 1.0,
                            bounds: (min: (0.0, -8.0, 0.0), max: (32.0, 8.0, 32.0)),
                            base_height: 0.0,
                            features: [],
                        ),
                        anchors: [(name: "west", pos: (0.0, 0.0, 16.0), yaw: 180.0)],
                    ),
                ],
                placements: [
                    Root(segment: "a"),
                    Join(segment: "b", anchor: "west", to: "a.east", gap: {gap}),
                ],
                player_spawn: (4.0, 2.0, 4.0),
            )"#
        );
        let mut level: Level = ron::from_str(&ron).expect("test level should parse");
        let frames = crate::level::resolve_placements(&level).expect("should resolve");
        level.frames = frames;
        level
    }

    fn errors(report: &Report) -> Vec<String> {
        report
            .findings
            .iter()
            .filter(|f| f.severity == Severity::Error)
            .map(|f| f.message.clone())
            .collect()
    }

    /// The case an author actually writes: two islands a jumpable gap apart.
    /// A bounds-based or allocation-based contention check would reject this,
    /// because each segment's seam shell reaches a full chunk into the other's
    /// space.
    #[test]
    fn two_segments_six_metres_apart_do_not_contend() {
        let level = two_islands(6.0);
        let terrain = build_terrain(&level);
        let mut report = Report::default();
        check_contention(&terrain, &mut report);
        assert!(
            errors(&report).is_empty(),
            "6 m apart flagged as contention: {:?}",
            errors(&report)
        );
    }

    /// Genuinely overlapping segments must be rejected. A negative-looking
    /// placement is not expressible, so the overlap is built directly.
    #[test]
    fn genuinely_overlapping_segments_contend() {
        let ron = r#"Level(
            name: "Overlap",
            segments: [
                (
                    name: "a",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (0.0, -8.0, 0.0), max: (32.0, 8.0, 32.0)),
                        base_height: 0.0,
                        features: [],
                    ),
                ),
                (
                    name: "b",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (0.0, -8.0, 0.0), max: (32.0, 8.0, 32.0)),
                        base_height: 0.0,
                        features: [],
                    ),
                ),
            ],
            placements: [Root(segment: "a"), Root(segment: "b")],
            player_spawn: (4.0, 2.0, 4.0),
        )"#;
        let mut level: Level = ron::from_str(ron).expect("parse");
        // Two roots would be rejected by placement resolution, so the frames are
        // set by hand: this test is about the contention check, not the tree.
        level.frames = vec![
            crate::terrain::SegmentFrame::identity(),
            crate::terrain::SegmentFrame::new(nalgebra::Point3::new(8.0, 0.0, 8.0), 0),
        ];

        let terrain = build_terrain(&level);
        let mut report = Report::default();
        check_contention(&terrain, &mut report);
        assert_eq!(
            errors(&report).len(),
            1,
            "overlapping segments not flagged: {:?}",
            report.findings
        );
    }

    #[test]
    fn a_gap_within_standing_range_is_clean() {
        let level = two_islands(6.0);
        let mut report = Report::default();
        check_connections(&level, &mut report);
        assert!(
            report.findings.is_empty(),
            "6 m gap flagged: {:?}",
            report.findings
        );
    }

    #[test]
    fn a_gap_beyond_sprint_range_is_an_error() {
        let level = two_islands(20.0);
        let mut report = Report::default();
        check_connections(&level, &mut report);
        let errors = errors(&report);
        assert_eq!(errors.len(), 1, "expected one error, got {errors:?}");
        assert!(errors[0].contains("sprint-jump"), "{}", errors[0]);
    }

    /// Between the two ranges is a warning that names the move required.
    #[test]
    fn a_gap_past_standing_range_warns_with_the_move_needed() {
        let level = two_islands(9.0);
        let mut report = Report::default();
        check_connections(&level, &mut report);
        assert_eq!(report.error_count(), 0);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(report.findings[0].message.contains("jump"));
    }

    #[test]
    fn crossing_classification_follows_the_envelope() {
        let envelope =
            JumpEnvelope::derive(&PlayerConfig::default(), PhysicsConfig::default().gravity);
        assert_eq!(Crossing::for_gap(1.0, &envelope), Crossing::Standing);
        assert_eq!(
            Crossing::for_gap(envelope.sprint.flat_range + 1.0, &envelope),
            Crossing::Unreachable
        );
    }
}
