//! Checks that only exist once a level is built out of traversal primitives.
//!
//! Four questions, and the first one is the reason the rest are worth having:
//!
//! 1. **Is this gap walked or jumped?** A gap spanned by a `Path` is not a
//!    jump, and reporting it as one makes the report *worse* the moment an
//!    author bridges something — which teaches them to ignore it.
//! 2. **Can the player stand on this?** A deck narrower than the collider is a
//!    level bug, and the width it has to beat comes from the collider rather
//!    than from a number typed here.
//! 3. **Can they climb this?** A staircase's rise per step against the apex.
//! 4. **Will this render as what was authored?** A primitive finer than a few
//!    voxels comes out as a staircase in plan regardless of how it was
//!    described, and that is how a route becomes resolution-dependent.

use nalgebra::Point3;

use crate::collision::AABB;
use crate::level::{Level, TraversalInfo};
use crate::physics::PhysicsConfig;
use crate::player::PlayerConfig;
use crate::terrain::traversal::{route_plan, RoutePlan};
use crate::terrain::SegmentFrame;

use super::reach::{Footprint, JumpEnvelope};
use super::report::{Report, Section};

/// How many voxels a primitive's narrowest dimension has to span.
///
/// Two, because two is what it takes for the primitive to have an *interior*:
/// one lattice sample inside and one outside, which is the minimum the density
/// encoding needs to place both of a deck's faces where they were authored.
/// Below that the primitive's thickness is decided by the lattice rather than
/// by the author, and it is decided differently at every resolution — which is
/// exactly how a route becomes resolution-dependent.
const MIN_VOXELS_ACROSS: f32 = 2.0;

/// Report every traversal primitive in the level and check what it asks of the
/// player and of its segment's resolution.
pub fn route_section(level: &Level, report: &mut Report) -> Section {
    let player = PlayerConfig::default();
    let footprint = Footprint::derive(&player);
    let envelope = JumpEnvelope::derive(&player, PhysicsConfig::default().gravity);

    let mut section = Section::new("Traversal primitives");
    let mut count = 0;

    for segment in &level.segments {
        let voxel = segment.terrain.voxel_size;
        for (i, volume) in segment.terrain.volumes.iter().enumerate() {
            let Some(info) = volume.traversal_info() else {
                continue;
            };
            count += 1;
            let name = format!("{}.volumes[{i}]", segment.name);

            section.row(
                &name,
                format!(
                    "{} · finest {:.2} m at {voxel:.2} m voxels{}{}",
                    info.kind,
                    info.finest_detail,
                    info.walkable_width
                        .map(|w| format!(" · {w:.1} m walkable"))
                        .unwrap_or_default(),
                    info.step_rise
                        .map(|r| format!(" · {r:.2} m per step"))
                        .unwrap_or_default(),
                ),
            );

            check_width(&name, &info, &footprint, report);
            check_rise(&name, &info, &envelope, report);
            check_resolution(&name, &info, voxel, report);
        }
    }

    if count == 0 {
        section.row("None", "the level is shaped only by landscape features");
    }
    section.note(
        "A primitive's surface lands at its authored height at any resolution; what \
         resolution decides is whether its *edges* do.",
    );
    section
}

/// A deck the player cannot stand on is a level bug however good it looks.
fn check_width(name: &str, info: &TraversalInfo, footprint: &Footprint, report: &mut Report) {
    let Some(width) = info.walkable_width else {
        return;
    };
    if width < footprint.fits {
        report.error(
            "routes",
            format!(
                "{name}: the {} is {width:.2} m wide, narrower than the player's \
                 {:.2} m collider — they cannot stand on it",
                info.kind, footprint.fits
            ),
        );
    } else if width < footprint.walkable {
        report.warn(
            "routes",
            format!(
                "{name}: the {} is {width:.2} m wide against a {:.2} m walkable minimum; \
                 the player fits but has no margin either side",
                info.kind, footprint.walkable
            ),
        );
    }
}

/// A staircase is meant to be walked. A rise past half the jump apex means
/// jumping every step, and past the apex means not climbing it at all.
fn check_rise(name: &str, info: &TraversalInfo, envelope: &JumpEnvelope, report: &mut Report) {
    let Some(rise) = info.step_rise else {
        return;
    };
    let apex = envelope.max_apex();
    if rise > apex {
        report.error(
            "routes",
            format!(
                "{name}: each step rises {rise:.2} m, past the {apex:.2} m jump apex — \
                 the flight cannot be climbed"
            ),
        );
    } else if rise > apex * 0.5 {
        report.warn(
            "routes",
            format!(
                "{name}: each step rises {rise:.2} m, over half the {apex:.2} m jump apex; \
                 the player has to jump every step rather than walk up"
            ),
        );
    }
}

/// A primitive finer than a few voxels comes out as the lattice, not as what
/// was authored — and it does so differently at every resolution.
fn check_resolution(name: &str, info: &TraversalInfo, voxel: f32, report: &mut Report) {
    let needed = info.finest_detail / MIN_VOXELS_ACROSS;
    if voxel > needed {
        report.warn(
            "routes",
            format!(
                "{name}: the {} is {:.2} m across its finest dimension, under {MIN_VOXELS_ACROSS} \
                 voxels at this segment's {voxel:.2} m resolution; it needs {needed:.2} m voxels \
                 to read as authored",
                info.kind, info.finest_detail
            ),
        );
    }
}

// ---------------------------------------------------------------------------
// Route coverage — is a gap walked or jumped?
// ---------------------------------------------------------------------------

/// The level's traversal primitives, placed in world space.
///
/// Built from the same [`route_plan`] generation uses, so the check cannot
/// disagree with the geometry about where a deck is. Segments are queried by
/// transforming the world point into each one's local frame — exact under
/// quarter-turn placement — rather than by transforming the geometry out.
pub struct RouteMap {
    placed: Vec<PlacedRoute>,
}

/// One primitive, with everything needed to ask about it in world space.
struct PlacedRoute {
    frame: SegmentFrame,
    /// The owning segment's authored extent, in its own local frame.
    ///
    /// Generation clips every feature to this, so a deck reaching past it does
    /// not exist however it was authored. Without the same clip here the check
    /// would confidently report a gap as bridged by geometry that was never
    /// written — the worst kind of wrong, because it silences a real finding.
    local_bounds: AABB,
    plan: RoutePlan,
}

/// Vertical spacing of the probe when asking whether a deck stands at a point.
const PROBE_STEP: f32 = 0.25;

/// How many samples a gap is split into when asking whether it is bridged.
const SPAN_SAMPLES: usize = 12;

/// Fraction of a gap at each end that is not probed.
///
/// The two anchors sit on the ledges either side, inside their own segments'
/// terrain, so the ends of the line are never on the bridge. What matters is
/// that the middle is.
const SPAN_MARGIN: f32 = 0.15;

impl RouteMap {
    pub fn build(level: &Level) -> Self {
        let mut placed = Vec::new();
        for (index, segment) in level.segments.iter().enumerate() {
            let frame = level.frame(index);
            let local_bounds = segment.terrain.bounds.to_aabb();
            for volume in &segment.terrain.volumes {
                if let Some(plan) = route_plan(volume) {
                    placed.push(PlacedRoute {
                        frame,
                        local_bounds,
                        plan,
                    });
                }
            }
        }
        Self { placed }
    }

    pub fn is_empty(&self) -> bool {
        self.placed.is_empty()
    }

    /// Whether walkable route geometry stands at this world position, anywhere
    /// within `window` metres above or below it.
    pub fn covers(&self, world: Point3<f32>, window: f32) -> bool {
        let steps = (window / PROBE_STEP).ceil() as i32;
        for route in &self.placed {
            for solid in route.plan.walkable() {
                for i in -steps..=steps {
                    let probe = Point3::new(world.x, world.y + i as f32 * PROBE_STEP, world.z);
                    let local = route.frame.to_local(probe);
                    if !contains(&route.local_bounds, local) {
                        continue;
                    }
                    if solid.sample(local).distance <= 0.0 {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Whether a route runs the whole way between two world points.
    ///
    /// The line between two mated anchors is the gap the player would jump, so
    /// covering all of it — not merely touching it — is what makes the crossing
    /// a walk.
    pub fn spans(&self, from: Point3<f32>, to: Point3<f32>, window: f32) -> bool {
        if self.placed.is_empty() {
            return false;
        }
        for i in 0..SPAN_SAMPLES {
            let t =
                SPAN_MARGIN + (1.0 - 2.0 * SPAN_MARGIN) * (i as f32 / (SPAN_SAMPLES - 1) as f32);
            let p = from + (to - from) * t;
            if !self.covers(p, window) {
                return false;
            }
        }
        true
    }
}

/// Whether a local point lies inside a segment's authored extent.
fn contains(bounds: &AABB, p: Point3<f32>) -> bool {
    p.x >= bounds.min.x
        && p.x <= bounds.max.x
        && p.y >= bounds.min.y
        && p.y <= bounds.max.y
        && p.z >= bounds.min.z
        && p.z <= bounds.max.z
}

/// Vertical reach either side of an anchor within which a deck still counts as
/// continuing that ledge.
///
/// Derived rather than chosen: a bridge deck more than a jump apex below an
/// anchor is not a bridge from it, because the player could not get back up on
/// the far side.
pub fn bridge_window(envelope: &JumpEnvelope) -> f32 {
    envelope.max_apex()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::level::resolve_placements;
    use crate::level_check::report::Severity;
    use crate::level_check::segments::{check_connections, Crossing};

    /// Two 32 m islands 6 m apart, with whatever volume features are given
    /// authored into the first one's frame.
    ///
    /// 6 m is inside the standing jump, so without a bridge the crossing is
    /// reported as a jump and nothing is flagged; the bridged case has to change
    /// the *classification*, not merely silence a warning.
    fn two_islands(volumes: &str) -> Level {
        two_islands_bounded(volumes, 44.0)
    }

    /// As above, but with the root segment's authored extent under the
    /// author's control, so the clipping generation applies can be exercised.
    fn two_islands_bounded(volumes: &str, max_x: f32) -> Level {
        let ron = format!(
            r#"Level(
                name: "Bridged islands",
                segments: [
                    (
                        name: "a",
                        terrain: Terrain(
                            voxel_size: 0.5,
                            bounds: (min: (0.0, -8.0, 0.0), max: ({max_x}, 16.0, 32.0)),
                            base_height: 0.0,
                            features: [],
                            volumes: [{volumes}],
                        ),
                        anchors: [(name: "east", pos: (32.0, 0.0, 16.0))],
                    ),
                    (
                        name: "b",
                        terrain: Terrain(
                            voxel_size: 0.5,
                            bounds: (min: (0.0, -8.0, 0.0), max: (32.0, 16.0, 32.0)),
                            base_height: 0.0,
                            features: [],
                        ),
                        anchors: [(name: "west", pos: (0.0, 0.0, 16.0), yaw: 180.0)],
                    ),
                ],
                placements: [
                    Root(segment: "a"),
                    Join(segment: "b", anchor: "west", to: "a.east", gap: 6.0),
                ],
                player_spawn: (4.0, 2.0, 4.0),
            )"#
        );
        let mut level: Level = ron::from_str(&ron).expect("test level should parse");
        level.frames = resolve_placements(&level).expect("test placement should resolve");
        level
    }

    fn findings(level: &Level, severity: Severity) -> Vec<String> {
        let mut report = Report::default();
        route_section(level, &mut report);
        report
            .findings
            .iter()
            .filter(|f| f.severity == severity)
            .map(|f| f.message.clone())
            .collect()
    }

    /// The report of the crossing between the two islands.
    fn crossing_row(level: &Level) -> String {
        let mut report = Report::default();
        let section = check_connections(level, &mut report);
        section
            .rows
            .iter()
            .map(|r| r.1.clone())
            .collect::<Vec<_>>()
            .join(" | ")
    }

    /// Without a bridge, a 6 m gap is a jump.
    #[test]
    fn an_unbridged_gap_is_still_a_jump() {
        let level = two_islands("");
        assert!(
            crossing_row(&level).contains("standing jump"),
            "{}",
            crossing_row(&level)
        );
    }

    /// A `Path` running the whole way across turns the same gap into a walk.
    /// Anchors sit at local (32, 0, 16) and 6 m beyond, so the deck has to span
    /// x = 32..38 in the root segment's frame.
    #[test]
    fn a_gap_spanned_by_a_path_is_walkable() {
        let level = two_islands(
            r#"Path(
                points: [(28.0, 0.0, 16.0), (42.0, 0.0, 16.0)],
                width: 3.0,
                thickness: 1.0,
            )"#,
        );
        let row = crossing_row(&level);
        assert!(
            row.contains(Crossing::Walk.describe()),
            "the bridged gap is still reported as a jump: {row}"
        );
    }

    /// A path that stops short of the far side does not bridge anything, and
    /// must not be allowed to claim it does.
    #[test]
    fn a_path_that_does_not_reach_the_far_side_is_not_a_bridge() {
        let level = two_islands(
            r#"Path(
                points: [(28.0, 0.0, 16.0), (34.0, 0.0, 16.0)],
                width: 3.0,
                thickness: 1.0,
            )"#,
        );
        let row = crossing_row(&level);
        assert!(
            row.contains("standing jump"),
            "a half-length path claimed the crossing: {row}"
        );
    }

    /// A deck authored past its segment's extent is clipped away by
    /// generation, so it must not be allowed to claim a crossing either. This
    /// is the one failure mode that would make the check actively harmful:
    /// silencing a real jump with geometry that was never written.
    #[test]
    fn a_path_running_past_its_segments_bounds_does_not_bridge() {
        const DECK: &str = r#"Path(
            points: [(28.0, 0.0, 16.0), (42.0, 0.0, 16.0)],
            width: 3.0,
            thickness: 1.0,
        )"#;

        // The gap runs from x = 32 to x = 38. With the segment authored out to
        // 44 the deck exists and bridges it.
        assert!(
            crossing_row(&two_islands_bounded(DECK, 44.0)).contains(Crossing::Walk.describe()),
            "the deck inside its bounds failed to bridge"
        );
        // With the segment ending at 34 the same deck is generated only as far
        // as 34, and the crossing is a jump again.
        let clipped = crossing_row(&two_islands_bounded(DECK, 34.0));
        assert!(
            clipped.contains("standing jump"),
            "a deck clipped away by its segment's bounds still claimed the crossing: {clipped}"
        );
    }

    /// A deck the player physically cannot stand on.
    #[test]
    fn a_path_narrower_than_the_player_is_an_error() {
        let level = two_islands(
            r#"Path(points: [(4.0, 4.0, 4.0), (20.0, 4.0, 4.0)], width: 0.4, thickness: 1.0)"#,
        );
        let errors = findings(&level, Severity::Error);
        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(errors[0].contains("collider"), "{}", errors[0]);
    }

    /// Wide enough to stand on, too narrow to walk along with any margin.
    #[test]
    fn a_path_narrower_than_the_walkable_minimum_warns() {
        let level = two_islands(
            r#"Path(points: [(4.0, 4.0, 4.0), (20.0, 4.0, 4.0)], width: 0.7, thickness: 1.0)"#,
        );
        assert!(findings(&level, Severity::Error).is_empty());
        let warnings = findings(&level, Severity::Warning);
        assert!(
            warnings.iter().any(|w| w.contains("walkable minimum")),
            "{warnings:?}"
        );
    }

    /// A step the player cannot clear at all.
    #[test]
    fn a_staircase_rising_past_the_jump_apex_is_an_error() {
        let level = two_islands(
            r#"Staircase(from: (4.0, 0.0, 8.0), to: (20.0, 12.0, 8.0), width: 3.0, steps: 4)"#,
        );
        let errors = findings(&level, Severity::Error);
        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(errors[0].contains("apex"), "{}", errors[0]);
    }

    /// A primitive too fine for its segment cannot render as authored, and the
    /// warning has to name the resolution that would work.
    #[test]
    fn a_primitive_finer_than_its_segments_voxels_warns_with_the_resolution_needed() {
        let level = two_islands(
            r#"Path(points: [(4.0, 4.0, 4.0), (20.0, 4.0, 4.0)], width: 3.0, thickness: 0.6)"#,
        );
        let warnings = findings(&level, Severity::Warning);
        let hit = warnings
            .iter()
            .find(|w| w.contains("read as authored"))
            .unwrap_or_else(|| panic!("no resolution warning: {warnings:?}"));
        // 0.6 m over two voxels needs 0.30 m voxels; the segment runs 0.5.
        assert!(hit.contains("0.30 m voxels"), "{hit}");
    }
}
