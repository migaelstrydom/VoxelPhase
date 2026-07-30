//! Resolving the placement tree into one world frame per segment.
//!
//! ```text
//!            Root("plaza")                  placement edges form a TREE
//!                 │                         and are the only thing that
//!        ┌────────┴────────┐                derives a transform
//!   Join("tower")     Join("bridge")
//!                          │
//!                    Join("summit")
//!
//!   Connect("summit.back" ─ "plaza.gate")   connectivity is a GRAPH, and
//!                                           derives nothing — it is checked,
//!                                           not followed
//! ```
//!
//! Splitting the two is what makes a non-linear level well-defined. If a
//! shortcut looping back to an earlier area were also a placement, two paths
//! around the loop would each claim to determine the same segment's transform,
//! and they would disagree.

use std::collections::VecDeque;
use std::fmt;

use nalgebra::Point3;

use crate::terrain::{mate, SegmentFrame};

use super::data::{AnchorDef, Level, Placement};

/// Why a level's placement tree could not be resolved.
#[derive(Debug, Clone, PartialEq)]
pub enum PlacementError {
    /// Two segments share a name, so `segment.anchor` references are ambiguous.
    DuplicateSegment(String),
    /// A placement or connection names a segment that does not exist.
    UnknownSegment { referrer: String, segment: String },
    /// A join or connection names an anchor the segment does not declare.
    UnknownAnchor { segment: String, anchor: String },
    /// An anchor reference was not of the form `segment.anchor`.
    MalformedAnchorRef(String),
    /// No segment was declared as the placement root.
    NoRoot,
    /// More than one segment was declared as the placement root.
    MultipleRoots(Vec<String>),
    /// A segment has more than one placement, so its transform is
    /// over-determined.
    OverDetermined { segment: String, placements: usize },
    /// A segment has no placement at all.
    Orphan(String),
    /// Segments whose placements form a loop, so none of them can be resolved.
    Cycle(Vec<String>),
    /// A yaw that is not a multiple of 90°.
    BadYaw { what: String, yaw: f32 },
    /// A negative join gap.
    NegativeGap { segment: String, gap: f32 },
    /// A welded join, which is not implemented.
    WeldNotImplemented(String),
}

impl fmt::Display for PlacementError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PlacementError::DuplicateSegment(name) => {
                write!(f, "two segments are both named '{name}'")
            }
            PlacementError::UnknownSegment { referrer, segment } => {
                write!(f, "{referrer} refers to unknown segment '{segment}'")
            }
            PlacementError::UnknownAnchor { segment, anchor } => {
                write!(f, "segment '{segment}' has no anchor named '{anchor}'")
            }
            PlacementError::MalformedAnchorRef(text) => write!(
                f,
                "anchor reference '{text}' is not of the form 'segment.anchor'"
            ),
            PlacementError::NoRoot => write!(
                f,
                "no Root placement: exactly one segment must be placed at an explicit \
                 world transform"
            ),
            PlacementError::MultipleRoots(names) => write!(
                f,
                "{} Root placements ({}): placement is a tree and has exactly one root",
                names.len(),
                names.join(", ")
            ),
            PlacementError::OverDetermined {
                segment,
                placements,
            } => write!(
                f,
                "segment '{segment}' has {placements} placements; its transform is \
                 over-determined. Express the extra join as a Connect assertion instead."
            ),
            PlacementError::Orphan(name) => write!(
                f,
                "segment '{name}' has no placement: give it a Join, or make it the Root"
            ),
            PlacementError::Cycle(names) => write!(
                f,
                "segments {} form a placement cycle and cannot be reached from the root; \
                 break the loop and express the closing join as a Connect assertion",
                names.join(", ")
            ),
            PlacementError::BadYaw { what, yaw } => write!(
                f,
                "{what} has yaw {yaw}°, which is not a multiple of 90°. Rotation is \
                 restricted to quarter turns; pitch and roll are not supported."
            ),
            PlacementError::NegativeGap { segment, gap } => {
                write!(
                    f,
                    "join for segment '{segment}' has a negative gap of {gap}"
                )
            }
            PlacementError::WeldNotImplemented(name) => write!(
                f,
                "join for segment '{name}' requests weld: true, which is not implemented. \
                 Marching cubes reads the unallocated neighbour as air, so a welded \
                 boundary emits a cap surface sealing the join. Use a gap, or express the \
                 continuous structure as a spawnable."
            ),
        }
    }
}

/// A parsed `segment.anchor` reference.
pub struct AnchorRef<'a> {
    pub segment: &'a str,
    pub anchor: &'a str,
}

/// Split `segment.anchor`. The segment name may not contain a dot.
pub fn parse_anchor_ref(text: &str) -> Result<AnchorRef<'_>, PlacementError> {
    match text.split_once('.') {
        Some((segment, anchor)) if !segment.is_empty() && !anchor.is_empty() => {
            Ok(AnchorRef { segment, anchor })
        }
        _ => Err(PlacementError::MalformedAnchorRef(text.to_string())),
    }
}

/// Resolve every segment's world frame from the level's placement tree.
///
/// The result is parallel to `level.segments`.
pub fn resolve_placements(level: &Level) -> Result<Vec<SegmentFrame>, PlacementError> {
    let index_of = build_index(level)?;
    let placements = group_placements(level, &index_of)?;

    let mut frames: Vec<Option<SegmentFrame>> = vec![None; level.segments.len()];

    // Children of each segment, so a placed segment can place its dependants.
    let mut children: Vec<Vec<usize>> = vec![Vec::new(); level.segments.len()];
    let mut queue = VecDeque::new();

    for (child, placement) in placements.iter().enumerate() {
        match placement {
            Placement::Root {
                segment,
                origin,
                yaw,
            } => {
                let origin = Point3::new(origin.0, origin.1, origin.2);
                frames[child] =
                    Some(SegmentFrame::from_degrees(origin, *yaw).ok_or_else(|| {
                        PlacementError::BadYaw {
                            what: format!("root segment '{segment}'"),
                            yaw: *yaw,
                        }
                    })?);
                queue.push_back(child);
            }
            Placement::Join { segment, to, .. } => {
                let parent_ref = parse_anchor_ref(to)?;
                let parent = index_of
                    .iter()
                    .find(|(n, _)| *n == parent_ref.segment)
                    .map(|(_, i)| *i)
                    .ok_or_else(|| PlacementError::UnknownSegment {
                        referrer: format!("join for segment '{segment}'"),
                        segment: parent_ref.segment.to_string(),
                    })?;
                children[parent].push(child);
            }
        }
    }

    while let Some(parent) = queue.pop_front() {
        let parent_frame = frames[parent].expect("queued segments are placed");
        for child in std::mem::take(&mut children[parent]) {
            let Placement::Join {
                segment,
                anchor,
                to,
                gap,
                weld,
            } = &placements[child]
            else {
                unreachable!("only joins are recorded as children");
            };

            if *weld {
                return Err(PlacementError::WeldNotImplemented(segment.clone()));
            }
            if *gap < 0.0 {
                return Err(PlacementError::NegativeGap {
                    segment: segment.clone(),
                    gap: *gap,
                });
            }

            let parent_ref = parse_anchor_ref(to)?;
            let parent_anchor = anchor_frame(level, parent, parent_ref.anchor)?;
            let child_anchor = anchor_frame(level, child, anchor)?;

            frames[child] = Some(mate(
                &parent_frame.compose(&parent_anchor),
                &child_anchor,
                *gap,
            ));
            queue.push_back(child);
        }
    }

    // Every segment has exactly one placement and is not the root, so anything
    // still unplaced is only reachable through a loop.
    let unplaced: Vec<String> = frames
        .iter()
        .enumerate()
        .filter(|(_, f)| f.is_none())
        .map(|(i, _)| level.segments[i].name.clone())
        .collect();
    if !unplaced.is_empty() {
        return Err(PlacementError::Cycle(unplaced));
    }

    Ok(frames.into_iter().map(Option::unwrap).collect())
}

/// The world frame of an anchor, given its segment's already-resolved frame.
pub fn world_anchor(
    level: &Level,
    frames: &[SegmentFrame],
    reference: &str,
) -> Result<SegmentFrame, PlacementError> {
    let parsed = parse_anchor_ref(reference)?;
    let index =
        level
            .segment_index(parsed.segment)
            .ok_or_else(|| PlacementError::UnknownSegment {
                referrer: format!("anchor reference '{reference}'"),
                segment: parsed.segment.to_string(),
            })?;
    let local = anchor_frame(level, index, parsed.anchor)?;
    Ok(frames[index].compose(&local))
}

/// Segment-local frame of one anchor.
fn anchor_frame(
    level: &Level,
    segment: usize,
    anchor: &str,
) -> Result<SegmentFrame, PlacementError> {
    let def = level.segments[segment]
        .anchors
        .iter()
        .find(|a| a.name == anchor)
        .ok_or_else(|| PlacementError::UnknownAnchor {
            segment: level.segments[segment].name.clone(),
            anchor: anchor.to_string(),
        })?;
    local_frame(&level.segments[segment].name, def)
}

/// An anchor definition as a segment-local frame, rejecting non-quarter yaws.
pub fn local_frame(segment: &str, def: &AnchorDef) -> Result<SegmentFrame, PlacementError> {
    let origin = Point3::new(def.pos.0, def.pos.1, def.pos.2);
    SegmentFrame::from_degrees(origin, def.yaw).ok_or_else(|| PlacementError::BadYaw {
        what: format!("anchor '{}.{}'", segment, def.name),
        yaw: def.yaw,
    })
}

/// Segment names paired with their index, rejecting duplicates.
fn build_index(level: &Level) -> Result<Vec<(&str, usize)>, PlacementError> {
    let mut index: Vec<(&str, usize)> = Vec::with_capacity(level.segments.len());
    for (i, segment) in level.segments.iter().enumerate() {
        if index.iter().any(|(name, _)| *name == segment.name) {
            return Err(PlacementError::DuplicateSegment(segment.name.clone()));
        }
        index.push((segment.name.as_str(), i));
    }
    Ok(index)
}

/// Exactly one placement per segment, in segment order.
fn group_placements<'a>(
    level: &'a Level,
    index_of: &[(&str, usize)],
) -> Result<Vec<&'a Placement>, PlacementError> {
    let mut per_segment: Vec<Vec<&Placement>> = vec![Vec::new(); level.segments.len()];
    let mut roots = Vec::new();

    for placement in &level.placements {
        let (name, is_root) = match placement {
            Placement::Root { segment, .. } => (segment, true),
            Placement::Join { segment, .. } => (segment, false),
        };
        let index = index_of
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, i)| *i)
            .ok_or_else(|| PlacementError::UnknownSegment {
                referrer: "a placement".to_string(),
                segment: name.clone(),
            })?;
        per_segment[index].push(placement);
        if is_root {
            roots.push(name.clone());
        }
    }

    if roots.is_empty() {
        return Err(PlacementError::NoRoot);
    }
    if roots.len() > 1 {
        return Err(PlacementError::MultipleRoots(roots));
    }

    for (index, placements) in per_segment.iter().enumerate() {
        match placements.len() {
            0 => return Err(PlacementError::Orphan(level.segments[index].name.clone())),
            1 => {}
            n => {
                return Err(PlacementError::OverDetermined {
                    segment: level.segments[index].name.clone(),
                    placements: n,
                })
            }
        }
    }

    Ok(per_segment.into_iter().map(|mut p| p.remove(0)).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::outward;

    /// A level of `n` segments with the given placement and connection clauses.
    ///
    /// Every segment is a bare 32 m box of flat terrain with two anchors: `east`
    /// facing local `+X`, and `west` facing local `−X`.
    fn level(names: &[&str], placements: &str, connections: &str) -> Level {
        let segments: String = names
            .iter()
            .map(|name| {
                format!(
                    r#"(
                        name: "{name}",
                        terrain: Terrain(
                            voxel_size: 1.0,
                            bounds: (min: (0.0, -16.0, 0.0), max: (32.0, 16.0, 32.0)),
                            base_height: 0.0,
                            features: [],
                        ),
                        anchors: [
                            (name: "east", pos: (32.0, 0.0, 16.0), yaw: 0.0),
                            (name: "west", pos: (0.0, 0.0, 16.0), yaw: 180.0),
                        ],
                    ),"#
                )
            })
            .collect();

        let ron = format!(
            r#"Level(
                name: "Placement test",
                segments: [{segments}],
                placements: [{placements}],
                connections: [{connections}],
                player_spawn: (4.0, 2.0, 4.0),
            )"#
        );
        ron::from_str(&ron).expect("test level should parse")
    }

    #[test]
    fn a_root_is_placed_at_its_declared_transform() {
        let l = level(
            &["a"],
            r#"Root(segment: "a", origin: (10.0, 2.0, -4.0), yaw: 90.0)"#,
            "",
        );
        let frames = resolve_placements(&l).expect("should resolve");
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].origin(), Point3::new(10.0, 2.0, -4.0));
        assert_eq!(frames[0].quarter_turns(), 1);
    }

    /// The load-bearing property: a joined segment's anchor ends up `gap` from
    /// its parent's anchor, facing back at it.
    #[test]
    fn a_join_separates_the_anchors_by_the_gap() {
        let l = level(
            &["a", "b"],
            r#"Root(segment: "a"),
               Join(segment: "b", anchor: "west", to: "a.east", gap: 6.0)"#,
            "",
        );
        let frames = resolve_placements(&l).expect("should resolve");

        let parent = world_anchor(&l, &frames, "a.east").unwrap();
        let child = world_anchor(&l, &frames, "b.west").unwrap();
        assert!(((child.origin() - parent.origin()).norm() - 6.0).abs() < 1e-4);
        assert!((outward(&parent) + outward(&child)).norm() < 1e-4);

        // 'a' spans x ∈ [0, 32] with its east anchor at x = 32, so 'b' starts
        // 6 m further on.
        assert!((frames[1].origin().x - 38.0).abs() < 1e-4);
    }

    #[test]
    fn a_segment_with_two_placements_is_over_determined() {
        let l = level(
            &["a", "b"],
            r#"Root(segment: "a"),
               Join(segment: "b", anchor: "west", to: "a.east", gap: 4.0),
               Join(segment: "b", anchor: "east", to: "a.west", gap: 4.0)"#,
            "",
        );
        assert!(matches!(
            resolve_placements(&l),
            Err(PlacementError::OverDetermined { .. })
        ));
    }

    #[test]
    fn a_segment_with_no_placement_is_an_orphan() {
        let l = level(&["a", "b"], r#"Root(segment: "a")"#, "");
        assert_eq!(
            resolve_placements(&l),
            Err(PlacementError::Orphan("b".to_string()))
        );
    }

    #[test]
    fn a_placement_loop_is_a_cycle() {
        let l = level(
            &["a", "b", "c"],
            r#"Root(segment: "a"),
               Join(segment: "b", anchor: "west", to: "c.east", gap: 4.0),
               Join(segment: "c", anchor: "west", to: "b.east", gap: 4.0)"#,
            "",
        );
        match resolve_placements(&l) {
            Err(PlacementError::Cycle(names)) => {
                assert_eq!(names, vec!["b".to_string(), "c".to_string()])
            }
            other => panic!("expected a cycle, got {other:?}"),
        }
    }

    #[test]
    fn two_roots_are_rejected() {
        let l = level(&["a", "b"], r#"Root(segment: "a"), Root(segment: "b")"#, "");
        assert!(matches!(
            resolve_placements(&l),
            Err(PlacementError::MultipleRoots(_))
        ));
    }

    #[test]
    fn no_root_is_rejected() {
        let l = level(
            &["a", "b"],
            r#"Join(segment: "a", anchor: "west", to: "b.east", gap: 4.0),
               Join(segment: "b", anchor: "west", to: "a.east", gap: 4.0)"#,
            "",
        );
        assert_eq!(resolve_placements(&l), Err(PlacementError::NoRoot));
    }

    #[test]
    fn a_welded_join_is_rejected_as_not_implemented() {
        let l = level(
            &["a", "b"],
            r#"Root(segment: "a"),
               Join(segment: "b", anchor: "west", to: "a.east", gap: 0.0, weld: true)"#,
            "",
        );
        assert_eq!(
            resolve_placements(&l),
            Err(PlacementError::WeldNotImplemented("b".to_string()))
        );
    }

    #[test]
    fn a_non_quarter_yaw_is_rejected() {
        let l = level(&["a"], r#"Root(segment: "a", yaw: 45.0)"#, "");
        assert!(matches!(
            resolve_placements(&l),
            Err(PlacementError::BadYaw { .. })
        ));
    }

    #[test]
    fn an_unknown_anchor_is_rejected() {
        let l = level(
            &["a", "b"],
            r#"Root(segment: "a"),
               Join(segment: "b", anchor: "west", to: "a.nowhere", gap: 4.0)"#,
            "",
        );
        assert!(matches!(
            resolve_placements(&l),
            Err(PlacementError::UnknownAnchor { .. })
        ));
    }

    #[test]
    fn a_malformed_anchor_reference_is_rejected() {
        let l = level(
            &["a", "b"],
            r#"Root(segment: "a"),
               Join(segment: "b", anchor: "west", to: "aeast", gap: 4.0)"#,
            "",
        );
        assert!(matches!(
            resolve_placements(&l),
            Err(PlacementError::MalformedAnchorRef(_))
        ));
    }

    /// A chain through a rotated parent still lands where the anchors say, which
    /// is the point of deriving transforms rather than authoring them.
    #[test]
    fn joins_compose_through_a_rotated_parent() {
        let l = level(
            &["a", "b", "c"],
            r#"Root(segment: "a", yaw: 90.0),
               Join(segment: "b", anchor: "west", to: "a.east", gap: 5.0),
               Join(segment: "c", anchor: "west", to: "b.east", gap: 5.0)"#,
            "",
        );
        let frames = resolve_placements(&l).expect("should resolve");
        for (parent, child) in [("a.east", "b.west"), ("b.east", "c.west")] {
            let p = world_anchor(&l, &frames, parent).unwrap();
            let c = world_anchor(&l, &frames, child).unwrap();
            assert!(
                ((c.origin() - p.origin()).norm() - 5.0).abs() < 1e-3,
                "{parent} → {child}"
            );
        }
    }
}
