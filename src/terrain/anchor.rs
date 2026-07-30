//! Named local frames within a segment.
//!
//! An anchor is *not* only a place where two segments join. It is a named frame
//! in segment-local space, and it exists to serve three jobs: joining segments
//! (implemented), mounting mobile geometry, and supplying waypoints for a motion
//! path (neither implemented — see `LEVEL_SEGMENTS_PLAN.md`). Modelling it as a
//! frame rather than as a join point is what keeps the other two from needing a
//! redesign.
//!
//! # Facing convention
//!
//! **An anchor's local `+X` points outward, out of the segment.**
//!
//! ```text
//!        segment "plaza"                    segment "tower"
//!   ┌───────────────────────┐   gap   ┌───────────────────────┐
//!   │                       │◀──────▶ │                       │
//!   │              exit_east│         │entry                  │
//!   │                    ●──┼──▶ +X   │  +X ◀──●              │
//!   │                       │         │                       │
//!   └───────────────────────┘         └───────────────────────┘
//! ```
//!
//! Mating two anchors is therefore a **180° relative yaw**: the child's anchor
//! is turned to face back at the parent's, and the two origins are separated by
//! `gap` metres along the parent anchor's outward direction. An anchor on the
//! east face of a segment has yaw 0; one on the north face (`−Z`) has yaw 90°,
//! because a quarter turn maps local `+X` onto `−Z`.
//!
//! Getting this backwards is the mistake every level author makes first: if a
//! segment lands *inside* its neighbour instead of beside it, the anchor is
//! facing inward.

use nalgebra::{Point3, Vector3};

use super::frame::SegmentFrame;

/// A named frame inside a segment.
#[derive(Debug, Clone, PartialEq)]
pub struct Anchor {
    /// Unique within its segment. Referred to as `segment.anchor` in level files.
    name: String,
    /// Position and yaw in segment-local coordinates.
    frame: SegmentFrame,
}

impl Anchor {
    pub fn new(name: impl Into<String>, frame: SegmentFrame) -> Self {
        Self {
            name: name.into(),
            frame,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// Position and yaw in segment-local coordinates.
    pub fn frame(&self) -> &SegmentFrame {
        &self.frame
    }

    /// Segment-local position.
    pub fn position(&self) -> Point3<f32> {
        self.frame.origin()
    }

    /// Lift this anchor into world space under its segment's frame.
    pub fn world_frame(&self, segment: &SegmentFrame) -> SegmentFrame {
        segment.compose(&self.frame)
    }
}

/// The outward direction of an anchor frame — its local `+X`.
pub fn outward(frame: &SegmentFrame) -> Vector3<f32> {
    frame.rotate_to_world(Vector3::x())
}

/// The segment frame that mates `child_anchor` onto an already-placed
/// `parent_anchor`, separated by `gap` metres.
///
/// `parent_anchor` is in world space; `child_anchor` is in the child segment's
/// local space. The result is the child segment's world frame.
///
/// The mating rule is the facing convention above: the child anchor ends up
/// `gap` metres along the parent anchor's outward direction, turned 180° so the
/// two outward directions oppose.
pub fn mate(parent_anchor: &SegmentFrame, child_anchor: &SegmentFrame, gap: f32) -> SegmentFrame {
    let target = SegmentFrame::new(
        parent_anchor.origin() + outward(parent_anchor) * gap,
        parent_anchor.quarter_turns() as i32 + 2,
    );
    // Solve F ∘ child_anchor = target for the child segment's frame F.
    target.compose(&child_anchor.inverse())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The defining property: after mating, the two anchors are `gap` apart and
    /// face each other.
    #[test]
    fn mated_anchors_face_each_other_across_the_gap() {
        for parent_turns in 0..4 {
            for child_turns in 0..4 {
                let parent = SegmentFrame::new(Point3::new(12.0, 3.0, -7.0), parent_turns);
                let child_local = SegmentFrame::new(Point3::new(-4.0, 1.0, 2.0), child_turns);
                let gap = 6.0;

                let child_segment = mate(&parent, &child_local, gap);
                let child_world = child_segment.compose(&child_local);

                let separation = child_world.origin() - parent.origin();
                assert!(
                    (separation.norm() - gap).abs() < 1e-3,
                    "parent {parent_turns} child {child_turns}: separation {}",
                    separation.norm()
                );
                assert!(
                    (outward(&parent) + outward(&child_world)).norm() < 1e-4,
                    "parent {parent_turns} child {child_turns}: anchors do not oppose"
                );
                // The child anchor lies along the parent's outward direction.
                assert!((separation.normalize() - outward(&parent)).norm() < 1e-3);
            }
        }
    }

    /// A zero gap makes the two anchor frames coincide in position — the welded
    /// configuration, which the loader rejects but the maths still describes.
    #[test]
    fn zero_gap_makes_the_anchors_coincident() {
        let parent = SegmentFrame::new(Point3::new(1.0, 2.0, 3.0), 2);
        let child_local = SegmentFrame::new(Point3::new(8.0, 0.0, -1.0), 1);
        let world = mate(&parent, &child_local, 0.0).compose(&child_local);
        assert!((world.origin() - parent.origin()).norm() < 1e-4);
    }

    #[test]
    fn outward_is_local_plus_x() {
        assert!((outward(&SegmentFrame::new(Point3::origin(), 0)) - Vector3::x()).norm() < 1e-6);
        assert!((outward(&SegmentFrame::new(Point3::origin(), 1)) + Vector3::z()).norm() < 1e-6);
    }
}
