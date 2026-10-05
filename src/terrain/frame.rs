//! The rigid transform that places a segment in the world.
//!
//! A segment frame is a translation plus a yaw restricted to whole quarter
//! turns. That restriction is the reason the rest of the terrain code can stay
//! simple: an axis-aligned box stays axis-aligned under the transform, so a
//! world AABB query converts into a segment-local AABB query *exactly* rather
//! than conservatively, and no triangle ever needs an interpolated rotation.
//!
//! ```text
//!        local                       world
//!          +z                          +x
//!          │        quarter_turns=1     │
//!    ──────┼──── +x       ──▶     +z ───┼──────
//!          │                            │
//! ```
//!
//! The yaw sense matches nalgebra's rotation about `+Y`: for `θ = k · 90°`,
//! `x' = x·cosθ + z·sinθ` and `z' = −x·sinθ + z·cosθ`. So one quarter turn maps
//! local `+X` onto world `−Z`, and local `+Z` onto world `+X`.

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use crate::collision::AABB;

/// Degrees in one permitted yaw step.
pub const YAW_STEP_DEGREES: f32 = 90.0;

/// Tolerance when deciding whether an authored yaw is a multiple of 90°.
const YAW_SNAP_TOLERANCE: f32 = 1e-3;

/// A segment's placement: where its local origin sits, and how it is turned.
///
/// Rotation is whole quarter turns about `+Y` only. Pitch and roll are not
/// representable, by design — see design rule 2 in `LEVEL_SEGMENTS_PLAN.md`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentFrame {
    /// World position of segment-local `(0, 0, 0)`.
    origin: Point3<f32>,
    /// Yaw in quarter turns about `+Y`, normalised to `0..=3`.
    quarter_turns: u8,
}

impl Default for SegmentFrame {
    fn default() -> Self {
        Self::identity()
    }
}

impl SegmentFrame {
    /// The frame that leaves local coordinates untouched.
    pub fn identity() -> Self {
        Self {
            origin: Point3::origin(),
            quarter_turns: 0,
        }
    }

    /// A frame at `origin` turned by `quarter_turns` × 90° about `+Y`.
    pub fn new(origin: Point3<f32>, quarter_turns: i32) -> Self {
        Self {
            origin,
            quarter_turns: quarter_turns.rem_euclid(4) as u8,
        }
    }

    /// A frame from an authored yaw in degrees.
    ///
    /// Returns `None` for anything that is not a multiple of 90°, which the
    /// level loader turns into an error naming the offending segment.
    pub fn from_degrees(origin: Point3<f32>, yaw_degrees: f32) -> Option<Self> {
        let turns = yaw_degrees / YAW_STEP_DEGREES;
        let rounded = turns.round();
        if (turns - rounded).abs() > YAW_SNAP_TOLERANCE {
            return None;
        }
        Some(Self::new(origin, rounded as i32))
    }

    /// World position of segment-local `(0, 0, 0)`.
    pub fn origin(&self) -> Point3<f32> {
        self.origin
    }

    /// Yaw in quarter turns about `+Y`, in `0..=3`.
    pub fn quarter_turns(&self) -> u8 {
        self.quarter_turns
    }

    /// Yaw in degrees, in `0..360`.
    pub fn yaw_degrees(&self) -> f32 {
        self.quarter_turns as f32 * YAW_STEP_DEGREES
    }

    /// This frame's rotation as a quaternion, for code that needs a general
    /// orientation (object spawning, debug rendering).
    pub fn rotation(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), self.yaw_degrees().to_radians())
    }

    /// The frame as an isometry taking segment-local positions to the world.
    pub fn isometry(&self) -> Isometry3<f32> {
        Isometry3::from_parts(Translation3::from(self.origin.coords), self.rotation())
    }

    /// Rotate a local direction into world space. Exact: a permutation and sign
    /// flips, never an interpolation.
    pub fn rotate_to_world(&self, local: Vector3<f32>) -> Vector3<f32> {
        let (x, z) = match self.quarter_turns {
            0 => (local.x, local.z),
            1 => (local.z, -local.x),
            2 => (-local.x, -local.z),
            _ => (-local.z, local.x),
        };
        Vector3::new(x, local.y, z)
    }

    /// Rotate a world direction into this frame.
    pub fn rotate_to_local(&self, world: Vector3<f32>) -> Vector3<f32> {
        self.inverse_rotation().rotate_to_world(world)
    }

    /// Transform a segment-local position into world space.
    pub fn to_world(&self, local: Point3<f32>) -> Point3<f32> {
        self.origin + self.rotate_to_world(local.coords)
    }

    /// Transform a world position into this segment's frame.
    pub fn to_local(&self, world: Point3<f32>) -> Point3<f32> {
        Point3::from(self.rotate_to_local(world - self.origin))
    }

    /// Transform a segment-local AABB into world space.
    ///
    /// Exact, not conservative: a quarter turn permutes the axes, so the
    /// transformed corners are still the extremes of an axis-aligned box.
    pub fn aabb_to_world(&self, local: &AABB) -> AABB {
        span(self.to_world(local.min), self.to_world(local.max))
    }

    /// Transform a world AABB into this segment's frame.
    pub fn aabb_to_local(&self, world: &AABB) -> AABB {
        span(self.to_local(world.min), self.to_local(world.max))
    }

    /// The frame that undoes this one's rotation, about the world origin.
    fn inverse_rotation(&self) -> Self {
        Self {
            origin: Point3::origin(),
            quarter_turns: ((4 - self.quarter_turns as i32) % 4) as u8,
        }
    }

    /// This frame with a further whole-quarter-turn yaw applied about its own
    /// origin.
    pub fn turned(&self, extra_quarter_turns: i32) -> Self {
        Self::new(self.origin, self.quarter_turns as i32 + extra_quarter_turns)
    }

    /// `self ∘ inner`: the frame that maps `inner`'s local space to the space
    /// `self` maps into.
    ///
    /// Used to lift an anchor (a frame inside a segment) into world space by
    /// composing it under the segment's own frame.
    pub fn compose(&self, inner: &Self) -> Self {
        Self::new(
            self.to_world(inner.origin),
            self.quarter_turns as i32 + inner.quarter_turns as i32,
        )
    }

    /// The frame that undoes this one, so `self.compose(&self.inverse())` is the
    /// identity.
    pub fn inverse(&self) -> Self {
        let rotation = self.inverse_rotation();
        Self {
            origin: Point3::from(-rotation.rotate_to_world(self.origin.coords)),
            quarter_turns: rotation.quarter_turns,
        }
    }
}

/// An AABB from two corners in any order.
fn span(a: Point3<f32>, b: Point3<f32>) -> AABB {
    AABB::new(
        Point3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)),
        Point3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_frame_is_a_no_op() {
        let f = SegmentFrame::identity();
        let p = Point3::new(3.0, -2.0, 7.5);
        assert_eq!(f.to_world(p), p);
        assert_eq!(f.to_local(p), p);
    }

    #[test]
    fn world_local_round_trips_for_every_quarter_turn() {
        for turns in 0..4 {
            let f = SegmentFrame::new(Point3::new(10.0, -4.0, 2.5), turns);
            let world = Point3::new(-7.0, 3.0, 11.0);
            let local = f.to_local(world);
            let back = f.to_world(local);
            assert!(
                (back - world).norm() < 1e-5,
                "turns {turns}: {world:?} -> {local:?} -> {back:?}"
            );
        }
    }

    /// The exact-integer rotation must agree with the general quaternion form,
    /// or anything that mixes the two (object spawning, debug draw) is subtly
    /// wrong.
    #[test]
    fn quarter_turn_rotation_matches_the_quaternion() {
        for turns in 0..4 {
            let f = SegmentFrame::new(Point3::origin(), turns);
            let v = Vector3::new(1.0, 2.0, 3.0);
            let exact = f.rotate_to_world(v);
            let general = f.rotation() * v;
            assert!(
                (exact - general).norm() < 1e-5,
                "turns {turns}: exact {exact:?} vs quaternion {general:?}"
            );
        }
    }

    /// One quarter turn takes local `+X` to world `−Z`, which is the convention
    /// the anchor facing rules are written against.
    #[test]
    fn one_quarter_turn_maps_plus_x_to_minus_z() {
        let f = SegmentFrame::new(Point3::origin(), 1);
        let out = f.rotate_to_world(Vector3::x());
        assert!(
            (out - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-6,
            "{out:?}"
        );
    }

    #[test]
    fn aabb_stays_axis_aligned_and_keeps_its_size() {
        let local = AABB::new(Point3::new(-1.0, 0.0, -3.0), Point3::new(5.0, 2.0, 4.0));
        for turns in 0..4 {
            let f = SegmentFrame::new(Point3::new(100.0, 0.0, -50.0), turns);
            let world = f.aabb_to_world(&local);
            let size = world.size();
            let expected = local.size();
            let matches = (size.y - expected.y).abs() < 1e-4
                && ((size.x - expected.x).abs() < 1e-4 && (size.z - expected.z).abs() < 1e-4
                    || (size.x - expected.z).abs() < 1e-4 && (size.z - expected.x).abs() < 1e-4);
            assert!(matches, "turns {turns}: {size:?} from {expected:?}");

            let back = f.aabb_to_local(&world);
            assert!((back.min - local.min).norm() < 1e-4);
            assert!((back.max - local.max).norm() < 1e-4);
        }
    }

    #[test]
    fn only_multiples_of_ninety_degrees_are_accepted() {
        let o = Point3::origin();
        assert!(SegmentFrame::from_degrees(o, 0.0).is_some());
        assert!(SegmentFrame::from_degrees(o, 90.0).is_some());
        assert!(SegmentFrame::from_degrees(o, -270.0).is_some());
        assert!(SegmentFrame::from_degrees(o, 360.0).is_some());
        assert!(SegmentFrame::from_degrees(o, 45.0).is_none());
        assert!(SegmentFrame::from_degrees(o, 89.0).is_none());
    }

    /// Composition has to agree with applying the two transforms in turn, or
    /// anchor placement (which composes a segment frame with an anchor frame)
    /// puts segments in the wrong place.
    #[test]
    fn composition_matches_applying_both_transforms() {
        let outer = SegmentFrame::new(Point3::new(5.0, 1.0, -2.0), 1);
        let inner = SegmentFrame::new(Point3::new(-3.0, 0.5, 4.0), 2);
        let combined = outer.compose(&inner);

        let p = Point3::new(1.0, 2.0, 3.0);
        let stepwise = outer.to_world(inner.to_world(p));
        assert!((combined.to_world(p) - stepwise).norm() < 1e-4);
    }

    #[test]
    fn inverse_undoes_the_frame() {
        for turns in 0..4 {
            let f = SegmentFrame::new(Point3::new(-8.0, 3.0, 12.0), turns);
            let identity = f.compose(&f.inverse());
            assert_eq!(identity.quarter_turns(), 0, "turns {turns}");
            assert!(identity.origin().coords.norm() < 1e-4, "turns {turns}");

            let p = Point3::new(2.0, -1.0, 6.0);
            assert!((f.inverse().to_world(f.to_world(p)) - p).norm() < 1e-4);
        }
    }

    #[test]
    fn negative_yaw_normalises_into_range() {
        assert_eq!(
            SegmentFrame::from_degrees(Point3::origin(), -90.0)
                .unwrap()
                .quarter_turns(),
            3
        );
    }
}
