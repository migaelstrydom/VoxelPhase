use nalgebra::{Isometry3, Point3, Unit, UnitVector3, Vector3};
use traversal_planner::section::Footing;

use crate::physics::{PhysicsWorld, RigidBodyHandle};

/// A lip the hands can hold, in the frame of the body it belongs to: wherever
/// that body takes it, the edge goes too.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge {
    /// The body the lip is on; `None` for terrain and other static geometry.
    pub body: Option<RigidBodyHandle>,
    /// Where each hand holds, on the top just behind the lip, in the body's
    /// frame (the world's for terrain).
    pub hands: [Point3<f32>; 2],
    /// Horizontal at the grab, pointing out of the face, away from the top:
    /// the way a drop or a back-jump leaves. In the body's frame.
    pub outward: UnitVector3<f32>,
    /// How far behind the face the hands hold, in metres: the face is this
    /// far out from them, along `outward`.
    pub face_depth: f32,
    /// Where the body's axis stands after a pull-up, on the top, in the
    /// body's frame; `None` if the top is too narrow, too steep or too low
    /// under a roof to stand on.
    pub stand: Option<Point3<f32>>,
    /// What the body is as footing.
    pub footing: Footing,
}

/// An edge the hands found this frame, as the locomotion state reads it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensedEdge {
    /// The edge, in its body's frame.
    pub edge: Edge,
    /// Where it is this frame.
    pub at: WorldEdge,
    /// Whether the edge carries on past each hand far enough to shimmy
    /// that way: `room[i]` is towards `at.hands[i]`.
    pub room: [bool; 2],
}

/// An [`Edge`] read in the world this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldEdge {
    /// Where each hand holds.
    pub hands: [Point3<f32>; 2],
    /// Out of the face, away from the top.
    pub outward: UnitVector3<f32>,
    /// How far behind the face the hands hold, in metres.
    pub face_depth: f32,
    /// Where the body's axis stands after a pull-up.
    pub stand: Option<Point3<f32>>,
}

impl Edge {
    /// The edge found at world points, kept in the frame of `body` as it
    /// stands in `world` now.
    pub fn from_world(
        world: &PhysicsWorld,
        body: Option<RigidBodyHandle>,
        found: &WorldEdge,
        footing: Footing,
    ) -> Option<Self> {
        let to_local = frame_of(world, body)?.inverse();
        Some(Self {
            body,
            hands: found.hands.map(|hand| to_local * hand),
            outward: Unit::new_unchecked(to_local * found.outward.into_inner()),
            face_depth: found.face_depth,
            stand: found.stand.map(|stand| to_local * stand),
            footing,
        })
    }

    /// The edge where its body has taken it; `None` once the body is gone.
    pub fn in_world(&self, world: &PhysicsWorld) -> Option<WorldEdge> {
        let to_world = frame_of(world, self.body)?;
        Some(WorldEdge {
            hands: self.hands.map(|hand| to_world * hand),
            outward: Unit::new_unchecked(to_world * self.outward.into_inner()),
            face_depth: self.face_depth,
            stand: self.stand.map(|stand| to_world * stand),
        })
    }

    /// Whether the body can be pulled up onto the top.
    pub fn climbable(&self) -> bool {
        self.stand.is_some()
    }

    /// `found`, this lip found again, with the hands where this edge has
    /// them along it and then `slide` metres on towards `found.hands[1]`: its
    /// height, face and way out as found, its place along it as held. Taken
    /// from the casts alone, the place would follow the body, which drifts
    /// behind an edge that moves, and the hands would slip along the lip.
    pub fn held_along(&self, found: &Edge, slide: f32) -> Self {
        let mid = |e: &Edge| nalgebra::center(&e.hands[0], &e.hands[1]);
        let span = found.hands[1] - found.hands[0];
        let outward = found.outward.into_inner();
        let Some(along) = (span - outward * span.dot(&outward)).try_normalize(1.0e-4) else {
            return *found;
        };
        let shift = along * ((mid(self) - mid(found)).dot(&along) + slide);
        Self {
            hands: found.hands.map(|hand| hand + shift),
            stand: found.stand.map(|stand| stand + shift),
            ..*found
        }
    }
}

impl WorldEdge {
    /// Midway between the hands: where the grip holds.
    pub fn grip(&self) -> Point3<f32> {
        nalgebra::center(&self.hands[0], &self.hands[1])
    }

    /// On the face, level with the grip and out from it.
    pub fn face(&self) -> Point3<f32> {
        self.grip() + self.outward.into_inner() * self.face_depth
    }

    /// Along the edge, from the first hand to the second, level and unit
    /// length.
    pub fn along(&self) -> Vector3<f32> {
        let level = Vector3::y().cross(&self.outward).normalize();
        if level.dot(&(self.hands[1] - self.hands[0])) < 0.0 {
            -level
        } else {
            level
        }
    }
}

/// The pose of `body` in the world: the identity for static geometry, and
/// `None` for a body that is gone.
pub fn frame_of(world: &PhysicsWorld, body: Option<RigidBodyHandle>) -> Option<Isometry3<f32>> {
    match body {
        None => Some(Isometry3::identity()),
        Some(handle) => world
            .body(handle)
            .map(|body| Isometry3::from_parts(body.position().coords.into(), body.rotation())),
    }
}
