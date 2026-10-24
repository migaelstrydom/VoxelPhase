use nalgebra::{Point3, Vector3};
use traversal_planner::params::BodyDims;

use crate::physics::{CastObstacles, PhysicsWorld, RigidBodyHandle, SphereCast, StaticGeometry};

use super::path_frame::PathFrame;

/// How far off parallel to a wall the heading may meet it and still slide
/// along it, in degrees. Steeper, the path runs into the wall and the plan
/// stops at it.
const SLIDE_ANGLE_DEGREES: f32 = 30.0;
/// How far the path's line keeps from a wall beside it beyond the capsule's
/// radius, in metres, so the strip's casts along it clear the wall's edge.
const WALL_CLEARANCE: f32 = 0.03;
/// How far beyond the clearance a wall beside the body is looked for, in
/// metres.
const SIDE_REACH: f32 = 0.05;
/// How far back from the body a look sideways starts, in metres, so a wall
/// the capsule has sunk into a little is still met from in front.
const SIDE_START_BACK: f32 = 0.1;
/// Most a face's normal may lean up or down for it to be a wall, as a
/// fraction of its length.
const WALL_NORMAL_Y: f32 = 0.3;

/// A path laid by [`WallSlide`].
pub struct LaidPath {
    pub frame: PathFrame,
    /// Whether a wall turned it from the heading.
    pub slides: bool,
}

/// Where the path is laid, as the capsule will move: along the heading, or
/// along a wall the heading meets nearly parallel, as the capsule slides on
/// it.
///
/// ```text
///   top view       wall ════════════════════
///                         ↗ heading
///                  body ●──────────────────→ path, clear of the wall
/// ```
///
/// The walls are found by the capsule's top sphere: lips lower than the
/// capsule's middle pass under it, and stay the plan's to hop.
pub struct WallSlide<'a> {
    pub world: &'a PhysicsWorld,
    pub scenery: &'a dyn StaticGeometry,
    /// Bodies the casts pass through: the player's own.
    pub exclude: &'a [RigidBodyHandle],
    pub body: &'a BodyDims,
}

impl WallSlide<'_> {
    /// The path for a body whose bottom-sphere centre is at `bottom`, heading
    /// along horizontal unit `heading`, looking `reach` metres ahead for a
    /// wall to slide along.
    pub fn path(&self, bottom: Point3<f32>, heading: Vector3<f32>, reach: f32) -> LaidPath {
        let top = bottom + Vector3::y() * self.body.spine();
        let mut origin = bottom;
        let mut forward = heading;
        let mut slides = false;
        let mut slide = |forward: &mut Vector3<f32>, normal: Vector3<f32>| {
            let into = -forward.dot(&normal);
            if into > 0.0 && into < SLIDE_ANGLE_DEGREES.to_radians().sin() {
                *forward = along(*forward, normal);
                slides = true;
            }
        };

        let left = Vector3::y().cross(&heading);
        for side in [left, -left] {
            let from = top - side * SIDE_START_BACK;
            let length = SIDE_START_BACK + WALL_CLEARANCE + SIDE_REACH;
            if let Some((normal, distance)) = self.wall(from, side, length) {
                origin += normal * (WALL_CLEARANCE + SIDE_START_BACK - distance).max(0.0);
                slide(&mut forward, normal);
            }
        }
        if let Some((normal, _)) = self.wall(top + (origin - bottom), forward, reach) {
            slide(&mut forward, normal);
        }
        LaidPath {
            frame: PathFrame::along(origin, forward),
            slides,
        }
    }

    /// The wall a sphere the capsule's size cast from `from` along `direction`
    /// meets within `length`: its horizontal normal and how far the sphere
    /// got.
    fn wall(
        &self,
        from: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Option<(Vector3<f32>, f32)> {
        let cast = SphereCast {
            origin: from,
            direction,
            length,
            radius: self.body.radius,
            exclude: self.exclude,
            solid_bodies: false,
            obstacles: CastObstacles::Everything,
        };
        let hit = self.world.cast_sphere(self.scenery, &cast)?;
        let normal = hit.face_normal;
        if normal.y.abs() > WALL_NORMAL_Y {
            return None;
        }
        let horizontal = Vector3::new(normal.x, 0.0, normal.z).try_normalize(f32::EPSILON)?;
        Some((horizontal, hit.distance))
    }
}

/// `forward`, pressing into the wall of horizontal normal `normal`, turned
/// along it.
fn along(forward: Vector3<f32>, normal: Vector3<f32>) -> Vector3<f32> {
    (forward - normal * forward.dot(&normal))
        .try_normalize(f32::EPSILON)
        .unwrap_or(forward)
}
