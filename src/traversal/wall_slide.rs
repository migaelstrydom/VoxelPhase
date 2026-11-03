use nalgebra::{Point3, Vector3};
use traversal_planner::params::BodyDims;

use crate::physics::{
    CastObstacles, CollisionFilter, PhysicsWorld, RigidBodyHandle, SphereCast, StaticGeometry,
};

use super::path_frame::PathFrame;

/// How far off parallel to a wall the heading may meet it and still slide
/// along it, in degrees. Steeper, the path runs into the wall and the plan
/// stops at it.
const SLIDE_ANGLE_DEGREES: f32 = 30.0;
/// How far off parallel to a face the bottom sphere meets the heading may be
/// for the path to slide along it, in degrees: a bank's foot converging on
/// the path, not a lip to hop.
const GRAZE_ANGLE_DEGREES: f32 = 5.0;
/// How far the path's line keeps from a wall beside it beyond the capsule's
/// radius, in metres, so the strip's casts along it clear the wall's edge.
const WALL_CLEARANCE: f32 = 0.03;
/// How far beyond the clearance a wall beside the body is looked for, in
/// metres.
const SIDE_REACH: f32 = 0.05;
/// How far back from the body a look sideways starts, in metres, so a wall
/// the capsule has sunk into a little is still met from in front.
const SIDE_START_BACK: f32 = 0.1;
/// How far ahead a wall the heading meets is slid along, in metres: about a
/// sprint's stopping distance, so the path turns before a plan into the wall
/// would brake for it, and a column metres off does not turn it.
const LOOK_AHEAD: f32 = 2.0;

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
/// capsule's middle pass under it, and stay the plan's to hop. Its bottom
/// sphere keeps the line off whatever it stands against, so the strip's floor
/// under the body is the ground, not the foot of a bank wedging the cast,
/// and follows a face it runs all but parallel to.
pub struct WallSlide<'a> {
    pub world: &'a PhysicsWorld,
    pub scenery: &'a dyn StaticGeometry,
    /// Bodies the casts pass through: the player's own.
    pub exclude: &'a [RigidBodyHandle],
    /// What the casts are, as colliders: the caster's own body's filter
    /// (`PhysicsWorld::filter_of`), so they touch what it would.
    pub filter: CollisionFilter,
    pub body: &'a BodyDims,
}

impl WallSlide<'_> {
    /// The path for a body whose bottom-sphere centre is at `bottom`, heading
    /// along horizontal unit `heading`.
    pub fn path(&self, bottom: Point3<f32>, heading: Vector3<f32>) -> LaidPath {
        let top = bottom + Vector3::y() * self.body.spine();
        let mut origin = bottom;
        let mut forward = heading;
        let mut slides = false;
        let mut slide = |forward: &mut Vector3<f32>, normal: Vector3<f32>, limit_degrees: f32| {
            let into = -forward.dot(&normal);
            if into > 0.0 && into < limit_degrees.to_radians().sin() {
                *forward = along(*forward, normal);
                slides = true;
            }
        };

        // The top sphere slides along a wall; the bottom one keeps the line
        // clear of what it stands against and slides only along a face it
        // all but parallels, so a lip met at an angle stays a hop.
        let spheres = [(bottom, GRAZE_ANGLE_DEGREES), (top, SLIDE_ANGLE_DEGREES)];
        let left = Vector3::y().cross(&heading);
        for side in [left, -left] {
            for (sphere, limit) in spheres {
                let from = sphere - side * SIDE_START_BACK;
                let length = SIDE_START_BACK + WALL_CLEARANCE + SIDE_REACH;
                if let Some((normal, distance)) = self.wall(from, side, length) {
                    origin += normal * (WALL_CLEARANCE + SIDE_START_BACK - distance).max(0.0);
                    slide(&mut forward, normal, limit);
                }
            }
        }
        for (sphere, limit) in spheres {
            if let Some((normal, _)) = self.wall(sphere + (origin - bottom), forward, LOOK_AHEAD) {
                slide(&mut forward, normal, limit);
            }
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
            filter: self.filter,
            obstacles: CastObstacles::Everything,
        };
        let hit = self.world.cast_sphere(self.scenery, &cast)?;
        // A wall is anything steeper than the body rides.
        let normal = hit.face_normal;
        if normal.y.abs() > self.body.steepest_ride().cos() {
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

#[cfg(test)]
mod tests {
    use nalgebra::UnitQuaternion;
    use traversal_planner::GaitParams;

    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    use super::*;

    /// How far a bank's face leans back from upright, in degrees: steeper
    /// than any ride, too shallow for the top sphere to meet.
    const BANK_LEAN: f32 = 18.0;

    /// A world with a fixed box of `half` extents whose `-x` face, leaned
    /// back by `lean` degrees, meets the ground (`y = 0`) at `x = foot`.
    fn world_with_face(foot: f32, half: Vector3<f32>, lean: f32) -> PhysicsWorld {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -lean.to_radians());
        let ground_corner = rotation * Vector3::new(-half.x, -half.y, 0.0);
        let block = world.create_body(
            RigidBodyDesc::static_body()
                .position(Point3::new(foot, 0.0, 0.0) - ground_corner)
                .rotation(rotation),
        );
        world.attach_collider(block, ColliderDesc::box_shape(half));
        world
    }

    fn laid(world: &PhysicsWorld, heading_degrees: f32) -> LaidPath {
        let body = GaitParams::default().body;
        let (sin, cos) = heading_degrees.to_radians().sin_cos();
        WallSlide {
            world,
            scenery: &EmptyGeometry,
            exclude: &[],
            filter: CollisionFilter::DEFAULT,
            body: &body,
        }
        .path(
            Point3::new(0.0, body.radius, 0.0),
            Vector3::new(sin, 0.0, cos),
        )
    }

    /// The bottom sphere 2 cm off a bank's leaning face: the line is moved
    /// clear of it, and a heading converging on it by 2° slides along it.
    #[test]
    fn the_line_keeps_clear_of_a_leaning_bank_and_follows_its_foot() {
        let radius = GaitParams::default().body.radius;
        let lean = BANK_LEAN.to_radians();
        // The face's distance from the bottom sphere's centre is the foot's
        // across it plus the sphere's height along its normal.
        let foot = (radius + 0.02 - radius * lean.sin()) / lean.cos();
        let world = world_with_face(foot, Vector3::new(1.0, 3.0, 20.0), BANK_LEAN);

        let along = laid(&world, 0.0);
        assert!(
            along
                .frame
                .world_point(traversal_planner::PathVec::new(0.0, 0.0, 0.0))
                .x
                < -0.005,
            "moved off the bank"
        );
        assert!(!along.slides, "parallel to it already");

        let converging = laid(&world, 2.0);
        assert!(converging.slides, "slides along the bank's foot");
        assert!(
            converging.frame.forward().x.abs() < 1.0e-3,
            "{:?}",
            converging.frame.forward()
        );
    }

    /// A 25 cm lip 0.6 m to the side, met at 20°, stays the plan's to hop.
    #[test]
    fn a_lip_met_at_an_angle_is_not_slid_along() {
        let world = world_with_face(0.6, Vector3::new(1.0, 0.125, 20.0), 0.0);
        let laid = laid(&world, 20.0);
        assert!(!laid.slides);
    }
}
