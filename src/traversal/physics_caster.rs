use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use nalgebra::Vector3;
use traversal_planner::section::{BodyKey, CastHit, Footing, SphereCaster, StripLayout};
use traversal_planner::{GaitParams, PathVec};

use crate::physics::{CastObstacles, PhysicsWorld, RigidBodyHandle, SphereCast, StaticGeometry};

use super::path_frame::PathFrame;

/// The key of the static geometry, which like a fixed authored shape is one
/// body.
const STATIC_GEOMETRY: BodyKey = BodyKey(0);
/// How far above the steepest rise the strip could ride its casts reach, and
/// how far below where the path was laid, in metres.
const REACH_ABOVE_RISE: f32 = 3.0;
const REACH_BELOW: f32 = 6.0;

/// How far above and below where a path is laid its strip's casts reach, in
/// metres: up, past the steepest slope the body rides climbing the strip's
/// whole length; down, a drop of a few metres. The strip follows the body's
/// layer, so reaching high reads nothing overhead as floor.
pub fn strip_reach(layout: &StripLayout, params: &GaitParams) -> (f32, f32) {
    let steepest = params.body.max_ground_slope_degrees.to_radians().tan();
    (REACH_ABOVE_RISE + layout.ahead * steepest, REACH_BELOW)
}

/// How fast a dynamic body's surface may move, in m/s, for the strip to see
/// it: faster, it is in flight, not ground, and gone from where it is before
/// a step could land on it. A blast's pieces are passed through until they
/// slow.
pub const IN_FLIGHT_SPEED: f32 = 2.0;
/// How fast a dynamic body's surface may move, in m/s, and still be resting
/// footing. Whatever the player stands on is awake, so asleep is no test.
const RESTING_SPEED: f32 = 0.05;

/// Whether a dynamic body is at rest as footing: asleep, or awake but still.
pub fn is_resting(world: &PhysicsWorld, handle: RigidBodyHandle) -> bool {
    world.is_sleeping(handle) || world.surface_speed(handle) <= RESTING_SPEED
}

/// The planner's sphere casts in the physics world: the terrain and every
/// body, rubble and spawnables included, but those in flight.
///
/// Casts reach from `bottom` to `top`: one asked to start beyond either
/// starts there.
pub struct PhysicsCaster<'a> {
    pub world: &'a PhysicsWorld,
    pub scenery: &'a dyn StaticGeometry,
    pub frame: PathFrame,
    /// Bodies the casts pass through: the player's own.
    pub exclude: &'a [RigidBodyHandle],
    /// World height casts up give up at, in metres, and casts down start
    /// from at most.
    pub top: f32,
    /// World height casts down give up at, in metres, and casts up start
    /// from at least.
    pub bottom: f32,
}

impl PhysicsCaster<'_> {
    fn cast(
        &self,
        from: PathVec,
        direction: Vector3<f32>,
        length: f32,
        radius: f32,
    ) -> Option<CastHit> {
        let cast = SphereCast {
            origin: self.frame.world_point(from),
            direction,
            length,
            radius,
            exclude: self.exclude,
            solid_bodies: false,
            obstacles: CastObstacles::Settled {
                fastest: IN_FLIGHT_SPEED,
            },
        };
        let hit = self.world.cast_sphere(self.scenery, &cast)?;
        let (body, footing) = match hit.body {
            None => (STATIC_GEOMETRY, Footing::Fixed),
            Some(handle) => (body_key(handle), self.footing(handle)),
        };
        Some(CastHit {
            z: cast.centre_at(hit.distance).y,
            normal: self.frame.path_vector(hit.normal),
            body,
            footing,
        })
    }

    fn footing(&self, handle: RigidBodyHandle) -> Footing {
        match self.world.body(handle) {
            Some(body) if body.is_dynamic() => Footing::Loose {
                mass: body.mass(),
                resting: is_resting(self.world, handle),
            },
            _ => Footing::Fixed,
        }
    }
}

/// A key that tells one body from another within a capture.
fn body_key(handle: RigidBodyHandle) -> BodyKey {
    let mut hasher = DefaultHasher::new();
    handle.hash(&mut hasher);
    BodyKey(hasher.finish().max(1))
}

impl SphereCaster for PhysicsCaster<'_> {
    fn cast_down(&self, s: f32, d: f32, radius: f32, from: f32) -> Option<CastHit> {
        let from = from.min(self.top);
        if from <= self.bottom {
            return None;
        }
        self.cast(
            PathVec::new(s, d, from),
            -Vector3::y(),
            from - self.bottom,
            radius,
        )
    }

    fn cast_up(&self, s: f32, d: f32, radius: f32, from: f32) -> Option<CastHit> {
        let from = from.max(self.bottom);
        if from >= self.top {
            return None;
        }
        self.cast(
            PathVec::new(s, d, from),
            Vector3::y(),
            self.top - from,
            radius,
        )
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    use super::*;

    /// The footing a cast down onto a dynamic box reads: resting asleep, and
    /// awake but still, as whatever the player stands on is; not sliding.
    fn footing_of_box(awake: bool, speed: f32) -> Footing {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let slab = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, 0.5, 0.0))
                .linear_velocity(Vector3::x() * speed),
        );
        world.attach_collider(slab, ColliderDesc::box_shape(Vector3::new(2.0, 0.5, 2.0)));
        if awake {
            world.wake_body(slab);
        }
        let caster = PhysicsCaster {
            world: &world,
            scenery: &EmptyGeometry,
            frame: PathFrame::new(Point3::origin(), 0.0),
            exclude: &[],
            top: 5.0,
            bottom: -5.0,
        };
        caster
            .cast_down(0.0, 0.0, 0.25, f32::INFINITY)
            .expect("the box under the cast")
            .footing
    }

    #[test]
    fn a_still_body_is_resting_footing_awake_or_asleep() {
        let resting = |footing| matches!(footing, Footing::Loose { resting, .. } if resting);
        assert!(resting(footing_of_box(false, 0.0)), "asleep");
        assert!(resting(footing_of_box(true, 0.0)), "awake and still");
        assert!(!resting(footing_of_box(true, 0.5)), "sliding");
    }
}
