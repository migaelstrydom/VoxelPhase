use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use nalgebra::Vector3;
use traversal_planner::section::{BodyKey, CastHit, SphereCaster, StripLayout};
use traversal_planner::{GaitParams, PathVec};

use crate::physics::{CollisionFilter, PhysicsWorld, RigidBodyHandle, StaticGeometry};

use super::path_frame::PathFrame;
use super::world_caster::WorldCaster;

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

/// The planner's sphere casts in the physics world: the terrain and every
/// body, rubble and spawnables included, but those in flight. A carrier
/// (`super::carriers`) is seen however fast it moves, and read as fixed
/// ground where it stands.
///
/// Casts reach from `bottom` to `top`: one asked to start beyond either
/// starts there.
pub struct PhysicsCaster<'a> {
    pub world: &'a PhysicsWorld,
    pub scenery: &'a dyn StaticGeometry,
    pub frame: PathFrame,
    /// Bodies the casts pass through: the player's own.
    pub exclude: &'a [RigidBodyHandle],
    /// What the casts are, as colliders: the caster's own body's filter
    /// (`PhysicsWorld::filter_of`), so they touch what it would.
    pub filter: CollisionFilter,
    /// Bodies that carry what stands on them.
    pub carriers: &'a [RigidBodyHandle],
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
        let hit =
            self.world_caster()
                .cast(self.frame.world_point(from), direction, length, radius)?;
        Some(CastHit {
            z: hit.centre.y,
            normal: self.frame.path_vector(hit.contact.normal),
            body: hit.contact.body.map_or(STATIC_GEOMETRY, body_key),
            footing: hit.footing,
        })
    }

    fn world_caster(&self) -> WorldCaster<'_> {
        WorldCaster {
            world: self.world,
            scenery: self.scenery,
            exclude: self.exclude,
            filter: self.filter,
            carriers: self.carriers,
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
    use traversal_planner::section::Footing;

    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    use super::super::world_caster::IN_FLIGHT_SPEED;
    use super::*;

    /// The footing a cast down onto a dynamic box reads, if it sees it:
    /// resting asleep, and awake but still, as whatever the player stands on
    /// is; not sliding; fixed if it is a carrier.
    fn footing_of_box(awake: bool, speed: f32, carrier: bool) -> Option<Footing> {
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
        let carriers = [slab];
        let caster = PhysicsCaster {
            world: &world,
            scenery: &EmptyGeometry,
            frame: PathFrame::new(Point3::origin(), 0.0),
            exclude: &[],
            filter: CollisionFilter::DEFAULT,
            carriers: if carrier { &carriers } else { &[] },
            top: 5.0,
            bottom: -5.0,
        };
        caster
            .cast_down(0.0, 0.0, 0.25, f32::INFINITY)
            .map(|hit| hit.footing)
    }

    #[test]
    fn a_still_body_is_resting_footing_awake_or_asleep() {
        let resting = |footing| matches!(footing, Some(Footing::Loose { resting, .. }) if resting);
        assert!(resting(footing_of_box(false, 0.0, false)), "asleep");
        assert!(resting(footing_of_box(true, 0.0, false)), "awake and still");
        assert!(!resting(footing_of_box(true, 0.5, false)), "sliding");
    }

    /// A carrier is fixed ground however fast it moves, where a loose body
    /// as fast is in flight and passed through, and one slower is footing in
    /// motion.
    #[test]
    fn a_carrier_is_fixed_ground_at_any_speed() {
        for speed in [0.0, 0.5, IN_FLIGHT_SPEED, 2.5, 4.0] {
            assert_eq!(
                footing_of_box(true, speed, true),
                Some(Footing::Fixed),
                "a carrier at {speed} m/s"
            );
        }
        assert_eq!(footing_of_box(true, 2.5, false), None, "in flight");
        assert!(matches!(
            footing_of_box(true, 1.0, false),
            Some(Footing::Loose { resting: false, .. })
        ));
    }
}
