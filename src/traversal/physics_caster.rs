use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use nalgebra::Vector3;
use traversal_planner::section::{BodyKey, CastHit, Footing, SphereCaster};
use traversal_planner::PathVec;

use crate::physics::{CastObstacles, PhysicsWorld, RigidBodyHandle, SphereCast, StaticGeometry};

use super::path_frame::PathFrame;

/// The key of the static geometry, which like a fixed authored shape is one
/// body.
const STATIC_GEOMETRY: BodyKey = BodyKey(0);
/// How far an upward cast starts above the floor it rises from, in metres,
/// so it does not begin touching it.
const CEILING_START_GAP: f32 = 0.01;

/// The planner's sphere casts in the physics world: the terrain and every
/// body, rubble and spawnables included.
///
/// Casts down start at `top`, so an overhang below it reads as floor; casts
/// up rise from the floor under the same point.
pub struct PhysicsCaster<'a> {
    pub world: &'a PhysicsWorld,
    pub scenery: &'a dyn StaticGeometry,
    pub frame: PathFrame,
    /// Bodies the casts pass through: the player's own.
    pub exclude: &'a [RigidBodyHandle],
    /// World height casts down start from, in metres.
    pub top: f32,
    /// World height casts down give up at, in metres.
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
            obstacles: CastObstacles::Everything,
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
                resting: self.world.is_sleeping(handle),
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
    fn cast_down(&self, s: f32, d: f32, radius: f32) -> Option<CastHit> {
        self.cast(
            PathVec::new(s, d, self.top),
            -Vector3::y(),
            self.top - self.bottom,
            radius,
        )
    }

    fn cast_up(&self, s: f32, d: f32, radius: f32) -> Option<CastHit> {
        let floor = self.cast_down(s, d, radius)?;
        let from = floor.z + CEILING_START_GAP;
        self.cast(
            PathVec::new(s, d, from),
            Vector3::y(),
            self.top - from,
            radius,
        )
    }
}
