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
/// The planner's sphere casts in the physics world: the terrain and every
/// body, rubble and spawnables included.
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
