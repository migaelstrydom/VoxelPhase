use nalgebra::{Point3, Vector3};
use traversal_planner::section::Footing;

use crate::physics::{
    CastObstacles, Layers, PhysicsWorld, RigidBodyHandle, SphereCast, SphereCastHit, StaticGeometry,
};

/// How fast a dynamic body's surface may move, in m/s, to be seen as
/// standing: faster, it is in flight, not ground, and gone from where it is
/// before a foot or a hand could hold it. A blast's pieces are passed through
/// until they slow.
pub const IN_FLIGHT_SPEED: f32 = 2.0;
/// How fast a dynamic body's surface may move, in m/s, and still be resting
/// footing. Whatever the player stands on is awake, so asleep is no test.
const RESTING_SPEED: f32 = 0.05;

/// Whether a dynamic body is at rest as footing: asleep, or awake but still.
pub fn is_resting(world: &PhysicsWorld, handle: RigidBodyHandle) -> bool {
    world.is_sleeping(handle) || world.surface_speed(handle) <= RESTING_SPEED
}

/// Sphere casts in world space through the terrain and every body but those
/// in flight, each hit read as footing. A carrier is seen however fast it
/// moves, and is fixed ground where it stands. The planner's strip
/// (`PhysicsCaster`) and the hands' ledge sensor both cast through it.
pub struct WorldCaster<'a> {
    pub world: &'a PhysicsWorld,
    pub scenery: &'a dyn StaticGeometry,
    /// Bodies the casts pass through: the caster's own.
    pub exclude: &'a [RigidBodyHandle],
    /// Bodies that carry what stands on them.
    pub carriers: &'a [RigidBodyHandle],
}

/// What a [`WorldCaster`] cast touched.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldHit {
    /// The touch, as the physics world reports it.
    pub contact: SphereCastHit,
    /// The sphere's centre at the touch.
    pub centre: Point3<f32>,
    /// What the body touched is as footing.
    pub footing: Footing,
}

impl WorldCaster<'_> {
    /// A sphere of `radius` swept from `origin` along the unit `direction`
    /// for `length` metres.
    pub fn cast(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
        radius: f32,
    ) -> Option<WorldHit> {
        let cast = SphereCast {
            origin,
            direction,
            length,
            radius,
            exclude: self.exclude,
            solid_bodies: false,
            layers: Layers::ALL,
            obstacles: CastObstacles::Settled {
                fastest: IN_FLIGHT_SPEED,
                seen: self.carriers,
            },
        };
        let contact = self.world.cast_sphere(self.scenery, &cast)?;
        Some(WorldHit {
            contact,
            centre: cast.centre_at(contact.distance),
            footing: contact
                .body
                .map_or(Footing::Fixed, |handle| self.footing(handle)),
        })
    }

    /// What `handle` is as footing: loose if it is dynamic and carries
    /// nothing, fixed otherwise.
    pub fn footing(&self, handle: RigidBodyHandle) -> Footing {
        match self.world.body(handle) {
            Some(body) if body.is_dynamic() && !self.carriers.contains(&handle) => Footing::Loose {
                mass: body.mass(),
                resting: is_resting(self.world, handle),
            },
            _ => Footing::Fixed,
        }
    }
}
