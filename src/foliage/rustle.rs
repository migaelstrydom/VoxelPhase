//! Leaves knocked off by whatever moves through a clump.
//!
//! A body moving through leaves sheds them in proportion to the distance it
//! travels inside them, more per metre the faster it goes: a walk through a
//! hedge leaves a trickle, a crate thrown through one a spray. Each body's
//! share in each clump gathers until it makes a burst, thrown from the body.

use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;
use specs::{Builder, Entities, Entity, Join, LazyUpdate, Read, ReadExpect, ReadStorage, System};

use super::components::Foliage;
use super::drag::immersion_in;
use super::systems::leaf_emitter;
use crate::components::{Position, RigidBodyComponent, Velocity};
use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// How a body moving through leaves knocks them off.
#[derive(Clone, Copy, Debug)]
pub struct Rustle {
    /// Slowest speed through the leaves, m/s, that knocks any off.
    pub least_speed: f32,
    /// Leaves knocked off per metre travelled wholly inside leaves at
    /// `reference_speed`. Faster sheds more per metre, by the square root of
    /// the speed.
    pub leaves_per_metre: f32,
    /// The speed, m/s, `leaves_per_metre` is measured at: a wade.
    pub reference_speed: f32,
    /// Leaves gathered before a burst is thrown, so a slow wade throws a few
    /// small bursts rather than a leaf a frame.
    pub least_burst: f32,
    /// The most leaves one frame's burst throws, however fast the body.
    pub most_burst: u32,
    /// Share of the body's velocity its leaves are thrown with: swept along
    /// behind it.
    pub carried: f32,
}

impl Default for Rustle {
    fn default() -> Self {
        Self {
            least_speed: 0.5,
            leaves_per_metre: 25.0,
            reference_speed: 3.0,
            least_burst: 3.0,
            most_burst: 40,
            carried: 0.3,
        }
    }
}

impl Rustle {
    /// Leaves knocked off by `dt` seconds at `speed` through leaves `depth`
    /// deep (0 clear of them, 1 wholly inside).
    pub fn leaves(&self, depth: f32, speed: f32, dt: f32) -> f32 {
        if speed < self.least_speed {
            return 0.0;
        }
        let per_metre = self.leaves_per_metre * (speed / self.reference_speed).sqrt();
        per_metre * depth * speed * dt
    }
}

/// Throws leaves off clumps where bodies move through them.
///
/// Runs after physics, on the velocities the step left.
#[derive(Default)]
pub struct FoliageRustleSystem {
    rustle: Rustle,
    /// Leaves owed by each (clump, body) pair still in contact, short of a
    /// burst. A pair that parts forgets what it owed.
    owed: FxHashMap<(Entity, RigidBodyHandle), f32>,
}

impl<'a> System<'a> for FoliageRustleSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, PhysicsResource>,
        Read<'a, Time>,
        Read<'a, LazyUpdate>,
        ReadStorage<'a, Foliage>,
        ReadStorage<'a, RigidBodyComponent>,
    );

    fn run(&mut self, (entities, physics, time, lazy, foliage, bodies): Self::SystemData) {
        let world = &physics.world;
        let dt = time.delta_seconds();
        let movers: Vec<Mover> = bodies
            .join()
            .filter_map(|b| Mover::of(world, b.0, self.rustle.least_speed))
            .collect();
        let mut owed = FxHashMap::default();
        if !movers.is_empty() {
            for (clump_entity, clump) in (&entities, &foliage).join() {
                let Some(reach) = Reach::of(clump, world) else {
                    continue;
                };
                let volume = clump.drag_volume();
                let carrier = clump.body();
                let clump_velocity = carrier
                    .and_then(|h| world.body(h))
                    .map_or(Vector3::zeros(), |b| b.linear_velocity());
                for mover in movers.iter().filter(|m| Some(m.handle) != carrier) {
                    if !reach.touches(mover) {
                        continue;
                    }
                    let speed = (mover.velocity - clump_velocity).magnitude();
                    let depth = immersion_in(&volume, world, mover.centre, mover.radius);
                    let fresh = self.rustle.leaves(depth, speed, dt);
                    if fresh <= 0.0 {
                        continue;
                    }
                    let key = (clump_entity, mover.handle);
                    let total = self.owed.get(&key).copied().unwrap_or(0.0) + fresh;
                    if total < self.rustle.least_burst {
                        owed.insert(key, total);
                        continue;
                    }
                    let count = (total as u32).min(self.rustle.most_burst);
                    lazy.create_entity(&entities)
                        .with(Position(mover.centre.coords))
                        .with(Velocity(mover.velocity))
                        .with(leaf_emitter(count).with_velocity_inheritance(self.rustle.carried))
                        .build();
                    owed.insert(key, total.fract());
                }
            }
        }
        self.owed = owed;
    }
}

/// An awake body moving fast enough to knock leaves off.
struct Mover {
    handle: RigidBodyHandle,
    centre: Point3<f32>,
    radius: f32,
    velocity: Vector3<f32>,
}

impl Mover {
    fn of(world: &PhysicsWorld, handle: RigidBodyHandle, least_speed: f32) -> Option<Self> {
        let body = world.body(handle)?;
        if !body.is_dynamic() || world.is_sleeping(handle) {
            return None;
        }
        let velocity = body.linear_velocity();
        (velocity.magnitude() >= least_speed).then(|| Self {
            handle,
            centre: body.position(),
            radius: world.reach(handle),
            velocity,
        })
    }
}

/// A sphere round a clump's leaves, to pass over bodies nowhere near it
/// before measuring how deep they are.
struct Reach {
    centre: Point3<f32>,
    radius: f32,
}

impl Reach {
    fn of(clump: &Foliage, world: &PhysicsWorld) -> Option<Self> {
        let pose = clump.pose(world)?;
        let (low, high) = clump.shape.bounds();
        Some(Self {
            centre: pose * Point3::from((low.coords + high.coords) * 0.5),
            radius: (high - low).magnitude() * 0.5,
        })
    }

    fn touches(&self, mover: &Mover) -> bool {
        (mover.centre - self.centre).magnitude() < self.radius + mover.radius
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_slow_body_knocks_nothing_off() {
        let rustle = Rustle::default();
        assert_eq!(rustle.leaves(1.0, 0.4, 1.0), 0.0);
        assert!(rustle.leaves(1.0, 0.6, 1.0) > 0.0);
    }

    #[test]
    fn a_throw_through_a_shrub_sheds_more_than_a_wade() {
        let rustle = Rustle::default();
        // A wade: 2 m of hedge at 3 m/s, half in it.
        let wade = rustle.leaves(0.5, 3.0, 2.0 / 3.0);
        // A throw: 1.5 m of hedge at 20 m/s, mostly in it.
        let throw = rustle.leaves(0.8, 20.0, 1.5 / 20.0);
        assert!(
            (wade - rustle.leaves_per_metre).abs() < 0.5,
            "wade sheds {wade}"
        );
        assert!(throw > 2.5 * wade, "throw sheds {throw}, wade {wade}");
    }
}
