//! A trail of leaves behind a loose clump flying through the air.
//!
//! The air strips leaves off a clump moving fast through it, a steady few
//! per metre flown: a clump thrown or blasted draws a trail behind it, one
//! rolling slowly to rest does not. The trail is a continuous emitter on the
//! clump's own entity, which spreads its leaves along the path flown each
//! frame.

use specs::{Entities, Join, ReadExpect, ReadStorage, System, WriteStorage};

use super::components::Foliage;
use super::systems::leaves;
use crate::particles::ParticleEmitter;
use crate::physics::PhysicsWorld;
use crate::systems::PhysicsResource;

/// How the air strips leaves off a flying clump.
#[derive(Clone, Copy, Debug)]
pub struct Trail {
    /// Slowest speed, m/s, at which the air strips leaves: above a clump
    /// rolling or tumbling to rest.
    pub least_speed: f32,
    /// Leaves stripped per metre flown.
    pub leaves_per_metre: f32,
    /// Share of the clump's velocity its leaves leave with: they peel off
    /// it rather than stop dead beside it.
    pub carried: f32,
}

impl Default for Trail {
    fn default() -> Self {
        Self {
            least_speed: 4.0,
            leaves_per_metre: 3.0,
            carried: 0.4,
        }
    }
}

impl Trail {
    /// Leaves a second stripped off a clump flying at `speed`.
    pub fn rate(&self, speed: f32) -> f32 {
        if speed < self.least_speed {
            0.0
        } else {
            self.leaves_per_metre * speed
        }
    }
}

/// Keeps each loose clump's trail emitter at the rate its speed calls for.
///
/// Runs after physics, on the velocities the step left.
#[derive(Default)]
pub struct FoliageTrailSystem {
    trail: Trail,
}

impl<'a> System<'a> for FoliageTrailSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, PhysicsResource>,
        ReadStorage<'a, Foliage>,
        WriteStorage<'a, ParticleEmitter>,
    );

    fn run(&mut self, (entities, physics, foliage, mut emitters): Self::SystemData) {
        for (entity, clump) in (&entities, &foliage).join() {
            let rate = self.trail.rate(air_speed(clump, &physics.world));
            match emitters.get_mut(entity) {
                Some(emitter) => emitter.spawn_rate = rate,
                None if rate > 0.0 => {
                    let emitter = leaves()
                        .with_spawn_rate(rate)
                        .with_velocity_inheritance(self.trail.carried);
                    let _ = emitters.insert(entity, emitter);
                }
                None => {}
            }
        }
    }
}

/// How fast a clump moves through the still air: its core's speed, or 0
/// rooted or asleep.
fn air_speed(clump: &Foliage, world: &PhysicsWorld) -> f32 {
    clump
        .body()
        .filter(|&h| !world.is_sleeping(h))
        .and_then(|h| world.body(h))
        .map_or(0.0, |b| b.linear_velocity().magnitude())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_a_fast_clump_trails_leaves() {
        let trail = Trail::default();
        assert_eq!(trail.rate(2.0), 0.0);
        // A clump blasted 30 m strips a few dozen leaves along the way.
        let flight = 30.0;
        let leaves = trail.rate(15.0) * flight / 15.0;
        assert!(
            (60.0..=120.0).contains(&leaves),
            "{leaves} leaves over {flight} m"
        );
    }
}
