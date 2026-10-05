//! Keeping the boulders a level holds to a number the solver can carry.
//!
//! A hill's shell comes down as thirty bodies, and every blast adds more. Past
//! `keep`, the smallest boulder that has had its `grace` crumbles to dust
//! where it lies, as scree does where it lands, and leaves the physics world.
//! Boulders have a budget of their own, not the props' shards': a collapse
//! would otherwise take every shard of a broken window with it, and the
//! shards' budget is sized for glass, not rock.
//!
//! ```text
//!   RubbleSpawnSystem ──▶ boulder + ResidentBoulder { volume, crumble, age }
//!                                         │
//!   BoulderCullSystem ── rank by volume ──┘
//!     past `keep` and `grace`: crumble at its position, body removed,
//!     entity deleted
//! ```
//!
//! This is the exception: a boulder at rest is deposited back into the
//! terrain (`settle`), and only those that never settle are left to the cull.

use specs::{
    Builder, Component, Entities, Entity, Join, Read, ReadStorage, System, VecStorage, Write,
    WriteStorage,
};

use super::dust::{Crumble, CrumbleSize};
use crate::components::{Position, RigidBodyComponent};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// A boulder the budget may take away.
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct ResidentBoulder {
    /// Its volume, the measure boulders are ranked by, in m³.
    pub volume: f32,
    /// The crumble it leaves if it is taken.
    pub crumble: CrumbleSize,
    /// Seconds since it broke away.
    pub age: f32,
}

impl ResidentBoulder {
    pub fn new(volume: f32, crumble: CrumbleSize) -> Self {
        Self {
            volume,
            crumble,
            age: 0.0,
        }
    }
}

/// Takes away the smallest boulders past the budget.
pub struct BoulderCullSystem {
    /// Most boulders kept, the largest first.
    pub keep: usize,
    /// Seconds a boulder is guaranteed, however small: a collapse is seen
    /// whole before anything is taken from it.
    pub grace: f32,
    crumble: Crumble,
}

impl Default for BoulderCullSystem {
    fn default() -> Self {
        Self {
            keep: 120,
            grace: 4.0,
            crumble: Crumble::default(),
        }
    }
}

impl BoulderCullSystem {
    /// Which of `boulders` (entity, volume, age) go: everything outside the
    /// largest `keep` that has had its grace. Young boulders count in the
    /// ranking, so a new large one displaces an old small one.
    pub fn over_budget(&self, boulders: &[(Entity, f32, f32)]) -> Vec<Entity> {
        let mut by_size: Vec<&(Entity, f32, f32)> = boulders.iter().collect();
        by_size.sort_by(|a, b| b.1.total_cmp(&a.1));
        by_size
            .into_iter()
            .skip(self.keep)
            .filter(|(_, _, age)| *age >= self.grace)
            .map(|(entity, _, _)| *entity)
            .collect()
    }
}

impl<'a> System<'a> for BoulderCullSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        Read<'a, Time>,
        Read<'a, specs::LazyUpdate>,
        WriteStorage<'a, ResidentBoulder>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Position>,
    );

    fn run(
        &mut self,
        (entities, mut physics, time, lazy, mut boulders, bodies, positions): Self::SystemData,
    ) {
        let dt = time.delta_seconds();
        let ranked: Vec<(Entity, f32, f32)> = (&entities, &mut boulders)
            .join()
            .map(|(entity, boulder)| {
                boulder.age += dt;
                (entity, boulder.volume, boulder.age)
            })
            .collect();

        for entity in self.over_budget(&ranked) {
            if let Some(body) = bodies.get(entity) {
                physics.world.remove_body(body.0);
            }
            if let (Some(position), Some(boulder)) = (positions.get(entity), boulders.get(entity)) {
                lazy.create_entity(&entities)
                    .with(*position)
                    .with(self.crumble.emitter(boulder.crumble))
                    .build();
            }
            let _ = entities.delete(entity);
        }
    }
}

#[cfg(test)]
mod tests {
    use specs::{World, WorldExt};

    use super::*;

    fn cull(keep: usize, grace: f32) -> BoulderCullSystem {
        BoulderCullSystem {
            keep,
            grace,
            ..BoulderCullSystem::default()
        }
    }

    #[test]
    fn the_smallest_old_boulders_go_first() {
        let mut world = World::new();
        let e: Vec<Entity> = (0..4).map(|_| world.create_entity().build()).collect();
        let boulders = vec![
            (e[0], 5.0, 10.0),
            (e[1], 0.5, 10.0),
            (e[2], 2.0, 10.0),
            (e[3], 0.3, 0.5),
        ];
        // Four boulders, two kept: the 0.5 m³ one goes; the 0.3 m³ one is
        // smaller but still in its grace.
        assert_eq!(cull(2, 4.0).over_budget(&boulders), vec![e[1]]);
        assert!(cull(4, 4.0).over_budget(&boulders).is_empty());
    }
}
