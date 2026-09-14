//! Keeping the pieces of a break from outliving their usefulness.
//!
//! A break is sold in its first second: the burst, the tumble, the shards
//! skittering across the floor. What is left after that is a pile of small
//! bodies leaning on each other, every pair a narrowphase query and a set of
//! solver rows, and thin enough that their manifolds never settle and the
//! island never sleeps. The pile costs more than the break did and shows less.
//!
//! ```text
//!   FractureSystem ──▶ freed piece + Debris{origin, size, age}
//!                                         │
//!                DebrisBudget ──▶ DebrisCullSystem ──▶ rank by size,
//!                                         │            per origin and overall
//!                                         ▼
//!                       past grace and over budget: glitter burst, body
//!                       removed, entity deleted
//! ```
//!
//! Pieces are ranked by size, not age, so the survivors are the large shards a
//! player would look at rather than whichever happened to be freed first; the
//! grace period is what lets every piece take part in the burst before the
//! small ones are taken away.

use serde::Deserialize;
use specs::{
    Builder, Component, Entities, Entity, Join, Read, ReadStorage, System, VecStorage, Write,
};

use crate::components::{Position, RigidBodyComponent};
use crate::particles::{ParticleEffectType, ParticleEmitter};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// A freed piece that the debris budget may take away.
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct Debris {
    /// The compound this piece came off, so a budget can be kept per object
    /// rather than letting one shattered window evict every other break's
    /// pieces.
    pub origin: Entity,
    /// Volume of the piece, the measure the budget ranks by. A shard's area
    /// times the sheet's thickness; a box's full volume.
    pub size: f32,
    /// Seconds since the piece was freed.
    pub age: f32,
}

impl Debris {
    pub fn new(origin: Entity, size: f32) -> Self {
        Self {
            origin,
            size,
            age: 0.0,
        }
    }
}

/// How many pieces of a break are allowed to stay.
///
/// Level-wide, so a level can trade pile fidelity against frame time in one
/// place. Missing from the level file means the defaults below.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default)]
pub struct DebrisBudget {
    /// Largest pieces kept per broken object.
    pub keep_per_origin: usize,
    /// Largest pieces kept across every broken object put together.
    pub keep_total: usize,
    /// Seconds a piece is guaranteed before the budget may take it, however
    /// small. The burst of a break is made of the small pieces; taking them
    /// as they are freed makes every break look thin.
    pub grace: f32,
    /// Particles sparkled off a piece as it goes, to cover its going.
    pub glitter: u32,
}

impl Default for DebrisBudget {
    fn default() -> Self {
        Self {
            keep_per_origin: 10,
            keep_total: 40,
            grace: 1.5,
            glitter: 8,
        }
    }
}

impl DebrisBudget {
    /// Which of `pieces` the budget does not keep: everything outside the
    /// largest `keep_per_origin` of its origin or the largest `keep_total`
    /// overall, once it has had its grace.
    ///
    /// Ranking counts every piece, young ones included, so a young large
    /// piece displaces an old small one and never the other way round: the
    /// set of survivors depends only on what exists, not on the order pieces
    /// were freed or judged in.
    pub fn over_budget(&self, pieces: &[(Entity, Debris)]) -> Vec<Entity> {
        let mut by_size: Vec<&(Entity, Debris)> = pieces.iter().collect();
        by_size.sort_by(|a, b| b.1.size.total_cmp(&a.1.size));

        let mut per_origin: Vec<(Entity, usize)> = Vec::new();
        let mut culled = Vec::new();
        for (rank, (entity, debris)) in by_size.iter().enumerate() {
            let origin_rank = match per_origin.iter_mut().find(|(o, _)| *o == debris.origin) {
                Some((_, count)) => {
                    *count += 1;
                    *count
                }
                None => {
                    per_origin.push((debris.origin, 1));
                    1
                }
            };
            let kept = rank < self.keep_total && origin_rank <= self.keep_per_origin;
            if !kept && debris.age >= self.grace {
                culled.push(*entity);
            }
        }
        culled
    }
}

/// Takes away the pieces the [`DebrisBudget`] does not keep.
///
/// A culled piece leaves the physics world and the ECS in the same frame and
/// leaves a glitter burst behind at its last position, so the eye has
/// something to read where a shard was rather than a shard that is not.
pub struct DebrisCullSystem;

impl<'a> System<'a> for DebrisCullSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        Read<'a, DebrisBudget>,
        Read<'a, Time>,
        Read<'a, specs::LazyUpdate>,
        specs::WriteStorage<'a, Debris>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Position>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, mut physics, budget, time, lazy, mut debris, bodies, positions) = data;
        let dt = time.delta_seconds();

        let pieces: Vec<(Entity, Debris)> = (&entities, &mut debris)
            .join()
            .map(|(entity, piece)| {
                piece.age += dt;
                (entity, *piece)
            })
            .collect();

        for entity in budget.over_budget(&pieces) {
            if let Some(body) = bodies.get(entity) {
                physics.world.remove_body(body.0);
            }
            if let Some(position) = positions.get(entity) {
                sparkle(&entities, &lazy, *position, budget.glitter);
            }
            let _ = entities.delete(entity);
        }
    }
}

/// Seconds a glitter emitter lives. It fires its burst on its first update,
/// and an emitter's life runs down *before* it fires, so a zero-length life
/// dies with nothing spawned; this only has to outlast one frame.
const GLITTER_EMITTER_LIFETIME: f32 = 0.05;

/// The emitter for one glitter burst of `count` flecks, which deletes itself
/// once the burst is out.
fn glitter_emitter(count: u32) -> ParticleEmitter {
    ParticleEmitter::new(ParticleEffectType::ShardGlitter)
        .with_burst(count)
        .with_lifetime(GLITTER_EMITTER_LIFETIME)
}

/// A one-off glitter burst at `position`.
fn sparkle(entities: &Entities, lazy: &specs::LazyUpdate, position: Position, count: u32) {
    if count == 0 {
        return;
    }
    lazy.create_entity(entities)
        .with(position)
        .with(glitter_emitter(count))
        .build();
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;
    use crate::particles::{step_emitter, EmitterMotion, ParticleConfig, ParticlePool};
    use crate::physics::RigidBodyDesc;

    fn entities(n: usize) -> (World, Vec<Entity>) {
        let mut world = World::new();
        let made = (0..n).map(|_| world.create_entity().build()).collect();
        (world, made)
    }

    fn budget() -> DebrisBudget {
        DebrisBudget {
            keep_per_origin: 2,
            keep_total: 3,
            grace: 1.0,
            glitter: 4,
        }
    }

    fn aged(origin: Entity, size: f32, age: f32) -> Debris {
        Debris { origin, size, age }
    }

    #[test]
    fn the_smallest_pieces_of_one_break_go_first() {
        let (_world, e) = entities(4);
        let pieces = vec![
            (e[1], aged(e[0], 0.5, 2.0)),
            (e[2], aged(e[0], 0.1, 2.0)),
            (e[3], aged(e[0], 0.3, 2.0)),
        ];
        assert_eq!(budget().over_budget(&pieces), vec![e[2]]);
    }

    #[test]
    fn a_young_piece_is_spared_but_still_counts() {
        let (_world, e) = entities(4);
        let pieces = vec![
            (e[1], aged(e[0], 0.5, 0.2)),
            (e[2], aged(e[0], 0.4, 0.2)),
            (e[3], aged(e[0], 0.1, 2.0)),
        ];
        assert_eq!(budget().over_budget(&pieces), vec![e[3]]);

        let all_young: Vec<_> = pieces
            .iter()
            .map(|(en, d)| (*en, aged(d.origin, d.size, 0.0)))
            .collect();
        assert!(budget().over_budget(&all_young).is_empty());
    }

    #[test]
    fn a_culled_piece_leaves_the_physics_world_and_a_glint_behind() {
        let mut world = World::new();
        world.register::<Debris>();
        world.register::<Position>();
        world.register::<RigidBodyComponent>();
        world.register::<ParticleEmitter>();
        world.insert(Time::fixed(1.0));
        world.insert(PhysicsResource::default());
        world.insert(DebrisBudget {
            keep_per_origin: 1,
            ..budget()
        });

        let origin = world.create_entity().build();
        let mut pieces = Vec::new();
        for size in [0.5, 0.1] {
            let handle = world
                .write_resource::<PhysicsResource>()
                .world
                .create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 2.0, 0.0)));
            let piece = world
                .create_entity()
                .with(Debris::new(origin, size))
                .with(Position(Vector3::new(0.0, 2.0, 0.0)))
                .with(RigidBodyComponent(handle))
                .build();
            pieces.push((piece, handle));
        }

        for _ in 0..2 {
            DebrisCullSystem.run_now(&world);
            world.maintain();
        }

        let (kept, kept_body) = pieces[0];
        let (culled, culled_body) = pieces[1];
        assert!(world.is_alive(kept));
        assert!(!world.is_alive(culled));
        let physics = world.read_resource::<PhysicsResource>();
        assert!(physics.world.body(kept_body).is_some());
        assert!(physics.world.body(culled_body).is_none());
        let glints = world.read_storage::<ParticleEmitter>().join().count();
        assert_eq!(glints, 1);
    }

    #[test]
    fn a_glitter_emitter_actually_spawns_its_flecks() {
        let config = ParticleConfig::default();
        let mut pool = ParticlePool::default();
        let mut emitter = glitter_emitter(6);
        let motion = EmitterMotion {
            previous: Vector3::zeros(),
            current: Vector3::zeros(),
            inherited_velocity: Vector3::zeros(),
        };
        step_emitter(
            &mut emitter,
            config.spec(ParticleEffectType::ShardGlitter),
            1.0 / 60.0,
            motion,
            &mut pool,
            &mut rand::thread_rng(),
        );
        assert_eq!(pool.count(), 6);
    }

    #[test]
    fn the_overall_cap_reaches_across_breaks() {
        let (_world, e) = entities(6);
        let pieces = vec![
            (e[2], aged(e[0], 0.9, 2.0)),
            (e[3], aged(e[0], 0.8, 2.0)),
            (e[4], aged(e[1], 0.7, 2.0)),
            (e[5], aged(e[1], 0.6, 2.0)),
        ];
        assert_eq!(budget().over_budget(&pieces), vec![e[5]]);
    }
}
