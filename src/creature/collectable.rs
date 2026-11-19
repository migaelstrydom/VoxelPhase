//! Things the player catches by running into them.
//!
//! The counterpart to a creature you avoid: a creature you chase. The
//! catching is deliberately the whole of the interaction — no button, no
//! range check the player has to aim — because what makes it a chase is
//! that the creature is faster to turn than you are.

use nalgebra::{Point3, Vector3};
use specs::{
    Builder, Component, DenseVecStorage, Entities, Join, LazyUpdate, Read, ReadStorage, System,
    Write, WriteStorage,
};

use crate::components::Position;
use crate::damage::Health;
use crate::debug::DebugLines;
use crate::objective::LevelProgress;
use crate::particles::{Palette, ParticleEffectType, ParticleEmitter};
use crate::player::Player;
use crate::time::Time;

/// What catching something is worth.
#[derive(Clone, Copy, Debug)]
pub enum Reward {
    /// Hit points, restored to the catcher.
    Heart { value: f32 },
    /// One of the level's Platonic solids.
    Solid,
}

/// Something the player collects by touching it.
#[derive(Component, Clone, Copy, Debug)]
#[storage(DenseVecStorage)]
pub struct Collectable {
    /// How close the player has to get. The sum of the two bodies' radii
    /// plus a little slack — a collectable that demands an exact overlap
    /// reads as broken rather than as difficult.
    pub reach: f32,
    pub reward: Reward,
    /// Seconds before this can be caught at all.
    ///
    /// Without it, a creature that spawns on top of the player is
    /// collected on its first frame and the chase never happens.
    pub arming_delay: f32,
}

impl Collectable {
    /// A heart worth `value` hit points, catchable from `reach` metres.
    pub fn heart(reach: f32, value: f32) -> Self {
        Self {
            reach,
            reward: Reward::Heart { value },
            arming_delay: 0.5,
        }
    }

    /// One of the level's solids, taken from `reach` metres.
    ///
    /// Armed immediately: a solid does not move, so there is no chase to
    /// spoil and nothing to gain by making the player wait.
    pub fn solid(reach: f32) -> Self {
        Self {
            reach,
            reward: Reward::Solid,
            arming_delay: 0.0,
        }
    }
}

/// Sparks shed by a solid as it is taken, in its gold.
const GLINT_COLOURS: [Vector3<f32>; 2] =
    [Vector3::new(2.6, 1.9, 0.9), Vector3::new(2.2, 1.3, 0.45)];
/// How many, how far they are thrown (as a scale on the glitter's own),
/// and how long the emitter lives once it has thrown them, s.
const GLINT_SPARKS: u32 = 48;
const GLINT_SCALE: f32 = 2.2;
const GLINT_LIFETIME: f32 = 0.1;

/// The burst of gold a solid goes out in when it is taken.
fn solid_glint() -> ParticleEmitter {
    ParticleEmitter::new(ParticleEffectType::ShardGlitter)
        .with_palette(Palette::mixed(&[
            (GLINT_COLOURS[0], 1.0),
            (GLINT_COLOURS[1], 1.0),
        ]))
        .with_scale(GLINT_SCALE)
        .with_burst(GLINT_SPARKS)
        .with_lifetime(GLINT_LIFETIME)
}

/// Hands collectables to the player who runs into them.
///
/// Deletes the entity outright. Nothing about a caught collectable
/// survives, so there is no corpse to despawn later and no state to get
/// out of step.
pub struct CollectionSystem;

impl<'a> System<'a> for CollectionSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, Collectable>,
        WriteStorage<'a, Health>,
        Write<'a, LevelProgress>,
        Write<'a, DebugLines>,
        Read<'a, LazyUpdate>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            time,
            players,
            positions,
            mut collectables,
            mut healths,
            mut progress,
            mut debug,
            lazy,
        ) = data;
        let dt = time.delta_seconds();

        let Some((player, player_position)) = (&entities, &players, &positions)
            .join()
            .next()
            .map(|(entity, _, position)| (entity, Point3::from(position.0)))
        else {
            return;
        };

        let mut caught = Vec::new();
        for (entity, position, collectable) in (&entities, &positions, &mut collectables).join() {
            if collectable.arming_delay > 0.0 {
                collectable.arming_delay -= dt;
                continue;
            }
            if (Point3::from(position.0) - player_position).magnitude() <= collectable.reach {
                caught.push((entity, collectable.reward, position.0));
            }
        }

        for (entity, reward, at) in caught {
            match reward {
                Reward::Heart { value } => {
                    if let Some(health) = healths.get_mut(player) {
                        health.current = (health.current + value).min(health.max);
                        debug.add("Caught", format!("heart +{value:.0}"));
                    }
                }
                Reward::Solid => {
                    progress.take_solid();
                    lazy.create_entity(&entities)
                        .with(Position(at))
                        .with(solid_glint())
                        .build();
                }
            }
            let _ = entities.delete(entity);
        }
    }
}
