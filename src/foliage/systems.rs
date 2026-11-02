//! Foliage's ECS systems: the skin kept up with its core, and clumps torn
//! loose by blasts.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Vector3};
use specs::{
    Builder, Entities, Join, LazyUpdate, Read, ReadExpect, ReadStorage, System, Write, WriteStorage,
};

use super::components::{Foliage, Rooting};
use crate::components::{ModelInstance, Position, ProceduralMesh, RigidBodyComponent, Velocity};
use crate::explosion::Explosion;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::particles::{Palette, ParticleEffectType, ParticleEmitter};
use crate::physics::{ContactEvent, PhysicsWorld, RigidBodyHandle};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// Speed, m/s, a clump must strike something at to squash and shed leaves:
/// above what settling and rolling put into its contacts.
const IMPACT_SPEED: f32 = 1.2;
/// Leaves shed per m/s of impact.
const LEAVES_PER_IMPACT: f32 = 5.0;
/// The most leaves one impact sheds.
const MOST_LEAVES: u32 = 40;
/// Seconds a leaf emitter lives: long enough to fire its burst.
const LEAF_EMITTER_LIFETIME: f32 = 0.05;

/// The greens a clump's shed leaves are drawn in.
pub const LEAF_COLOURS: [Vector3<f32>; 2] = [
    Vector3::new(0.16, 0.27, 0.06),
    Vector3::new(0.10, 0.20, 0.04),
];

/// Keeps each loose clump's skin pressed against what it rests on and
/// squashed after a landing, and sheds leaves where it lands.
///
/// A clump moving, or still ringing from a landing, is drawn through a
/// `ProceduralMesh`, rebuilt and uploaded every frame. One that is still is
/// drawn as a `ModelInstance` of the skin as it came to rest, uploaded once:
/// most clumps are still most of the time, and a skin is thousands of
/// vertices.
///
/// Runs after physics, reading the contacts the step made.
pub struct FoliageSkinSystem;

impl<'a> System<'a> for FoliageSkinSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, PhysicsResource>,
        Read<'a, Time>,
        Read<'a, LazyUpdate>,
        WriteStorage<'a, Foliage>,
        WriteStorage<'a, ProceduralMesh>,
        WriteStorage<'a, ModelInstance>,
    );

    fn run(
        &mut self,
        (entities, physics, time, lazy, mut foliage, mut meshes, mut models): Self::SystemData,
    ) {
        let dt = time.delta_seconds();
        let world = &physics.world;
        let mut scratch = Vec::new();
        for (entity, clump) in (&entities, &mut foliage).join() {
            let Rooting::Loose { body } = clump.rooting else {
                continue;
            };
            let Some(core) = world.body(body) else {
                continue;
            };
            let pose = Isometry3::from_parts(core.position().coords.into(), core.rotation());
            let asleep = world.is_sleeping(body);
            if !asleep {
                if let Some(impact) = feel_contacts(clump, world, body, &pose) {
                    shed_leaves(&entities, &lazy, impact.point, impact.speed);
                }
                clump.press.age(dt);
                clump.last_velocity = core.linear_velocity();
            }
            clump.squash.step(dt, &clump.give);

            let moving = !asleep || !clump.squash.is_settled();
            if !moving && clump.still_drawn {
                continue;
            }
            let drawn = clump.drawn(&pose, &mut scratch);
            if moving {
                models.remove(entity);
                let _ = meshes.insert(
                    entity,
                    ProceduralMesh {
                        primitives: vec![drawn],
                    },
                );
            } else {
                meshes.remove(entity);
                let _ = models.insert(entity, still_model(drawn));
            }
            clump.still_drawn = !moving;
        }
    }
}

/// A skin that has stopped changing, as a model uploaded once.
pub fn still_model(drawn: MeshPrimitive) -> ModelInstance {
    ModelInstance::new(Arc::new(Model::flat(vec![ModelPart::new(vec![drawn])])))
}

/// The hardest landing among a clump's contacts this frame.
struct Impact {
    point: Point3<f32>,
    speed: f32,
}

/// Press the clump's skin against every surface its core touches, and kick
/// its squash if one of them stopped it hard.
///
/// A landing is told from resting by the velocity the contact took away:
/// the core's change in velocity since last frame along the contact's
/// normal. Gravity alone puts a sixth of a metre a second into that each
/// frame, well under [`IMPACT_SPEED`].
fn feel_contacts(
    clump: &mut Foliage,
    world: &PhysicsWorld,
    body: RigidBodyHandle,
    pose: &Isometry3<f32>,
) -> Option<Impact> {
    let core = world.body(body)?;
    let centre = core.position();
    let stopped = core.linear_velocity() - clump.last_velocity;
    let mut hardest: Option<Impact> = None;
    for contact in world.contact_events().iter().filter(|c| involves(c, body)) {
        // Out of the surface and into the clump, whichever body is which.
        let normal = if (centre - contact.point).dot(&contact.normal) < 0.0 {
            -contact.normal
        } else {
            contact.normal
        };
        clump.press.touch(contact.point, normal, &clump.give);
        let speed = stopped.dot(&normal);
        if speed > IMPACT_SPEED && hardest.as_ref().is_none_or(|h| speed > h.speed) {
            hardest = Some(Impact {
                point: contact.point,
                speed,
            });
            let local = pose.inverse();
            clump
                .squash
                .kick(local * normal, local * contact.point, speed, &clump.give);
        }
    }
    hardest
}

fn involves(contact: &ContactEvent, body: RigidBodyHandle) -> bool {
    contact.body_b == body || contact.body_a == Some(body)
}

/// A burst of leaves at `point`, more for a harder landing.
fn shed_leaves(entities: &Entities, lazy: &LazyUpdate, point: Point3<f32>, speed: f32) {
    let count = ((speed * LEAVES_PER_IMPACT).round() as u32).min(MOST_LEAVES);
    if count == 0 {
        return;
    }
    lazy.create_entity(entities)
        .with(Position(point.coords))
        .with(leaf_emitter(count))
        .build();
}

/// An emitter that throws `count` leaves once and is gone.
pub fn leaf_emitter(count: u32) -> ParticleEmitter {
    ParticleEmitter::new(ParticleEffectType::Leaves)
        .with_burst(count)
        .with_palette(Palette::mixed(&[
            (LEAF_COLOURS[0], 1.0),
            (LEAF_COLOURS[1], 1.0),
        ]))
        .with_lifetime(LEAF_EMITTER_LIFETIME)
}

/// Share of an explosion's knockback radius within which it tears a rooted
/// clump loose.
const UPROOT_SHARE: f32 = 0.6;
/// Leaves a clump sheds as it is torn loose.
const UPROOT_LEAVES: u32 = 60;

/// Tears rooted clumps loose when a blast goes off near them: each becomes a
/// loose clump on a core of its own, which the blast's shove, delivered by
/// the next physics step, throws.
///
/// Runs after `ExplosionSystem` queued the shove, so the new core is there
/// to receive it.
pub struct FoliageUprootSystem;

impl<'a> System<'a> for FoliageUprootSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        Read<'a, LazyUpdate>,
        ReadStorage<'a, Explosion>,
        WriteStorage<'a, Foliage>,
        WriteStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, Velocity>,
    );

    fn run(
        &mut self,
        (entities, mut physics, lazy, explosions, mut foliage, mut bodies, mut velocities): Self::SystemData,
    ) {
        let blasts: Vec<(Point3<f32>, f32)> = explosions
            .join()
            .map(|e| (e.center, e.blast_radius * UPROOT_SHARE))
            .collect();
        if blasts.is_empty() {
            return;
        }
        for (entity, clump) in (&entities, &mut foliage).join() {
            let Rooting::Rooted { pose } = clump.rooting else {
                continue;
            };
            let centre = Point3::from(pose.translation.vector);
            if !blasts
                .iter()
                .any(|(at, reach)| (centre - at).magnitude() < *reach)
            {
                continue;
            }
            let body = clump.core.spawn(&mut physics.world, &clump.shape, pose);
            physics.world.wake_body(body);
            clump.rooting = Rooting::Loose { body };
            clump.still_drawn = false;
            let _ = bodies.insert(entity, RigidBodyComponent(body));
            let _ = velocities.insert(entity, Velocity(Vector3::zeros()));
            lazy.create_entity(&entities)
                .with(Position(centre.coords))
                .with(leaf_emitter(UPROOT_LEAVES))
                .build();
        }
    }
}
