//! Foliage's ECS systems: the skin kept up with its core, and clumps torn
//! loose by blasts.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Vector3};
use smallvec::SmallVec;
use specs::{
    Builder, Entities, Join, LazyUpdate, Read, ReadExpect, ReadStorage, System, Write, WriteStorage,
};

use super::components::{Foliage, Rooting};
use super::deform::Pusher;
use crate::components::{ModelInstance, Position, ProceduralMesh, RigidBodyComponent, Velocity};
use crate::explosion::Explosion;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::particles::{Palette, ParticleEffectType, ParticleEmitter};
use crate::physics::{ColliderShape, ContactEvent, PhysicsWorld, RigidBodyHandle};
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

/// Keeps each clump's skin giving: pressed against what a loose clump rests
/// on, squashed after its landings, and parted round whatever pushes
/// through any clump's leaves; and sheds leaves where a clump lands.
///
/// A clump whose skin is changing is drawn through a `ProceduralMesh`,
/// rebuilt and uploaded every frame. One that is still is drawn as a
/// `ModelInstance` of the skin as it came to rest, uploaded once: most
/// clumps are still most of the time, and a skin is thousands of vertices.
///
/// Runs after physics, reading the contacts the step made.
pub struct FoliageSkinSystem;

impl<'a> System<'a> for FoliageSkinSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, PhysicsResource>,
        Read<'a, Time>,
        Read<'a, LazyUpdate>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, Foliage>,
        WriteStorage<'a, ProceduralMesh>,
        WriteStorage<'a, ModelInstance>,
    );

    fn run(
        &mut self,
        (entities, physics, time, lazy, bodies, mut foliage, mut meshes, mut models): Self::SystemData,
    ) {
        if foliage.is_empty() {
            return;
        }
        let dt = time.delta_seconds();
        let world = &physics.world;
        let passers = Passer::moving(world, (&bodies).join().map(|b| b.0));
        let mut scratch = Vec::new();
        for (entity, clump) in (&entities, &mut foliage).join() {
            let Some(pose) = clump.pose(world) else {
                continue;
            };
            let asleep = match clump.rooting {
                Rooting::Rooted { .. } => true,
                Rooting::Loose { body } => {
                    let asleep = world.is_sleeping(body);
                    if !asleep {
                        if let Some(impact) = feel_contacts(clump, world, body, &pose) {
                            shed_leaves(&entities, &lazy, impact.point, impact.speed);
                        }
                        clump.bending.press.age(dt);
                        clump.last_velocity = world
                            .body(body)
                            .map_or(Vector3::zeros(), |b| b.linear_velocity());
                    }
                    asleep
                }
            };
            clump.bending.squash.step(dt, &clump.give);
            clump.bending.parting.age(dt);
            let centre = Point3::from(pose.translation.vector);
            let reach = clump.shape.bounding_radius();
            let own = clump.body();
            for passer in passers.iter().filter(|p| Some(p.body) != own) {
                if passer.reaches(centre, reach) {
                    let give = clump.give;
                    for &pusher in &passer.pushers {
                        clump.bending.parting.push(passer.body, pusher, &give);
                    }
                }
            }

            let changing = !asleep || clump.bending.is_changing();
            if !changing && clump.still_drawn {
                continue;
            }
            let drawn = clump.drawn(&pose, &mut scratch);
            if changing {
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
            clump.still_drawn = !changing;
        }
    }
}

/// An awake body that might be pushing through leaves, as the capsules its
/// colliders are.
struct Passer {
    body: RigidBodyHandle,
    centre: Point3<f32>,
    reach: f32,
    pushers: SmallVec<[Pusher; 2]>,
}

impl Passer {
    /// Every awake dynamic body among `candidates`.
    fn moving(
        world: &PhysicsWorld,
        candidates: impl Iterator<Item = RigidBodyHandle>,
    ) -> Vec<Self> {
        candidates
            .filter(|&h| !world.is_sleeping(h))
            .filter_map(|body| {
                let rigid = world.body(body).filter(|b| b.is_dynamic())?;
                let pushers = rigid
                    .colliders()
                    .iter()
                    .filter_map(|&c| world.collider(c))
                    .map(|collider| {
                        pusher_of(
                            collider.shape(),
                            collider.world_transform(rigid.position(), rigid.rotation()),
                        )
                    })
                    .collect();
                Some(Self {
                    body,
                    centre: rigid.position(),
                    reach: world.reach(body),
                    pushers,
                })
            })
            .collect()
    }

    /// Whether it is near enough a clump at `centre` of `reach` to be in its
    /// leaves.
    fn reaches(&self, centre: Point3<f32>, reach: f32) -> bool {
        (self.centre - centre).magnitude() < reach + self.reach
    }
}

/// A collider as the capsule that parts leaves round it: a capsule as
/// itself, a sphere as a point, anything else as a sphere a little inside
/// its bounds.
fn pusher_of(shape: &ColliderShape, at: Isometry3<f32>) -> Pusher {
    let centre = Point3::from(at.translation.vector);
    match *shape {
        ColliderShape::Capsule {
            half_height,
            radius,
        } => {
            let half = at.rotation * Vector3::y() * (half_height - radius).max(0.0);
            Pusher {
                a: centre - half,
                b: centre + half,
                radius,
            }
        }
        ColliderShape::Sphere { radius } => Pusher {
            a: centre,
            b: centre,
            radius,
        },
        ref other => Pusher {
            a: centre,
            b: centre,
            radius: other.bounding_radius() * 0.7,
        },
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

/// Press the clump's skin against every fixed surface its core touches, and
/// kick its squash if anything stopped it hard.
///
/// Only what cannot move presses: the ground, walls, a platform. A moving
/// body (a character, a crate, another clump's core) parts the leaves round
/// its own shape instead (`Parting`), and two clumps' leaves interleave
/// rather than meeting on a flat face.
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
        if presses(world, contact, body) {
            clump
                .bending
                .press
                .touch(contact.point, normal, &clump.give);
        }
        let speed = stopped.dot(&normal);
        if speed > IMPACT_SPEED && hardest.as_ref().is_none_or(|h| speed > h.speed) {
            hardest = Some(Impact {
                point: contact.point,
                speed,
            });
            let local = pose.inverse();
            clump
                .bending
                .squash
                .kick(local * normal, local * contact.point, speed, &clump.give);
        }
    }
    hardest
}

/// Whether what `body` touches in `contact` presses its leaves flat: the
/// static geometry, or a body that does not move by itself.
fn presses(world: &PhysicsWorld, contact: &ContactEvent, body: RigidBodyHandle) -> bool {
    let other = if contact.body_b == body {
        contact.body_a
    } else {
        Some(contact.body_b)
    };
    other.is_none_or(|h| world.body(h).is_none_or(|b| !b.is_dynamic()))
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
