//! Foliage's ECS systems: leaves shed where a loose clump lands, and clumps
//! torn loose by blasts.

use nalgebra::{Point3, Translation3, Vector3};
use specs::{
    Builder, Entities, Join, LazyUpdate, Read, ReadExpect, ReadStorage, System, Write, WriteStorage,
};

use super::components::{Foliage, Rooting};
use crate::components::{Position, RigidBodyComponent, Velocity};
use crate::explosion::Explosion;
use crate::particles::{Palette, ParticleEffectType, ParticleEmitter};
use crate::physics::{ContactEvent, PhysicsWorld, RigidBodyHandle};
use crate::systems::PhysicsResource;

/// Speed, m/s, a clump must strike something at to shed leaves: above what
/// settling and rolling put into its contacts.
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

/// Sheds leaves where a loose clump lands hard.
///
/// Runs after physics, reading the contacts the step made.
pub struct FoliageLandingSystem;

impl<'a> System<'a> for FoliageLandingSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, PhysicsResource>,
        Read<'a, LazyUpdate>,
        WriteStorage<'a, Foliage>,
    );

    fn run(&mut self, (entities, physics, lazy, mut foliage): Self::SystemData) {
        let world = &physics.world;
        for clump in (&mut foliage).join() {
            let Some(body) = clump.body() else {
                continue;
            };
            if world.is_sleeping(body) {
                continue;
            }
            if let Some(impact) = hardest_landing(clump, world, body) {
                shed_leaves(&entities, &lazy, impact.point, impact.speed);
            }
            clump.last_velocity = world
                .body(body)
                .map_or(Vector3::zeros(), |b| b.linear_velocity());
        }
    }
}

/// The hardest landing among a clump's contacts this frame.
struct Impact {
    point: Point3<f32>,
    speed: f32,
}

/// The hardest of the clump's contacts this frame, if any stopped it hard.
///
/// A landing is told from resting by the velocity the contact took away:
/// the core's change in velocity since last frame along the contact's
/// normal. Gravity alone puts a sixth of a metre a second into that each
/// frame, well under [`IMPACT_SPEED`]. Only the speed it was moving into the
/// surface at can be taken away, so a clump launched off the ground, by a
/// blast or a kick, does not land.
fn hardest_landing(clump: &Foliage, world: &PhysicsWorld, body: RigidBodyHandle) -> Option<Impact> {
    let core = world.body(body)?;
    let centre = core.position();
    let stopped = core.linear_velocity() - clump.last_velocity;
    world
        .contact_events()
        .iter()
        .filter(|c| involves(c, body))
        .filter_map(|contact| {
            // Out of the surface and into the clump, whichever body is which.
            let normal = if (centre - contact.point).dot(&contact.normal) < 0.0 {
                -contact.normal
            } else {
                contact.normal
            };
            let speed = landing_speed(stopped, clump.last_velocity, normal);
            (speed > IMPACT_SPEED).then_some(Impact {
                point: contact.point,
                speed,
            })
        })
        .max_by(|a, b| a.speed.total_cmp(&b.speed))
}

/// The speed a contact with outward `normal` stopped a core at, given the
/// velocity it took away and the core's velocity before: no more than the
/// core was moving into the surface.
fn landing_speed(stopped: Vector3<f32>, before: Vector3<f32>, normal: Vector3<f32>) -> f32 {
    stopped.dot(&normal).min(-before.dot(&normal))
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
    leaves()
        .with_burst(count)
        .with_lifetime(LEAF_EMITTER_LIFETIME)
}

/// Leaves in a clump's greens, emitted as the caller sets.
pub fn leaves() -> ParticleEmitter {
    ParticleEmitter::new(ParticleEffectType::Leaves).with_palette(Palette::mixed(&[
        (LEAF_COLOURS[0], 1.0),
        (LEAF_COLOURS[1], 1.0),
    ]))
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
            let Rooting::Rooted { pose, sunk } = clump.rooting else {
                continue;
            };
            let centre = Point3::from(pose.translation.vector);
            if !blasts
                .iter()
                .any(|(at, reach)| (centre - at).magnitude() < *reach)
            {
                continue;
            }
            // Lifted clear of the ground it grew sunk into.
            let freed = Translation3::new(0.0, sunk, 0.0) * pose;
            let body = clump.core.spawn(&mut physics.world, &clump.shape, freed);
            physics.world.wake_body(body);
            clump.rooting = Rooting::Loose { body };
            let _ = bodies.insert(entity, RigidBodyComponent(body));
            let _ = velocities.insert(entity, Velocity(Vector3::zeros()));
            lazy.create_entity(&entities)
                .with(Position(centre.coords))
                .with(leaf_emitter(UPROOT_LEAVES))
                .build();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_landing_is_a_fall_stopped_not_a_launch() {
        let up = Vector3::y();
        let falling = Vector3::new(1.0, -5.0, 0.0);
        let landed = landing_speed(Vector3::new(0.0, 5.0, 0.0), falling, up);
        assert!((landed - 5.0).abs() < 1e-6);
        // Launched off the ground at 15 m/s: not a landing.
        let launched = landing_speed(Vector3::new(0.0, 15.0, 0.0), Vector3::zeros(), up);
        assert!(launched < IMPACT_SPEED);
    }
}
