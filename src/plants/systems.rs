//! A plant's ECS systems: crowns broken off their limbs by blasts.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Vector3};
use specs::{Builder, Entities, Join, LazyUpdate, Read, ReadStorage, System, Write, WriteStorage};

use super::components::HeldCrown;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::explosion::Explosion;
use crate::foliage::{leaf_emitter, Foliage, Rooting};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::particles::ParticleEmitter;
use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::systems::PhysicsResource;

/// Share of an explosion's knockback radius within which it breaks a crown
/// off its limb, measured to the crown's centre: closer than a rooted shrub
/// is torn loose from (60%), since wood holds it. A grenade caught in a
/// crown, or at the foot of a small tree, takes its crown off.
const BREAK_SHARE: f32 = 0.5;
/// Leaves a crown sheds as it breaks off.
const BREAK_LEAVES: u32 = 60;

/// Breaks crowns off their limbs when a blast goes off close to them: each
/// becomes a loose clump on a core of its own, as a shrub torn loose does,
/// and its limb is left bare.
///
/// Runs after `ExplosionSystem` queued the shove, so the new core is there
/// to receive it.
pub struct CrownBreakSystem;

impl<'a> System<'a> for CrownBreakSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        Read<'a, LazyUpdate>,
        ReadStorage<'a, Explosion>,
        WriteStorage<'a, Foliage>,
        WriteStorage<'a, HeldCrown>,
        WriteStorage<'a, ModelInstance>,
        WriteStorage<'a, ParticleEmitter>,
    );

    fn run(
        &mut self,
        (
            entities,
            mut physics,
            lazy,
            explosions,
            mut foliage,
            mut crowns,
            mut models,
            mut emitters,
        ): Self::SystemData,
    ) {
        let blasts: Vec<(Point3<f32>, f32)> = explosions
            .join()
            .map(|e| (e.center, e.blast_radius * BREAK_SHARE))
            .collect();
        if blasts.is_empty() {
            return;
        }
        let world = &mut physics.world;
        let broken: Vec<_> = (&entities, &foliage, &crowns)
            .join()
            .filter_map(|(entity, clump, _)| {
                let pose = clump.pose(world)?;
                let centre = Point3::from(pose.translation.vector);
                blasts
                    .iter()
                    .any(|(at, reach)| (centre - at).magnitude() < *reach)
                    .then_some(entity)
            })
            .collect();
        for limb in broken {
            let (Some(clump), Some(crown)) = (foliage.remove(limb), crowns.remove(limb)) else {
                continue;
            };
            let Rooting::Loose { body, .. } = clump.rooting else {
                continue;
            };
            let Some(pose) = clump.pose(world) else {
                continue;
            };
            let velocity = velocity_at(world, body, pose.translation.vector.into());
            let spin = world
                .body(body)
                .map_or(Vector3::zeros(), |b| b.angular_velocity());
            for collider in &crown.colliders {
                world.detach_collider(body, *collider);
            }
            let moved = world.recenter_on_colliders(body);
            let _ = models.insert(limb, ModelInstance::new(bare_limb(crown.wood, moved)));
            // Its leaves' trail goes with them.
            emitters.remove(limb);

            let core = clump.core.spawn(world, &clump.shape, pose);
            world.set_body_velocity(core, velocity, spin);
            world.wake_body(core);
            let centre = pose.translation.vector;
            lazy.create_entity(&entities)
                .with(Position(centre))
                .with(Orientation(pose.rotation))
                .with(Velocity(velocity))
                .with(RigidBodyComponent(core))
                .with(ModelInstance::new(crown.skin))
                .with(Foliage::new(clump.shape.clone(), Rooting::on(core)))
                .with(Renderable)
                .build();
            lazy.create_entity(&entities)
                .with(Position(centre))
                .with(leaf_emitter(BREAK_LEAVES))
                .build();
        }
    }
}

/// The velocity of the point `at` on `body`.
fn velocity_at(world: &PhysicsWorld, body: RigidBodyHandle, at: Point3<f32>) -> Vector3<f32> {
    world.body(body).map_or(Vector3::zeros(), |b| {
        b.linear_velocity() + b.angular_velocity().cross(&(at - b.position()))
    })
}

/// The limb drawn bare, its body's origin having moved by `moved` in its
/// own frame.
fn bare_limb(mut wood: MeshPrimitive, moved: Vector3<f32>) -> Arc<Model> {
    let shift = Isometry3::translation(-moved.x, -moved.y, -moved.z);
    for vertex in &mut wood.vertices {
        vertex.pos = (shift * Point3::from(vertex.pos)).coords;
    }
    Arc::new(Model::flat(vec![ModelPart::new(vec![wood])]))
}
