//! A plant's ECS systems: branches broken off by blasts.

use nalgebra::Vector3;
use specs::{Builder, Entities, Join, LazyUpdate, Read, System, Write, WriteStorage};

use super::components::PlantPiece;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::foliage::Foliage;
use crate::physics::{PhysicsImpulseQueue, PhysicsWorld};
use crate::systems::PhysicsResource;

/// Breaks branches off plants when a blast strikes their crowns harder than
/// they hold (`Joinery::branch_strength`): each becomes a piece of its own,
/// thrown by the blast, and the piece it left is redrawn without it.
///
/// Runs after the physics step that applied the blast, reading its impulses
/// as the fracture system does: the step pushed the whole plant, and the
/// branch leaves with its share.
pub struct BranchBreakSystem;

impl<'a> System<'a> for BranchBreakSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        Read<'a, PhysicsImpulseQueue>,
        Read<'a, LazyUpdate>,
        WriteStorage<'a, PlantPiece>,
        WriteStorage<'a, Foliage>,
        WriteStorage<'a, ModelInstance>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Orientation>,
    );

    fn run(
        &mut self,
        (
            entities,
            mut physics,
            impulses,
            lazy,
            mut pieces,
            mut foliage,
            mut models,
            mut positions,
            mut orientations,
        ): Self::SystemData,
    ) {
        let blasts = impulses.last_impulses();
        if blasts.is_empty() {
            return;
        }
        let world = &mut physics.world;
        for (entity, piece) in (&entities, &mut pieces).join() {
            let mut broke = false;
            for branch in piece.body.branches(&piece.plant.plant) {
                if !piece.body.holds(branch) {
                    continue;
                }
                let grown = piece.plant.clone();
                let joinery = &grown.joinery;
                let kick = blasts
                    .iter()
                    .filter_map(|blast| {
                        joinery.blast_on_branch(world, &grown.plant, &piece.body, branch, blast)
                    })
                    .sum::<Vector3<f32>>();
                if kick.magnitude() <= joinery.branch_strength {
                    continue;
                }
                let Some(broken) = joinery.break_off(world, &grown.plant, &mut piece.body, branch)
                else {
                    continue;
                };
                world.apply_impulse(broken.body, kick);
                spawn_piece(
                    &entities,
                    &lazy,
                    world,
                    PlantPiece {
                        plant: grown.clone(),
                        body: broken,
                    },
                );
                broke = true;
            }
            if !broke {
                continue;
            }
            let _ = models.insert(entity, ModelInstance::new(piece.model()));
            match piece.foliage() {
                Some(clump) => {
                    let _ = foliage.insert(entity, clump);
                }
                None => {
                    foliage.remove(entity);
                }
            }
            // The body re-centred on what it kept after this frame's sync:
            // the model, drawn from the new origin, must be placed there.
            if let Some(body) = world.body(piece.body.body) {
                let _ = positions.insert(entity, Position(body.position().coords));
                let _ = orientations.insert(entity, Orientation(body.rotation()));
            }
        }
    }
}

/// An entity for `piece`, drawn and dragged as it now stands.
pub fn spawn_piece(
    entities: &Entities,
    lazy: &LazyUpdate,
    world: &PhysicsWorld,
    piece: PlantPiece,
) {
    let Some(body) = world.body(piece.body.body) else {
        return;
    };
    let mut builder = lazy
        .create_entity(entities)
        .with(Position(body.position().coords))
        .with(Orientation(body.rotation()))
        .with(Velocity(body.linear_velocity()))
        .with(RigidBodyComponent(piece.body.body))
        .with(ModelInstance::new(piece.model()))
        .with(Renderable);
    if let Some(clump) = piece.foliage() {
        builder = builder.with(clump);
    }
    builder.with(piece).build();
}
