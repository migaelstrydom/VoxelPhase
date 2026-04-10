//! Terrain anchor system: releases anchored bodies when their terrain is destroyed.

use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::components::{ModelInstance, RigidBodyComponent, TerrainAnchored};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainManager;

/// Checks terrain solidity under each anchored body and removes constraints
/// when the terrain is gone. Optionally swaps the collider to its full-size
/// version so the freed body has correct collision geometry.
pub struct TerrainAnchorSystem;

impl<'a> System<'a> for TerrainAnchorSystem {
    type SystemData = (
        Entities<'a>,
        WriteStorage<'a, TerrainAnchored>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, ModelInstance>,
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainManager>>,
    );

    fn run(
        &mut self,
        (entities, mut anchored, bodies, mut models, mut physics, terrain_opt): Self::SystemData,
    ) {
        let Some(terrain) = terrain_opt else {
            return;
        };

        let mut released = Vec::new();

        for (entity, anchor, body) in (&entities, &mut anchored, &bodies).join() {
            let p = anchor.anchor_world;
            if !terrain.is_solid_at(p.x, p.y, p.z) {
                physics.world.remove_constraint(anchor.anchor_handle);
                physics.world.remove_constraint(anchor.upright_handle);

                if let Some(released_collider) = anchor.released_collider.take() {
                    let body_handle = body.0;
                    let existing: Vec<_> = physics
                        .world
                        .body(body_handle)
                        .map(|b| b.colliders().to_vec())
                        .unwrap_or_default();
                    for ch in existing {
                        physics.world.detach_collider(body_handle, ch);
                    }
                    physics
                        .world
                        .attach_collider(body_handle, released_collider);
                }

                if let Some(released_model) = anchor.released_model.take() {
                    if let Some(model_instance) = models.get_mut(entity) {
                        model_instance.model = released_model;
                    }
                }

                released.push(entity);
            }
        }

        for entity in released {
            anchored.remove(entity);
        }
    }
}
