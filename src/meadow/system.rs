//! System that keeps the meadow grown over the terrain as it stands.

use nalgebra::Vector3;
use specs::{Entities, Read, System, Write, WriteStorage};

use super::meadow::{Meadow, TileChange};
use crate::components::{ModelInstance, Position, Renderable};
use crate::terrain::TerrainWorld;

/// Regrows the [`Meadow`] whenever the terrain's drawn mesh has changed: a
/// tile's entity gets a new model, a new tile a new entity, and a tile left
/// bare loses its own. Does nothing in a level without a meadow.
pub struct MeadowSystem;

impl<'a> System<'a> for MeadowSystem {
    type SystemData = (
        Entities<'a>,
        Option<Write<'a, Meadow>>,
        Option<Read<'a, TerrainWorld>>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, ModelInstance>,
        WriteStorage<'a, Renderable>,
    );

    fn run(
        &mut self,
        (entities, meadow, terrain, mut positions, mut models, mut renderables): Self::SystemData,
    ) {
        let (Some(mut meadow), Some(terrain)) = (meadow, terrain) else {
            return;
        };
        for change in meadow.regrow(&terrain) {
            match change {
                TileChange::Grown { tile, model } => {
                    // Sprigs are meshed where they stand in the world.
                    let entity = entities
                        .build_entity()
                        .with(Position(Vector3::zeros()), &mut positions)
                        .with(ModelInstance::new(model), &mut models)
                        .with(Renderable, &mut renderables)
                        .build();
                    meadow.drawn_by(tile, entity);
                }
                TileChange::Regrown { entity, model } => {
                    // The entity is the meadow's own and alive: nothing
                    // else takes a tile's entity out.
                    let _ = models.insert(entity, ModelInstance::new(model));
                }
                TileChange::Bare { entity } => {
                    let _ = entities.delete(entity);
                }
            }
        }
    }
}
