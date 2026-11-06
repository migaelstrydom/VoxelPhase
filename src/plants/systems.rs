//! A plant's ECS systems: its share of a piece split off it.

use specs::{Read, ReadStorage, System, WriteStorage};

use super::components::PlantPiece;
use crate::components::RigidBodyComponent;
use crate::foliage::Foliage;
use crate::fracture::PieceSplits;
use crate::model::PartLooks;
use crate::systems::PhysicsResource;

/// Gives a piece split off a plant what is the plant's own: its
/// `PlantPiece`, the `Foliage` of the crowns it holds and the snapped bark
/// of a branch broken off the limb it grew from. The piece it came off
/// gets its leaves again too, riding on its re-centred body.
///
/// Runs after the fracture system, which split the body and shared out the
/// looks, and before the looks are drawn.
pub struct PlantShareSystem;

impl<'a> System<'a> for PlantShareSystem {
    type SystemData = (
        Read<'a, PieceSplits>,
        Read<'a, PhysicsResource>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, PlantPiece>,
        WriteStorage<'a, Foliage>,
        WriteStorage<'a, PartLooks>,
    );

    fn run(
        &mut self,
        (splits, physics, bodies, mut pieces, mut foliage, mut looks): Self::SystemData,
    ) {
        let world = &physics.world;
        for split in splits.iter() {
            let Some(plant) = pieces.get(split.from).map(|piece| piece.plant.clone()) else {
                continue;
            };
            let _ = pieces.insert(
                split.piece,
                PlantPiece {
                    plant: plant.clone(),
                },
            );
            for entity in [split.from, split.piece] {
                let Some(body) = bodies.get(entity).map(|b| b.0) else {
                    continue;
                };
                let limbs = plant.limbs_on(world, body);
                let clump = limbs
                    .first()
                    .and_then(|&l| plant.frame_on(world, l))
                    .and_then(|frame| plant.foliage(&limbs, body, frame));
                match clump {
                    Some(clump) => {
                        let _ = foliage.insert(entity, clump);
                    }
                    None => {
                        foliage.remove(entity);
                    }
                }
                if let Some(looks) = looks.get_mut(entity) {
                    plant.snap_broken(&limbs, looks);
                }
            }
        }
    }
}
