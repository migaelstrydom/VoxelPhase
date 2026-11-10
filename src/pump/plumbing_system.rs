//! System that gives each piece of a broken plumbed body the pump its
//! share of the pipe makes.

use specs::{Read, ReadStorage, System, WriteStorage};

use super::component::WaterPump;
use super::plumbing::Plumbing;
use crate::components::RigidBodyComponent;
use crate::fracture::PieceSplits;
use crate::systems::PhysicsResource;

/// After a split, hands the piece its compound's [`Plumbing`] and works out
/// both pieces' pumps again: the one holding the intake throws from where
/// its pipe now ends, the other has none.
pub struct PlumbingShareSystem;

impl<'a> System<'a> for PlumbingShareSystem {
    type SystemData = (
        Read<'a, PieceSplits>,
        Read<'a, PhysicsResource>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, Plumbing>,
        WriteStorage<'a, WaterPump>,
    );

    fn run(&mut self, (splits, physics, bodies, mut plumbing, mut pumps): Self::SystemData) {
        for split in splits.iter() {
            let Some(pipes) = plumbing.get(split.from).cloned() else {
                continue;
            };
            for entity in [split.from, split.piece] {
                let pump = bodies
                    .get(entity)
                    .and_then(|body| pipes.pump_on(&physics.world, body.0));
                match pump {
                    Some(pump) => {
                        let _ = pumps.insert(entity, pump);
                    }
                    None => {
                        pumps.remove(entity);
                    }
                }
            }
            let _ = plumbing.insert(split.piece, pipes);
        }
    }
}
