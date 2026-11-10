//! System that keeps the water's pumps where their bodies are.

use nalgebra::{Isometry3, Translation3};
use specs::{Entities, Entity, Join, ReadStorage, System, Write};

use super::component::WaterPump;
use crate::components::{Orientation, Position};
use crate::water::{PumpId, WaterWorld};

/// Stands every [`WaterPump`] in the water at its body's pose, and takes
/// out the pumps of bodies that are gone. Runs before the water steps, so
/// a frame's pumping happens where the bodies are that frame.
pub struct PumpSystem;

/// The field's name for spout `spout` of `entity`'s pump: unlike any other
/// living or dead entity's.
pub(super) fn pump_id(entity: Entity, spout: usize) -> PumpId {
    PumpId {
        owner: ((entity.gen().id() as u64) << 32) | entity.id() as u64,
        spout: spout as u32,
    }
}

impl<'a> System<'a> for PumpSystem {
    type SystemData = (
        Entities<'a>,
        Option<Write<'a, WaterWorld>>,
        ReadStorage<'a, WaterPump>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Orientation>,
    );

    fn run(&mut self, (entities, water, pumps, positions, orientations): Self::SystemData) {
        let Some(mut water) = water else {
            return;
        };
        let mut standing = Vec::new();
        for (entity, pump, position, orientation) in
            (&entities, &pumps, &positions, &orientations).join()
        {
            let pose = Isometry3::from_parts(Translation3::from(position.0), orientation.0);
            for (spout, placement) in pump.placements(&pose).enumerate() {
                let id = pump_id(entity, spout);
                water.set_pump(id, placement);
                standing.push(id);
            }
        }
        water.retain_pumps(|id| standing.contains(&id));
    }
}
