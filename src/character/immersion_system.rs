use nalgebra::Point3;
use specs::{Join, Read, ReadStorage, System, WriteStorage};

use super::immersion::{Immersion, WaterAtBody};
use crate::components::Position;
use crate::water::WaterWorld;

/// Measures [`Immersion`] for every body that carries one, from the water in
/// its column.
///
/// The one writer of `Immersion`. Runs after physics, like the grounding
/// system: the animator reads it in the same frame and `character_control` at
/// the top of the next. A level with no water leaves every body dry.
pub struct ImmersionSystem;

impl<'a> System<'a> for ImmersionSystem {
    type SystemData = (
        Option<Read<'a, WaterWorld>>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, Immersion>,
    );

    fn run(&mut self, (water, positions, mut immersions): Self::SystemData) {
        let query = water.as_deref().map(WaterWorld::query);
        for (position, immersion) in (&positions, &mut immersions).join() {
            let at = Point3::from(position.0);
            *immersion = query
                .as_ref()
                .and_then(|q| {
                    let sample = q.sample(at)?;
                    Some(WaterAtBody {
                        level: q.level_at(at).unwrap_or(sample.surface),
                        surface: sample.surface,
                        floor: sample.floor,
                        current: sample.velocity,
                    })
                })
                .map_or_else(Immersion::dry, Immersion::in_water);
        }
    }
}
