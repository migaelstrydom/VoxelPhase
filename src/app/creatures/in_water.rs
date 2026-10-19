//! Where a swimming creature starts: in the water column over an authored
//! (x, z), at its share of the way down.

use nalgebra::Point3;
use specs::{World, WorldExt};

use crate::terrain::TerrainWorld;
use crate::water::WaterWorld;

/// The point at `depth_share` of the way down the water over `pos` (0 just
/// under the surface, 1 just over the floor), kept `clearance` off both, or
/// `None` where there is no water there deep enough to hold the creature.
pub fn spawn_point_in_water(
    world: &World,
    pos: (f32, f32),
    depth_share: f32,
    clearance: f32,
) -> Option<Point3<f32>> {
    let floor = world
        .read_resource::<TerrainWorld>()
        .mesh_surface_height_at(pos.0, pos.1)?;
    let water = world.try_fetch::<WaterWorld>()?;
    let here = water
        .query()
        .sample(Point3::new(pos.0, floor + clearance, pos.1))?;
    let top = here.surface - clearance;
    let bottom = here.floor + clearance;
    if top <= bottom {
        return None;
    }
    Some(Point3::new(
        pos.0,
        top + (bottom - top) * depth_share,
        pos.1,
    ))
}
