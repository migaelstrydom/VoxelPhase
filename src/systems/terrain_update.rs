//! System that updates terrain mesh and collision data when terrain is modified.

use specs::{System, Write};

use crate::terrain::TerrainManager;

/// System that updates terrain mesh and collision data if terrain was modified.
///
/// This should run after all systems that modify terrain (e.g., ExplosionSystem)
/// and before systems that read terrain (e.g., TerrainCollisionSystem, RenderSystem).
pub struct TerrainUpdateSystem;

impl<'a> System<'a> for TerrainUpdateSystem {
    type SystemData = Option<Write<'a, TerrainManager>>;

    fn run(&mut self, terrain_manager_opt: Self::SystemData) {
        if let Some(mut terrain_manager) = terrain_manager_opt {
            terrain_manager.update();
        }
    }
}
