//! System that updates terrain mesh and collision data when terrain is modified.

use specs::{System, Write};

use crate::debug::DebugLog;
use crate::terrain::{TerrainManager, UpdateTimings};

/// System that updates terrain mesh and collision data if terrain was modified.
///
/// This should run after all systems that modify terrain (e.g., ExplosionSystem)
/// and before systems that read terrain (e.g., TerrainCollisionSystem, RenderSystem).
///
/// Also reports the cost breakdown of the most recent terrain rebuild through
/// [`DebugLog`] (printed on F3). The phases scale differently — remesh with the
/// number of chunks dirtied, buffer concatenation with the size of the whole
/// level — so they are reported apart.
pub struct TerrainUpdateSystem;

impl<'a> System<'a> for TerrainUpdateSystem {
    type SystemData = (Option<Write<'a, TerrainManager>>, Write<'a, DebugLog>);

    fn run(&mut self, (terrain_manager_opt, mut debug_log): Self::SystemData) {
        let Some(mut terrain_manager) = terrain_manager_opt else {
            return;
        };

        terrain_manager.update();

        if let Some(timings) = terrain_manager.last_update_timings() {
            report(&timings, &mut debug_log);
        }
    }
}

/// Write one rebuild's timing breakdown into the debug log.
///
/// These describe the last rebuild that had work to do, not the current frame —
/// terrain is idle on almost every frame, and a number that vanished the frame
/// after a grenade would be unreadable.
fn report(timings: &UpdateTimings, debug_log: &mut DebugLog) {
    let ms = |d: std::time::Duration| format!("{:.2} ms", d.as_secs_f64() * 1000.0);

    debug_log.add(
        "Terrain/LastUpdate/Chunks",
        timings.chunks_dirtied.to_string(),
    );
    debug_log.add("Terrain/LastUpdate/1 Remesh", ms(timings.remesh));
    debug_log.add("Terrain/LastUpdate/2 Adjacency", ms(timings.adjacency));
    debug_log.add("Terrain/LastUpdate/3 BufferConcat", ms(timings.concat));
    debug_log.add("Terrain/LastUpdate/4 Total", ms(timings.total()));
    debug_log.add("Terrain/Triangles", timings.triangles.to_string());
}
