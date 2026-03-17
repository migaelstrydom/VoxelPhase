//! System that steps the water simulation and provides debug visualisation.

use nalgebra::Point3;
use specs::{Read, System, Write};

use crate::debug::{DebugLines, DebugOverlays};
use crate::rendering::Colour;
use crate::terrain::TerrainManager;
use crate::time::Time;
use crate::water::{WaterGrid, WaveGrid};

/// Steps the water flow simulation and wave equation each frame.
///
/// Reads terrain dirty_regions to detect floor changes under water, then
/// advances the flow sim by the frame's delta time. The wave equation
/// simulation runs after the flow sim, producing fine-resolution surface
/// ripples.
pub struct WaterSystem;

const WATER_DEBUG_COLOUR: Colour = Colour {
    r: 0.2,
    g: 0.5,
    b: 1.0,
    a: 0.8,
};

struct WaterDebugConfig {
    draw_wet_cells: bool,
    debug_colour: Colour,
    sphere_radius: f32,
}

const WATER_DEBUG_CONFIG: WaterDebugConfig = WaterDebugConfig {
    draw_wet_cells: false,
    debug_colour: WATER_DEBUG_COLOUR,
    sphere_radius: 0.3,
};

impl<'a> System<'a> for WaterSystem {
    type SystemData = (
        Option<Write<'a, WaterGrid>>,
        Option<Write<'a, WaveGrid>>,
        Option<Read<'a, TerrainManager>>,
        Read<'a, Time>,
        Write<'a, DebugOverlays>,
        Write<'a, DebugLines>,
    );

    fn run(
        &mut self,
        (water_opt, wave_opt, terrain_opt, time, mut debug_overlays, mut debug_lines): Self::SystemData,
    ) {
        let Some(mut grid) = water_opt else {
            return;
        };

        let dt = time.delta_seconds();

        // Propagate terrain damage to water floor levels.
        if let Some(ref terrain) = terrain_opt {
            let dirty = terrain.dirty_regions();
            if !dirty.is_empty() {
                let mut dirty_cells = Vec::new();
                for region in dirty {
                    // Convert AABB to grid cells that overlap.
                    let min_i = grid.world_to_grid(region.min.x, region.min.z);
                    let max_i = grid.world_to_grid(region.max.x, region.max.z);

                    if let (Some((i0, j0)), Some((i1, j1))) = (min_i, max_i) {
                        for j in j0..=j1 {
                            for i in i0..=i1 {
                                dirty_cells.push((i, j));
                            }
                        }
                    }
                }
                if !dirty_cells.is_empty() {
                    grid.mark_dirty_floors(&dirty_cells);
                }
            }
        }

        // Step the flow simulation.
        let terrain_ref = terrain_opt.as_deref();
        grid.step(dt, |x, z| {
            terrain_ref.and_then(|t| t.surface_height_at(x, z))
        });

        // Step the wave equation simulation.
        if let Some(mut wave_grid) = wave_opt {
            wave_grid.step(dt, &grid);
        }

        // Debug visualisation: spheres at each wet cell's surface.
        if WATER_DEBUG_CONFIG.draw_wet_cells {
            let dims = grid.dims();
            let cell_area = grid.cell_area();
            let mut wet_count = 0u32;

            for j in 0..dims.1 {
                for i in 0..dims.0 {
                    let cell = grid.cell(i, j);
                    if cell.volume <= 0.0 {
                        continue;
                    }
                    wet_count += 1;
                    let x = grid.cell_center_x(i);
                    let z = grid.cell_center_z(j);
                    let y = cell.surface_level(cell_area);
                    debug_overlays.add_sphere(
                        Point3::new(x, y, z),
                        WATER_DEBUG_CONFIG.sphere_radius,
                        WATER_DEBUG_CONFIG.debug_colour,
                    );
                }
            }

            debug_lines.add("Water/Wet cells", wet_count.to_string());
            debug_lines.add("Water/Settled", grid.is_settled().to_string());
        }
    }
}
