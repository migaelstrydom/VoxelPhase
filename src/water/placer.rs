//! Flood-fill based water placement.
//!
//! Determines which water grid cells belong to a pool by flood-filling from a
//! seed point through terrain that is below the target surface level. The
//! filled region is dilated by one cell to cover marching-cubes shore smoothing.

use std::collections::VecDeque;

use crate::terrain::TerrainManager;

use super::WaterGrid;

/// Fill a pool by flood-filling from `seed` at `surface_level`.
///
/// Queries the SVO via `terrain.is_solid_at()` to determine extent: a cell is
/// inside the pool if the voxel at `(cell_center_x, surface_level, cell_center_z)`
/// is air. After flood-fill, the region is dilated by one cell so the water
/// mesh covers marching-cubes shore smoothing.
///
/// Floor levels are resolved via `WaterGrid` sampled floor admission helpers.
pub fn fill_pool(
    grid: &mut WaterGrid,
    terrain: &TerrainManager,
    seed: (f32, f32),
    surface_level: f32,
) {
    let Some((si, sj)) = grid.world_to_grid(seed.0, seed.1) else {
        log::warn!(
            "Pool seed ({:.1}, {:.1}) is outside the water grid",
            seed.0,
            seed.1
        );
        return;
    };

    let dims = grid.dims();

    // Phase 1: flood-fill to find interior cells.
    let mut interior = vec![false; dims.0 * dims.1];
    let mut queue = VecDeque::new();

    if is_pool_cell(grid, terrain, si, sj, surface_level) {
        interior[sj * dims.0 + si] = true;
        queue.push_back((si, sj));
    } else {
        log::warn!(
            "Pool seed cell ({}, {}) is solid at surface_level {:.1}",
            si,
            sj,
            surface_level
        );
        return;
    }

    while let Some((i, j)) = queue.pop_front() {
        for (di, dj) in [(-1i32, 0), (1, 0), (0, -1i32), (0, 1)] {
            let ni = i as i32 + di;
            let nj = j as i32 + dj;
            if ni < 0 || nj < 0 || (ni as usize) >= dims.0 || (nj as usize) >= dims.1 {
                continue;
            }
            let ni = ni as usize;
            let nj = nj as usize;
            let idx = nj * dims.0 + ni;
            if !interior[idx] && is_pool_cell(grid, terrain, ni, nj, surface_level) {
                interior[idx] = true;
                queue.push_back((ni, nj));
            }
        }
    }

    // Phase 2: set floor levels and add water volume.
    let mut floor_query = |x: f32, z: f32| terrain.approx_surface_height_at(x, z);
    let mut filled_count = 0u32;

    for j in 0..dims.1 {
        for i in 0..dims.0 {
            if !interior[j * dims.0 + i] {
                continue;
            }

            if grid.fill_cell_to_surface(i, j, surface_level, &mut floor_query) {
                filled_count += 1;
            }
        }
    }

    log::info!(
        "Pool at ({:.1}, {:.1}): filled {} cells at surface_level {:.1}",
        seed.0,
        seed.1,
        filled_count,
        surface_level,
    );
}

/// Check whether any part of a cell is inside a pool: if any of the 9 sample
/// points is air at `surface_level`, the cell is a candidate. This extends
/// the flood-fill into shore cells where MC smoothing dips below the water.
fn is_pool_cell(
    grid: &WaterGrid,
    terrain: &TerrainManager,
    i: usize,
    j: usize,
    surface_level: f32,
) -> bool {
    grid.cell_sample_points(i, j)
        .iter()
        .any(|&(x, z)| !terrain.is_solid_at(x, surface_level, z))
}
