//! Water grid: 2D heightfield simulation with flow equalization.

use nalgebra::Vector3;

/// Configuration for the water grid.
pub struct WaterGridConfig {
    /// World-space width/depth of each cell.
    pub cell_size: f32,

    /// Grid dimensions (x, z).
    pub dims: (usize, usize),

    /// World-space position of grid corner (0, 0).
    pub origin: Vector3<f32>,

    /// Sea level for the infinite ocean plane. `None` disables ocean
    /// coupling so boundary cells behave like regular terrain cells.
    pub ocean_level: Option<f32>,

    /// Flow rate multiplier. Higher = faster equalization.
    pub flow_rate: f32,

    /// Surface level difference below which the simulation is considered settled.
    pub settle_epsilon: f32,

    /// Fluid density in kg/m³. Used by the buoyancy system.
    pub fluid_density: f32,
}

impl Default for WaterGridConfig {
    fn default() -> Self {
        Self {
            cell_size: 2.0,
            dims: (64, 64),
            origin: Vector3::new(-64.0, 0.0, -64.0),
            ocean_level: None,
            flow_rate: 4.0,
            settle_epsilon: 0.001,
            fluid_density: 1000.0,
        }
    }
}

/// A single cell in the water grid.
#[derive(Debug, Clone, Copy)]
pub struct WaterCell {
    /// Volume of water in this cell (cubic units). Zero means dry.
    pub volume: f32,

    /// Terrain surface this water rests on (world Y). Set when water is
    /// placed or flows into a cell.
    pub floor_level: f32,
}

impl Default for WaterCell {
    fn default() -> Self {
        Self {
            volume: 0.0,
            floor_level: 0.0,
        }
    }
}

impl WaterCell {
    /// Compute the water surface level from floor + volume.
    #[inline]
    pub fn surface_level(&self, cell_area: f32) -> f32 {
        self.floor_level + self.volume / cell_area
    }
}

/// 2D water heightfield simulation.
///
/// The grid is axis-aligned in the XZ plane. Cell `(i, j)` covers the
/// world-space rectangle:
///   `[origin.x + i*cell_size .. origin.x + (i+1)*cell_size]` x
///   `[origin.z + j*cell_size .. origin.z + (j+1)*cell_size]`
pub struct WaterGrid {
    cells: Vec<WaterCell>,
    dims: (usize, usize),
    cell_size: f32,
    cell_area: f32,
    origin: Vector3<f32>,
    ocean_level: Option<f32>,
    flow_rate: f32,
    settle_epsilon: f32,
    fluid_density: f32,

    /// True when all active cells have reached equilibrium.
    settled: bool,

    /// Per-cell flow accumulator, reused across steps to avoid allocation.
    delta_volume: Vec<f32>,

    /// Cells that need floor-level rechecks due to terrain damage.
    dirty_floors: Vec<(usize, usize)>,
}

impl WaterGrid {
    /// Volume below which a cell is snapped to zero (dry).
    /// Prevents residual volumes from the 50% outflow cap lingering forever.
    const MIN_VOLUME: f32 = 1e-2;

    /// Create a new water grid from configuration.
    pub fn new(config: WaterGridConfig) -> Self {
        let total = config.dims.0 * config.dims.1;
        let cell_area = config.cell_size * config.cell_size;

        Self {
            cells: vec![WaterCell::default(); total],
            dims: config.dims,
            cell_size: config.cell_size,
            cell_area,
            origin: config.origin,
            ocean_level: config.ocean_level,
            flow_rate: config.flow_rate,
            settle_epsilon: config.settle_epsilon,
            fluid_density: config.fluid_density,
            settled: false,
            delta_volume: vec![0.0; total],
            dirty_floors: Vec::new(),
        }
    }

    /// Grid dimensions (x, z).
    pub fn dims(&self) -> (usize, usize) {
        self.dims
    }

    /// World-space cell size.
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }

    /// Cell area (cell_size^2).
    pub fn cell_area(&self) -> f32 {
        self.cell_area
    }

    /// World-space origin of the grid corner.
    pub fn origin(&self) -> Vector3<f32> {
        self.origin
    }

    /// Ocean level (sea level), if ocean boundary coupling is enabled.
    pub fn ocean_level(&self) -> Option<f32> {
        self.ocean_level
    }

    /// Fluid density in kg/m³.
    pub fn fluid_density(&self) -> f32 {
        self.fluid_density
    }

    /// Whether the simulation has settled (no significant flow).
    pub fn is_settled(&self) -> bool {
        self.settled
    }

    /// Get a cell by grid coordinates.
    pub fn cell(&self, i: usize, j: usize) -> &WaterCell {
        &self.cells[j * self.dims.0 + i]
    }

    /// Get a mutable cell by grid coordinates.
    pub fn cell_mut(&mut self, i: usize, j: usize) -> &mut WaterCell {
        &mut self.cells[j * self.dims.0 + i]
    }

    /// Flat index from grid coordinates.
    #[inline]
    fn index(&self, i: usize, j: usize) -> usize {
        j * self.dims.0 + i
    }

    /// World-space center X of cell (i, _).
    #[inline]
    pub fn cell_center_x(&self, i: usize) -> f32 {
        self.origin.x + (i as f32 + 0.5) * self.cell_size
    }

    /// World-space center Z of cell (_, j).
    #[inline]
    pub fn cell_center_z(&self, j: usize) -> f32 {
        self.origin.z + (j as f32 + 0.5) * self.cell_size
    }

    /// Convert a world-space (x, z) to grid coordinates, if within bounds.
    pub fn world_to_grid(&self, x: f32, z: f32) -> Option<(usize, usize)> {
        let fi = (x - self.origin.x) / self.cell_size;
        let fj = (z - self.origin.z) / self.cell_size;

        if fi < 0.0 || fj < 0.0 {
            return None;
        }

        let i = fi as usize;
        let j = fj as usize;

        if i >= self.dims.0 || j >= self.dims.1 {
            return None;
        }

        Some((i, j))
    }

    /// Whether this cell is on the grid boundary (acts as ocean source/sink).
    #[inline]
    fn is_boundary(&self, i: usize, j: usize) -> bool {
        i == 0 || j == 0 || i == self.dims.0 - 1 || j == self.dims.1 - 1
    }

    /// Whether the ocean can couple to this boundary cell.
    #[inline]
    fn boundary_ocean_active(&self, i: usize, j: usize) -> bool {
        debug_assert!(self.is_boundary(i, j));
        self.ocean_level
            .is_some_and(|ocean_level| self.cell(i, j).floor_level <= ocean_level)
    }

    /// Stable surface level for a boundary cell connected to the ocean.
    #[inline]
    fn boundary_surface_level(&self, i: usize, j: usize) -> f32 {
        debug_assert!(self.is_boundary(i, j));
        let cell = self.cell(i, j);
        if let Some(ocean_level) = self
            .ocean_level
            .filter(|_| self.boundary_ocean_active(i, j))
        {
            ocean_level
        } else {
            cell.floor_level
        }
    }

    /// Stable water volume for a boundary cell connected to the ocean.
    #[inline]
    fn boundary_target_volume(&self, i: usize, j: usize) -> f32 {
        debug_assert!(self.is_boundary(i, j));
        if let Some(ocean_level) = self
            .ocean_level
            .filter(|_| self.boundary_ocean_active(i, j))
        {
            let cell = self.cell(i, j);
            (ocean_level - cell.floor_level).max(0.0) * self.cell_area
        } else {
            0.0
        }
    }

    /// Inject water volume at a grid cell, setting the floor level.
    ///
    /// Clears the settled flag so the flow simulation runs.
    pub fn add_water(&mut self, i: usize, j: usize, volume: f32, floor_level: f32) {
        let cell = self.cell_mut(i, j);
        cell.volume += volume;
        cell.floor_level = floor_level;
        self.settled = false;
    }

    /// Mark cells as needing floor-level rechecks (terrain was damaged).
    pub fn mark_dirty_floors(&mut self, cells: &[(usize, usize)]) {
        self.dirty_floors.extend_from_slice(cells);
        self.settled = false;
    }

    /// Get the water surface level at a world-space (x, z) position.
    ///
    /// Returns `None` if the position is outside the grid or the cell is dry
    /// and not ocean-coupled.
    pub fn surface_level_at(&self, x: f32, z: f32) -> Option<f32> {
        let (i, j) = self.world_to_grid(x, z)?;
        let cell = self.cell(i, j);
        if cell.volume <= 0.0 {
            if self.is_boundary(i, j) && self.boundary_ocean_active(i, j) {
                return Some(self.boundary_surface_level(i, j));
            }
            return None;
        }
        Some(cell.surface_level(self.cell_area))
    }

    /// Total water volume across all cells.
    pub fn total_volume(&self) -> f32 {
        self.cells.iter().map(|c| c.volume).sum()
    }

    /// Step the flow simulation. Call once per physics step.
    ///
    /// `dt` is the time step in seconds. `floor_query` is called for cells
    /// in `dirty_floors` to refresh their floor level from terrain.
    pub fn step<F>(&mut self, dt: f32, mut floor_query: F)
    where
        F: FnMut(f32, f32) -> Option<f32>,
    {
        // Handle dirty floors first.
        self.process_dirty_floors(&mut floor_query);

        if self.settled {
            return;
        }

        // Zero the delta accumulator for all cells.
        for d in self.delta_volume.iter_mut() {
            *d = 0.0;
        }

        let mut max_delta: f32 = 0.0;

        // Flow equalization: for each cell with water, compute the target
        // equalization level with all lower neighbors and distribute flow.
        for j in 0..self.dims.1 {
            for i in 0..self.dims.0 {
                let idx = self.index(i, j);
                let cell = &self.cells[idx];
                let is_boundary = self.is_boundary(i, j);
                let is_ocean_source = is_boundary
                    && self.boundary_ocean_active(i, j)
                    && cell.volume <= self.boundary_target_volume(i, j);

                // Determine effective surface level for this cell.
                let h_self = if is_ocean_source {
                    self.boundary_surface_level(i, j)
                } else if cell.volume <= 0.0 {
                    continue;
                } else {
                    cell.surface_level(self.cell_area)
                };

                // Collect lower neighbors and compute total flow demand.
                let offsets: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];
                let mut total_demand = 0.0f32;
                let mut demands = [0.0f32; 4];
                let mut neighbor_indices = [0usize; 4];

                for (k, (di, dj)) in offsets.iter().enumerate() {
                    let ni = i as i32 + di;
                    let nj = j as i32 + dj;

                    if ni < 0 || nj < 0 || ni >= self.dims.0 as i32 || nj >= self.dims.1 as i32 {
                        continue;
                    }

                    let ni = ni as usize;
                    let nj = nj as usize;
                    let n_idx = self.index(ni, nj);
                    neighbor_indices[k] = n_idx;

                    let neighbor = &self.cells[n_idx];
                    let h_neighbor = if neighbor.volume <= 0.0 && self.is_boundary(ni, nj) {
                        self.boundary_surface_level(ni, nj)
                    } else if neighbor.volume <= 0.0 {
                        neighbor.floor_level
                    } else {
                        neighbor.surface_level(self.cell_area)
                    };

                    let delta_h = h_self - h_neighbor;
                    if delta_h > 0.0 {
                        let demand = delta_h * self.flow_rate * dt;
                        demands[k] = demand;
                        total_demand += demand;
                    }
                }

                if total_demand <= 0.0 {
                    continue;
                }

                // Cap total outflow at 50% of cell volume (stability).
                let max_outflow = if is_ocean_source {
                    total_demand
                } else {
                    total_demand.min(cell.volume * 0.5)
                };

                // Distribute proportionally among neighbors.
                let scale = max_outflow / total_demand;
                for k in 0..4 {
                    if demands[k] > 0.0 {
                        let flow = demands[k] * scale;
                        self.delta_volume[idx] -= flow;
                        self.delta_volume[neighbor_indices[k]] += flow;
                        max_delta = max_delta.max(flow);
                    }
                }
            }
        }

        // Apply accumulated deltas.
        for j in 0..self.dims.1 {
            for i in 0..self.dims.0 {
                let idx = self.index(i, j);
                let delta = self.delta_volume[idx];
                let should_clamp_boundary =
                    self.is_boundary(i, j) && self.boundary_ocean_active(i, j);
                if delta == 0.0 && !should_clamp_boundary {
                    continue;
                }

                let was_dry = self.cells[idx].volume <= 0.0;
                self.cells[idx].volume = (self.cells[idx].volume + delta).max(0.0);

                // Snap negligible volumes to zero so cells dry out cleanly.
                // Only snap when draining (delta <= 0) to avoid killing small
                // volumes that are still growing from inflow.
                if delta <= 0.0 && self.cells[idx].volume < Self::MIN_VOLUME {
                    self.cells[idx].volume = 0.0;
                }

                // Set floor level for newly-wet cells.
                if was_dry && self.cells[idx].volume > 0.0 {
                    let cx = self.cell_center_x(i);
                    let cz = self.cell_center_z(j);
                    if let Some(floor) = floor_query(cx, cz) {
                        self.cells[idx].floor_level = floor;
                    }
                }

                if should_clamp_boundary {
                    self.cells[idx].volume = self.boundary_target_volume(i, j);
                }
            }
        }

        // Check if settled: compare maximum surface height difference
        // between any wet cell and its neighbors. This catches cases where
        // individual flows are tiny but the system hasn't converged.
        let mut max_surface_diff: f32 = 0.0;
        for j in 0..self.dims.1 {
            for i in 0..self.dims.0 {
                let idx = self.index(i, j);
                let cell = &self.cells[idx];

                let h = if cell.volume > 0.0 {
                    cell.surface_level(self.cell_area)
                } else if self.is_boundary(i, j) && self.boundary_ocean_active(i, j) {
                    self.boundary_surface_level(i, j)
                } else {
                    continue;
                };

                let offsets: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];
                for (di, dj) in offsets {
                    let ni = i as i32 + di;
                    let nj = j as i32 + dj;
                    if ni < 0 || nj < 0 || ni >= self.dims.0 as i32 || nj >= self.dims.1 as i32 {
                        continue;
                    }
                    let ni = ni as usize;
                    let nj = nj as usize;
                    let n_idx = self.index(ni, nj);
                    let neighbor = &self.cells[n_idx];

                    let h_n = if neighbor.volume > 0.0 {
                        neighbor.surface_level(self.cell_area)
                    } else if self.is_boundary(ni, nj) && self.boundary_ocean_active(ni, nj) {
                        self.boundary_surface_level(ni, nj)
                    } else {
                        continue;
                    };

                    max_surface_diff = max_surface_diff.max((h - h_n).abs());
                }
            }
        }
        self.settled = max_surface_diff < self.settle_epsilon;
    }

    /// Process dirty floor cells: recheck terrain heights and handle drains.
    fn process_dirty_floors<F>(&mut self, floor_query: &mut F)
    where
        F: FnMut(f32, f32) -> Option<f32>,
    {
        if self.dirty_floors.is_empty() {
            return;
        }

        let dirty: Vec<(usize, usize)> = self.dirty_floors.drain(..).collect();

        for (i, j) in dirty {
            if i >= self.dims.0 || j >= self.dims.1 {
                continue;
            }

            let idx = self.index(i, j);
            let cx = self.cell_center_x(i);
            let cz = self.cell_center_z(j);

            match floor_query(cx, cz) {
                None => {
                    // Floor completely destroyed — drain all water.
                    self.cells[idx].volume = 0.0;
                }
                Some(new_floor) => {
                    let old_floor = self.cells[idx].floor_level;
                    let threshold = 0.1;
                    if (new_floor - old_floor).abs() > threshold {
                        // Floor changed — update it. For wet cells the volume
                        // stays the same so surface_level shifts, creating a
                        // height differential the flow sim resolves. For dry
                        // cells the updated floor lets neighbors flow in.
                        self.cells[idx].floor_level = new_floor;
                    }
                }
            }
        }

        self.settled = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_grid(dims: (usize, usize), cell_size: f32, ocean_level: Option<f32>) -> WaterGrid {
        make_grid_with_flow_rate(dims, cell_size, ocean_level, 4.0)
    }

    fn make_grid_with_flow_rate(
        dims: (usize, usize),
        cell_size: f32,
        ocean_level: Option<f32>,
        flow_rate: f32,
    ) -> WaterGrid {
        WaterGrid::new(WaterGridConfig {
            cell_size,
            dims,
            origin: Vector3::new(0.0, 0.0, 0.0),
            ocean_level,
            flow_rate,
            ..Default::default()
        })
    }

    /// No-op floor query that returns a flat floor at y=0.
    fn flat_floor(_x: f32, _z: f32) -> Option<f32> {
        Some(0.0)
    }

    #[test]
    fn volume_conservation_symmetric() {
        // 3x3 grid, water in center. Flow should spread evenly and conserve volume.
        let mut grid = make_grid((3, 3), 1.0, None);
        let initial_volume = 10.0;
        grid.add_water(1, 1, initial_volume, 0.0);

        // Set floor levels for all cells.
        for j in 0..3 {
            for i in 0..3 {
                grid.cell_mut(i, j).floor_level = 0.0;
            }
        }

        // Run many steps to reach equilibrium with ocean coupling disabled.
        let dt = 1.0 / 60.0;
        for _ in 0..500 {
            grid.step(dt, flat_floor);
        }

        let total = grid.total_volume();
        assert!(
            (total - initial_volume).abs() < 0.01,
            "Volume not conserved: expected {initial_volume}, got {total}"
        );
    }

    #[test]
    fn flow_direction_downhill() {
        // 5x5 grid, flat floor. Water placed in one cell should spread to
        // all neighbors and reach equilibrium (equal surface levels).
        let mut grid = make_grid((5, 5), 1.0, None);

        for j in 0..5 {
            for i in 0..5 {
                grid.cell_mut(i, j).floor_level = 0.0;
            }
        }

        // Place water at (1, 2) — interior, near left boundary.
        grid.add_water(1, 2, 5.0, 0.0);

        let dt = 1.0 / 60.0;
        for _ in 0..50000 {
            grid.step(dt, flat_floor);
        }

        // Cell (3, 2) — downstream — should have received some water.
        // Both cells should have similar surface levels.
        let src = grid.cell(1, 2);
        let dst = grid.cell(3, 2);
        assert!(
            dst.volume > 0.0,
            "Downstream cell should have received water, vol={}",
            dst.volume
        );
        let src_surface = src.surface_level(grid.cell_area());
        let dst_surface = dst.surface_level(grid.cell_area());
        assert!(
            (src_surface - dst_surface).abs() < 0.5,
            "Surface levels should equalize: src={src_surface}, dst={dst_surface}"
        );
    }

    #[test]
    fn boundary_cells_ocean_source() {
        // 5x5 grid with ocean_level = 5.0. Boundary cells should flood interior.
        let mut grid = make_grid((5, 5), 1.0, Some(5.0));

        // Set all floor levels to 0.
        for j in 0..5 {
            for i in 0..5 {
                grid.cell_mut(i, j).floor_level = 0.0;
            }
        }

        let dt = 1.0 / 60.0;
        for _ in 0..100000 {
            grid.step(dt, flat_floor);
        }

        // Interior cells should have water near ocean_level.
        let center = grid.cell(2, 2);
        let surface = center.surface_level(grid.cell_area());
        assert!(
            center.volume > 0.0,
            "Center cell should have water from ocean"
        );
        assert!(
            (surface - 5.0).abs() < 0.5,
            "Center surface should approach ocean_level=5.0, got {surface}"
        );
    }

    #[test]
    fn boundary_cells_ocean_sink() {
        // 5x5 grid with ocean_level = 0. Put a tall column of water in the center.
        // It should drain to ocean level via boundary cells.
        let mut grid = make_grid((5, 5), 1.0, Some(0.0));

        for j in 0..5 {
            for i in 0..5 {
                grid.cell_mut(i, j).floor_level = 0.0;
            }
        }

        grid.add_water(2, 2, 100.0, 0.0);

        let dt = 1.0 / 60.0;
        for _ in 0..30000 {
            grid.step(dt, flat_floor);
        }

        // Center surface should be near ocean_level (0.0), not sky-high.
        let surface = grid.cell(2, 2).surface_level(grid.cell_area());
        assert!(
            surface < 1.0,
            "Center surface should drain toward ocean_level=0, got {surface}"
        );
    }

    #[test]
    fn settled_detection() {
        // A grid with no water settles after one step (no flow occurs).
        let mut grid = make_grid((3, 3), 1.0, None);
        assert!(!grid.is_settled());
        grid.step(1.0 / 60.0, flat_floor);
        assert!(grid.is_settled());
    }

    #[test]
    fn settled_after_equilibrium() {
        // Add water, step until settled.
        let mut grid = make_grid((3, 3), 1.0, None);
        for j in 0..3 {
            for i in 0..3 {
                grid.cell_mut(i, j).floor_level = 0.0;
            }
        }
        grid.add_water(1, 1, 5.0, 0.0);

        assert!(!grid.is_settled());

        let dt = 1.0 / 60.0;
        for _ in 0..1000 {
            grid.step(dt, flat_floor);
            if grid.is_settled() {
                return;
            }
        }
        panic!("Grid should have settled after sufficient steps");
    }

    #[test]
    fn dirty_floor_drain() {
        // Cell has water on a floor at y=5. After marking dirty, the floor
        // query returns None (floor destroyed). Water should drain.
        let mut grid = make_grid((3, 3), 1.0, None);
        grid.add_water(1, 1, 10.0, 5.0);

        grid.mark_dirty_floors(&[(1, 1)]);

        grid.step(1.0 / 60.0, |_x, _z| None);

        assert!(
            grid.cell(1, 1).volume <= 0.0,
            "Water should drain when floor is destroyed"
        );
    }

    #[test]
    fn dirty_floor_drop() {
        // Cell has water on floor at y=10. After damage, floor drops to y=2.
        // Use 5x5 grid so center (2,2) is not adjacent to boundary cells.
        let mut grid = make_grid((5, 5), 1.0, None);
        for j in 0..5 {
            for i in 0..5 {
                grid.cell_mut(i, j).floor_level = 10.0;
            }
        }
        grid.add_water(2, 2, 10.0, 10.0);

        grid.mark_dirty_floors(&[(2, 2)]);

        // One step: dirty floor processing sets floor to 2.0, then flow runs.
        // Neighbors at floor=10 are above the new surface (2 + 10/1 = 12),
        // so minimal flow occurs.
        grid.step(1.0 / 60.0, |_x, _z| Some(2.0));

        let cell = grid.cell(2, 2);
        assert!(
            (cell.floor_level - 2.0).abs() < 0.01,
            "Floor should drop to 2.0, got {}",
            cell.floor_level
        );
        assert!(
            cell.volume > 0.0,
            "Water volume should be preserved after floor drop"
        );
    }

    #[test]
    fn world_to_grid_conversion() {
        let grid = make_grid((10, 10), 2.0, None);

        assert_eq!(grid.world_to_grid(1.0, 1.0), Some((0, 0)));
        assert_eq!(grid.world_to_grid(3.0, 5.0), Some((1, 2)));
        assert_eq!(grid.world_to_grid(-1.0, 0.0), None);
        assert_eq!(grid.world_to_grid(20.0, 0.0), None);
    }

    #[test]
    fn surface_level_at_query() {
        let mut grid = make_grid((5, 5), 2.0, None);
        grid.add_water(2, 2, 8.0, 3.0);

        // cell_area = 4.0, so surface = 3.0 + 8.0/4.0 = 5.0
        let level = grid.surface_level_at(5.0, 5.0).unwrap();
        assert!(
            (level - 5.0).abs() < 0.001,
            "Expected surface at 5.0, got {level}"
        );

        // Dry cell returns None.
        assert!(grid.surface_level_at(1.0, 1.0).is_none());
    }

    #[test]
    fn surface_level_at_ocean_boundary_query() {
        let mut grid = make_grid((5, 5), 1.0, Some(2.0));
        for j in 0..5 {
            for i in 0..5 {
                grid.cell_mut(i, j).floor_level = 0.0;
            }
        }

        let level = grid.surface_level_at(0.5, 2.5).unwrap();
        assert!(
            (level - 2.0).abs() < 0.001,
            "Expected ocean boundary surface at 2.0, got {level}"
        );
    }
}
