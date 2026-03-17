//! Wave equation surface simulation on a fine-resolution grid.
//!
//! Runs the 2D wave equation on top of the coarse flow grid to produce
//! visible ripples that propagate from disturbances. Each wave cell stores
//! a displacement (offset from the bulk water level) and a velocity.
//!
//! A wave cell is "wet" if the flow cell it maps to has volume > 0.
//! The wave equation only steps wet cells. Dry or out-of-bounds neighbors are
//! treated as fixed-height boundaries with zero displacement.

use nalgebra::Vector3;

use super::WaterGrid;

/// Configuration for the wave grid.
pub struct WaveGridConfig {
    /// World-space width/depth of each wave cell (~0.1m for 10cm resolution).
    pub cell_size: f32,

    /// Grid dimensions (x, z). Derived from the flow grid extent and cell size.
    pub dims: (usize, usize),

    /// World-space position of grid corner (matches flow grid origin).
    pub origin: Vector3<f32>,

    /// Wave propagation speed in m/s.
    pub wave_speed: f32,

    /// Wave damping coefficient in 1/s.
    pub wave_damping: f32,

    /// Number of wave cells per flow cell in each direction.
    pub cells_per_flow_cell: usize,
}

/// A single cell in the wave grid.
#[derive(Clone, Copy)]
pub struct WaveCell {
    /// Surface displacement from the bulk water level.
    pub displacement: f32,

    /// Time derivative of displacement (vertical velocity of the surface).
    pub velocity: f32,
}

impl Default for WaveCell {
    fn default() -> Self {
        Self {
            displacement: 0.0,
            velocity: 0.0,
        }
    }
}

/// Fine-resolution wave equation simulation grid.
///
/// Sits on top of the coarse `WaterGrid` (flow grid). Each wave cell tracks
/// surface displacement and velocity. The 2D wave equation propagates
/// ripples across the wet surface. Wet/dry status is derived from the flow
/// grid — no per-cell state tracking needed.
pub struct WaveGrid {
    cells: Vec<WaveCell>,
    dims: (usize, usize),
    cell_size: f32,
    origin: Vector3<f32>,
    wave_speed: f32,
    wave_damping: f32,

    /// Number of wave cells per flow cell in each direction.
    cells_per_flow_cell: usize,

    /// Per-cell acceleration scratch buffer, reused each step.
    acceleration: Vec<f32>,
}

impl WaveGrid {
    /// Create a new wave grid from configuration.
    pub fn new(config: WaveGridConfig) -> Self {
        let total = config.dims.0 * config.dims.1;
        Self {
            cells: vec![WaveCell::default(); total],
            dims: config.dims,
            cell_size: config.cell_size,
            origin: config.origin,
            wave_speed: config.wave_speed,
            wave_damping: config.wave_damping,
            cells_per_flow_cell: config.cells_per_flow_cell,
            acceleration: vec![0.0; total],
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

    /// World-space origin of the grid corner.
    pub fn origin(&self) -> Vector3<f32> {
        self.origin
    }

    /// Number of wave cells per flow cell in each direction.
    pub fn cells_per_flow_cell(&self) -> usize {
        self.cells_per_flow_cell
    }

    /// Get a wave cell by grid coordinates.
    pub fn cell(&self, wi: usize, wj: usize) -> &WaveCell {
        &self.cells[wj * self.dims.0 + wi]
    }

    /// Get a mutable wave cell by grid coordinates.
    pub fn cell_mut(&mut self, wi: usize, wj: usize) -> &mut WaveCell {
        &mut self.cells[wj * self.dims.0 + wi]
    }

    /// Convert a world-space (x, z) to wave grid coordinates, if within bounds.
    pub fn world_to_wave(&self, x: f32, z: f32) -> Option<(usize, usize)> {
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

    /// Inject a displacement impulse at a world-space position.
    ///
    /// Adds `amount` to the velocity of the wave cell at (x, z), causing a
    /// disturbance that radiates outward. Use negative values to push the
    /// surface down (e.g. body impact).
    pub fn inject_velocity_at(&mut self, x: f32, z: f32, amount: f32) {
        if let Some((wi, wj)) = self.world_to_wave(x, z) {
            self.cells[wj * self.dims.0 + wi].velocity += amount;
        }
    }

    /// Inject a displacement at a world-space position.
    pub fn inject_displacement_at(&mut self, x: f32, z: f32, amount: f32) {
        if let Some((wi, wj)) = self.world_to_wave(x, z) {
            self.cells[wj * self.dims.0 + wi].displacement += amount;
        }
    }

    /// Step the wave equation simulation with CFL-safe substepping.
    pub fn step(&mut self, dt: f32, flow_grid: &WaterGrid) {
        if self.wave_speed <= 0.0 || dt <= 0.0 {
            return;
        }

        // CFL stability: c * dt / dx < 1/sqrt(2) ≈ 0.707
        // Use 0.5 as safety margin.
        let max_dt = 0.5 * self.cell_size / self.wave_speed;
        let mut remaining = dt;
        while remaining > 0.0 {
            let sub_dt = remaining.min(max_dt);
            self.step_internal(sub_dt, flow_grid);
            remaining -= sub_dt;
        }
    }

    /// Single substep of the wave equation.
    ///
    /// Uses a two-pass approach to avoid order-dependent energy artifacts:
    /// 1. Compute acceleration for all wet cells from current state.
    /// 2. Apply velocity and displacement updates.
    fn step_internal(&mut self, dt: f32, flow_grid: &WaterGrid) {
        let flow_dims = flow_grid.dims();
        let n = self.cells_per_flow_cell;
        let c2 = self.wave_speed * self.wave_speed;
        let inv_dx2 = 1.0 / (self.cell_size * self.cell_size);
        let damping = self.wave_damping;

        // Pass 1: compute accelerations from current state (read-only on cells).
        for fj in 0..flow_dims.1 {
            for fi in 0..flow_dims.0 {
                let flow_cell = flow_grid.cell(fi, fj);

                let w_start_i = fi * n;
                let w_start_j = fj * n;
                let w_end_i = ((fi + 1) * n).min(self.dims.0);
                let w_end_j = ((fj + 1) * n).min(self.dims.1);

                if flow_cell.volume <= 0.0 {
                    // Dry flow cell: zero all wave cells and accelerations.
                    for wj in w_start_j..w_end_j {
                        let row_start = wj * self.dims.0;
                        for wi in w_start_i..w_end_i {
                            let idx = row_start + wi;
                            self.cells[idx].displacement = 0.0;
                            self.cells[idx].velocity = 0.0;
                            self.acceleration[idx] = 0.0;
                        }
                    }
                    continue;
                }

                for wj in w_start_j..w_end_j {
                    for wi in w_start_i..w_end_i {
                        let idx = wj * self.dims.0 + wi;
                        let h = self.cells[idx].displacement;

                        let h_left = self.neighbor_disp(wi, wj, -1, 0, flow_grid);
                        let h_right = self.neighbor_disp(wi, wj, 1, 0, flow_grid);
                        let h_back = self.neighbor_disp(wi, wj, 0, -1, flow_grid);
                        let h_front = self.neighbor_disp(wi, wj, 0, 1, flow_grid);

                        let lap = (h_left + h_right + h_back + h_front - 4.0 * h) * inv_dx2;
                        let vel = self.cells[idx].velocity;
                        self.acceleration[idx] = c2 * lap - damping * vel;
                    }
                }
            }
        }

        // Pass 2: apply updates (symplectic Euler: velocity first, then position).
        for fj in 0..flow_dims.1 {
            for fi in 0..flow_dims.0 {
                if flow_grid.cell(fi, fj).volume <= 0.0 {
                    continue;
                }

                let w_start_i = fi * n;
                let w_start_j = fj * n;
                let w_end_i = ((fi + 1) * n).min(self.dims.0);
                let w_end_j = ((fj + 1) * n).min(self.dims.1);

                for wj in w_start_j..w_end_j {
                    for wi in w_start_i..w_end_i {
                        let idx = wj * self.dims.0 + wi;
                        self.cells[idx].velocity += self.acceleration[idx] * dt;
                        self.cells[idx].displacement += self.cells[idx].velocity * dt;
                        self.cells[idx].velocity =
                            self.cells[idx].velocity.clamp(-MAX_WAVE_VELOCITY, MAX_WAVE_VELOCITY);
                        self.cells[idx].displacement = self.cells[idx]
                            .displacement
                            .clamp(-MAX_WAVE_DISPLACEMENT, MAX_WAVE_DISPLACEMENT);
                    }
                }
            }
        }
    }

    /// Get displacement of a neighbor wave cell.
    ///
    /// Dry or out-of-bounds neighbors clamp to zero displacement so the water
    /// surface meets the shoreline without a free-slope boundary.
    #[inline]
    fn neighbor_disp(
        &self,
        wi: usize,
        wj: usize,
        di: i32,
        dj: i32,
        flow_grid: &WaterGrid,
    ) -> f32 {
        let ni = wi as i32 + di;
        let nj = wj as i32 + dj;

        if ni < 0 || nj < 0 || ni >= self.dims.0 as i32 || nj >= self.dims.1 as i32 {
            return 0.0;
        }

        let ni = ni as usize;
        let nj = nj as usize;

        // Check if the neighbor's flow cell is wet.
        let fi = ni / self.cells_per_flow_cell;
        let fj = nj / self.cells_per_flow_cell;
        let flow_dims = flow_grid.dims();
        if fi >= flow_dims.0 || fj >= flow_dims.1 {
            return 0.0;
        }
        if flow_grid.cell(fi, fj).volume <= 0.0 {
            return 0.0;
        }

        self.cells[nj * self.dims.0 + ni].displacement
    }

    /// Map a wave cell to its parent flow cell indices.
    #[inline]
    pub fn wave_to_flow(&self, wi: usize, wj: usize) -> (usize, usize) {
        (wi / self.cells_per_flow_cell, wj / self.cells_per_flow_cell)
    }

    /// Check if a wave cell is wet (its parent flow cell has volume > 0).
    #[inline]
    pub fn is_wet(&self, wi: usize, wj: usize, flow_grid: &WaterGrid) -> bool {
        let (fi, fj) = self.wave_to_flow(wi, wj);
        let flow_dims = flow_grid.dims();
        if fi >= flow_dims.0 || fj >= flow_dims.1 {
            return false;
        }
        flow_grid.cell(fi, fj).volume > 0.0
    }
}

const MAX_WAVE_VELOCITY: f32 = 2.0;
const MAX_WAVE_DISPLACEMENT: f32 = 1.0;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::WaterGridConfig;

    fn make_flow_grid(dims: (usize, usize), cell_size: f32) -> WaterGrid {
        WaterGrid::new(WaterGridConfig {
            cell_size,
            dims,
            origin: Vector3::new(0.0, 0.0, 0.0),
            ocean_level: None,
            flow_rate: 4.0,
            ..Default::default()
        })
    }

    fn make_wave_grid(flow_grid: &WaterGrid, wave_cell_size: f32) -> WaveGrid {
        let flow_dims = flow_grid.dims();
        let flow_cell_size = flow_grid.cell_size();
        let n = (flow_cell_size / wave_cell_size).round() as usize;
        let wave_dims = (flow_dims.0 * n, flow_dims.1 * n);

        WaveGrid::new(WaveGridConfig {
            cell_size: wave_cell_size,
            dims: wave_dims,
            origin: flow_grid.origin(),
            wave_speed: 4.0,
            wave_damping: 0.5,
            cells_per_flow_cell: n,
        })
    }

    /// Helper: fill all flow cells with water at a uniform level.
    fn fill_flow_grid(flow_grid: &mut WaterGrid, surface_level: f32) {
        let dims = flow_grid.dims();
        let cell_area = flow_grid.cell_area();
        for j in 0..dims.1 {
            for i in 0..dims.0 {
                let floor = 0.0;
                let volume = (surface_level - floor) * cell_area;
                flow_grid.add_water(i, j, volume, floor);
            }
        }
    }

    #[test]
    fn ripple_propagation() {
        let mut flow_grid = make_flow_grid((5, 5), 1.0);
        fill_flow_grid(&mut flow_grid, 5.0);

        let mut wave_grid = make_wave_grid(&flow_grid, 0.5);

        // Inject displacement at the center.
        let center_wi = wave_grid.dims().0 / 2;
        let center_wj = wave_grid.dims().1 / 2;
        wave_grid.cell_mut(center_wi, center_wj).displacement = 1.0;

        let dt = 1.0 / 60.0;
        for _ in 0..20 {
            wave_grid.step(dt, &flow_grid);
        }

        // Center should have rebounded (displacement changed from initial).
        let center_disp = wave_grid.cell(center_wi, center_wj).displacement;
        assert!(
            center_disp < 0.9,
            "Center should have spread energy outward, got {center_disp}"
        );

        // A neighbor should have received displacement.
        let neighbor_disp = wave_grid.cell(center_wi + 1, center_wj).displacement;
        assert!(
            neighbor_disp.abs() > 0.001,
            "Neighbor should have received displacement, got {neighbor_disp}"
        );
    }

    #[test]
    fn energy_decay() {
        let mut flow_grid = make_flow_grid((5, 5), 1.0);
        fill_flow_grid(&mut flow_grid, 5.0);

        let mut wave_grid = make_wave_grid(&flow_grid, 0.5);

        // Inject displacement at center.
        let center_wi = wave_grid.dims().0 / 2;
        let center_wj = wave_grid.dims().1 / 2;
        wave_grid.cell_mut(center_wi, center_wj).displacement = 1.0;

        // Measure total activity (displacement² + velocity²) as a proxy for
        // whether damping is working. After the initial transient where energy
        // redistributes from a point source, the damped system should settle.
        let activity_of = |wg: &WaveGrid| -> f32 {
            wg.cells
                .iter()
                .map(|c| c.displacement * c.displacement + c.velocity * c.velocity)
                .sum::<f32>()
        };

        let dt = 1.0 / 60.0;

        // Let the initial transient settle (energy redistributes from point source).
        for _ in 0..200 {
            wave_grid.step(dt, &flow_grid);
        }
        let mid_activity = activity_of(&wave_grid);

        // Continue stepping — activity should decrease due to damping.
        for _ in 0..500 {
            wave_grid.step(dt, &flow_grid);
        }
        let late_activity = activity_of(&wave_grid);

        assert!(
            late_activity < mid_activity * 0.5,
            "Activity should decrease significantly due to damping: mid={mid_activity}, late={late_activity}"
        );

        // After many more steps, should be nearly zero.
        for _ in 0..1000 {
            wave_grid.step(dt, &flow_grid);
        }
        let final_activity = activity_of(&wave_grid);
        assert!(
            final_activity < 0.02,
            "Damped system should settle to near-zero activity, got {final_activity}"
        );
    }

    #[test]
    fn dry_cell_isolation() {
        // 3x3 flow grid, only center cell is wet.
        let mut flow_grid = make_flow_grid((3, 3), 1.0);
        let cell_area = flow_grid.cell_area();
        flow_grid.add_water(1, 1, 5.0 * cell_area, 0.0);

        let mut wave_grid = make_wave_grid(&flow_grid, 0.5);

        // Inject displacement in a wet wave cell.
        let n = wave_grid.cells_per_flow_cell();
        let wi = 1 * n + n / 2;
        let wj = 1 * n + n / 2;
        wave_grid.cell_mut(wi, wj).displacement = 1.0;

        let dt = 1.0 / 60.0;
        for _ in 0..50 {
            wave_grid.step(dt, &flow_grid);
        }

        // Wave cells in dry flow cells should remain zero.
        for cwj in 0..n {
            for cwi in 0..n {
                let cell = wave_grid.cell(cwi, cwj); // flow cell (0,0) is dry
                assert!(
                    cell.displacement.abs() < 1e-6 && cell.velocity.abs() < 1e-6,
                    "Dry cell ({cwi},{cwj}) should be zero, got disp={}, vel={}",
                    cell.displacement,
                    cell.velocity
                );
            }
        }
    }

    #[test]
    fn cfl_stability_large_dt() {
        let mut flow_grid = make_flow_grid((3, 3), 1.0);
        fill_flow_grid(&mut flow_grid, 5.0);

        let mut wave_grid = make_wave_grid(&flow_grid, 0.5);

        // Inject a large displacement.
        let center_wi = wave_grid.dims().0 / 2;
        let center_wj = wave_grid.dims().1 / 2;
        wave_grid.cell_mut(center_wi, center_wj).displacement = 5.0;

        // Step with a very large dt — the substep guard should keep it stable.
        wave_grid.step(1.0, &flow_grid);

        // Verify no NaN or explosion.
        for cell in &wave_grid.cells {
            assert!(
                cell.displacement.is_finite(),
                "Displacement should be finite after large dt"
            );
            assert!(
                cell.velocity.is_finite(),
                "Velocity should be finite after large dt"
            );
            assert!(
                cell.displacement.abs() < 100.0,
                "Displacement should not explode: {}",
                cell.displacement
            );
        }
    }

    #[test]
    fn bank_reflection() {
        // 5x5 flow grid, fill all cells. Inject displacement near the edge.
        // After enough steps, energy should reflect back.
        let mut flow_grid = make_flow_grid((5, 5), 1.0);
        fill_flow_grid(&mut flow_grid, 5.0);

        let mut wave_grid = make_wave_grid(&flow_grid, 0.5);
        let n = wave_grid.cells_per_flow_cell();

        // Inject near the left edge of the grid.
        let wi = n; // second wave cell from left edge
        let wj = wave_grid.dims().1 / 2;
        wave_grid.cell_mut(wi, wj).displacement = 1.0;

        let dt = 1.0 / 60.0;

        // Record initial displacement on the right side of injection point.
        let probe_wi = wi + 2 * n;
        let probe_wj = wj;

        // Step until the ripple has had time to propagate and reflect.
        for _ in 0..200 {
            wave_grid.step(dt, &flow_grid);
        }

        // The probe should have seen some displacement (either direct or reflected).
        let probe_disp = wave_grid.cell(probe_wi, probe_wj).displacement;
        assert!(
            probe_disp.abs() > 1e-4,
            "Probe should have received reflected/propagated displacement, got {probe_disp}"
        );
    }

    #[test]
    fn pond_merge() {
        // Two separate wet regions with a dry gap. Inject in one, make gap wet,
        // verify ripples propagate through.
        let mut flow_grid = make_flow_grid((5, 5), 1.0);
        let cell_area = flow_grid.cell_area();

        // Wet columns 0-1 and 3-4, dry column 2.
        for j in 0..5 {
            for i in 0..5 {
                if i != 2 {
                    flow_grid.add_water(i, j, 5.0 * cell_area, 0.0);
                }
            }
        }

        let mut wave_grid = make_wave_grid(&flow_grid, 0.5);
        let n = wave_grid.cells_per_flow_cell();

        // Inject in left region.
        let wi = n / 2;
        let wj = wave_grid.dims().1 / 2;
        wave_grid.cell_mut(wi, wj).displacement = 2.0;

        let dt = 1.0 / 60.0;
        for _ in 0..50 {
            wave_grid.step(dt, &flow_grid);
        }

        // Right region should have no displacement (gap is dry).
        let right_wi = 3 * n + n / 2;
        let right_disp = wave_grid.cell(right_wi, wj).displacement;
        assert!(
            right_disp.abs() < 1e-4,
            "Right region should be isolated before merge, got {right_disp}"
        );

        // Now make the gap wet.
        for j in 0..5 {
            flow_grid.add_water(2, j, 5.0 * cell_area, 0.0);
        }

        // Inject again and step.
        wave_grid.cell_mut(wi, wj).displacement = 2.0;
        for _ in 0..200 {
            wave_grid.step(dt, &flow_grid);
        }

        // Right region should now have displacement.
        let right_disp = wave_grid.cell(right_wi, wj).displacement;
        assert!(
            right_disp.abs() > 1e-4,
            "Right region should receive ripples after merge, got {right_disp}"
        );
    }

    #[test]
    fn newly_wet_cells_remain_quiet_without_explicit_impulse() {
        let mut flow_grid = make_flow_grid((4, 4), 1.0);
        let cell_area = flow_grid.cell_area();

        for j in 0..4 {
            for i in 0..4 {
                if i < 2 {
                    flow_grid.add_water(i, j, 5.0 * cell_area, 0.0);
                }
            }
        }

        let mut wave_grid = make_wave_grid(&flow_grid, 0.5);
        let dt = 1.0 / 60.0;

        wave_grid.step(dt, &flow_grid);

        for j in 0..4 {
            for i in 2..4 {
                flow_grid.add_water(i, j, 3.0 * cell_area, 0.0);
            }
        }

        for _ in 0..10 {
            wave_grid.step(dt, &flow_grid);
        }

        let n = wave_grid.cells_per_flow_cell();
        let left_interface = wave_grid.cell(2 * n - 1, 2 * n).displacement;
        let right_interface = wave_grid.cell(2 * n, 2 * n).displacement;

        assert!(
            left_interface.abs() < 1e-4 && right_interface.abs() < 1e-4,
            "Newly wet cells should stay quiet without an explicit impulse: left={left_interface}, right={right_interface}"
        );
    }
}
