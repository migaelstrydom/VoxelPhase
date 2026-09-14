//! A still body of water for a bench shot to stand things in.
//!
//! Water is the one thing in the frame that is neither opaque geometry nor part
//! of the scene's own blended pass: it is drawn after the tonemap resolve,
//! against the depth the scene left behind. That makes it the only way to test
//! how the blended pass and the post-resolve passes agree about depth — and
//! they did not, which is why this exists.
//!
//! The grids are the game's own, filled directly rather than simulated: a shot
//! has to frame the same water every run, and a settling pool does not.

use nalgebra::Vector3;

use crate::water::{WaterGrid, WaterGridConfig, WaterProperties, WaveGrid, WaveGridConfig};

/// Flow cell size, matching what a level of this size would resolve to.
const FLOW_CELL_SIZE: f32 = 1.0;

/// Wave cell size. The game's own, so ripple scale reads as it does in a level.
const WAVE_CELL_SIZE: f32 = 0.5;

/// A rectangular pool at a fixed level, centred on the origin in x and z.
#[derive(Clone, Copy, Debug)]
pub struct ScenePool {
    /// World height of the water surface.
    pub level: f32,

    /// World height of the floor the water rests on. The difference from
    /// `level` is the depth the water shader shades by, so a pool that is too
    /// shallow reads as clear glass rather than as water.
    pub floor: f32,

    /// Half-width of the pool in x and z.
    pub half_extent: f32,
}

impl ScenePool {
    pub fn new(level: f32, floor: f32, half_extent: f32) -> Self {
        Self {
            level,
            floor,
            half_extent,
        }
    }

    /// The pair of grids the water renderer draws from.
    ///
    /// Every cell is filled to exactly `level`, and the wave grid is left at
    /// rest: what this pool is for is depth and ordering, and a moving surface
    /// would only make two runs of the same shot disagree.
    pub fn grids(&self) -> (WaterGrid, WaveGrid) {
        let properties = WaterProperties::default();

        let span = (self.half_extent * 2.0 / FLOW_CELL_SIZE).ceil() as usize;
        let origin = Vector3::new(-self.half_extent, 0.0, -self.half_extent);

        let mut flow_grid = WaterGrid::new(
            WaterGridConfig {
                cell_size: FLOW_CELL_SIZE,
                dims: (span, span),
                origin,
                ocean_level: None,
            },
            &properties,
        );

        let volume = (self.level - self.floor) * flow_grid.cell_area();
        for j in 0..span {
            for i in 0..span {
                flow_grid.add_water(i, j, volume, self.floor);
            }
        }

        let cells_per_flow_cell = (FLOW_CELL_SIZE / WAVE_CELL_SIZE).round() as usize;
        let wave_grid = WaveGrid::new(
            WaveGridConfig {
                cell_size: WAVE_CELL_SIZE,
                dims: (span * cells_per_flow_cell, span * cells_per_flow_cell),
                origin,
                cells_per_flow_cell,
            },
            &properties,
        );

        (flow_grid, wave_grid)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The surface has to come out at the level asked for, or every shot that
    /// stands something half in the water is framed wrong.
    #[test]
    fn the_pool_fills_to_the_level_it_was_given() {
        let pool = ScenePool::new(0.4, -1.0, 3.0);
        let (flow, _) = pool.grids();

        let (width, depth) = flow.dims();
        for j in 0..depth {
            for i in 0..width {
                let surface = flow.cell(i, j).surface_level(flow.cell_area());
                assert!(
                    (surface - pool.level).abs() < 1e-3,
                    "cell ({i}, {j}) sits at {surface}, not {}",
                    pool.level
                );
            }
        }
    }

    /// The two grids are addressed in the same world frame, and the wave grid
    /// has to divide the flow grid exactly or the renderer interpolates the
    /// bulk level off the end of it.
    #[test]
    fn the_wave_grid_covers_the_flow_grid() {
        let pool = ScenePool::new(0.4, -1.0, 3.0);
        let (flow, wave) = pool.grids();

        assert_eq!(flow.origin(), wave.origin());
        assert_eq!(
            wave.dims().0 as f32 * WAVE_CELL_SIZE,
            flow.dims().0 as f32 * FLOW_CELL_SIZE
        );
    }
}
