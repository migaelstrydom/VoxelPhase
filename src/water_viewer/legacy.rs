//! The heightfield water the hydrology design replaces, driven headlessly.
//!
//! Stepped exactly as `WaterSystem` steps it — terrain changes marked, the flow
//! grid advanced, the wave grid advanced over it — so the harness and the perf
//! bench measure the system the game runs, not a copy of it.

use std::time::{Duration, Instant};

use nalgebra::Point3;

use crate::level::{create_level_water, Level};
use crate::rendering::water::WaterRenderer;
use crate::terrain::TerrainWorld;
use crate::water::{WaterGrid, WaveGrid};

/// Where one tick of the legacy water spent its time.
#[derive(Debug, Clone, Copy, Default)]
pub struct LegacyTickTimings {
    /// Floor rechecks from terrain changes plus the flow step.
    pub flow: Duration,
    /// The wave equation step.
    pub wave: Duration,
}

/// The legacy flow grid and wave grid of one level.
pub struct LegacyWater {
    flow: WaterGrid,
    wave: WaveGrid,
}

impl LegacyWater {
    /// The level's water, placed as the game places it. `None` for a level
    /// without water.
    pub fn from_level(level: &Level, terrain: &TerrainWorld) -> Option<Self> {
        let (flow, wave) = create_level_water(level, terrain)?;
        Some(Self { flow, wave })
    }

    /// Advance by one tick, after `terrain.update()` has published this
    /// frame's changes.
    pub fn tick(&mut self, terrain: &TerrainWorld, dt: f32) -> LegacyTickTimings {
        let started = Instant::now();
        self.flow.mark_changed_regions(terrain.changed_regions());
        self.flow
            .step(dt, |x, z| terrain.mesh_surface_height_at(x, z));
        let flow = started.elapsed();

        let started = Instant::now();
        self.wave.step(dt, &self.flow);
        LegacyTickTimings {
            flow,
            wave: started.elapsed(),
        }
    }

    /// Build the surface mesh the renderer would upload this frame, and say
    /// how long it took and how many indices it produced.
    pub fn build_mesh(&self) -> (Duration, usize) {
        let started = Instant::now();
        let (_, indices) = WaterRenderer::generate_mesh(&self.flow, &self.wave);
        (started.elapsed(), indices.len())
    }

    /// Water surface level in the column containing `point`. The heightfield
    /// holds one water per column, so `point.y` cannot pick between two.
    pub fn level_at(&self, point: Point3<f32>) -> Option<f32> {
        self.flow.surface_level_at(point.x, point.z)
    }

    /// Water held across the whole grid, in m³.
    pub fn volume(&self) -> f64 {
        self.flow.total_volume() as f64
    }

    /// Cells holding any water.
    pub fn wet_cells(&self) -> usize {
        let (width, depth) = self.flow.dims();
        (0..depth)
            .flat_map(|j| (0..width).map(move |i| (i, j)))
            .filter(|&(i, j)| self.flow.cell(i, j).volume > 0.0)
            .count()
    }

    /// World (x, z) of wet cells that border a dry one: the shoreline.
    pub fn shoreline(&self) -> Vec<(f32, f32)> {
        let (width, depth) = self.flow.dims();
        let wet = |i: i64, j: i64| {
            i >= 0
                && j >= 0
                && (i as usize) < width
                && (j as usize) < depth
                && self.flow.cell(i as usize, j as usize).volume > 0.0
        };
        let mut out = Vec::new();
        for j in 0..depth as i64 {
            for i in 0..width as i64 {
                if !wet(i, j) {
                    continue;
                }
                let dry_neighbour = [(-1, 0), (1, 0), (0, -1), (0, 1)]
                    .iter()
                    .any(|&(di, dj)| !wet(i + di, j + dj));
                if dry_neighbour {
                    out.push((
                        self.flow.cell_center_x(i as usize),
                        self.flow.cell_center_z(j as usize),
                    ));
                }
            }
        }
        out
    }
}
