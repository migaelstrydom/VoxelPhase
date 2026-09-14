//! Where each authored body of water actually ends up.
//!
//! A `Pool` is a seed and a surface level; its extent is a flood fill through
//! every cell with a sample at or under that level. The author pictures a
//! basin. What the fill sees is whichever cells happen to dip low enough,
//! including the rough ground *around* the basin and the void past the edge of
//! the terrain, so a pool authored a hand's breadth under the plain drowns the
//! level. Nothing else in the pipeline says so: the game just starts wet.
//!
//! Each body is placed into its own fresh grid so that its footprint is its
//! own. The combined grid the game uses cannot attribute a cell to a body.

use crate::level::spawner::{empty_flow_grid, fill_body};
use crate::level::{Level, WaterBody};
use crate::terrain::TerrainWorld;
use crate::water::WaterGrid;

use super::report::{Report, Section};

/// Share of the level's footprint past which a single body is a flood rather
/// than a pool. Half the map under one body is not a design anyone authored
/// by picking a seed point.
const FLOOD_FRACTION: f32 = 0.5;

/// The measured footprint of one placed body.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyExtent {
    /// Grid cells holding water.
    pub cells: usize,
    /// Grid cells in the whole level footprint.
    pub grid_cells: usize,
    /// World-space bounding box of the wet cells, as (min_x, min_z, max_x, max_z).
    pub bounds: Option<(f32, f32, f32, f32)>,
    /// Whether any wet cell lies on the outermost ring of the grid: the fill
    /// ran to the edge of the world and would have kept going.
    pub reaches_edge: bool,
}

impl BodyExtent {
    /// Fraction of the level footprint this body covers.
    pub fn coverage(&self) -> f32 {
        if self.grid_cells == 0 {
            0.0
        } else {
            self.cells as f32 / self.grid_cells as f32
        }
    }

    /// Whether this body escaped the basin it was authored in.
    pub fn is_flood(&self) -> bool {
        self.reaches_edge || self.coverage() > FLOOD_FRACTION
    }
}

/// Measure the wet cells of a filled grid.
pub fn measure(grid: &WaterGrid) -> BodyExtent {
    let (width, depth) = grid.dims();
    let cell = grid.cell_size();
    let mut cells = 0;
    let mut reaches_edge = false;
    let mut bounds: Option<(f32, f32, f32, f32)> = None;

    for j in 0..depth {
        for i in 0..width {
            if grid.cell(i, j).volume <= 0.0 {
                continue;
            }
            cells += 1;
            if i == 0 || j == 0 || i + 1 == width || j + 1 == depth {
                reaches_edge = true;
            }
            let (x, z) = (grid.cell_center_x(i), grid.cell_center_z(j));
            let half = cell * 0.5;
            bounds = Some(match bounds {
                None => (x - half, z - half, x + half, z + half),
                Some((x0, z0, x1, z1)) => (
                    x0.min(x - half),
                    z0.min(z - half),
                    x1.max(x + half),
                    z1.max(z + half),
                ),
            });
        }
    }

    BodyExtent {
        cells,
        grid_cells: width * depth,
        bounds,
        reaches_edge,
    }
}

/// Place every body on its own and report where each one went.
///
/// Returns `None` for a level without water, so the section is simply absent
/// rather than empty.
pub fn check_water(level: &Level, terrain: &TerrainWorld, report: &mut Report) -> Option<Section> {
    let config = level.water.as_ref()?;
    let mut section = Section::new("Water");

    if let Some(level_y) = config.ocean_level {
        section.row("Ocean", format!("surface {level_y:.1}"));
    }

    for (index, body) in config.bodies.iter().enumerate() {
        let mut grid = empty_flow_grid(config, terrain);
        fill_body(&mut grid, terrain, body);
        let extent = measure(&grid);

        let WaterBody::Pool {
            seed,
            surface_level,
        } = body;
        let label = format!(
            "Pool #{} ({:.0}, {:.0}) @ {surface_level:.1}",
            index + 1,
            seed.0,
            seed.1
        );

        let value =
            match extent.bounds {
                None => "dry — the seed cell is above the surface level".to_string(),
                Some((x0, z0, x1, z1)) => {
                    format!(
                "{} cells · {:.0}% of the footprint · ({x0:.0}, {z0:.0})..({x1:.0}, {z1:.0}){}",
                extent.cells,
                extent.coverage() * 100.0,
                if extent.reaches_edge { " · reaches the edge of the world" } else { "" },
            )
                }
            };
        section.row(label, value);

        if extent.bounds.is_none() {
            report.warn(
                "water",
                format!(
                    "water body #{}: seed ({:.1}, {:.1}) sits above its surface level {surface_level:.1}, \
                     so the pool is empty",
                    index + 1,
                    seed.0,
                    seed.1
                ),
            );
        } else if extent.is_flood() {
            report.error(
                "water",
                format!(
                    "water body #{}: the fill from ({:.1}, {:.1}) at {surface_level:.1} escaped its basin — \
                     {:.0}% of the level is under it{}. Raise the ground around the basin \
                     clear of the surface level, roughness included, or lower the surface.",
                    index + 1,
                    seed.0,
                    seed.1,
                    extent.coverage() * 100.0,
                    if extent.reaches_edge { " and it reaches the edge of the world" } else { "" },
                ),
            );
        }
    }

    section.note(
        "A pool claims every cell with a sample at or under its surface level, connected to \
         the seed. Ground within the terrain roughness of the surface is a leak.",
    );
    Some(section)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn extent(cells: usize, grid_cells: usize, reaches_edge: bool) -> BodyExtent {
        BodyExtent {
            cells,
            grid_cells,
            bounds: Some((0.0, 0.0, 1.0, 1.0)),
            reaches_edge,
        }
    }

    #[test]
    fn a_pool_inside_its_basin_is_not_a_flood() {
        assert!(!extent(100, 10_000, false).is_flood());
    }

    #[test]
    fn covering_most_of_the_map_is_a_flood() {
        assert!(extent(6_000, 10_000, false).is_flood());
    }

    #[test]
    fn touching_the_edge_of_the_world_is_a_flood_however_small() {
        assert!(extent(3, 10_000, true).is_flood());
    }
}
