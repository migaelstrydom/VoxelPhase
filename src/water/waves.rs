//! The waves laid over the field's surface: ripples where bodies stir still
//! water, and the sea's swell.
//!
//! ```text
//!   field surface ──▶ + swell   where the water stands near sea level,
//!                               fading out over SEA_BAND either side
//!                 ──▶ + ripples in awake tiles over still water: a tile
//!                               wakes only where its water is flat to FLAT
//! ```
//!
//! The field carries the water; the waves carry nothing. They only move the
//! surface that is drawn and that floating bodies ride, and both read them
//! the same way, so a body bobs on the swell it is drawn on.

use nalgebra::Point3;

use crate::level::WaterConfig;
use crate::water::coupling::{Disturbance, Disturbances};
use crate::water::field::{WaterField, WET_DEPTH};
use crate::water::geometry::{Column, SpanChunkCoord, CHUNK_COLUMNS, COLUMNS_PER_CHUNK};
use crate::water::ids::WaterBodyId;
use crate::water::query::FIELD_BODY;
use crate::water::surface::{
    corner_floor, MaskSource, RippleConfig, RippleTiles, Swell, TileMask, TILE_CORNERS,
};

/// How far from sea level water still takes some of the swell, m: full at
/// sea level, none this far above or below it. A river running into the sea
/// takes it up over its last stretch rather than at a line.
pub const SEA_BAND: f32 = 0.5;

/// Most a tile's water may vary in height and still ripple, m: still water.
/// Running water carries no ripple tile.
const FLAT: f32 = 0.03;

/// The sea beyond the map's open edges, as its swell sees it.
#[derive(Debug, Clone, Copy)]
pub struct Sea {
    pub level: f32,
    pub swell: Swell,
}

/// Ripples and the sea's swell.
pub struct SurfaceWaves {
    ripples: RippleTiles,
    sea: Option<Sea>,
}

impl SurfaceWaves {
    pub fn new(config: &WaterConfig) -> Self {
        Self {
            ripples: RippleTiles::new(RippleConfig::default()),
            sea: config.ocean.as_ref().map(|ocean| Sea {
                level: ocean.level,
                swell: Swell {
                    amplitude: ocean.swell,
                    phase: 0.0,
                },
            }),
        }
    }

    /// Re-mask the awake tiles to the water as it now stands, and step them.
    pub fn step(&mut self, field: &WaterField, dt: f32) {
        let masks = FieldMasks { field };
        self.ripples.refresh_masks(&masks);
        self.ripples.step(dt, &masks);
    }

    /// Apply every disturbance the coupler collected this frame.
    pub fn apply(&mut self, field: &WaterField, disturbances: &Disturbances) {
        let masks = FieldMasks { field };
        for &(at, radius, disturbance) in &disturbances.pending {
            self.disturb(&masks, at, radius, disturbance);
        }
    }

    fn disturb(
        &mut self,
        masks: &FieldMasks,
        at: Point3<f32>,
        radius: f32,
        disturbance: Disturbance,
    ) {
        self.ripples
            .disturb(FIELD_BODY, at.x, at.z, radius, disturbance, masks);
    }

    /// Where the camera is, for ranking ripple tiles against their budget.
    pub fn set_focus(&mut self, focus: Option<Point3<f32>>) {
        self.ripples.set_focus(focus);
    }

    pub fn ripples(&self) -> &RippleTiles {
        &self.ripples
    }

    /// The ripples' displacement at (x, z); zero where no tile is awake.
    pub fn ripple_at(&self, x: f32, z: f32) -> f32 {
        self.ripples.height_at(FIELD_BODY, x, z)
    }

    pub fn sea(&self) -> Option<Sea> {
        self.sea
    }

    /// The swell on water whose still surface stands at `surface`: the
    /// sea's, scaled by how near sea level it stands.
    pub fn swell_at(&self, surface: f32) -> Swell {
        self.sea.map_or(Swell::default(), |sea| Swell {
            amplitude: sea.swell.amplitude * swell_share(sea.level, surface),
            phase: sea.swell.phase,
        })
    }
}

/// How much of the sea's swell water standing at `surface` takes: all at sea
/// level, none `SEA_BAND` above or below it. Mirrored in `river.vert`.
pub fn swell_share(sea_level: f32, surface: f32) -> f32 {
    (1.0 - (surface - sea_level).abs() / SEA_BAND).clamp(0.0, 1.0)
}

/// Tile masks read off the field: where a tile's water is still, the
/// columns it covers and the floor under each.
struct FieldMasks<'a> {
    field: &'a WaterField,
}

impl MaskSource for FieldMasks<'_> {
    fn mask(&self, tile: SpanChunkCoord, body: WaterBodyId) -> Option<TileMask> {
        if body != FIELD_BODY {
            return None;
        }
        let level = self.still_level(tile)?;
        // The tile's columns, a ring of its neighbours' for the corners on
        // its edges, and one more ring to find the banks beside them.
        const SIDE: usize = CHUNK_COLUMNS as usize + 4;
        let origin = tile.column(0).offset(-2, -2);
        let at = |i: usize, k: usize| origin.offset(i as i32, k as i32);
        let mut wet = [false; SIDE * SIDE];
        let mut floor = [f32::NAN; SIDE * SIDE];
        for k in 0..SIDE {
            for i in 0..SIDE {
                if let Some((f, w)) = self.column(at(i, k), level) {
                    floor[k * SIDE + i] = f;
                    wet[k * SIDE + i] = w;
                }
            }
        }
        // A dry column is water's only where it banks wet water: its foot
        // under the level, beside a wet column.
        let drawn = |i: usize, k: usize| -> f32 {
            let here = k * SIDE + i;
            let beside = [(1, 0), (-1, 0), (0, 1), (0, -1)].iter().any(|&(di, dk)| {
                let (ni, nk) = (i as i32 + di, k as i32 + dk);
                (0..SIDE as i32).contains(&ni)
                    && (0..SIDE as i32).contains(&nk)
                    && wet[nk as usize * SIDE + ni as usize]
            });
            if wet[here] || beside {
                floor[here]
            } else {
                f32::NAN
            }
        };
        let n = CHUNK_COLUMNS as usize;
        let mut mask = TileMask {
            floors: [f32::NAN; COLUMNS_PER_CHUNK],
            corners: [f32::NAN; TILE_CORNERS * TILE_CORNERS],
            level,
        };
        for k in 0..n {
            for i in 0..n {
                mask.floors[k * n + i] = drawn(i + 2, k + 2);
            }
        }
        for k in 0..TILE_CORNERS {
            for i in 0..TILE_CORNERS {
                mask.corners[k * TILE_CORNERS + i] = corner_floor([
                    drawn(i + 1, k + 1),
                    drawn(i + 2, k + 1),
                    drawn(i + 1, k + 2),
                    drawn(i + 2, k + 2),
                ]);
            }
        }
        (!mask.is_empty()).then_some(mask)
    }
}

impl FieldMasks<'_> {
    /// The mean surface of the tile's wet cells, if they all stand within
    /// `FLAT` of one another.
    fn still_level(&self, tile: SpanChunkCoord) -> Option<f32> {
        let cells = self.field.cells();
        let slot = cells.slot_of(tile)?;
        let chunk = &cells.chunks[slot];
        let (mut low, mut high, mut sum, mut count) = (f64::INFINITY, f64::NEG_INFINITY, 0.0, 0);
        for flat in 0..chunk.depth.len() {
            if chunk.depth[flat] > WET_DEPTH {
                let surface = chunk.surface(flat);
                low = low.min(surface);
                high = high.max(surface);
                sum += surface;
                count += 1;
            }
        }
        (count > 0 && high - low <= FLAT as f64).then(|| (sum / count as f64) as f32)
    }

    /// The floor of a column's span at `level`, if water there would cover
    /// some of it, and whether it is wet at that level.
    fn column(&self, column: Column, level: f32) -> Option<(f32, bool)> {
        let graph = self.field.graph();
        if !graph.contains_column(column) {
            return None;
        }
        let span = graph.span_at(column, level)?;
        let floor = graph.span(span).floor_min;
        if floor >= level {
            return None;
        }
        let cells = self.field.cells();
        let wet = cells
            .index(graph, span.column, span.ordinal)
            .is_some_and(|at| {
                cells.depth(at) > WET_DEPTH && (cells.surface(at) as f32 - level).abs() <= FLAT
            });
        Some((floor, wet))
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;
    use crate::level::load_level;
    use crate::level_check::build_terrain;
    use crate::water::WaterWorld;

    /// A splash on island_sea's pool wakes a tile, whose ripples spread
    /// out from it and move the surface floating bodies read.
    #[test]
    #[ignore = "release: opens island_sea"]
    fn a_splash_on_still_water_ripples_it() {
        let level = load_level(Path::new("levels/island_sea.level.ron")).unwrap();
        let terrain = build_terrain(&level);
        let mut water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
        let at = Point3::new(-4.0, 1.0, 12.0);
        let still = water.query().level_at(at).expect("the pool is wet there");
        water.apply(&Disturbances {
            pending: vec![(at, 0.5, Disturbance::Velocity(-3.0))],
        });
        assert_eq!(water.waves().ripples().active_count(), 1, "one tile woken");
        let mut moved: f32 = 0.0;
        for _ in 0..60 {
            water.step(1.0 / 60.0);
            let ring = water.query().sample(Point3::new(-2.5, 1.0, 12.0)).unwrap();
            moved = moved.max((ring.surface - still).abs());
        }
        assert!(moved > 1.0e-3, "1.5 m out the surface moved {moved} m");
    }
}
