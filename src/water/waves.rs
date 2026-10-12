//! The waves laid over the field's surface: ripples where bodies stir still
//! water, and the sea's swell.
//!
//! ```text
//!   field surface ──▶ + swell   where the water stands near sea level,
//!                               fading out over SEA_BAND either side
//!                 ──▶ + ripples in awake tiles, still or running: the
//!                               current under them carries them, and they
//!                               die where the surface breaks
//! ```
//!
//! The field carries the water; the waves carry nothing. They only move the
//! surface that is drawn and that floating bodies ride, and both read them
//! the same way, so a body bobs on the swell it is drawn on.

use nalgebra::{Point3, Vector2};

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

/// Most a tile's water may vary in height and still stand still, m: still
/// water, which the renderer divides blended surfaces by as one level.
const FLAT: f32 = 0.03;

/// Least step between neighbouring columns' surfaces at which the surface
/// breaks, m: a fall's lip and foot, or a step in rapids, where ripples die.
/// A river's own slope steps a few centimetres a column.
const BREAK_STEP: f32 = 0.15;

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

/// The mean surface of a tile's wet cells, if they all stand within `FLAT`
/// of one another: still water, which ripples and which the renderer divides
/// blended surfaces by.
pub fn still_level(field: &WaterField, tile: SpanChunkCoord) -> Option<f32> {
    let cells = field.cells();
    let chunk = &cells.chunks[cells.slot_of(tile)?];
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

/// Tile masks read off the field: the columns a tile's water covers, the
/// floor and surface under each and the current through it, still or
/// running.
struct FieldMasks<'a> {
    field: &'a WaterField,
}

/// A column's water as a tile's mask reads it.
#[derive(Clone, Copy)]
struct ColumnWater {
    floor: f32,
    surface: f32,
    flow: Vector2<f32>,
    /// Whether it holds water itself, rather than banking the water beside
    /// it.
    wet: bool,
}

impl MaskSource for FieldMasks<'_> {
    fn mask(&self, tile: SpanChunkCoord, body: WaterBodyId) -> Option<TileMask> {
        if body != FIELD_BODY {
            return None;
        }
        // The tile's columns, a ring of its neighbours' for the corners on
        // its edges, and one more ring to find the banks beside them.
        const SIDE: usize = CHUNK_COLUMNS as usize + 4;
        const ORTHOGONAL: [(i32, i32); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];
        let origin = tile.column(0).offset(-2, -2);
        let at = |i: usize, k: usize| origin.offset(i as i32, k as i32);
        let beside = |i: usize, k: usize| {
            ORTHOGONAL.iter().filter_map(move |&(di, dk)| {
                let (ni, nk) = (i as i32 + di, k as i32 + dk);
                ((0..SIDE as i32).contains(&ni) && (0..SIDE as i32).contains(&nk))
                    .then(|| nk as usize * SIDE + ni as usize)
            })
        };
        let mut water: [Option<ColumnWater>; SIDE * SIDE] = [None; SIDE * SIDE];
        for k in 0..SIDE {
            for i in 0..SIDE {
                water[k * SIDE + i] = self.wet_column(at(i, k));
            }
        }
        // A dry column is water's only where it banks wet water: its foot
        // under the surface beside it.
        let mut drawn = water;
        for k in 0..SIDE {
            for i in 0..SIDE {
                if water[k * SIDE + i].is_some() {
                    continue;
                }
                let surface = beside(i, k)
                    .filter_map(|n| water[n].map(|w| w.surface))
                    .fold(f32::NAN, f32::max);
                if !surface.is_nan() {
                    drawn[k * SIDE + i] = self.bank(at(i, k), surface);
                }
            }
        }

        let n = CHUNK_COLUMNS as usize;
        let mut mask = TileMask {
            floors: [f32::NAN; COLUMNS_PER_CHUNK],
            corners: [f32::NAN; TILE_CORNERS * TILE_CORNERS],
            surfaces: [f32::NAN; COLUMNS_PER_CHUNK],
            flows: [Vector2::zeros(); COLUMNS_PER_CHUNK],
            breaks: [false; COLUMNS_PER_CHUNK],
            level: 0.0,
        };
        let (mut sum, mut count) = (0.0, 0);
        for k in 0..n {
            for i in 0..n {
                let here = (k + 2) * SIDE + i + 2;
                let Some(column) = drawn[here] else {
                    continue;
                };
                let at = k * n + i;
                mask.floors[at] = column.floor;
                mask.surfaces[at] = column.surface;
                mask.flows[at] = column.flow;
                mask.breaks[at] = column.wet
                    && beside(i + 2, k + 2).any(|b| {
                        water[b].is_some_and(|w| (w.surface - column.surface).abs() > BREAK_STEP)
                    });
                if column.wet {
                    sum += column.surface;
                    count += 1;
                }
            }
        }
        if count == 0 {
            return None;
        }
        mask.level = sum / count as f32;
        let floor = |i: usize, k: usize| drawn[k * SIDE + i].map_or(f32::NAN, |c| c.floor);
        for k in 0..TILE_CORNERS {
            for i in 0..TILE_CORNERS {
                mask.corners[k * TILE_CORNERS + i] = corner_floor([
                    floor(i + 1, k + 1),
                    floor(i + 2, k + 1),
                    floor(i + 1, k + 2),
                    floor(i + 2, k + 2),
                ]);
            }
        }
        Some(mask)
    }
}

impl FieldMasks<'_> {
    /// A column's water: of its wet spans, the one standing highest, which
    /// is what is seen from above.
    fn wet_column(&self, column: Column) -> Option<ColumnWater> {
        let graph = self.field.graph();
        if !graph.contains_column(column) {
            return None;
        }
        let cells = self.field.cells();
        graph
            .spans(column)
            .iter()
            .enumerate()
            .filter_map(|(ordinal, span)| {
                let at = cells.index(graph, column, ordinal as u8)?;
                (cells.depth(at) > WET_DEPTH).then(|| ColumnWater {
                    floor: span.floor_min,
                    surface: cells.surface(at) as f32,
                    flow: self.field.velocity(at),
                    wet: true,
                })
            })
            .max_by(|a, b| a.surface.total_cmp(&b.surface))
    }

    /// A dry column beside water standing at `surface`, if its floor dips
    /// under that surface: the water's bank, ripples running up it.
    fn bank(&self, column: Column, surface: f32) -> Option<ColumnWater> {
        let graph = self.field.graph();
        if !graph.contains_column(column) {
            return None;
        }
        let floor = graph.span(graph.span_at(column, surface)?).floor_min;
        (floor < surface).then_some(ColumnWater {
            floor,
            surface,
            flow: Vector2::zeros(),
            wet: false,
        })
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;
    use crate::level::load_level;
    use crate::level_check::build_terrain;
    use crate::water::WaterWorld;

    /// A splash on island_sea's open sea wakes a tile, whose ripples spread
    /// out from it and move the surface floating bodies read.
    #[test]
    #[ignore = "release: opens island_sea"]
    fn a_splash_on_still_water_ripples_it() {
        let level = load_level(Path::new("levels/island_sea.level.ron")).unwrap();
        let terrain = build_terrain(&level);
        let mut water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
        let at = Point3::new(-30.0, 0.0, 36.0);
        let still = water.query().level_at(at).expect("the sea is wet there");
        water.apply(&Disturbances {
            pending: vec![(at, 0.5, Disturbance::Velocity(-3.0))],
        });
        assert_eq!(water.waves().ripples().active_count(), 1, "one tile woken");
        let mut moved: f32 = 0.0;
        for _ in 0..60 {
            water.step(1.0 / 60.0);
            let ring = water.query().sample(Point3::new(-28.5, 0.0, 36.0)).unwrap();
            moved = moved.max((ring.surface - still).abs());
        }
        assert!(moved > 1.0e-3, "1.5 m out the surface moved {moved} m");
    }

    /// A splash on island_sea's river below the cascade ripples it, and the
    /// current, faster there than the waves, carries every ripple
    /// downstream: none reaches as far upstream. Read from the ripples
    /// themselves, as the river is still settling under them.
    #[test]
    #[ignore = "release: opens island_sea"]
    fn a_splash_on_a_river_is_carried_downstream() {
        let level = load_level(Path::new("levels/island_sea.level.ron")).unwrap();
        let terrain = build_terrain(&level);
        let mut water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
        let at = Point3::new(13.75, 10.0, -10.75);
        let cell = water.field().cell_at(at).expect("the river runs there");
        let current = water.field().velocity(cell);
        assert!(current.norm() > 1.5, "the river runs at {current:?} there");
        let along = current.normalize() * 1.25;
        water.apply(&Disturbances {
            pending: vec![(at, 0.3, Disturbance::Velocity(-3.0))],
        });
        assert!(water.waves().ripples().active_count() >= 1, "a tile woken");
        let ripple = |water: &WaterWorld, sign: f32| {
            water
                .waves()
                .ripple_at(at.x + sign * along.x, at.z + sign * along.y)
                .abs()
        };
        let (mut downstream, mut upstream): (f32, f32) = (0.0, 0.0);
        for _ in 0..60 {
            water.step(1.0 / 60.0);
            downstream = downstream.max(ripple(&water, 1.0));
            upstream = upstream.max(ripple(&water, -1.0));
        }
        assert!(
            downstream > 1.0e-2 && upstream < 1.0e-3,
            "ripples {downstream} m downstream, {upstream} m upstream"
        );
    }
}
