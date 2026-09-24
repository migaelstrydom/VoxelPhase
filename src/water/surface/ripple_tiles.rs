//! Ripple tiles: the short waves bodies make, simulated only near them.
//!
//! ```text
//!   8 m tile (one span chunk) × 64 × 64 cells of 0.125 m, keyed by (tile, body)
//!   woken by a disturbance, or by energy crossing in from an active neighbour
//!   asleep after 2 s quiet; at most 32 awake, the weakest and farthest evicted
//! ```
//!
//! Keyed by body, so an island pool and the sea beneath it ripple apart, and
//! each tile is masked to its body's columns. A merge or split makes a new
//! body, so its tiles restart calm. The edges of a tile with no awake
//! neighbour absorb what reaches them through a sponge, rather than
//! reflecting it back.

use std::collections::BTreeMap;

use nalgebra::Point3;

use crate::water::coupling::Disturbance;
use crate::water::geometry::{
    Column, SpanChunkCoord, CHUNK_COLUMNS, COLUMNS_PER_CHUNK, COLUMN_SIZE,
};
use crate::water::ids::WaterBodyId;

/// Cells along each side of a tile.
pub const TILE_CELLS: usize = 64;

/// Cells in a tile.
pub const CELLS_PER_TILE: usize = TILE_CELLS * TILE_CELLS;

/// Width of a cell, m.
pub const RIPPLE_CELL: f32 = COLUMN_SIZE * CHUNK_COLUMNS as f32 / TILE_CELLS as f32;

/// Cells along each side of a column.
const CELLS_PER_COLUMN: usize = TILE_CELLS / CHUNK_COLUMNS as usize;

/// A tile's identity: where it is, and whose water it ripples.
pub type RippleKey = (SpanChunkCoord, WaterBodyId);

/// Which columns of a tile hold the body's water, and the floor under each.
#[derive(Debug, Clone, PartialEq)]
pub struct TileMask {
    /// Per column, row-major; NaN where the body holds no water there.
    pub floors: [f32; COLUMNS_PER_CHUNK],
}

impl TileMask {
    pub fn is_empty(&self) -> bool {
        self.floors.iter().all(|f| f.is_nan())
    }

    fn wet_cell(&self, cell: usize) -> bool {
        let (ci, ck) = (cell % TILE_CELLS, cell / TILE_CELLS);
        let column = (ck / CELLS_PER_COLUMN) * CHUNK_COLUMNS as usize + ci / CELLS_PER_COLUMN;
        !self.floors[column].is_nan()
    }
}

/// Where tile masks come from: the network, which knows each body's columns.
pub trait MaskSource {
    /// The body's mask over a tile, or `None` if it has no water there.
    fn mask(&self, tile: SpanChunkCoord, body: WaterBodyId) -> Option<TileMask>;
}

/// How ripples move and settle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RippleConfig {
    /// Wave speed, m/s.
    pub wave_speed: f32,
    /// Velocity damping, 1/s.
    pub damping: f32,
    /// A tile quieter than `sleep_energy` for this long sleeps, s.
    pub sleep_after: f32,
    pub sleep_energy: f32,
    /// Energy in a tile's edge strip that wakes the neighbour it faces.
    pub wake_energy: f32,
    /// Most tiles awake at once.
    pub max_active: usize,
    /// Cells of sponge along an edge with nothing awake beyond it.
    pub sponge: usize,
}

impl Default for RippleConfig {
    fn default() -> Self {
        Self {
            wave_speed: 2.0,
            damping: 0.5,
            sleep_after: 2.0,
            sleep_energy: 1e-4,
            wake_energy: 1e-3,
            max_active: 32,
            sponge: 6,
        }
    }
}

/// One awake tile.
#[derive(Debug, Clone)]
pub struct RippleTile {
    pub mask: TileMask,
    /// Surface displacement per cell, m.
    pub height: Vec<f32>,
    velocity: Vec<f32>,
    /// 1 where the cell holds the body's water, 0 where it is dry: the
    /// mask per cell, as a factor the step multiplies through.
    wet: Vec<f32>,
    /// Scratch for the step's accelerations.
    acceleration: Vec<f32>,
    /// Seconds spent below the sleep energy.
    quiet: f32,
    /// Energy after the last step.
    energy: f32,
}

impl RippleTile {
    fn new(mask: TileMask) -> Self {
        let mut tile = Self {
            mask,
            height: vec![0.0; CELLS_PER_TILE],
            velocity: vec![0.0; CELLS_PER_TILE],
            wet: vec![0.0; CELLS_PER_TILE],
            acceleration: vec![0.0; CELLS_PER_TILE],
            quiet: 0.0,
            energy: 0.0,
        };
        tile.set_mask(tile.mask.clone());
        tile
    }

    /// Take a new mask, stilling any cell that has gone dry.
    fn set_mask(&mut self, mask: TileMask) {
        for cell in 0..CELLS_PER_TILE {
            let wet = mask.wet_cell(cell);
            self.wet[cell] = if wet { 1.0 } else { 0.0 };
            if !wet {
                self.height[cell] = 0.0;
                self.velocity[cell] = 0.0;
            }
        }
        self.mask = mask;
    }

    pub fn energy(&self) -> f32 {
        self.energy
    }
}

/// Every awake ripple tile.
#[derive(Debug, Default)]
pub struct RippleTiles {
    tiles: BTreeMap<RippleKey, RippleTile>,
    config: RippleConfig,
    /// Where the camera is, for ranking tiles against the budget.
    focus: Option<Point3<f32>>,
}

/// The four edge neighbours of a tile, with the edge they share.
const EDGES: [(i32, i32); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];

impl RippleTiles {
    pub fn new(config: RippleConfig) -> Self {
        Self {
            tiles: BTreeMap::new(),
            config,
            focus: None,
        }
    }

    pub fn set_focus(&mut self, focus: Option<Point3<f32>>) {
        self.focus = focus;
    }

    /// Awake tiles, in key order.
    pub fn active(&self) -> impl Iterator<Item = (&RippleKey, &RippleTile)> {
        self.tiles.iter()
    }

    pub fn active_count(&self) -> usize {
        self.tiles.len()
    }

    /// Drop tiles whose body is gone or whose mask no longer holds water.
    pub fn retain(&mut self, mut keep: impl FnMut(&RippleKey) -> bool) {
        self.tiles.retain(|key, _| keep(key));
    }

    /// Refresh the mask of every awake tile, as a body's water moves.
    pub fn refresh_masks(&mut self, masks: &dyn MaskSource) {
        let keys: Vec<RippleKey> = self.tiles.keys().copied().collect();
        for key in keys {
            match masks.mask(key.0, key.1) {
                Some(mask) if !mask.is_empty() => {
                    let tile = self.tiles.get_mut(&key).expect("key from this map");
                    tile.set_mask(mask);
                }
                _ => {
                    self.tiles.remove(&key);
                }
            }
        }
    }

    /// Disturb a body's surface over a disc about (x, z), waking the tile
    /// under it if it sleeps.
    pub fn disturb(
        &mut self,
        body: WaterBodyId,
        x: f32,
        z: f32,
        radius: f32,
        disturbance: Disturbance,
        masks: &dyn MaskSource,
    ) {
        let reach = radius.max(RIPPLE_CELL * 0.5);
        let lo = Column::containing(x - reach, z - reach).chunk();
        let hi = Column::containing(x + reach, z + reach).chunk();
        for tz in lo.z..=hi.z {
            for tx in lo.x..=hi.x {
                let coord = SpanChunkCoord { x: tx, z: tz };
                let Some(tile) = self.wake((coord, body), masks) else {
                    continue;
                };
                let (x0, z0) = coord.column(0).min_corner();
                let span = |lo: f32, hi: f32, origin: f32| {
                    let a = ((lo - origin) / RIPPLE_CELL - 0.5).floor().max(0.0) as usize;
                    let b = ((hi - origin) / RIPPLE_CELL - 0.5).ceil().max(0.0) as usize;
                    a..(b + 1).min(TILE_CELLS)
                };
                for ck in span(z - reach, z + reach, z0) {
                    for ci in span(x - reach, x + reach, x0) {
                        let cx = x0 + (ci as f32 + 0.5) * RIPPLE_CELL;
                        let cz = z0 + (ck as f32 + 0.5) * RIPPLE_CELL;
                        let d = ((cx - x).powi(2) + (cz - z).powi(2)).sqrt();
                        let t = if radius > 0.0 {
                            1.0 - d / radius
                        } else if d <= RIPPLE_CELL * 0.71 {
                            1.0
                        } else {
                            0.0
                        };
                        let cell = ck * TILE_CELLS + ci;
                        if t <= 0.0 || !tile.mask.wet_cell(cell) {
                            continue;
                        }
                        match disturbance {
                            Disturbance::Velocity(a) => tile.velocity[cell] += a * t,
                            Disturbance::Displacement(a) => tile.height[cell] += a * t,
                        }
                    }
                }
                tile.quiet = 0.0;
                // Ranked against the budget before its next step.
                tile.energy = tile
                    .height
                    .iter()
                    .zip(&tile.velocity)
                    .map(|(h, v)| h * h + v * v / 3600.0)
                    .sum();
            }
        }
    }

    /// The tile for a key, woken with a fresh mask if it was asleep. `None`
    /// if the body has no water there.
    fn wake(&mut self, key: RippleKey, masks: &dyn MaskSource) -> Option<&mut RippleTile> {
        if !self.tiles.contains_key(&key) {
            let mask = masks.mask(key.0, key.1).filter(|m| !m.is_empty())?;
            self.tiles.insert(key, RippleTile::new(mask));
        }
        self.tiles.get_mut(&key)
    }

    /// Displacement of a body's surface at (x, z); zero where no tile is awake.
    pub fn height_at(&self, body: WaterBodyId, x: f32, z: f32) -> f32 {
        let coord = Column::containing(x, z).chunk();
        let Some(tile) = self.tiles.get(&(coord, body)) else {
            return 0.0;
        };
        let (x0, z0) = coord.column(0).min_corner();
        let fx = ((x - x0) / RIPPLE_CELL - 0.5).clamp(0.0, (TILE_CELLS - 1) as f32);
        let fz = ((z - z0) / RIPPLE_CELL - 0.5).clamp(0.0, (TILE_CELLS - 1) as f32);
        let (i0, k0) = (fx.floor() as usize, fz.floor() as usize);
        let (i1, k1) = ((i0 + 1).min(TILE_CELLS - 1), (k0 + 1).min(TILE_CELLS - 1));
        let (tx, tz) = (fx - i0 as f32, fz - k0 as f32);
        let h = |i: usize, k: usize| tile.height[k * TILE_CELLS + i];
        let a = h(i0, k0) * (1.0 - tx) + h(i1, k0) * tx;
        let b = h(i0, k1) * (1.0 - tx) + h(i1, k1) * tx;
        a * (1.0 - tz) + b * tz
    }

    /// Advance every awake tile by `dt`, wake neighbours that energy crosses
    /// into, put quiet tiles to sleep and hold the budget.
    ///
    /// Dry cells hold still water at zero. Each tile is copied into a padded
    /// buffer whose border is the facing edge of an awake neighbour, or still
    /// water where there is none, so the stencil runs without a branch.
    pub fn step(&mut self, dt: f32, masks: &dyn MaskSource) {
        // Disturbances since the last step may have woken more than the
        // budget; only the ones that stay are worth stepping.
        self.hold_budget();
        if self.tiles.is_empty() {
            return;
        }
        let c2_dx2 = self.config.wave_speed.powi(2) / (RIPPLE_CELL * RIPPLE_CELL);
        let keys: Vec<RippleKey> = self.tiles.keys().copied().collect();
        const P: usize = TILE_CELLS + 2;

        // Accelerations first, reading every tile's heights, so neighbouring
        // tiles see each other's edges from the same instant.
        let mut accelerations: Vec<Vec<f32>> = keys
            .iter()
            .map(|k| {
                std::mem::take(
                    &mut self
                        .tiles
                        .get_mut(k)
                        .expect("key from this map")
                        .acceleration,
                )
            })
            .collect();
        let mut wakes: Vec<RippleKey> = Vec::new();
        let mut padded = vec![0.0f32; P * P];
        let mut sponge_x = [0.0f32; TILE_CELLS];
        let mut sponge_z = [0.0f32; TILE_CELLS];
        for (key, acc) in keys.iter().zip(accelerations.iter_mut()) {
            let tile = &self.tiles[key];
            let neighbours: [Option<&RippleTile>; 4] = EDGES.map(|(dx, dz)| {
                self.tiles.get(&(
                    SpanChunkCoord {
                        x: key.0.x + dx,
                        z: key.0.z + dz,
                    },
                    key.1,
                ))
            });
            padded.fill(0.0);
            for k in 0..TILE_CELLS {
                let row = &tile.height[k * TILE_CELLS..(k + 1) * TILE_CELLS];
                padded[(k + 1) * P + 1..(k + 1) * P + 1 + TILE_CELLS].copy_from_slice(row);
            }
            for along in 0..TILE_CELLS {
                if let Some(n) = neighbours[0] {
                    padded[(along + 1) * P + P - 1] = n.height[along * TILE_CELLS];
                }
                if let Some(n) = neighbours[1] {
                    padded[(along + 1) * P] = n.height[along * TILE_CELLS + TILE_CELLS - 1];
                }
                if let Some(n) = neighbours[2] {
                    padded[(P - 1) * P + along + 1] = n.height[along];
                }
                if let Some(n) = neighbours[3] {
                    padded[along + 1] = n.height[(TILE_CELLS - 1) * TILE_CELLS + along];
                }
            }
            self.sponge_profile(&neighbours, &mut sponge_x, &mut sponge_z);

            for k in 0..TILE_CELLS {
                let above = &padded[k * P + 1..k * P + 1 + TILE_CELLS];
                let here = &padded[(k + 1) * P..(k + 2) * P];
                let below = &padded[(k + 2) * P + 1..(k + 2) * P + 1 + TILE_CELLS];
                let row = k * TILE_CELLS..(k + 1) * TILE_CELLS;
                let velocity = &tile.velocity[row.clone()];
                let wet = &tile.wet[row.clone()];
                let base = self.config.damping;
                let damp_z = sponge_z[k];
                for (((((out, w), (a, b)), v), wet), sx) in acc[row]
                    .iter_mut()
                    .zip(here.windows(3))
                    .zip(above.iter().zip(below))
                    .zip(velocity)
                    .zip(wet)
                    .zip(&sponge_x)
                {
                    let laplacian = w[0] + w[2] + a + b - 4.0 * w[1];
                    let damping = base + damp_z.max(*sx);
                    *out = (c2_dx2 * laplacian - damping * v) * wet;
                }
            }
            // Energy in an edge strip facing a sleeping neighbour wakes it.
            for (edge, (dx, dz)) in EDGES.iter().enumerate() {
                if neighbours[edge].is_none() && edge_energy(tile, edge) > self.config.wake_energy {
                    wakes.push((
                        SpanChunkCoord {
                            x: key.0.x + dx,
                            z: key.0.z + dz,
                        },
                        key.1,
                    ));
                }
            }
        }

        for (key, acc) in keys.iter().zip(accelerations) {
            let tile = self.tiles.get_mut(key).expect("key from this map");
            let mut energy = 0.0;
            for cell in 0..CELLS_PER_TILE {
                let v = tile.velocity[cell] + acc[cell] * dt;
                let h = tile.height[cell] + v * dt;
                tile.velocity[cell] = v;
                tile.height[cell] = h;
                energy += h * h + (v * dt) * (v * dt);
            }
            tile.energy = energy;
            tile.acceleration = acc;
            if energy < self.config.sleep_energy {
                tile.quiet += dt;
            } else {
                tile.quiet = 0.0;
            }
        }

        let sleep_after = self.config.sleep_after;
        self.tiles.retain(|_, t| t.quiet < sleep_after);
        for key in wakes {
            if self.tiles.len() >= self.config.max_active {
                break;
            }
            self.wake(key, masks);
        }
        self.hold_budget();
    }

    /// Extra damping along each axis from the sponges on edges with no
    /// awake neighbour; a cell takes the larger of its two.
    fn sponge_profile(
        &self,
        neighbours: &[Option<&RippleTile>; 4],
        along_x: &mut [f32; TILE_CELLS],
        along_z: &mut [f32; TILE_CELLS],
    ) {
        let width = self.config.sponge;
        let ramp = |d: usize| {
            if d < width {
                let t = 1.0 - d as f32 / width as f32;
                SPONGE_DAMPING * t * t
            } else {
                0.0
            }
        };
        for c in 0..TILE_CELLS {
            let (to_high, to_low) = (TILE_CELLS - 1 - c, c);
            along_x[c] = 0.0f32
                .max(if neighbours[0].is_none() {
                    ramp(to_high)
                } else {
                    0.0
                })
                .max(if neighbours[1].is_none() {
                    ramp(to_low)
                } else {
                    0.0
                });
            along_z[c] = 0.0f32
                .max(if neighbours[2].is_none() {
                    ramp(to_high)
                } else {
                    0.0
                })
                .max(if neighbours[3].is_none() {
                    ramp(to_low)
                } else {
                    0.0
                });
        }
    }

    /// Evict the weakest, farthest tiles until the budget holds.
    fn hold_budget(&mut self) {
        while self.tiles.len() > self.config.max_active {
            let focus = self.focus;
            let weakest = self
                .tiles
                .iter()
                .map(|(key, tile)| (*key, priority(key, tile, focus)))
                .min_by(|a, b| a.1.total_cmp(&b.1).then(b.0.cmp(&a.0)))
                .map(|(key, _)| key);
            match weakest {
                Some(key) => {
                    self.tiles.remove(&key);
                }
                None => break,
            }
        }
    }
}

/// Damping at the outer edge of a sponge, 1/s.
const SPONGE_DAMPING: f32 = 30.0;

/// Width of the strip whose energy wakes a neighbour, in cells.
const WAKE_STRIP: usize = 2;

fn edge_energy(tile: &RippleTile, edge: usize) -> f32 {
    let mut energy = 0.0;
    for along in 0..TILE_CELLS {
        for depth in 0..WAKE_STRIP {
            let (i, k) = match edge {
                0 => (TILE_CELLS - 1 - depth, along),
                1 => (depth, along),
                2 => (along, TILE_CELLS - 1 - depth),
                _ => (along, depth),
            };
            energy += tile.height[k * TILE_CELLS + i].powi(2);
        }
    }
    energy
}

/// How much a tile deserves to stay awake: its energy, less with distance
/// from the camera.
fn priority(key: &RippleKey, tile: &RippleTile, focus: Option<Point3<f32>>) -> f32 {
    let Some(focus) = focus else {
        return tile.energy;
    };
    let (x0, z0) = key.0.column(0).min_corner();
    let half = CHUNK_COLUMNS as f32 * COLUMN_SIZE * 0.5;
    let d2 = (x0 + half - focus.x).powi(2) + (z0 + half - focus.z).powi(2);
    tile.energy / (1.0 + d2 / 64.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::ids::StoreId;

    /// Water everywhere, one metre deep.
    struct Everywhere;

    impl MaskSource for Everywhere {
        fn mask(&self, _tile: SpanChunkCoord, _body: WaterBodyId) -> Option<TileMask> {
            Some(TileMask {
                floors: [-1.0; COLUMNS_PER_CHUNK],
            })
        }
    }

    const BODY: WaterBodyId = StoreId(0);

    #[test]
    fn a_splash_spreads_and_dies_away() {
        let mut ripples = RippleTiles::new(RippleConfig::default());
        ripples.disturb(
            BODY,
            4.0,
            4.0,
            0.5,
            Disturbance::Velocity(-2.0),
            &Everywhere,
        );
        assert_eq!(ripples.active_count(), 1);
        for _ in 0..30 {
            ripples.step(1.0 / 60.0, &Everywhere);
        }
        // The ring has left the centre.
        assert!(ripples.height_at(BODY, 5.5, 4.0).abs() > 1e-4);
        for _ in 0..60 * 30 {
            ripples.step(1.0 / 60.0, &Everywhere);
        }
        assert_eq!(ripples.active_count(), 0, "a quiet tile sleeps");
    }

    #[test]
    fn energy_reaching_an_edge_wakes_the_neighbour() {
        let mut ripples = RippleTiles::new(RippleConfig::default());
        ripples.disturb(
            BODY,
            7.5,
            4.0,
            0.5,
            Disturbance::Displacement(0.3),
            &Everywhere,
        );
        for _ in 0..20 {
            ripples.step(1.0 / 60.0, &Everywhere);
        }
        assert!(ripples.active_count() >= 2);
    }

    #[test]
    fn the_budget_evicts_the_weakest() {
        let config = RippleConfig {
            max_active: 2,
            ..RippleConfig::default()
        };
        let mut ripples = RippleTiles::new(config);
        for (i, strength) in [0.1, 0.5, 0.3].into_iter().enumerate() {
            let x = 4.0 + 40.0 * i as f32;
            ripples.disturb(
                BODY,
                x,
                4.0,
                1.0,
                Disturbance::Displacement(strength),
                &Everywhere,
            );
        }
        ripples.step(1.0 / 60.0, &Everywhere);
        assert_eq!(ripples.active_count(), 2);
        assert_eq!(ripples.height_at(BODY, 4.0, 4.0), 0.0, "the weakest went");
    }
}
