//! Ripple tiles: the short waves bodies make, simulated only near them.
//!
//! ```text
//!   8 m tile (one span chunk) × 32 × 32 cells of 0.25 m, keyed by (tile, body)
//!   woken by a disturbance, or by energy crossing in from an active neighbour
//!   asleep after 2 s quiet; at most 32 awake, a tile woken past that kept only
//!   if it outranks the weakest and farthest by a margin, which it displaces
//!   each step: waves (c = 1.25 m/s) ─▶ carried along by the current under them
//! ```
//!
//! Ripples ride the water they are on: on a river the current carries them
//! downstream, so one faster than the waves sweeps a splash's ring away
//! instead of letting it spread upstream. Where the surface breaks, down a
//! fall or a step in rapids, they die.
//!
//! Keyed by body, so an island pool and the sea beneath it ripple apart, and
//! each tile is masked to its body's columns. A merge or split makes a new
//! body, so its tiles restart calm. The edges of a tile with no awake
//! neighbour absorb what reaches them through a sponge, rather than
//! reflecting it back; energy entering the sponge wakes the neighbour first.
//!
//! The step is explicit, so it never takes more than [`RippleTiles::stable_dt`]:
//! a long frame runs the ripples slow rather than past the stencil's limit.
//! Damping is implicit, stable at any rate.

use std::collections::BTreeMap;

use nalgebra::{Point3, Vector2};

use crate::water::coupling::Disturbance;
use crate::water::geometry::{
    Column, SpanChunkCoord, CHUNK_COLUMNS, COLUMNS_PER_CHUNK, COLUMN_SIZE,
};
use crate::water::ids::WaterBodyId;

/// Cells along each side of a tile.
pub const TILE_CELLS: usize = 32;

/// Cells in a tile.
pub const CELLS_PER_TILE: usize = TILE_CELLS * TILE_CELLS;

/// Cells of apron around a tile as drawn: as far past its edge as the
/// shader's slope reaches, a central difference of one cell either side of a
/// point interpolated between cell centres.
pub const APRON_CELLS: usize = 2;

/// Cells along each side of a tile as drawn: the tile and its apron.
pub const PADDED_CELLS: usize = TILE_CELLS + 2 * APRON_CELLS;

/// Cells in a tile as drawn.
pub const PADDED_CELLS_PER_TILE: usize = PADDED_CELLS * PADDED_CELLS;

/// Column corners along each side of a tile.
pub const TILE_CORNERS: usize = CHUNK_COLUMNS as usize + 1;

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
    /// Per column corner, row-major over `TILE_CORNERS`²: the
    /// [`corner_floor`](super::corner_floor) of the columns touching it, this tile's and its
    /// neighbours'.
    pub corners: [f32; TILE_CORNERS * TILE_CORNERS],
    /// Per column: the surface its water stands at, the one a dry column
    /// banking the water takes from the water beside it; NaN where the body
    /// holds no water there.
    pub surfaces: [f32; COLUMNS_PER_CHUNK],
    /// Per column: the water's current, m/s, x and z.
    pub flows: [Vector2<f32>; COLUMNS_PER_CHUNK],
    /// Per column: whether the surface breaks there, stepping down a fall
    /// or through rapids, where ripples die.
    pub breaks: [bool; COLUMNS_PER_CHUNK],
    /// The mean surface of the tile's water when the mask was taken, m.
    pub level: f32,
}

impl TileMask {
    /// Still water at `level` over the columns whose floor is not NaN.
    pub fn still(
        floors: [f32; COLUMNS_PER_CHUNK],
        corners: [f32; TILE_CORNERS * TILE_CORNERS],
        level: f32,
    ) -> Self {
        Self {
            floors,
            corners,
            surfaces: floors.map(|f| if f.is_nan() { f32::NAN } else { level }),
            flows: [Vector2::zeros(); COLUMNS_PER_CHUNK],
            breaks: [false; COLUMNS_PER_CHUNK],
            level,
        }
    }

    /// Whether any of the tile's water runs.
    fn flowing(&self) -> bool {
        self.flows.iter().any(|f| *f != Vector2::zeros())
    }

    pub fn is_empty(&self) -> bool {
        self.floors.iter().all(|f| f.is_nan())
    }

    fn wet_cell(&self, cell: usize) -> bool {
        !self.floor_under(cell).is_nan()
    }

    fn floor_under(&self, cell: usize) -> f32 {
        self.floors[column_of(cell)]
    }

    /// The current at a cell's centre, bilinear between column centres and
    /// held at the tile's edge.
    fn flow_at(&self, cell: usize) -> Vector2<f32> {
        let n = CHUNK_COLUMNS as usize;
        let at = |c: usize| (c as f32 + 0.5) / CELLS_PER_COLUMN as f32 - 0.5;
        let (fx, fz) = (
            at(cell % TILE_CELLS).clamp(0.0, (n - 1) as f32),
            at(cell / TILE_CELLS).clamp(0.0, (n - 1) as f32),
        );
        let (i0, k0) = (fx.floor() as usize, fz.floor() as usize);
        let (i1, k1) = ((i0 + 1).min(n - 1), (k0 + 1).min(n - 1));
        let (tx, tz) = (fx - i0 as f32, fz - k0 as f32);
        let f = |i: usize, k: usize| self.flows[k * n + i];
        let a = f(i0, k0) * (1.0 - tx) + f(i1, k0) * tx;
        let b = f(i0, k1) * (1.0 - tx) + f(i1, k1) * tx;
        a * (1.0 - tz) + b * tz
    }
}

/// The column holding a cell, row-major over the tile's columns.
fn column_of(cell: usize) -> usize {
    let (ci, ck) = (cell % TILE_CELLS, cell / TILE_CELLS);
    (ck / CELLS_PER_COLUMN) * CHUNK_COLUMNS as usize + ci / CELLS_PER_COLUMN
}

/// Where tile masks come from: whatever knows each body's columns.
pub trait MaskSource {
    /// The body's mask over a tile, or `None` if it has no water there.
    fn mask(&self, tile: SpanChunkCoord, body: WaterBodyId) -> Option<TileMask>;
}

/// How ripples move and settle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RippleConfig {
    /// Wave speed, m/s. The wave equation runs every wavelength at one
    /// speed, where real ripples disperse: c = √(gλ/2π + 2πσ/ρλ). This is
    /// water's at λ = 1 m, four cells, the shortest wave the grid draws
    /// cleanly: 1.25 m/s. Shorter ones run somewhat fast, longer ones slow.
    pub wave_speed: f32,
    /// Velocity damping, 1/s.
    pub damping: f32,
    /// A tile quieter than `sleep_energy` for this long sleeps, s.
    pub sleep_after: f32,
    /// Σh² over the tile, m²: an RMS height of 0.16 mm.
    pub sleep_energy: f32,
    /// Σh² in a tile's edge strip that wakes the neighbour it faces, m²: an
    /// RMS height of 2.8 mm.
    pub wake_energy: f32,
    /// Most tiles awake at once.
    pub max_active: usize,
    /// Cells of sponge along an edge with nothing awake beyond it.
    pub sponge: usize,
    /// The highest a ripple stands above or below the surface, m.
    pub max_height: f32,
    /// ... and at most this share of the water's depth under it.
    pub height_per_depth: f32,
}

impl Default for RippleConfig {
    fn default() -> Self {
        Self {
            wave_speed: 1.25,
            damping: 0.5,
            sleep_after: 2.0,
            sleep_energy: 2.5e-5,
            wake_energy: 5e-4,
            max_active: 32,
            sponge: 4,
            max_height: 0.4,
            height_per_depth: 0.5,
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
    /// The most a cell's height may stray from the surface, m.
    cap: Vec<f32>,
    /// The current under each cell, m/s, x and z; empty where the tile's
    /// water stands still.
    flow: Vec<Vector2<f32>>,
    /// Damping where the surface breaks, 1/s, per cell.
    breaking: Vec<f32>,
    /// Scratch for the step's new velocities.
    next_velocity: Vec<f32>,
    /// Seconds spent below the sleep energy.
    quiet: f32,
    /// Energy after the last step.
    energy: f32,
}

impl RippleTile {
    fn new(mask: TileMask, config: &RippleConfig) -> Self {
        let mut tile = Self {
            mask: mask.clone(),
            height: vec![0.0; CELLS_PER_TILE],
            velocity: vec![0.0; CELLS_PER_TILE],
            wet: vec![0.0; CELLS_PER_TILE],
            cap: vec![0.0; CELLS_PER_TILE],
            flow: Vec::new(),
            breaking: vec![0.0; CELLS_PER_TILE],
            next_velocity: vec![0.0; CELLS_PER_TILE],
            quiet: 0.0,
            energy: 0.0,
        };
        tile.set_mask(mask, config);
        tile
    }

    /// Take a new mask, stilling any cell that has gone dry.
    fn set_mask(&mut self, mask: TileMask, config: &RippleConfig) {
        self.flow.clear();
        if mask.flowing() {
            self.flow
                .extend((0..CELLS_PER_TILE).map(|cell| mask.flow_at(cell)));
        }
        for cell in 0..CELLS_PER_TILE {
            let column = column_of(cell);
            let floor = mask.floors[column];
            let wet = !floor.is_nan();
            self.wet[cell] = if wet { 1.0 } else { 0.0 };
            self.cap[cell] = if wet {
                (config.height_per_depth * (mask.surfaces[column] - floor))
                    .clamp(0.0, config.max_height)
            } else {
                0.0
            };
            self.breaking[cell] = if mask.breaks[column] {
                BREAK_DAMPING
            } else {
                0.0
            };
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
                    tile.set_mask(mask, &self.config);
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
                let key = (coord, body);
                let woken = !self.tiles.contains_key(&key);
                let Some(tile) = self.wake(key, masks) else {
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
                // Ranked against the budget by what it now holds.
                tile.energy = tile
                    .height
                    .iter()
                    .zip(&tile.velocity)
                    .map(|(h, v)| h * h + v * v / 3600.0)
                    .sum();
                if woken {
                    self.admit(key);
                }
            }
        }
    }

    /// The tile for a key, woken with a fresh mask if it was asleep. `None`
    /// if the body has no water there.
    fn wake(&mut self, key: RippleKey, masks: &dyn MaskSource) -> Option<&mut RippleTile> {
        if !self.tiles.contains_key(&key) {
            let mask = masks.mask(key.0, key.1).filter(|m| !m.is_empty())?;
            self.tiles.insert(key, RippleTile::new(mask, &self.config));
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

    /// The longest step the explicit stencil takes stably, s: half the
    /// time a wave takes to cross a cell, inside the 2D limit of 1/√2.
    pub fn stable_dt(&self) -> f32 {
        0.5 * RIPPLE_CELL / self.config.wave_speed
    }

    /// Advance every awake tile by `dt`, at most [`Self::stable_dt`], wake
    /// neighbours that energy crosses into, put quiet tiles to sleep and
    /// hold the budget.
    ///
    /// Dry cells hold still water at zero. Each tile is copied into a padded
    /// buffer whose border is the facing edge of an awake neighbour, or still
    /// water where there is none, so the stencil runs without a branch.
    pub fn step(&mut self, dt: f32, masks: &dyn MaskSource) {
        if self.tiles.is_empty() {
            return;
        }
        let dt = dt.min(self.stable_dt());
        let c2_dx2 = self.config.wave_speed.powi(2) / (RIPPLE_CELL * RIPPLE_CELL);
        let keys: Vec<RippleKey> = self.tiles.keys().copied().collect();
        const P: usize = TILE_CELLS + 2;

        // New velocities first, reading every tile's heights, so neighbouring
        // tiles see each other's edges from the same instant.
        let mut next_velocities: Vec<Vec<f32>> = keys
            .iter()
            .map(|k| {
                std::mem::take(
                    &mut self
                        .tiles
                        .get_mut(k)
                        .expect("key from this map")
                        .next_velocity,
                )
            })
            .collect();
        let mut sponges: Vec<([f32; TILE_CELLS], [f32; TILE_CELLS])> =
            Vec::with_capacity(keys.len());
        let mut wakes: Vec<RippleKey> = Vec::new();
        let mut padded = vec![0.0f32; P * P];
        for (key, next) in keys.iter().zip(next_velocities.iter_mut()) {
            let tile = &self.tiles[key];
            let neighbours = self.neighbours(key);
            pad(
                &tile.height,
                neighbours.map(|n| n.map(|t| &t.height[..])),
                &mut padded,
            );
            let mut sponge_x = [0.0f32; TILE_CELLS];
            let mut sponge_z = [0.0f32; TILE_CELLS];
            self.sponge_profile(&neighbours, &mut sponge_x, &mut sponge_z);

            for k in 0..TILE_CELLS {
                let above = &padded[k * P + 1..k * P + 1 + TILE_CELLS];
                let here = &padded[(k + 1) * P..(k + 2) * P];
                let below = &padded[(k + 2) * P + 1..(k + 2) * P + 1 + TILE_CELLS];
                let row = k * TILE_CELLS..(k + 1) * TILE_CELLS;
                let velocity = &tile.velocity[row.clone()];
                let wet = &tile.wet[row.clone()];
                let breaking = &tile.breaking[row.clone()];
                let base = self.config.damping;
                let damp_z = sponge_z[k];
                for ((((((out, w), (a, b)), v), wet), sx), br) in next[row]
                    .iter_mut()
                    .zip(here.windows(3))
                    .zip(above.iter().zip(below))
                    .zip(velocity)
                    .zip(wet)
                    .zip(&sponge_x)
                    .zip(breaking)
                {
                    let laplacian = w[0] + w[2] + a + b - 4.0 * w[1];
                    let damping = base + damp_z.max(*sx) + br;
                    *out = (v + c2_dx2 * laplacian * dt) / (1.0 + damping * dt) * wet;
                }
            }
            // Energy entering the sponge on an edge facing a sleeping
            // neighbour wakes it, before the sponge can absorb it.
            for (edge, (dx, dz)) in EDGES.iter().enumerate() {
                if neighbours[edge].is_none()
                    && edge_energy(tile, edge, self.config.sponge) > self.config.wake_energy
                {
                    wakes.push((
                        SpanChunkCoord {
                            x: key.0.x + dx,
                            z: key.0.z + dz,
                        },
                        key.1,
                    ));
                }
            }
            sponges.push((sponge_x, sponge_z));
        }

        for ((key, next), (sponge_x, sponge_z)) in keys.iter().zip(next_velocities).zip(&sponges) {
            let tile = self.tiles.get_mut(key).expect("key from this map");
            let mut energy = 0.0;
            for k in 0..TILE_CELLS {
                for i in 0..TILE_CELLS {
                    let cell = k * TILE_CELLS + i;
                    let v = next[cell];
                    // The sponge settles heights as well as motion, so the
                    // edge it guards comes to rest at the still surface.
                    let settle = 1.0 + (sponge_z[k].max(sponge_x[i]) + tile.breaking[cell]) * dt;
                    let cap = tile.cap[cell];
                    let h = ((tile.height[cell] + v * dt) / settle).clamp(-cap, cap);
                    tile.velocity[cell] = v;
                    tile.height[cell] = h;
                    energy += h * h + (v * dt) * (v * dt);
                }
            }
            tile.energy = energy;
            tile.next_velocity = next;
            if energy < self.config.sleep_energy {
                tile.quiet += dt;
            } else {
                tile.quiet = 0.0;
            }
        }

        self.carry(dt, &keys);

        let sleep_after = self.config.sleep_after;
        self.tiles.retain(|_, t| t.quiet < sleep_after);
        for key in wakes {
            if self.tiles.len() >= self.config.max_active {
                break;
            }
            self.wake(key, masks);
        }
    }

    /// Carry each flowing tile's ripples along the current under them, `dt`
    /// seconds: each cell takes the heights and motion found upstream of it,
    /// at most a cell away. Plain semi-Lagrangian steps blur a ripple a
    /// little every frame, half a metre a second at a brisk current, which
    /// is a ripple's own size; MacCormack's correction undoes most of that,
    /// held within the values it sampled so it never overshoots. It costs
    /// some speed on the shortest ripples, which ride at 70–85 % of the
    /// current and die sooner than on still water. Every tile reads its
    /// neighbours' cells as they were before any moved.
    fn carry(&mut self, dt: f32, keys: &[RippleKey]) {
        let mut carried: Vec<(RippleKey, Vec<f32>, Vec<f32>)> = Vec::new();
        let mut scratch = Carry::default();
        for key in keys {
            let tile = &self.tiles[key];
            if tile.flow.is_empty() {
                continue;
            }
            let neighbours = self.neighbours(key);
            let (height, velocity) = scratch.carry(
                tile,
                neighbours.map(|n| n.map(|t| (&t.height[..], &t.velocity[..]))),
                dt / RIPPLE_CELL,
            );
            carried.push((*key, height, velocity));
        }
        for (key, height, velocity) in carried {
            let tile = self.tiles.get_mut(&key).expect("key from this map");
            tile.height = height;
            tile.velocity = velocity;
        }
    }

    /// The awake tiles beside a tile, in [`EDGES`] order.
    fn neighbours(&self, key: &RippleKey) -> [Option<&RippleTile>; 4] {
        EDGES.map(|(dx, dz)| {
            self.tiles.get(&(
                SpanChunkCoord {
                    x: key.0.x + dx,
                    z: key.0.z + dz,
                },
                key.1,
            ))
        })
    }

    /// Edges of a tile with no awake neighbour, one bit each in [`EDGES`]
    /// order: drawn beside the coarse surface, which stands still.
    pub fn sealed_edges(&self, key: &RippleKey) -> u32 {
        self.neighbours(key)
            .iter()
            .enumerate()
            .filter(|(_, n)| n.is_none())
            .fold(0, |bits, (edge, _)| bits | 1 << edge)
    }

    /// A tile's heights as drawn, row-major over [`PADDED_CELLS`]²: the
    /// tile and its apron. The apron holds an awake neighbour's cells, so
    /// two tiles interpolate their shared edge, and take its slope, from the
    /// same values. Over a sleeping neighbour it holds the tile's edge
    /// mirrored with its sign flipped, so the edge interpolates to the still
    /// surface the coarse tile beside it draws.
    pub fn write_padded(&self, key: &RippleKey, out: &mut [f32]) {
        let hood = Neighbourhood::around(&self.tiles, key);
        let n = TILE_CELLS as i32;
        let a = APRON_CELLS as i32;
        let tile = &hood.tiles[1][1].expect("the centre is awake").height;
        for k in -a..n + a {
            let row = (k + a) as usize * PADDED_CELLS;
            for i in -a..n + a {
                let inside = (0..n).contains(&i) && (0..n).contains(&k);
                out[row + (i + a) as usize] = if inside {
                    tile[k as usize * TILE_CELLS + i as usize]
                } else {
                    hood.apron(i, k)
                };
            }
        }
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

    /// Keep a tile just woken only if the budget has room for it, or it
    /// outranks the weakest awake tile by [`ADMISSION_MARGIN`] and displaces
    /// it. So the budget holds whenever the tiles are read, and two tiles of
    /// near-equal claim do not take turns being drawn.
    fn admit(&mut self, key: RippleKey) {
        if self.tiles.len() <= self.config.max_active {
            return;
        }
        let focus = self.focus;
        let claim = priority(&key, &self.tiles[&key], focus);
        let weakest = self
            .tiles
            .iter()
            .filter(|(k, _)| **k != key)
            .map(|(k, tile)| (*k, priority(k, tile, focus)))
            .min_by(|a, b| a.1.total_cmp(&b.1).then(b.0.cmp(&a.0)));
        let evicted = match weakest {
            Some((weakest, standing)) if claim > ADMISSION_MARGIN * standing => weakest,
            _ => key,
        };
        self.tiles.remove(&evicted);
    }
}

/// A tile's values in a buffer one cell wider all round, `(TILE_CELLS + 2)`
/// square: the border holds the facing edge of each awake neighbour, given
/// in [`EDGES`] order, and still water where there is none.
fn pad(values: &[f32], neighbours: [Option<&[f32]>; 4], out: &mut [f32]) {
    const P: usize = TILE_CELLS + 2;
    out.fill(0.0);
    for k in 0..TILE_CELLS {
        out[(k + 1) * P + 1..(k + 1) * P + 1 + TILE_CELLS]
            .copy_from_slice(&values[k * TILE_CELLS..(k + 1) * TILE_CELLS]);
    }
    for along in 0..TILE_CELLS {
        if let Some(n) = neighbours[0] {
            out[(along + 1) * P + P - 1] = n[along * TILE_CELLS];
        }
        if let Some(n) = neighbours[1] {
            out[(along + 1) * P] = n[along * TILE_CELLS + TILE_CELLS - 1];
        }
        if let Some(n) = neighbours[2] {
            out[(P - 1) * P + along + 1] = n[along];
        }
        if let Some(n) = neighbours[3] {
            out[along + 1] = n[(TILE_CELLS - 1) * TILE_CELLS + along];
        }
    }
}

/// Side of a tile's buffer padded with one cell all round.
const PADDED: usize = TILE_CELLS + 2;

/// Scratch for carrying a tile's heights and motion along its current.
#[derive(Default)]
struct Carry {
    heights: Vec<f32>,
    velocities: Vec<f32>,
    /// The forward step's heights and motion, and the least and most of
    /// the values each was taken between.
    forward: Vec<[f32; 2]>,
    bounds: Vec<[f32; 4]>,
}

/// Where a point between a padded buffer's cells lies: the cell at its
/// low corner and its weight toward each of the four around it.
#[derive(Clone, Copy)]
struct Sample {
    at: usize,
    weights: [f32; 4],
}

impl Sample {
    /// Cell `cell` of the tile, moved `shift` cells (each at most one).
    fn of(cell: usize, shift: Vector2<f32>) -> Self {
        let last = (PADDED - 2) as f32;
        let x = ((cell % TILE_CELLS + 1) as f32 + shift.x).clamp(0.0, last);
        let z = ((cell / TILE_CELLS + 1) as f32 + shift.y).clamp(0.0, last);
        let (x0, z0) = (x.floor(), z.floor());
        let (tx, tz) = (x - x0, z - z0);
        Self {
            at: z0 as usize * PADDED + x0 as usize,
            weights: [
                (1.0 - tx) * (1.0 - tz),
                tx * (1.0 - tz),
                (1.0 - tx) * tz,
                tx * tz,
            ],
        }
    }

    fn corners(self, values: &[f32]) -> [f32; 4] {
        let a = self.at;
        [
            values[a],
            values[a + 1],
            values[a + PADDED],
            values[a + PADDED + 1],
        ]
    }

    fn read(self, values: &[f32]) -> f32 {
        let c = self.corners(values);
        let w = self.weights;
        c[0] * w[0] + c[1] * w[1] + c[2] * w[2] + c[3] * w[3]
    }

    /// The value, and the least and most of the four it lies between.
    fn read_bounded(self, values: &[f32]) -> (f32, f32, f32) {
        let c = self.corners(values);
        let w = self.weights;
        (
            c[0] * w[0] + c[1] * w[1] + c[2] * w[2] + c[3] * w[3],
            c[0].min(c[1]).min(c[2].min(c[3])),
            c[0].max(c[1]).max(c[2].max(c[3])),
        )
    }
}

impl Carry {
    /// A tile's heights and motion, each wet cell's moved by its current
    /// over `cells_per_speed` (MacCormack): carried forward, carried back
    /// again, and corrected by half of what the round trip lost, within the
    /// values the forward step sampled. Heights stay within the tile's cap.
    fn carry(
        &mut self,
        tile: &RippleTile,
        neighbours: [Option<(&[f32], &[f32])>; 4],
        cells_per_speed: f32,
    ) -> (Vec<f32>, Vec<f32>) {
        let shift = |cell: usize| (tile.flow[cell] * cells_per_speed).map(|d| d.clamp(-1.0, 1.0));
        self.heights.resize(PADDED * PADDED, 0.0);
        self.velocities.resize(PADDED * PADDED, 0.0);
        pad(
            &tile.height,
            neighbours.map(|n| n.map(|n| n.0)),
            &mut self.heights,
        );
        pad(
            &tile.velocity,
            neighbours.map(|n| n.map(|n| n.1)),
            &mut self.velocities,
        );
        self.forward.clear();
        self.bounds.clear();
        for cell in 0..CELLS_PER_TILE {
            if tile.wet[cell] == 0.0 {
                let (h, v) = (tile.height[cell], tile.velocity[cell]);
                self.forward.push([h, v]);
                self.bounds.push([h, h, v, v]);
                continue;
            }
            let from = Sample::of(cell, -shift(cell));
            let (h, h_low, h_high) = from.read_bounded(&self.heights);
            let (v, v_low, v_high) = from.read_bounded(&self.velocities);
            self.forward.push([h, v]);
            self.bounds.push([h_low, h_high, v_low, v_high]);
        }
        let forward_h: Vec<f32> = self.forward.iter().map(|f| f[0]).collect();
        let forward_v: Vec<f32> = self.forward.iter().map(|f| f[1]).collect();
        pad(
            &forward_h,
            neighbours.map(|n| n.map(|n| n.0)),
            &mut self.heights,
        );
        pad(
            &forward_v,
            neighbours.map(|n| n.map(|n| n.1)),
            &mut self.velocities,
        );
        let mut height = forward_h;
        let mut velocity = forward_v;
        for cell in 0..CELLS_PER_TILE {
            if tile.wet[cell] == 0.0 {
                continue;
            }
            let back = Sample::of(cell, shift(cell));
            let [h_low, h_high, v_low, v_high] = self.bounds[cell];
            let cap = tile.cap[cell];
            let h = height[cell] + 0.5 * (tile.height[cell] - back.read(&self.heights));
            height[cell] = h.clamp(h_low, h_high).clamp(-cap, cap);
            let v = velocity[cell] + 0.5 * (tile.velocity[cell] - back.read(&self.velocities));
            velocity[cell] = v.clamp(v_low, v_high);
        }
        (height, velocity)
    }
}

/// Damping where the surface breaks, 1/s: a sponge's outer edge.
const BREAK_DAMPING: f32 = SPONGE_DAMPING;

/// How many times the weakest awake tile's priority a tile woken past the
/// budget needs to displace it.
const ADMISSION_MARGIN: f32 = 1.5;

/// Damping at the outer edge of a sponge, 1/s.
const SPONGE_DAMPING: f32 = 30.0;

/// Width of the strip whose energy wakes a neighbour, in cells.
const WAKE_STRIP: usize = 2;

/// Energy in the strip `inset` cells in from an edge.
fn edge_energy(tile: &RippleTile, edge: usize, inset: usize) -> f32 {
    let mut energy = 0.0;
    for along in 0..TILE_CELLS {
        for depth in inset..inset + WAKE_STRIP {
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

/// A tile and the eight around it, for reading cells across its edges.
struct Neighbourhood<'a> {
    /// `[dz + 1][dx + 1]`; `None` where the tile sleeps.
    tiles: [[Option<&'a RippleTile>; 3]; 3],
}

impl<'a> Neighbourhood<'a> {
    fn around(tiles: &'a BTreeMap<RippleKey, RippleTile>, key: &RippleKey) -> Self {
        let at = |dx: i32, dz: i32| {
            tiles.get(&(
                SpanChunkCoord {
                    x: key.0.x + dx,
                    z: key.0.z + dz,
                },
                key.1,
            ))
        };
        Self {
            tiles: [-1, 0, 1].map(|dz| [-1, 0, 1].map(|dx| at(dx, dz))),
        }
    }

    /// A cell's height, in cells from the centre tile's origin; `None` in
    /// a sleeping tile.
    fn cell(&self, i: i32, k: i32) -> Option<f32> {
        let n = TILE_CELLS as i32;
        let tile = self.tiles[(k.div_euclid(n) + 1) as usize][(i.div_euclid(n) + 1) as usize]?;
        Some(tile.height[k.rem_euclid(n) as usize * TILE_CELLS + i.rem_euclid(n) as usize])
    }

    /// The height drawn at a cell in the centre tile's apron. A cell in an
    /// awake tile is its own. One in a sleeping tile is the negated mean of
    /// its mirror images across the nearby tile edges that lie in awake
    /// tiles, or, with none, its image across both axes at a lone corner.
    /// Only whether tiles are awake and their heights decide it, so every
    /// tile that draws the cell draws the same value.
    fn apron(&self, i: i32, k: i32) -> f32 {
        if let Some(h) = self.cell(i, k) {
            return h;
        }
        let (mi, mk) = (mirror(i), mirror(k));
        let images = [mi.map(|i| (i, k)), mk.map(|k| (i, k))];
        let (sum, count) = images
            .into_iter()
            .flatten()
            .filter_map(|(i, k)| self.cell(i, k))
            .fold((0.0, 0), |(s, c), h| (s + h, c + 1));
        if count > 0 {
            return -sum / count as f32;
        }
        mi.zip(mk).and_then(|(i, k)| self.cell(i, k)).unwrap_or(0.0)
    }
}

/// A cell index's mirror image across the tile edge within [`APRON_CELLS`]
/// of it, in cells from the centre tile's origin; `None` if no edge is that
/// close.
fn mirror(index: i32) -> Option<i32> {
    let (n, a) = (TILE_CELLS as i32, APRON_CELLS as i32);
    let (tile, local) = (index.div_euclid(n), index.rem_euclid(n));
    if local < a {
        Some(tile * n - 1 - local)
    } else if local >= n - a {
        Some((tile + 1) * n + (n - 1 - local))
    } else {
        None
    }
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
    use crate::water::ids::SurfaceId;

    /// Water everywhere, one metre deep.
    struct Everywhere;

    impl MaskSource for Everywhere {
        fn mask(&self, _tile: SpanChunkCoord, _body: WaterBodyId) -> Option<TileMask> {
            Some(TileMask::still(
                [-1.0; COLUMNS_PER_CHUNK],
                [-1.0; TILE_CORNERS * TILE_CORNERS],
                0.0,
            ))
        }
    }

    const BODY: WaterBodyId = SurfaceId(0);

    /// Water everywhere, one metre deep, running along +x at `0` m/s.
    struct Running(f32);

    impl MaskSource for Running {
        fn mask(&self, tile: SpanChunkCoord, body: WaterBodyId) -> Option<TileMask> {
            let mut mask = Everywhere.mask(tile, body)?;
            mask.flows = [Vector2::new(self.0, 0.0); COLUMNS_PER_CHUNK];
            Some(mask)
        }
    }

    /// Where a tile's ripples stand, weighted by their height squared, m
    /// from its corner.
    fn centre_of(ripples: &RippleTiles) -> Vector2<f32> {
        let (_, tile) = ripples.active().next().expect("a tile awake");
        let (mut sum, mut weight) = (Vector2::zeros(), 0.0);
        for (cell, &h) in tile.height.iter().enumerate() {
            let at = Vector2::new((cell % TILE_CELLS) as f32, (cell / TILE_CELLS) as f32)
                .add_scalar(0.5)
                * RIPPLE_CELL;
            sum += at * h * h;
            weight += h * h;
        }
        sum / weight
    }

    /// The tile's ripples' energy, Σh².
    fn energy_of(ripples: &RippleTiles) -> f32 {
        let (_, tile) = ripples.active().next().expect("a tile awake");
        tile.height.iter().map(|h| h * h).sum()
    }

    /// Carrying ripples must not smear them: a ring carried by a current
    /// keeps nearly the energy it keeps in still water.
    #[test]
    fn a_current_carries_ripples_without_smearing_them() {
        let energy = |speed: f32| {
            let masks = Running(speed);
            let mut ripples = RippleTiles::new(RippleConfig::default());
            ripples.disturb(BODY, 3.0, 4.0, 0.3, Disturbance::Velocity(-3.0), &masks);
            for _ in 0..60 {
                ripples.step(1.0 / 60.0, &masks);
            }
            energy_of(&ripples)
        };
        let (still, carried) = (energy(0.0), energy(1.5));
        assert!(carried > 0.8 * still, "still {still}, carried {carried}");
    }

    #[test]
    fn a_current_carries_a_splash_downstream() {
        let drift = |speed: f32| {
            let masks = Running(speed);
            let mut ripples = RippleTiles::new(RippleConfig::default());
            ripples.disturb(BODY, 3.0, 4.0, 0.5, Disturbance::Displacement(0.1), &masks);
            let before = centre_of(&ripples);
            for _ in 0..60 {
                ripples.step(1.0 / 60.0, &masks);
            }
            centre_of(&ripples) - before
        };
        let still = drift(0.0);
        assert!(still.norm() < 0.1, "still water held it at {still:?}");
        // The correction that keeps ripples from smearing slows the
        // shortest of them: they ride at most of the current's speed.
        let carried = drift(1.0);
        assert!(
            (0.5..1.1).contains(&carried.x) && carried.y.abs() < 0.1,
            "a 1 m/s current moved it {carried:?} in a second"
        );
    }

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

    /// A budget of two, and a disturbance of `strength` in each of a row
    /// of tiles 40 m apart, in order.
    fn woken_in_turn(strengths: &[f32], focus: Option<Point3<f32>>) -> RippleTiles {
        let config = RippleConfig {
            max_active: 2,
            ..RippleConfig::default()
        };
        let mut ripples = RippleTiles::new(config);
        ripples.set_focus(focus);
        for (i, &strength) in strengths.iter().enumerate() {
            ripples.disturb(
                BODY,
                4.0 + 40.0 * i as f32,
                4.0,
                1.0,
                Disturbance::Displacement(strength),
                &Everywhere,
            );
        }
        ripples
    }

    #[test]
    fn the_budget_holds_as_tiles_are_woken() {
        let ripples = woken_in_turn(&[0.1, 0.5, 0.3], None);
        assert_eq!(ripples.active_count(), 2, "held before any step");
        assert_eq!(ripples.height_at(BODY, 4.0, 4.0), 0.0, "the weakest went");
    }

    #[test]
    fn a_tile_woken_past_the_budget_must_clearly_outrank_the_weakest() {
        let ripples = woken_in_turn(&[0.3, 0.5, 0.32], None);
        assert_eq!(ripples.active_count(), 2);
        assert_ne!(
            ripples.height_at(BODY, 4.0, 4.0),
            0.0,
            "the incumbent stays"
        );
        assert_eq!(
            ripples.height_at(BODY, 84.0, 4.0),
            0.0,
            "the newcomer is turned away"
        );
    }

    #[test]
    fn the_budget_keeps_the_tiles_near_the_camera() {
        let ripples = woken_in_turn(&[0.5, 0.5, 0.3], Some(Point3::new(84.0, 6.0, 4.0)));
        assert_eq!(ripples.active_count(), 2);
        assert_ne!(ripples.height_at(BODY, 84.0, 4.0), 0.0, "the nearest stays");
        assert_eq!(ripples.height_at(BODY, 4.0, 4.0), 0.0, "the farthest went");
    }

    #[test]
    fn a_long_frame_stays_calm() {
        let mut ripples = RippleTiles::new(RippleConfig::default());
        ripples.disturb(
            BODY,
            4.0,
            4.0,
            0.5,
            Disturbance::Velocity(-2.0),
            &Everywhere,
        );
        // The frame clock's cap: far past the stencil's own limit.
        for _ in 0..30 {
            ripples.step(0.1, &Everywhere);
        }
        let peak = ripples
            .active()
            .flat_map(|(_, t)| t.height.iter())
            .fold(0.0f32, |m, h| m.max(h.abs()));
        assert!(peak < 0.05, "the splash grew to {peak} m");
    }

    #[test]
    fn a_ring_wakes_the_neighbour_before_the_sponge_eats_it() {
        let mut ripples = RippleTiles::new(RippleConfig::default());
        ripples.disturb(
            BODY,
            4.0,
            4.0,
            0.5,
            Disturbance::Velocity(-2.0),
            &Everywhere,
        );
        for _ in 0..120 {
            ripples.step(1.0 / 60.0, &Everywhere);
        }
        assert!(
            ripples.active_count() >= 5,
            "the ring stopped at the tile's edges"
        );
    }

    /// A cell as drawn, in cells from the tile's origin; the apron lies
    /// below 0 and from `TILE_CELLS`.
    fn drawn(padded: &[f32], i: i32, k: i32) -> f32 {
        let a = APRON_CELLS as i32;
        padded[(k + a) as usize * PADDED_CELLS + (i + a) as usize]
    }

    /// The height the shader interpolates on a tile's low-x edge, at row k.
    fn low_x_edge(padded: &[f32], k: usize) -> f32 {
        0.5 * (drawn(padded, -1, k as i32) + drawn(padded, 0, k as i32))
    }

    /// ... and on its high-x edge.
    fn high_x_edge(padded: &[f32], k: usize) -> f32 {
        let n = TILE_CELLS as i32;
        0.5 * (drawn(padded, n - 1, k as i32) + drawn(padded, n, k as i32))
    }

    /// `rippleHeightAt` in ripple.glsl: bilinear between cell centres, at a
    /// tile-local position, m.
    fn shader_height(padded: &[f32], x: f32, z: f32) -> f32 {
        let a = APRON_CELLS as i32;
        let n = TILE_CELLS as i32;
        let cell = |i: i32, k: i32| drawn(padded, i.clamp(-a, n + a - 1), k.clamp(-a, n + a - 1));
        let (fx, fz) = (x / RIPPLE_CELL - 0.5, z / RIPPLE_CELL - 0.5);
        let (cx, cz) = (fx.floor() as i32, fz.floor() as i32);
        let (tx, tz) = (fx - cx as f32, fz - cz as f32);
        let lerp = |p: f32, q: f32, t: f32| p + (q - p) * t;
        let low = lerp(cell(cx, cz), cell(cx + 1, cz), tx);
        let high = lerp(cell(cx, cz + 1), cell(cx + 1, cz + 1), tx);
        lerp(low, high, tz)
    }

    /// `rippleGradientAt` in ripple.glsl.
    fn shader_gradient(padded: &[f32], x: f32, z: f32) -> (f32, f32) {
        let e = RIPPLE_CELL;
        (
            (shader_height(padded, x + e, z) - shader_height(padded, x - e, z)) / (2.0 * e),
            (shader_height(padded, x, z + e) - shader_height(padded, x, z - e)) / (2.0 * e),
        )
    }

    #[test]
    fn awake_tiles_draw_their_shared_edge_alike() {
        let mut ripples = RippleTiles::new(RippleConfig::default());
        ripples.disturb(
            BODY,
            8.0,
            4.0,
            1.0,
            Disturbance::Displacement(0.2),
            &Everywhere,
        );
        ripples.step(1.0 / 60.0, &Everywhere);
        let (west, east) = (
            (SpanChunkCoord { x: 0, z: 0 }, BODY),
            (SpanChunkCoord { x: 1, z: 0 }, BODY),
        );
        let mut a = vec![0.0; PADDED_CELLS_PER_TILE];
        let mut b = vec![0.0; PADDED_CELLS_PER_TILE];
        ripples.write_padded(&west, &mut a);
        ripples.write_padded(&east, &mut b);
        let mut largest = 0.0f32;
        for k in 0..TILE_CELLS {
            assert_eq!(high_x_edge(&a, k), low_x_edge(&b, k));
            largest = largest.max(high_x_edge(&a, k).abs());
        }
        assert!(largest > 1e-3, "the disturbance never reached the edge");
    }

    #[test]
    fn awake_tiles_slope_alike_across_their_shared_edge() {
        let mut ripples = RippleTiles::new(RippleConfig::default());
        // Off the edge, so the ring crosses it on a slope.
        ripples.disturb(
            BODY,
            7.0,
            4.0,
            1.0,
            Disturbance::Displacement(0.2),
            &Everywhere,
        );
        for _ in 0..10 {
            ripples.step(1.0 / 60.0, &Everywhere);
        }
        let (west, east) = (
            (SpanChunkCoord { x: 0, z: 0 }, BODY),
            (SpanChunkCoord { x: 1, z: 0 }, BODY),
        );
        let mut a = vec![0.0; PADDED_CELLS_PER_TILE];
        let mut b = vec![0.0; PADDED_CELLS_PER_TILE];
        ripples.write_padded(&west, &mut a);
        ripples.write_padded(&east, &mut b);
        let extent = TILE_CELLS as f32 * RIPPLE_CELL;
        let mut steepest = 0.0f32;
        for step in 0..=32 {
            let z = step as f32 * extent / 32.0;
            let (ax, az) = shader_gradient(&a, extent, z);
            let (bx, bz) = shader_gradient(&b, 0.0, z);
            assert!(
                (ax - bx).abs() < 1e-6 && (az - bz).abs() < 1e-6,
                "at z = {z}: ({ax}, {az}) vs ({bx}, {bz})"
            );
            steepest = steepest.max(ax.abs().max(az.abs()));
        }
        assert!(steepest > 1e-3, "the disturbance never reached the edge");
    }

    #[test]
    fn an_edge_beside_a_sleeping_tile_draws_still_water() {
        let mut ripples = RippleTiles::new(RippleConfig::default());
        ripples.disturb(
            BODY,
            7.45,
            4.0,
            0.5,
            Disturbance::Displacement(0.2),
            &Everywhere,
        );
        // One cell at the low corner.
        ripples.disturb(
            BODY,
            0.13,
            0.13,
            0.0,
            Disturbance::Displacement(0.2),
            &Everywhere,
        );
        let key = (SpanChunkCoord { x: 0, z: 0 }, BODY);
        assert_eq!(ripples.active_count(), 1);
        assert_eq!(ripples.sealed_edges(&key), 0b1111);
        let mut padded = vec![0.0; PADDED_CELLS_PER_TILE];
        ripples.write_padded(&key, &mut padded);
        let n = TILE_CELLS as i32;
        assert!(drawn(&padded, n - 1, 16).abs() > 1e-3);
        assert!(drawn(&padded, 0, 0).abs() > 1e-3);
        for k in 0..TILE_CELLS {
            assert_eq!(high_x_edge(&padded, k), 0.0);
            // The second apron cell mirrors the second cell in.
            let k = k as i32;
            assert_eq!(drawn(&padded, n + 1, k), -drawn(&padded, n - 2, k));
        }
        // The corner, where two sealed edges meet.
        for (i, k) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            let square = drawn(&padded, -1 - i, -1 - k)
                + drawn(&padded, i, -1 - k)
                + drawn(&padded, -1 - i, k)
                + drawn(&padded, i, k);
            assert_eq!(square, 0.0, "the corner's images at ({i}, {k})");
        }
    }

    #[test]
    fn the_shader_reads_the_layout_written_here() {
        let glsl =
            std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/shader/ripple.glsl"))
                .expect("shader/ripple.glsl");
        for line in [
            format!("const int RIPPLE_CELLS = {TILE_CELLS};"),
            format!("const int RIPPLE_APRON = {APRON_CELLS};"),
            format!("const float RIPPLE_CELL = {RIPPLE_CELL};"),
            format!("const int RIPPLE_COLUMNS = {CHUNK_COLUMNS};"),
            format!("const float RIPPLE_COLUMN = {COLUMN_SIZE};"),
        ] {
            assert!(glsl.contains(&line), "ripple.glsl lacks `{line}`");
        }
    }
}
