//! `WaterField`: water as a shallow-water field over the span graph.
//!
//! ```text
//!   TerrainWorld ──▶ SpanRasteriser ──▶ SpanGraph ──▶ FieldCells (+ faces)
//!        │ edit           │ re-pair         │ remap        │
//!        └────────────────┘                 └── carry ─────┤
//!   Sources ── pour ──▶                                    ▼
//!   Sinks   ◀─ drain ──                         FieldSolver (awake chunks)
//!                                                          │
//!                              Falls ◀── found again ──────┘
//!                    (each falling face's arc and landing: what the next
//!                     step sends into the air, what comes down where, and
//!                     what is drawn and strikes)
//!                                │ landings
//!                                ▼
//!                              Foam (made, carried on the faces, fading)
//! ```
//!
//! The water is the only state: depths in cells, discharges on faces. An
//! edit re-pairs the columns it touched, carries their water into the new
//! spans, and lays faces again around them. Nothing else changes.

use std::time::{Duration, Instant};

use nalgebra::{Point3, Vector2, Vector3};

use crate::level::{MapEdge, WaterBody, WaterConfig};
use crate::terrain::TerrainWorld;
use crate::water::geometry::{
    Column, SpanChunkCoord, SpanGraph, SpanIndex, SpanRasteriser, SpanRef, COLUMN_SIZE,
};

use super::activity;
use super::breaking;
use super::cells::{FieldCells, CELL_AREA};
use super::conduits;
use super::currents;
use super::faces::{lay_faces, link_faces, SeaBoundary};
use super::falls::{Fall, Falls};
use super::fill::{fill_fed, fill_pool, fill_sea, pool_seed, SpillLevels};
use super::foam::Foam;
use super::fronts::{self, Front};
use super::pumps::{Jet, PumpId, PumpPlacement, Pumps};
use super::remap::carry;
use super::solver::{FieldSolver, FlowParams, StepStats};
use super::sources::{Sink, Source};

/// Depth below which a cell counts as dry, m.
pub const WET_DEPTH: f64 = 2.0e-3;

/// Depth below which water seeps away, m: thinner than the drawn shoreline.
/// Friction grows without bound as a film thins, so a film on a slope would
/// otherwise creep on for ever, never seen and never at rest.
const FILM_DEPTH: f64 = 5.0e-3;

/// How fast a film seeps away, m/s.
const SEEP_RATE: f64 = 5.0e-4;

/// Seep films away in the awake chunks. Returns the volume lost, m³.
fn seep(cells: &mut FieldCells, dt: f32) -> f64 {
    let step = SEEP_RATE * dt as f64;
    let mut lost = 0.0;
    for chunk in cells.chunks.iter_mut().filter(|c| c.awake) {
        for depth in &mut chunk.depth {
            if *depth > 0.0 && *depth < FILM_DEPTH {
                let taken = depth.min(step);
                *depth -= taken;
                lost += taken * CELL_AREA;
            }
        }
    }
    lost
}

/// Where the field's water has come from and gone to since it was built.
#[derive(Debug, Clone, Copy, Default)]
pub struct FieldLedger {
    /// Water the field was built with, m³.
    pub initial: f64,
    /// Poured in by sources, m³.
    pub sourced: f64,
    /// Let out to sinks, the void and the sea, net of what the sea let in, m³.
    pub sunk: f64,
    /// Seeped away from films too thin to see, m³.
    pub lost: f64,
}

impl FieldLedger {
    /// What the ledger says the field should hold, less what it holds.
    pub fn error(&self, volume: f64) -> f64 {
        self.initial + self.sourced - self.sunk - self.lost - volume
    }
}

/// What the last frame cost and did.
#[derive(Debug, Clone, Copy, Default)]
pub struct FieldFrame {
    pub stats: StepStats,
    /// Chunks awake after the frame.
    pub awake: usize,
    pub step_time: Duration,
    /// Time the last terrain edit took to carry, on the frame it came.
    pub edit_time: Option<Duration>,
}

pub struct WaterField {
    rasteriser: SpanRasteriser,
    graph: SpanGraph,
    cells: FieldCells,
    solver: FieldSolver,
    sources: Vec<Source>,
    sinks: Vec<Sink>,
    pumps: Pumps,
    sea: Option<SeaBoundary>,
    /// The water going over edges as it stood after the last step or edit.
    falls: Falls,
    /// What steps the foam riding the water.
    foam: Foam,
    ledger: FieldLedger,
    frame: FieldFrame,
    /// Carrying time of an edit since the last step, reported with it.
    edit_time: Option<Duration>,
}

impl WaterField {
    /// Lay the field over the terrain and fill it as authored.
    pub fn build(config: &WaterConfig, terrain: &TerrainWorld) -> Self {
        Self::with_params(config, terrain, FlowParams::default())
    }

    pub fn with_params(config: &WaterConfig, terrain: &TerrainWorld, params: FlowParams) -> Self {
        let (rasteriser, graph) = SpanRasteriser::build(terrain);
        let mut cells = FieldCells::build(&graph);
        let sea = config.ocean.as_ref().map(|ocean| {
            let open = |edge| ocean.open_edges.contains(&edge);
            SeaBoundary {
                level: ocean.level,
                open: [
                    open(MapEdge::East),
                    open(MapEdge::North),
                    open(MapEdge::West),
                    open(MapEdge::South),
                ],
            }
        });
        for slot in 0..cells.chunks.len() {
            lay_faces(&mut cells.chunks[slot], &graph, sea);
        }
        let all: Vec<usize> = (0..cells.chunks.len()).collect();
        link_faces(&mut cells, &all);
        if let Some(sea) = &sea {
            fill_sea(&graph, &mut cells, sea);
        }
        let spill = SpillLevels::find(&graph, &cells, sea.as_ref());
        let mut sources = Vec::new();
        let mut sinks = Vec::new();
        for body in &config.bodies {
            match body {
                WaterBody::Pool {
                    seed,
                    surface_level,
                } => {
                    if let Some(span) = pool_seed(&graph, seed.0, seed.1, *surface_level) {
                        fill_pool(&graph, &mut cells, &spill, span, *surface_level);
                    } else {
                        log::warn!(
                            "pool at ({:.1}, {:.1}) has no ground under {:.2} m to fill",
                            seed.0,
                            seed.1,
                            surface_level
                        );
                    }
                }
                WaterBody::Spring {
                    position,
                    direction,
                    discharge,
                } => sources.push(Source::new(
                    Point3::new(position.0, position.1, position.2),
                    Vector3::new(direction.0, direction.1, direction.2),
                    *discharge,
                    &graph,
                    &cells,
                )),
                WaterBody::SkySource {
                    position,
                    discharge,
                } => sources.push(Source::new(
                    Point3::new(position.0, position.1, position.2),
                    Vector3::zeros(),
                    *discharge,
                    &graph,
                    &cells,
                )),
                WaterBody::Sink { min, max } => sinks.push(Sink::new(
                    Point3::new(min.0, min.1, min.2),
                    Point3::new(max.0, max.1, max.2),
                    &graph,
                    &cells,
                )),
            }
        }
        for slot in 0..cells.chunks.len() {
            if cells.chunks[slot].depth.iter().any(|&d| d > 0.0) {
                cells.wake(slot);
            }
        }
        let ledger = FieldLedger {
            initial: cells.volume(),
            ..FieldLedger::default()
        };
        Self {
            rasteriser,
            graph,
            cells,
            solver: FieldSolver::new(params),
            sources,
            sinks,
            pumps: Pumps::default(),
            sea,
            falls: Falls::default(),
            foam: Foam::default(),
            ledger,
            frame: FieldFrame::default(),
            edit_time: None,
        }
    }

    /// Run the field until it stops changing, or for `limit` seconds: the
    /// steady state a level opens in. Water nothing feeds is already as it
    /// stands, so a level without sources is left as authored. Returns the
    /// seconds it ran. The ledger starts again from what it holds then.
    ///
    /// Every hollow a source's water would fill on its way down is filled
    /// first, to its crest and a guess at the head over it, so the run only
    /// has to set the water flowing and settle the heads. Still
    /// means its volume held to a part in ten thousand over a window: waves
    /// under a fall never stop.
    pub fn settle(&mut self, dt: f32, limit: f32) -> f32 {
        const WINDOW: f32 = 5.0;
        if self.sources.is_empty() {
            return 0.0;
        }
        let starts: Vec<(SpanRef, f32)> = self
            .sources
            .iter()
            .filter_map(|s| Some((s.landing.first()?, s.discharge)))
            .map(|(at, discharge)| {
                let chunk = &self.cells.chunks[at.slot as usize];
                let (local, ordinal) = chunk.column_of(at.flat as usize);
                (SpanRef::key(chunk.coord.column(local), ordinal), discharge)
            })
            .collect();
        let held: Vec<f64> = self.cells.chunks.iter().map(|c| c.volume()).collect();
        let manning_n = self.solver.params.manning_n;
        for (start, discharge) in starts {
            fill_fed(&self.graph, &mut self.cells, start, discharge, manning_n);
        }
        for (slot, before) in held.into_iter().enumerate() {
            if self.cells.chunks[slot].volume() != before {
                self.cells.wake(slot);
            }
        }
        let mut time = 0.0;
        let mut since = 0.0;
        let mut before = self.volume();
        while time < limit {
            self.step(dt);
            time += dt;
            since += dt;
            if since >= WINDOW {
                let now = self.volume();
                let still = (now - before).abs() < (1.0e-4 * now).max(0.01);
                before = now;
                since = 0.0;
                if still {
                    break;
                }
            }
        }
        self.ledger = FieldLedger {
            initial: self.volume(),
            ..FieldLedger::default()
        };
        time
    }

    /// Advance by `dt` seconds.
    pub fn step(&mut self, dt: f32) {
        let started = Instant::now();
        for source in &self.sources {
            let sent = source.pour(&mut self.cells, dt);
            self.ledger.sourced += sent;
            if source.landing.is_empty() {
                self.ledger.sunk += sent;
            }
        }
        self.ledger.sunk += self.pumps.run(&mut self.cells, dt);
        // Chunks whose faces may carry water this step, before and after it:
        // their cells' velocities, and those of the cells their faces reach
        // into to the +x and +z, are read again after it.
        let mut moving: Vec<usize> = (0..self.cells.chunks.len())
            .filter(|&s| self.cells.chunks[s].awake)
            .collect();
        let stats = self.solver.step(&mut self.cells, dt);
        breaking::settle(&mut self.cells, self.solver.brinks());
        self.ledger.sunk += stats.boundary_out;
        for sink in &self.sinks {
            self.ledger.sunk += sink.drain(&mut self.cells);
        }
        self.ledger.lost += seep(&mut self.cells, dt);
        let awake = activity::settle(&mut self.cells, dt);
        moving.extend((0..self.cells.chunks.len()).filter(|&s| self.cells.chunks[s].awake));
        let downstream: Vec<usize> = moving
            .iter()
            .flat_map(|&s| {
                let c = self.cells.chunks[s].coord;
                [
                    SpanChunkCoord { x: c.x + 1, z: c.z },
                    SpanChunkCoord { x: c.x, z: c.z + 1 },
                ]
            })
            .filter_map(|c| self.cells.slot_of(c))
            .collect();
        moving.extend(downstream);
        moving.sort_unstable();
        moving.dedup();
        currents::refresh(&mut self.cells, &moving);
        conduits::find(&mut self.cells, self.solver.pressed());
        let brinks = Some(self.solver.brinks());
        self.ledger.sunk += self.falls.find(&self.graph, &mut self.cells, brinks, dt);
        let fast = self.solver.fast();
        self.foam
            .step(&mut self.cells, self.falls.landings(), fast, dt);
        self.frame = FieldFrame {
            stats,
            awake,
            step_time: started.elapsed(),
            edit_time: self.edit_time.take(),
        };
    }

    /// Catch up with the terrain's most recent update.
    pub fn on_terrain_update(&mut self, terrain: &TerrainWorld) {
        let rebuilt = terrain.rebuilt_chunks();
        if rebuilt.is_empty() {
            return;
        }
        let started = Instant::now();
        let remap =
            self.rasteriser
                .rebuild(terrain, &mut self.graph, rebuilt, terrain.changed_regions());
        self.ledger.sunk += carry(&mut self.cells, &self.graph, &remap, self.sea);
        for source in &mut self.sources {
            source.retrace(&self.graph, &self.cells);
        }
        for sink in &mut self.sinks {
            sink.relay(&self.graph, &self.cells);
        }
        self.pumps.retrace(&self.graph, &self.cells);
        conduits::forget(&mut self.cells);
        self.ledger.sunk += self.falls.find(&self.graph, &mut self.cells, None, 0.0);
        self.edit_time = Some(started.elapsed());
    }

    /// The water surface at a point: in the span whose band holds `y`, if
    /// that span is wet.
    pub fn level_at(&self, at: Point3<f32>) -> Option<f32> {
        let cell = self.cell_at(at)?;
        (self.cells.depth(cell) > WET_DEPTH).then(|| self.cells.surface(cell) as f32)
    }

    /// The cell whose span's band holds a point.
    pub fn cell_at(&self, at: Point3<f32>) -> Option<SpanIndex> {
        let column = Column::containing(at.x, at.z);
        if !self.graph.contains_column(column) {
            return None;
        }
        let span = self.graph.span_at(column, at.y)?;
        self.cells.index(&self.graph, column, span.ordinal)
    }

    /// Where the water runs out over ground it has not covered yet: all of
    /// it, or what may lie within `near`'s range of its point.
    pub fn fronts(&self, near: Option<(Point3<f32>, f32)>) -> impl Iterator<Item = Front> + '_ {
        fronts::fronts(&self.cells, near)
    }

    /// The water going over edges, as it stood after the last step or edit.
    pub fn falls(&self) -> &Falls {
        &self.falls
    }

    /// The fall from `cell` over its edge in `direction` (an index into
    /// `ORTHOGONAL`), if its water falls there.
    pub fn fall(&self, cell: SpanIndex, direction: usize) -> Option<&Fall> {
        self.falls.get(cell, direction)
    }

    /// The fall whose lip runs on from `fall`'s at its end `end`, and which
    /// of that fall's ends meets it there (`Falls::beside`).
    pub fn fall_beside(&self, fall: &Fall, end: usize) -> Option<(&Fall, usize)> {
        self.falls.beside(&self.graph, &self.cells, fall, end)
    }

    /// The foam riding a cell's water (`FieldChunk::foam`).
    pub fn foam(&self, cell: SpanIndex) -> f32 {
        self.cells.chunks[cell.slot as usize].foam[cell.flat as usize]
    }

    /// The chunks whose foam has changed by more than can be seen since
    /// this was last asked, by slot, and whether each one's change reaches
    /// the cells along its border.
    pub fn take_foam_moved(&mut self) -> Vec<(usize, bool)> {
        self.foam.take_moved()
    }

    /// Depth-averaged velocity in a cell, m/s, x and z.
    pub fn velocity(&self, cell: SpanIndex) -> Vector2<f32> {
        self.cells.chunks[cell.slot as usize].velocity[cell.flat as usize]
    }

    /// Water held, m³: in its cells, and in the air over its falls.
    pub fn volume(&self) -> f64 {
        self.cells.volume() + self.falls.aloft()
    }

    /// The shallowest cell and whether every depth and discharge is finite:
    /// the field's health, which no scenario may break.
    pub fn health(&self) -> (f64, bool) {
        let mut shallowest = f64::INFINITY;
        let mut finite = true;
        for chunk in &self.cells.chunks {
            for &d in &chunk.depth {
                shallowest = shallowest.min(d);
                finite &= d.is_finite();
            }
            finite &= chunk.faces.iter().all(|f| f.q.is_finite());
        }
        (shallowest, finite)
    }

    /// How high the water stands in all, m⁴: each cell's volume by the
    /// height of its middle. Water left alone only ever loses it.
    pub fn potential(&self) -> f64 {
        let mut potential = 0.0;
        for chunk in &self.cells.chunks {
            for (flat, &depth) in chunk.depth.iter().enumerate() {
                potential += depth * CELL_AREA * (chunk.bed[flat] as f64 + 0.5 * depth);
            }
        }
        potential
    }

    /// How far over its roof the water of any cell stands, m: nothing where
    /// every cell under a roof holds no more than it has room for.
    pub fn over_roof(&self) -> f64 {
        let mut over = 0.0f64;
        for chunk in &self.cells.chunks {
            for (flat, &depth) in chunk.depth.iter().enumerate() {
                let room = (chunk.ceiling[flat] - chunk.bed[flat]) as f64;
                over = over.max(depth - room);
            }
        }
        over
    }

    /// How far above its own lip any fall's water comes down, m: at its
    /// foot, or on the floor of the cell it lands in. Nothing where every
    /// fall falls.
    pub fn highest_landing(&self) -> f32 {
        let mut rise = 0.0f32;
        for fall in self.falls.iter() {
            rise = rise.max(fall.foot.y - fall.lip.y);
            if let Some(into) = fall.into.filter(|_| fall.spent.is_none()) {
                rise = rise.max(self.cells.bed(into) - fall.lip.y);
            }
        }
        rise
    }

    /// The fastest water anywhere, m/s: each face's discharge over the mean
    /// depth either side, where both sides hold at least `MEASURED_DEPTH`.
    pub fn max_speed(&self) -> f32 {
        const MEASURED_DEPTH: f64 = 0.01;
        let mut fastest = 0.0f32;
        for chunk in &self.cells.chunks {
            for face in &chunk.faces {
                let a = chunk.depth[face.a as usize];
                let b = self.cells.depth(face.b);
                if a > MEASURED_DEPTH && b > MEASURED_DEPTH {
                    fastest = fastest.max(face.q.abs() / (0.5 * (a + b)) as f32);
                }
            }
        }
        fastest
    }

    pub fn ledger(&self) -> FieldLedger {
        self.ledger
    }

    pub fn frame(&self) -> FieldFrame {
        self.frame
    }

    pub fn graph(&self) -> &SpanGraph {
        &self.graph
    }

    pub fn cells(&self) -> &FieldCells {
        &self.cells
    }

    /// Every source, as its water lands now.
    pub fn sources(&self) -> &[Source] {
        &self.sources
    }

    /// Stand pump `id` at `placement`: a new pump if the field holds none
    /// by that name, the same one moved or working differently if it does.
    pub fn set_pump(&mut self, id: PumpId, placement: PumpPlacement) {
        self.pumps.set(id, placement, &self.graph, &self.cells);
    }

    /// Take out every pump `keep` does not want.
    pub fn retain_pumps(&mut self, keep: impl FnMut(PumpId) -> bool) {
        self.pumps.retain(keep);
    }

    /// What pump `id` moved over the last step, m³/s.
    pub fn pump_discharge(&self, id: PumpId) -> Option<f32> {
        self.pumps.discharge(id)
    }

    /// Every stream thrown into the air from a point: each spring's whose
    /// water lands in the field, and each running pump's.
    pub fn jets(&self) -> impl Iterator<Item = Jet> + '_ {
        let springs = self.sources.iter().filter_map(|source| {
            Some(Jet {
                origin: source.origin,
                velocity: source.velocity,
                discharge: source.discharge,
                landing: *source.landing.first()?,
            })
        });
        springs.chain(self.pumps.jets())
    }

    /// Columns whose spans needed a parity repair: an open edge in the
    /// terrain's mesh there.
    pub fn repaired_columns(&self) -> impl Iterator<Item = Column> + '_ {
        self.rasteriser.repaired_columns()
    }

    /// The sea beyond the map's open edges, if the level has one.
    pub fn sea(&self) -> Option<SeaBoundary> {
        self.sea
    }

    pub fn params(&self) -> FlowParams {
        self.solver.params
    }

    /// Total discharge leaving through boundary faces now, m³/s.
    pub fn boundary_discharge(&self) -> f32 {
        self.cells
            .chunks
            .iter()
            .flat_map(|c| c.boundary.iter())
            .map(|f| f.q * COLUMN_SIZE)
            .sum()
    }
}
