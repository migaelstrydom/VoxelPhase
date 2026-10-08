//! `WaterField`: water as a shallow-water field over the span graph.
//!
//! ```text
//!   TerrainWorld ──▶ SpanRasteriser ──▶ SpanGraph ──▶ FieldCells (+ faces)
//!        │ edit           │ re-pair         │ remap        │
//!        └────────────────┘                 └── carry ─────┤
//!   Sources ── pour ──▶                                    ▼
//!   Sinks   ◀─ drain ──                         FieldSolver (awake chunks)
//! ```
//!
//! The water is the only state: depths in cells, discharges on faces. An
//! edit re-pairs the columns it touched, carries their water into the new
//! spans, and lays faces again around them. Nothing else changes.

use std::time::{Duration, Instant};

use nalgebra::{Point3, Vector2, Vector3};

use crate::level::{MapEdge, WaterBody, WaterConfig};
use crate::terrain::TerrainWorld;
use crate::water::geometry::{Column, SpanGraph, SpanIndex, SpanRasteriser, COLUMN_SIZE};

use super::activity;
use super::cells::{FieldCells, CELL_AREA};
use super::faces::{face_owners, lay_faces, link_faces, Axis, SeaBoundary};
use super::fill::{flood, pool_seed, sea_seeds};
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
    sea: Option<SeaBoundary>,
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
        if let Some(sea) = sea {
            flood(&graph, &mut cells, &sea_seeds(&graph, &sea), sea.level);
        }
        let mut sources = Vec::new();
        let mut sinks = Vec::new();
        for body in &config.bodies {
            match body {
                WaterBody::Pool {
                    seed,
                    surface_level,
                } => {
                    if let Some(span) = pool_seed(&graph, seed.0, seed.1, *surface_level) {
                        flood(&graph, &mut cells, &[span], *surface_level);
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
            sea,
            ledger,
            frame: FieldFrame::default(),
            edit_time: None,
        }
    }

    /// Run the field until it stops changing, or for `limit` seconds: the
    /// steady state a level opens in. Water nothing feeds is already as it
    /// stands, so a level without sources is left as authored. Returns the
    /// seconds it ran. The ledger
    /// starts again from what it holds then. Still means its volume held
    /// to a part in ten thousand over a window: waves under a fall never stop.
    pub fn settle(&mut self, dt: f32, limit: f32) -> f32 {
        const WINDOW: f32 = 5.0;
        if self.sources.is_empty() {
            return 0.0;
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
            let volume = source.discharge as f64 * dt as f64;
            self.ledger.sourced += volume;
            if source.landing.is_empty() {
                self.ledger.sunk += volume;
                continue;
            }
            let share = volume / source.landing.len() as f64 / CELL_AREA;
            for &at in &source.landing {
                *self.cells.depth_mut(at) += share;
                self.cells.wake(at.slot as usize);
            }
        }
        let stats = self.solver.step(&mut self.cells, dt);
        self.ledger.sunk += stats.boundary_out;
        for sink in &self.sinks {
            self.ledger.sunk += sink.drain(&mut self.cells);
        }
        self.ledger.lost += seep(&mut self.cells, dt);
        let awake = activity::settle(&mut self.cells, dt);
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

    /// Depth-averaged velocity in a cell, m/s: on each axis, the mean over
    /// its faces of the discharge over the mean depth either side, taken as
    /// at least `MEASURED_DEPTH`. Dividing by a film's own depth would make
    /// a trickle a torrent; a film that moves still reads as moving.
    pub fn velocity(&self, cell: SpanIndex) -> Vector2<f32> {
        const MEASURED_DEPTH: f64 = 0.01;
        let here = self.cells.depth(cell);
        if here <= WET_DEPTH {
            return Vector2::zeros();
        }
        let mut sum = Vector2::zeros();
        let mut count = Vector2::zeros();
        let mut add = |axis: Axis, q: f32, other: f64| {
            let depth = (0.5 * (here + other)).max(MEASURED_DEPTH);
            let u = q / depth as f32;
            match axis {
                Axis::X => {
                    sum.x += u;
                    count.x += 1.0;
                }
                Axis::Z => {
                    sum.y += u;
                    count.y += 1.0;
                }
            }
        };
        let chunk = &self.cells.chunks[cell.slot as usize];
        for face in &chunk.faces {
            if face.a == cell.flat {
                add(face.axis, face.q, self.cells.depth(face.b));
            }
        }
        for owner in self.inbound_owners(cell) {
            let chunk = &self.cells.chunks[owner];
            for face in &chunk.faces {
                if face.b == cell {
                    add(face.axis, face.q, chunk.depth[face.a as usize]);
                }
            }
        }
        let mean = |s: f32, n: f32| if n > 0.0 { s / n } else { 0.0 };
        Vector2::new(mean(sum.x, count.x), mean(sum.y, count.y))
    }

    /// Slots whose faces may end in `cell`: its own and its −x and −z
    /// neighbours.
    fn inbound_owners(&self, cell: SpanIndex) -> impl Iterator<Item = usize> + '_ {
        let coord = self.cells.chunks[cell.slot as usize].coord;
        face_owners(coord)
            .into_iter()
            .filter_map(|c| self.graph.slot(c))
    }

    pub fn volume(&self) -> f64 {
        self.cells.volume()
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
