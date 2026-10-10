//! The field, as the game runs it, on a real level: what it costs to open,
//! to step quiet, on a blast's frame, and while the water answers the blast;
//! and what rebuilding its drawn surface costs on the frames it moved.

use std::fmt::Write as _;
use std::path::Path;
use std::time::{Duration, Instant};

use nalgebra::Point3;

use crate::level::load_level;
use crate::level_check::build_terrain;
use crate::perf::stats::{as_ms, max_ms, mean_ms, percentile_ms};
use crate::rendering::water::RiverParts;
use crate::terrain::{BlastConfig, TerrainWorld};
use crate::water::field::{WaterField, WET_DEPTH};
use crate::water::geometry::{SpanIndex, SpanRef};
use crate::water::WaterWorld;
use crate::water_viewer::TICK_RATE;

use super::runner::{Durations, WARM_UP};

/// Depth a shore cell must hold to be the blast's mark, m: open water, not
/// a film on the bank.
const SHORE_DEPTH: f64 = 0.3;

/// Where one frame of the field went.
#[derive(Debug, Clone, Copy, Default)]
pub struct FieldFrameCost {
    /// Carrying the water over a terrain edit.
    pub edit: Duration,
    /// Stepping the field.
    pub step: Duration,
    /// Rebuilding the drawn surface, on the frames it moved.
    pub mesh: Duration,
    /// Chunks awake after the frame.
    pub awake: usize,
    /// The terrain's own rebuild, reported beside the water, not in it.
    pub terrain: Duration,
}

impl FieldFrameCost {
    /// What the frame's water costs the CPU.
    pub fn water(&self) -> Duration {
        self.edit + self.step + self.mesh
    }
}

/// One level's recorded frames, split by phase.
pub struct FieldSubject {
    pub label: String,
    /// Where the charge went off, if the level has a shore to blast.
    pub blast_at: Option<Point3<f32>>,
    pub quiet: Vec<FieldFrameCost>,
    pub blast: FieldFrameCost,
    pub transient: Vec<FieldFrameCost>,
    /// Building the field and filling it, then settling it: wall time.
    pub load: Duration,
    /// Simulated seconds the opening settle ran.
    pub settled: f32,
    /// Chunks in the field, and cells over all of them.
    pub chunks: usize,
    pub cells: usize,
    /// Largest drawn surface, in indices.
    pub mesh_indices: usize,
    /// Water held at the start of the quiet phase and at the end, m³, and
    /// the ledger's error at the end.
    pub volume: (f64, f64),
    pub ledger_error: f64,
    /// Let out to the void, sinks and the sea, and seeped from films, over
    /// the whole run, m³.
    pub sunk: f64,
    pub lost: f64,
}

/// A level from disk, blasted on its deepest shore.
pub fn run_field_level(path: &Path, durations: Durations) -> Result<FieldSubject, String> {
    let level = load_level(path).map_err(|e| e.to_string())?;
    let mut terrain = build_terrain(&level);
    let label = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let config = level
        .water
        .as_ref()
        .ok_or_else(|| format!("{label} has no water"))?;
    let mut water = WaterWorld::from_config(config, &terrain);
    let (settled, load) = water.opened();
    let dt = 1.0 / TICK_RATE;
    let frames = |seconds: f32| (seconds * TICK_RATE).round() as usize;
    let mut rivers = RiverParts::default();
    let mut indices = 0;
    let mut run = |terrain: &mut TerrainWorld, water: &mut WaterWorld, n: usize| {
        (0..n)
            .map(|_| frame(terrain, water, &mut rivers, &mut indices, dt))
            .collect::<Vec<_>>()
    };

    run(&mut terrain, &mut water, frames(WARM_UP));
    let before = water.volume();
    let quiet = run(&mut terrain, &mut water, frames(durations.quiet));
    let blast_at = shore_site(water.field(), &terrain);
    if let Some(at) = blast_at {
        terrain.detonate(at, &BlastConfig::default());
    }
    let blast = run(&mut terrain, &mut water, 1)[0];
    let transient = run(&mut terrain, &mut water, frames(durations.transient));
    let field = water.field();
    let volume = field.volume();
    Ok(FieldSubject {
        label,
        blast_at,
        quiet,
        blast,
        transient,
        load,
        settled,
        chunks: field.cells().chunks.len(),
        cells: field.cells().chunks.iter().map(|c| c.depth.len()).sum(),
        mesh_indices: indices,
        volume: (before, volume),
        ledger_error: field.ledger().error(volume),
        sunk: field.ledger().sunk,
        lost: field.ledger().lost,
    })
}

/// One game frame of terrain and water, and the surface rebuild the
/// renderer would make if the water moved.
fn frame(
    terrain: &mut TerrainWorld,
    water: &mut WaterWorld,
    rivers: &mut RiverParts,
    indices: &mut usize,
    dt: f32,
) -> FieldFrameCost {
    let started = Instant::now();
    terrain.update();
    let terrain_time = started.elapsed();
    water.on_terrain_update(terrain);
    water.step(dt);
    let field = water.field().frame();

    let started = Instant::now();
    let mut mesh = Duration::ZERO;
    if rivers.sync(&*water) {
        *indices = (*indices).max(rivers.mesh().indices.len());
        mesh = started.elapsed();
    }
    FieldFrameCost {
        edit: field.edit_time.unwrap_or_default(),
        step: field.step_time,
        mesh,
        awake: field.awake,
        terrain: terrain_time,
    }
}

/// The deepest water standing beside dry ground, on the terrain surface: a
/// blast there breaks the bank that holds the most back.
fn shore_site(field: &WaterField, terrain: &TerrainWorld) -> Option<Point3<f32>> {
    let graph = field.graph();
    let cells = field.cells();
    let depth = |span: SpanRef| {
        cells
            .index(graph, span.column, span.ordinal)
            .map_or(0.0, |at: SpanIndex| cells.depth(at))
    };
    let (span, _) = graph
        .all_refs()
        .map(|span| (span, depth(span)))
        .filter(|&(_, d)| d > SHORE_DEPTH)
        .filter(|&(span, _)| {
            graph
                .orthogonal_neighbours(span)
                .into_iter()
                .any(|n| depth(SpanRef::key(n.span.column, n.span.ordinal)) <= WET_DEPTH)
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))?;
    let (x, z) = span.column.centre();
    let y = terrain.mesh_surface_height_at(x, z)?;
    Some(Point3::new(x, y, z))
}

/// One level: opening, quiet frames, the blast frame, and the transient.
pub fn field_table(subject: &FieldSubject) -> String {
    let mut out = String::new();
    let blast = subject.blast_at.map_or("no shore to blast".into(), |p| {
        format!("blast at {:.1}, {:.1}, {:.1}", p.x, p.y, p.z)
    });
    let _ = writeln!(
        out,
        "{}  ({blast}; {} chunks, {} cells; mesh up to {} indices)",
        subject.label, subject.chunks, subject.cells, subject.mesh_indices
    );
    let _ = writeln!(
        out,
        "  opening {:.1} ms ({:.0} s settled); volume {:.1} -> {:.1} m³ (sunk {:.1}, seeped {:.1}), ledger error {:+.1e} m³",
        as_ms(subject.load),
        subject.settled,
        subject.volume.0,
        subject.volume.1,
        subject.sunk,
        subject.lost,
        subject.ledger_error
    );
    let _ = writeln!(
        out,
        "  {:<18} {:>14} {:>14} {:>14} {:>10} {:>24}",
        "phase (ms)", "edit", "step", "mesh", "awake", "water mean/p99/max"
    );
    out.push_str(&phase_row("quiet", &subject.quiet));
    out.push_str(&phase_row(
        "blast frame",
        std::slice::from_ref(&subject.blast),
    ));
    out.push_str(&phase_row("transient", &subject.transient));
    let _ = writeln!(
        out,
        "  terrain rebuild on the blast frame: {:.2} ms (not water)",
        as_ms(subject.blast.terrain)
    );
    out
}

fn phase_row(name: &str, frames: &[FieldFrameCost]) -> String {
    let pair = |f: fn(&FieldFrameCost) -> Duration| {
        let times: Vec<Duration> = frames.iter().map(f).collect();
        format!("{:.3}/{:.3}", mean_ms(&times), percentile_ms(&times, 0.99))
    };
    let water: Vec<Duration> = frames.iter().map(FieldFrameCost::water).collect();
    let awake = frames.iter().map(|f| f.awake).max().unwrap_or(0);
    format!(
        "  {:<18} {:>14} {:>14} {:>14} {:>10} {:>24}\n",
        format!("{name} ({})", frames.len()),
        pair(|f| f.edit),
        pair(|f| f.step),
        pair(|f| f.mesh),
        format!("≤{awake}"),
        format!(
            "{:.3} / {:.3} / {:.3}",
            mean_ms(&water),
            percentile_ms(&water, 0.99),
            max_ms(&water)
        ),
    )
}
