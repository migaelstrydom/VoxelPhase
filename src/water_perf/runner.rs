//! Runs the water on real terrain at the game's cadence and times each frame.

use std::path::Path;
use std::time::{Duration, Instant};

use nalgebra::Point3;

use crate::level::{load_level, Level};
use crate::level_check::build_terrain;
use crate::rendering::water::{MeshKey, WaterScene};
use crate::terrain::{BlastConfig, TerrainWorld};
use crate::water::WaterWorld;
use crate::water_viewer::{find, Action, TICK_RATE};

/// Seconds of frames discarded before any are recorded.
pub const WARM_UP: f32 = 0.2;

/// How long each phase of a run lasts, in simulated seconds.
#[derive(Debug, Clone, Copy)]
pub struct Durations {
    /// Undisturbed water before the blast.
    pub quiet: f32,
    /// Frames after the blast frame.
    pub transient: f32,
}

impl Default for Durations {
    fn default() -> Self {
        Self {
            quiet: 4.0,
            transient: 10.0,
        }
    }
}

/// Where one frame's water time went.
#[derive(Debug, Clone, Copy, Default)]
pub struct FrameCost {
    /// Re-pairing spans and repairing drainage after a terrain edit.
    pub geometry: Duration,
    /// Re-flooding the basins an edit touched.
    pub reregion: Duration,
    /// Between-tick topology.
    pub settle: Duration,
    /// Solver ticks.
    pub solve: Duration,
    /// Rebuilding the surface mesh, on the frames whose topology changed.
    pub mesh: Duration,
    /// The terrain's own rebuild, reported beside the water, not in it.
    pub terrain: Duration,
}

impl FrameCost {
    /// Every water stage: what the frame's water costs the CPU.
    pub fn water(&self) -> Duration {
        self.geometry + self.reregion + self.settle + self.solve + self.mesh
    }
}

/// One subject's recorded frames, split by phase.
pub struct Subject {
    pub label: String,
    /// Where the charge went off.
    pub blast_at: Point3<f32>,
    pub quiet: Vec<FrameCost>,
    pub blast: FrameCost,
    pub transient: Vec<FrameCost>,
    /// Surface mesh size, in indices, on the last quiet frame.
    pub mesh_indices: usize,
    /// Basins before the blast and at the end.
    pub basins: (usize, usize),
    /// Load: building spans, drainage and basins.
    pub load: Duration,
    /// Each basin before the blast and at the end, one line each.
    pub basin_lines: (Vec<String>, Vec<String>),
}

/// A level from disk, blasted at the lip of its largest basin.
pub fn run_level(path: &Path, durations: Durations) -> Result<Subject, String> {
    let level = load_level(path).map_err(|e| e.to_string())?;
    let terrain = build_terrain(&level);
    let label = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let config = level
        .water
        .as_ref()
        .ok_or_else(|| format!("{label} has no water"))?;
    let started = Instant::now();
    let (water, _) = WaterWorld::from_config(config, &terrain);
    let load = started.elapsed();
    let blast_at =
        lip_site(&water, &terrain).ok_or_else(|| format!("{label}: no basin to blast"))?;
    Ok(run_subject(
        label,
        &level,
        terrain,
        water,
        load,
        blast_at,
        &BlastConfig::default(),
        durations,
    ))
}

/// The water harness's dam breach, the transient case.
pub fn run_breach(durations: Durations) -> Result<Subject, String> {
    let scenario = find("breach").ok_or("the breach scenario is missing")?;
    let (level, terrain) = scenario.terrain()?;
    let config = level.water.as_ref().ok_or("breach has no water")?;
    let started = Instant::now();
    let (water, _) = WaterWorld::from_config(config, &terrain);
    let load = started.elapsed();
    let (centre, radius) = scenario
        .beats
        .iter()
        .map(|beat| match beat.action {
            Action::Blast { centre, radius } => (centre, radius),
        })
        .next()
        .ok_or("the breach scenario has no blast")?;
    Ok(run_subject(
        "breach (water_viewer)".into(),
        &level,
        terrain,
        water,
        load,
        centre,
        &BlastConfig::fixed_radius(radius),
        durations,
    ))
}

#[allow(clippy::too_many_arguments)]
fn run_subject(
    label: String,
    _level: &Level,
    mut terrain: TerrainWorld,
    mut water: WaterWorld,
    load: Duration,
    blast_at: Point3<f32>,
    charge: &BlastConfig,
    durations: Durations,
) -> Subject {
    let dt = 1.0 / TICK_RATE;
    let frames = |seconds: f32| (seconds * TICK_RATE).round() as usize;
    let mut key = MeshKey::new();
    let mut indices = 0;

    for _ in 0..frames(WARM_UP) {
        frame(&mut terrain, &mut water, &mut key, &mut indices, dt);
    }
    let quiet: Vec<FrameCost> = (0..frames(durations.quiet))
        .map(|_| frame(&mut terrain, &mut water, &mut key, &mut indices, dt))
        .collect();
    let mesh_indices = indices;
    let before = water.basins().count();
    let lines_before = basin_lines(&water);

    terrain.detonate(blast_at, charge);
    let blast = frame(&mut terrain, &mut water, &mut key, &mut indices, dt);
    let transient = (0..frames(durations.transient))
        .map(|_| frame(&mut terrain, &mut water, &mut key, &mut indices, dt))
        .collect();

    Subject {
        label,
        blast_at,
        quiet,
        blast,
        transient,
        mesh_indices,
        basins: (before, water.basins().count()),
        load,
        basin_lines: (lines_before, basin_lines(&water)),
    }
}

/// One line per basin: id, level, volume, region size, spill.
fn basin_lines(water: &WaterWorld) -> Vec<String> {
    water
        .basins()
        .map(|(id, b)| {
            format!(
                "basin {}: level {:.3}, {:.1} m³, {} spans, {} crests (lowest {:?}), spill {:?}, cap {:.2}",
                id.0,
                b.level(),
                b.volume,
                b.region.len(),
                b.crests.len(),
                b.crests.first().map(|c| (c.saddle, c.kind)),
                b.spill(),
                b.cap
            )
        })
        .collect()
}

/// One game frame of terrain and water, and the mesh rebuild the renderer
/// would make if the topology changed.
fn frame(
    terrain: &mut TerrainWorld,
    water: &mut WaterWorld,
    key: &mut MeshKey,
    indices: &mut usize,
    dt: f32,
) -> FrameCost {
    let started = Instant::now();
    terrain.update();
    let terrain_time = started.elapsed();
    water.on_terrain_update(terrain);
    let edit = water.pending_timings();
    water.step(dt);
    let step = water.last_step_timings();

    let started = Instant::now();
    let now = water.mesh_key();
    let mut mesh = Duration::ZERO;
    if now != *key {
        let built = water.build_mesh();
        *indices = built.indices.len();
        *key = now;
        mesh = started.elapsed();
    }
    FrameCost {
        geometry: edit.geometry,
        reregion: edit.reregion,
        settle: step.settle,
        solve: step.solve,
        mesh,
        terrain: terrain_time,
    }
}

/// The lip of the largest basin, on the terrain surface: a blast there
/// re-floods the largest region, and may breach it.
fn lip_site(water: &WaterWorld, terrain: &TerrainWorld) -> Option<Point3<f32>> {
    let (_, basin) = water
        .basins()
        .max_by(|a, b| a.1.region.len().cmp(&b.1.region.len()))?;
    let span = basin
        .crests
        .first()
        .map(|c| c.inside)
        .or_else(|| basin.region.first().map(|r| r.span))?;
    let (x, z) = span.column.centre();
    let y = terrain.mesh_surface_height_at(x, z)?;
    Some(Point3::new(x, y, z))
}

/// The worst case of §9.2: re-flooding the largest basin of a level, and
/// rebuilding the surface mesh after it, repeated for a stable figure.
pub struct WorstCase {
    pub label: String,
    pub spans: usize,
    pub reregion: Vec<Duration>,
    pub mesh: Vec<Duration>,
}

pub fn worst_case(path: &Path, repeats: usize) -> Result<WorstCase, String> {
    let level = load_level(path).map_err(|e| e.to_string())?;
    let terrain = build_terrain(&level);
    let label = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let config = level
        .water
        .as_ref()
        .ok_or_else(|| format!("{label} has no water"))?;
    let (mut water, _) = WaterWorld::from_config(config, &terrain);
    let (id, spans) = water
        .basins()
        .map(|(id, b)| (id, b.region.len()))
        .max_by_key(|(_, n)| *n)
        .ok_or_else(|| format!("{label} has no basin"))?;
    let mut out = WorstCase {
        label,
        spans,
        reregion: Vec::new(),
        mesh: Vec::new(),
    };
    for _ in 0..repeats {
        let started = Instant::now();
        water.reregion(id);
        out.reregion.push(started.elapsed());
        let started = Instant::now();
        let _ = water.build_mesh();
        out.mesh.push(started.elapsed());
    }
    Ok(out)
}
