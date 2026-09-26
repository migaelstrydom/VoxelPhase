//! Runs the water on real terrain at the game's cadence and times each frame.

use std::path::Path;
use std::time::{Duration, Instant};

use nalgebra::Point3;

use crate::level::{load_level, Level, Settle};
use crate::level_check::build_terrain;
use crate::rendering::water::{MeshKey, WaterScene, RIPPLE_TILE_STRIDE};
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
    /// Laying the network again over the edited ground.
    pub rebuild: Duration,
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
        self.geometry + self.rebuild + self.settle + self.solve + self.mesh
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
        geometry: step.geometry,
        rebuild: step.rebuild,
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
    // Flooded again, a basin is a new store: found again each time.
    let largest = |water: &WaterWorld| {
        water
            .basins()
            .map(|(id, b)| (id, b.region.len()))
            .max_by_key(|(_, n)| *n)
    };
    let (_, spans) = largest(&water).ok_or_else(|| format!("{label} has no basin"))?;
    let mut out = WorstCase {
        label,
        spans,
        reregion: Vec::new(),
        mesh: Vec::new(),
    };
    for _ in 0..repeats {
        let (id, _) = largest(&water).expect("found above");
        let started = Instant::now();
        water.reregion(id);
        out.reregion.push(started.elapsed());
        let started = Instant::now();
        let _ = water.build_mesh();
        out.mesh.push(started.elapsed());
    }
    Ok(out)
}

/// Ripple tiles at their budget: every awake tile of the largest basin
/// stepped, as many as the budget allows.
pub struct RippleCost {
    pub label: String,
    pub tiles: usize,
    pub steps: Vec<Duration>,
    /// Bytes the renderer uploads per frame for them.
    pub upload_bytes: usize,
}

pub fn ripple_cost(path: &Path, frames: usize) -> Result<RippleCost, String> {
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
    let points: Vec<Point3<f32>> = {
        let (_, basin) = water
            .basins()
            .max_by_key(|(_, b)| b.region.len())
            .ok_or_else(|| format!("{label} has no basin"))?;
        let level = basin.level();
        let mut tiles = std::collections::BTreeSet::new();
        basin
            .region
            .iter()
            .filter(|r| r.shape.floor_min < level)
            .filter(|r| tiles.insert(r.span.column.chunk()))
            .map(|r| {
                let (x, z) = r.span.column.centre();
                Point3::new(x, level, z)
            })
            .collect()
    };
    let dt = 1.0 / TICK_RATE;
    let mut steps = Vec::with_capacity(frames);
    for frame in 0..frames {
        // Keep every tile stirred, as a crowd of floats would.
        if frame % 30 == 0 {
            for p in &points {
                water.disturb(*p, 0.5, crate::water::Disturbance::Velocity(-1.0));
            }
        }
        water.step(dt);
        steps.push(water.last_step_timings().ripples);
    }
    let tiles = water.ripples().active_count();
    Ok(RippleCost {
        label,
        tiles,
        steps,
        upload_bytes: tiles * RIPPLE_TILE_STRIDE * std::mem::size_of::<f32>(),
    })
}

/// A level opened at rest: what the steady settle cost at load.
#[derive(Debug, Clone)]
pub struct SteadyLoad {
    pub label: String,
    /// The whole water load, settle included.
    pub load: Duration,
    pub sweeps: usize,
    pub converged: bool,
    /// Water the sources put in before the level opened, m³.
    pub filled: f64,
}

/// Scenarios with sources, each opened steady (§13, budget §19).
pub fn steady_loads() -> Result<Vec<SteadyLoad>, String> {
    ["staircase", "spring_pools"]
        .into_iter()
        .map(|name| {
            let scenario = find(name).ok_or(format!("the {name} scenario is missing"))?;
            let (level, terrain) = scenario.terrain()?;
            let mut config = level.water.clone().ok_or(format!("{name} has no water"))?;
            config.settle = Settle::Steady;
            let started = Instant::now();
            let (water, _) = WaterWorld::from_config(&config, &terrain);
            let load = started.elapsed();
            let report = water.steady_report().ok_or("opened without a settle")?;
            Ok(SteadyLoad {
                label: format!("{name} (water_viewer)"),
                load,
                sweeps: report.sweeps,
                converged: report.converged,
                filled: water.ledger().emitted,
            })
        })
        .collect()
}
