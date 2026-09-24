//! Runs the water on real terrain at the game's cadence and times each frame.

use std::path::Path;
use std::time::{Duration, Instant};

use nalgebra::Point3;

use crate::level::{load_level, Level};
use crate::level_check::build_terrain;
use crate::terrain::{BlastConfig, TerrainWorld};
use crate::water_viewer::{find, LegacyWater, TICK_RATE};

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
    /// Terrain-change handling plus the flow step.
    pub flow: Duration,
    /// The wave step.
    pub wave: Duration,
    /// Building the surface mesh the renderer uploads.
    pub mesh: Duration,
    /// The terrain's own rebuild, reported beside the water, not in it.
    pub terrain: Duration,
}

impl FrameCost {
    /// Every water stage: what the frame's water costs the CPU.
    pub fn water(&self) -> Duration {
        self.flow + self.wave + self.mesh
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
    /// Surface mesh size on the last quiet frame, in indices.
    pub mesh_indices: usize,
}

/// A level from disk, blasted at the shore of its water.
pub fn run_level(path: &Path, durations: Durations) -> Result<Subject, String> {
    let level = load_level(path).map_err(|e| e.to_string())?;
    let terrain = build_terrain(&level);
    let label = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let water =
        LegacyWater::from_level(&level, &terrain).ok_or_else(|| format!("{label} has no water"))?;
    let blast_at =
        shore_site(&water, &terrain).ok_or_else(|| format!("{label}: no shoreline to blast"))?;
    Ok(run_subject(
        label,
        &level,
        terrain,
        water,
        blast_at,
        &BlastConfig::default(),
        durations,
    ))
}

/// The water harness's pond breach, the transient case.
pub fn run_breach(durations: Durations) -> Result<Subject, String> {
    let scenario = find("breach").ok_or("the breach scenario is missing")?;
    let (level, terrain) = scenario.terrain()?;
    let water = LegacyWater::from_level(&level, &terrain).ok_or("breach has no water")?;
    let (centre, radius) = scenario
        .beats
        .iter()
        .map(|beat| match beat.action {
            crate::water_viewer::Action::Blast { centre, radius } => (centre, radius),
        })
        .next()
        .ok_or("the breach scenario has no blast")?;
    Ok(run_subject(
        "breach (water_viewer)".into(),
        &level,
        terrain,
        water,
        centre,
        &BlastConfig::fixed_radius(radius),
        durations,
    ))
}

fn run_subject(
    label: String,
    _level: &Level,
    mut terrain: TerrainWorld,
    mut water: LegacyWater,
    blast_at: Point3<f32>,
    charge: &BlastConfig,
    durations: Durations,
) -> Subject {
    let dt = 1.0 / TICK_RATE;
    let frames = |seconds: f32| (seconds * TICK_RATE).round() as usize;

    for _ in 0..frames(WARM_UP) {
        frame(&mut terrain, &mut water, dt);
    }
    let quiet: Vec<FrameCost> = (0..frames(durations.quiet))
        .map(|_| frame(&mut terrain, &mut water, dt))
        .collect();
    let (_, mesh_indices) = water.build_mesh();

    terrain.detonate(blast_at, charge);
    let blast = frame(&mut terrain, &mut water, dt);
    let transient = (0..frames(durations.transient))
        .map(|_| frame(&mut terrain, &mut water, dt))
        .collect();

    Subject {
        label,
        blast_at,
        quiet,
        blast,
        transient,
        mesh_indices,
    }
}

/// One game frame of terrain and water: update, tick, and the mesh build.
fn frame(terrain: &mut TerrainWorld, water: &mut LegacyWater, dt: f32) -> FrameCost {
    let started = Instant::now();
    terrain.update();
    let terrain_time = started.elapsed();
    let tick = water.tick(terrain, dt);
    let (mesh, _) = water.build_mesh();
    FrameCost {
        flow: tick.flow,
        wave: tick.wave,
        mesh,
        terrain: terrain_time,
    }
}

/// The shoreline point nearest the water's centroid, on the terrain surface.
///
/// Nearest the centroid is the narrowest part of the rim, the likeliest place
/// for a blast to reach the water from the shore.
fn shore_site(water: &LegacyWater, terrain: &TerrainWorld) -> Option<Point3<f32>> {
    let shore = water.shoreline();
    if shore.is_empty() {
        return None;
    }
    let n = shore.len() as f32;
    let (cx, cz) = shore
        .iter()
        .fold((0.0, 0.0), |(x, z), &(sx, sz)| (x + sx / n, z + sz / n));
    let &(x, z) = shore.iter().min_by(|a, b| {
        let da = (a.0 - cx).powi(2) + (a.1 - cz).powi(2);
        let db = (b.0 - cx).powi(2) + (b.1 - cz).powi(2);
        da.total_cmp(&db)
    })?;
    let y = terrain.mesh_surface_height_at(x, z)?;
    Some(Point3::new(x, y, z))
}
