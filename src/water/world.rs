//! The water façade: the single ECS resource, and the one thing physics,
//! rendering and gameplay hold.
//!
//! ```text
//!   WaterWorld
//!     field     WaterField       depths in cells, discharges on faces
//!     waves     SurfaceWaves     ripples and the sea's swell, over the field
//!     versions  SurfaceVersions  per chunk: moves when its surface visibly does
//!     falling   FallingWater     the streams in the air this frame
//!     clock     f64              seconds simulated: the shaders' and swell's clock
//! ```
//!
//! Each frame: carry the water over any terrain edit, step the field, find
//! its falling water, then step the ripples over it.

use std::time::{Duration, Instant};

use nalgebra::Point3;

use crate::debug::DebugLog;
use crate::level::{Settle, WaterConfig};
use crate::terrain::TerrainWorld;

use super::coupling::Disturbances;
use super::falling::FallingWater;
use super::field::{DrawnDepths, SurfaceVersions, WaterField};
use super::query::WaterQuery;
use super::waves::SurfaceWaves;

/// Step the field takes while the level opens steady, s: the solver takes
/// as many substeps within it as its water needs.
const SETTLE_STEP: f32 = 0.1;

/// Longest the opening settle runs, simulated s. The fill before it has
/// already wet the channels and raised each fed lake to a guess at its
/// head, so by then lakes stand within centimetres of steady and settle the
/// rest of the way in play.
const SETTLE_LIMIT: f32 = 30.0;

/// Longest frame the field steps in one go, s: a hitch is not a flood.
const LONGEST_FRAME: f32 = 0.1;

/// All of a level's water.
pub struct WaterWorld {
    field: WaterField,
    waves: SurfaceWaves,
    /// Each chunk's surface version, for the renderer.
    versions: SurfaceVersions,
    /// The field's falling water, as it stood after the last step or edit.
    falling: FallingWater,
    /// Seconds simulated: the clock the drawn currents run on.
    clock: f64,
    /// Simulated seconds the level's opening settle ran, and what it cost.
    opened: (f32, Duration),
}

impl WaterWorld {
    /// A level's water, laid over its terrain and filled as authored, then
    /// run until it is steady if the level asks for that.
    pub fn from_config(config: &WaterConfig, terrain: &TerrainWorld) -> Self {
        let started = Instant::now();
        let mut field = WaterField::build(config, terrain);
        let settled = match config.settle {
            Settle::Steady => field.settle(SETTLE_STEP, SETTLE_LIMIT),
            Settle::AsAuthored => 0.0,
        };
        Self {
            versions: SurfaceVersions::new(field.cells()),
            falling: FallingWater::build(&field),
            waves: SurfaceWaves::new(config),
            field,
            clock: 0.0,
            opened: (settled, started.elapsed()),
        }
    }

    /// Carry the water over the terrain's most recent update.
    pub fn on_terrain_update(&mut self, terrain: &TerrainWorld) {
        if terrain.rebuilt_chunks().is_empty() {
            return;
        }
        self.field.on_terrain_update(terrain);
        let cells = self.field.cells();
        self.versions.update(cells, 0..cells.chunks.len());
        self.falling = FallingWater::build(&self.field);
    }

    /// Advance by one frame.
    pub fn step(&mut self, frame_dt: f32) {
        let dt = frame_dt.min(LONGEST_FRAME);
        let awake = |field: &WaterField| -> Vec<usize> {
            let chunks = &field.cells().chunks;
            (0..chunks.len()).filter(|&s| chunks[s].awake).collect()
        };
        let mut moving = awake(&self.field);
        self.field.step(dt);
        moving.extend(awake(&self.field));
        moving.sort_unstable();
        moving.dedup();
        self.versions.update(self.field.cells(), moving);
        self.falling = FallingWater::build(&self.field);
        self.waves.step(&self.field, dt);
        self.clock += dt as f64;
    }

    /// Stir the ripples with every disturbance the coupler collected.
    pub fn apply(&mut self, disturbances: &Disturbances) {
        self.waves.apply(&self.field, disturbances);
    }

    /// Where the camera is, for ranking ripple tiles against their budget.
    pub fn set_focus(&mut self, focus: Option<Point3<f32>>) {
        self.waves.set_focus(focus);
    }

    /// Point queries against the water: buoyancy, currents, the player.
    pub fn query(&self) -> WaterQuery<'_> {
        WaterQuery::new(&self.field, &self.waves, &self.falling, self.clock())
    }

    /// The field's falling water this frame.
    pub fn falling(&self) -> &FallingWater {
        &self.falling
    }

    pub fn waves(&self) -> &SurfaceWaves {
        &self.waves
    }

    pub fn field(&self) -> &WaterField {
        &self.field
    }

    /// Each chunk's surface version, by slot: moves whenever its drawn
    /// surface would.
    pub fn versions(&self) -> &[u32] {
        self.versions.versions()
    }

    /// Each chunk's depths as its surface is drawn from them.
    pub fn drawn_depths(&self) -> DrawnDepths<'_> {
        self.versions.drawn()
    }

    /// Seconds simulated.
    pub fn clock(&self) -> f32 {
        self.clock as f32
    }

    /// Water held, m³.
    pub fn volume(&self) -> f64 {
        self.field.volume()
    }

    /// Simulated seconds the opening settle ran, and the wall time it and
    /// the build took.
    pub fn opened(&self) -> (f32, Duration) {
        self.opened
    }

    /// Whether the opening settle ran to its limit, so the level opens still
    /// settling.
    pub fn still_settling(&self) -> bool {
        self.opened.0 >= SETTLE_LIMIT
    }

    /// Write the F3 statistics.
    pub fn debug_log(&self, log: &mut DebugLog) {
        let ledger = self.field.ledger();
        let volume = self.field.volume();
        let frame = self.field.frame();
        log.add("Water/Ledger/Held", format!("{volume:.3} m³"));
        log.add(
            "Water/Ledger/Error",
            format!("{:+.3e} m³", ledger.error(volume)),
        );
        log.add("Water/Ledger/Sourced", format!("{:.3}", ledger.sourced));
        log.add("Water/Ledger/Sunk", format!("{:.3}", ledger.sunk));
        log.add("Water/Ledger/Lost", format!("{:.3}", ledger.lost));
        log.add(
            "Water/Field/Awake",
            format!(
                "{} of {} chunks, {} substeps",
                frame.awake,
                self.field.cells().chunks.len(),
                frame.stats.substeps
            ),
        );
        log.add(
            "Water/Ripples",
            format!("{} tiles awake", self.waves.ripples().active_count()),
        );
        log.add(
            "Water/Time/Step",
            format!("{:.3} ms", frame.step_time.as_secs_f64() * 1000.0),
        );
        if let Some(edit) = frame.edit_time {
            log.add(
                "Water/Time/LastEdit",
                format!("{:.3} ms", edit.as_secs_f64() * 1000.0),
            );
        }
        let (settled, took) = self.opened;
        log.add(
            "Water/Time/Opening",
            format!("{:.3} s ({settled:.0} s settled)", took.as_secs_f64()),
        );
    }
}
