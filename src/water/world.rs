//! The water façade: the single ECS resource, and the one thing physics,
//! rendering and gameplay hold.
//!
//! ```text
//!   WaterWorld
//!     field     WaterField       depths in cells, discharges on faces
//!     versions  SurfaceVersions  per chunk: moves when its surface visibly does
//!     clock     f64              seconds simulated: the shader's flow clock
//! ```
//!
//! Each frame: carry the water over any terrain edit, then step the field.

use std::time::{Duration, Instant};

use crate::debug::DebugLog;
use crate::level::{Settle, WaterConfig};
use crate::terrain::TerrainWorld;

use super::field::{SurfaceVersions, WaterField};
use super::query::WaterQuery;

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
    /// Each chunk's surface version, for the renderer.
    versions: SurfaceVersions,
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
        self.clock += dt as f64;
    }

    /// Point queries against the water: buoyancy, currents, the player.
    pub fn query(&self) -> WaterQuery<'_> {
        WaterQuery::new(&self.field)
    }

    pub fn field(&self) -> &WaterField {
        &self.field
    }

    /// Each chunk's surface version, by slot: moves whenever its drawn
    /// surface would.
    pub fn versions(&self) -> &[u32] {
        self.versions.versions()
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
