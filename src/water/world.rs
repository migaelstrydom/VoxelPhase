//! The water façade: the single ECS resource, and the one thing physics,
//! rendering and gameplay hold.
//!
//! ```text
//!   WaterWorld
//!     field     WaterField       depths in cells, discharges on faces
//!     waves     SurfaceWaves     ripples and the sea's swell, over the field
//!     versions  SurfaceVersions  per chunk: moves when its surface visibly does
//!     falling   FallingWater     the streams in the air this frame
//!     jets      Vec<Jet>         the jets the versions last took, to move them when a jet does
//!     falls     Vec<Vec<Fall>>   the falls the versions last took, likewise
//!     clock     f64              seconds simulated: the shaders' and swell's clock
//! ```
//!
//! Each frame: carry the water over any terrain edit, step the field, find
//! its falling water, stir the ripples where it lands, then step the ripples.

use std::time::{Duration, Instant};

use nalgebra::Point3;

use crate::debug::DebugLog;
use crate::level::{Settle, WaterConfig};
use crate::terrain::TerrainWorld;

use super::coupling::{Disturbance, Disturbances};
use super::falling::FallingWater;
use super::field::{DrawnDepths, Fall, Jet, PumpId, PumpPlacement, SurfaceVersions, WaterField};
use super::geometry::{SpanChunkCoord, ORTHOGONAL};
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

/// The least power a fall must land with to stir the water it lands in,
/// m³/s · (m/s)², and the power at which it stirs it hardest.
const STIR_LEAST_POWER: f32 = 2.0;
const STIR_FULL_POWER: f32 = 200.0;

/// The hardest a landing fall kicks the surface up or down in a frame, m/s,
/// and how far past its ends its kick reaches, m.
const STIR_SPEED: f32 = 0.8;
const STIR_MARGIN: f32 = 0.3;

/// All of a level's water.
pub struct WaterWorld {
    field: WaterField,
    waves: SurfaceWaves,
    /// Each chunk's surface version, for the renderer.
    versions: SurfaceVersions,
    /// The field's falling water, as it stood after the last step or edit.
    falling: FallingWater,
    /// The field's jets as the versions last took them.
    jets: Vec<Jet>,
    /// Each chunk's falls as the versions last took them, by slot.
    falls: Vec<Vec<Fall>>,
    /// The state of the generator that draws each landing's kick.
    stir: u32,
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
            jets: field.jets().collect(),
            falls: (0..field.cells().chunks.len())
                .map(|slot| field.falls().of_slot(slot).to_vec())
                .collect(),
            waves: SurfaceWaves::new(config, &field),
            stir: 0x9E37_79B9,
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
        self.note_falls();
        self.falling = FallingWater::build(&self.field);
        self.waves.measure_exposure(&self.field);
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
        self.note_jets();
        self.note_falls();
        self.falling = FallingWater::build(&self.field);
        self.stir_landings();
        self.waves.step(&self.field, dt);
        self.clock += dt as f64;
    }

    /// Stand pump `id` at `placement`: a new pump if the water holds none
    /// by that name, the same one moved or working differently if it does.
    pub fn set_pump(&mut self, id: PumpId, placement: PumpPlacement) {
        self.field.set_pump(id, placement);
    }

    /// Take out every pump `keep` does not want.
    pub fn retain_pumps(&mut self, keep: impl FnMut(PumpId) -> bool) {
        self.field.retain_pumps(keep);
    }

    /// Move the version of every chunk a jet has started or stopped landing
    /// in, or lands in differently enough to be seen: a pump moved, run dry
    /// or working harder. The jets kept are those last drawn, so a change
    /// too small to see on any one frame is still drawn once it adds up.
    fn note_jets(&mut self) {
        let now: Vec<Jet> = self.field.jets().collect();
        if drawn_alike(&now, &self.jets) {
            return;
        }
        let mut slots: Vec<u32> = (self.jets.iter().chain(&now))
            .map(|jet| jet.landing.slot)
            .collect();
        slots.sort_unstable();
        slots.dedup();
        for slot in slots {
            let landing = |jets: &[Jet]| -> Vec<Jet> {
                jets.iter()
                    .filter(|jet| jet.landing.slot == slot)
                    .copied()
                    .collect()
            };
            if !drawn_alike(&landing(&self.jets), &landing(&now)) {
                self.versions.touch(slot as usize);
            }
        }
        self.jets = now;
    }

    /// Move the version of every chunk whose falls have started, stopped or
    /// changed enough to be seen: a curtain is drawn with the surface it
    /// hangs from, along an arc that moves with the water's speed while the
    /// depths its version is taken from stand still. So does the version of
    /// each chunk beside it that holds falls, whose curtains may run on into
    /// its own. The falls kept are those last drawn, so a change too small
    /// to see on any one frame is still drawn once it adds up.
    fn note_falls(&mut self) {
        let cells = self.field.cells();
        let count = cells.chunks.len();
        self.falls.resize_with(count, Vec::new);
        let mut moved = Vec::new();
        for slot in 0..count {
            let now = self.field.falls().of_slot(slot);
            let drawn = &mut self.falls[slot];
            let alike =
                now.len() == drawn.len() && now.iter().zip(&*drawn).all(|(a, b)| a.drawn_alike(b));
            if !alike {
                drawn.clear();
                drawn.extend_from_slice(now);
                moved.push(slot);
            }
        }
        for slot in moved {
            self.versions.touch(slot);
            let coord = cells.chunks[slot].coord;
            let beside = ORTHOGONAL.iter().filter_map(|step| {
                cells.slot_of(SpanChunkCoord {
                    x: coord.x + step.di,
                    z: coord.z + step.dk,
                })
            });
            for near in beside {
                if !self.falls[near].is_empty() {
                    self.versions.touch(near);
                }
            }
        }
    }

    /// Stir the ripples where falls land in water that stays: each landing
    /// kicks the surface up or down at random, harder the more power it
    /// lands with, so rings run out from under a fall for as long as it
    /// falls. The kicks are drawn from the world's own generator, so a run
    /// repeats.
    fn stir_landings(&mut self) {
        let mut pending = Vec::new();
        for landing in self.falling.landings().filter(|landing| !landing.onward) {
            let power = landing.discharge * landing.velocity.norm_squared();
            if power < STIR_LEAST_POWER {
                continue;
            }
            // Xorshift: enough for a splash.
            self.stir ^= self.stir << 13;
            self.stir ^= self.stir >> 17;
            self.stir ^= self.stir << 5;
            let unit = (self.stir >> 8) as f32 / (1u32 << 23) as f32 - 1.0;
            let kick = unit * STIR_SPEED * (power / STIR_FULL_POWER).min(1.0).sqrt();
            let radius = landing.half_length + STIR_MARGIN;
            pending.push((landing.at, radius, Disturbance::Velocity(kick)));
        }
        if !pending.is_empty() {
            self.waves.apply(&self.field, &Disturbances { pending });
        }
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

/// Whether jets `a` would be drawn as jets `b` are, one for one.
fn drawn_alike(a: &[Jet], b: &[Jet]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(a, b)| a.drawn_alike(b))
}
