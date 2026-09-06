//! In-game recorder for the foot placer's input stream.
//!
//! Captures every `PlacerCtx` field (plus the suspend flag and the active
//! pose tag) once per tick, alongside the placer's outputs for the same
//! tick. The `replay` test module feeds a recording back through a fresh
//! `FootPlacer`, reproducing in-game behaviour deterministically offline —
//! the bridge between "looks wrong in game" and a debuggable trace.
//!
//! Nothing is written while the game runs, and nothing is kept until asked
//! for. `PLACER_REC` only offers the recorder; F4 arms it, and F4 again writes
//! the file. A tick then costs one push into a preallocated ring holding the
//! last `PLACER_REC_SECONDS` of play.
//!
//! Arming from a key rather than from the environment keeps the walk to the
//! artefact at full speed, and makes the recorder's cost measurable: it is
//! reported per tick on the debug overlay, so "recording slows the game down"
//! is a number rather than an impression. This is not an optimisation: a
//! recorder that appends to a file sixty times a second puts that file under
//! whatever is watching the directory — a sync daemon, an indexer, a backup —
//! and the cost lands on the machine rather than on the frame, as a slowdown
//! that gets worse the longer the recording runs. It also records the wrong
//! thing. What is wanted is the ten seconds where the artefact showed, not the
//! two minutes of walking there, and a ring gives exactly that without asking
//! anyone to guess when to start.
//!
//! ```bash
//! PLACER_REC=/tmp/temple.csv cargo run    # play; F4 to start, F4 to write
//! ```
//!
//! Float fields are written with `Display`, which round-trips `f32`
//! exactly, so a replay sees bit-identical inputs.

use std::collections::VecDeque;
use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;
use std::time::Instant;

use nalgebra::{Point3, Vector3};

use super::placer::{FootPlacer, PlacerCtx};

/// Environment variable holding the recording output path.
pub const RECORDER_ENV_VAR: &str = "PLACER_REC";

/// Environment variable holding the recorded window, in seconds.
pub const WINDOW_ENV_VAR: &str = "PLACER_REC_SECONDS";

/// Seconds of play kept when `PLACER_REC_SECONDS` says nothing. Long enough to
/// hold the approach to an artefact as well as the artefact.
const DEFAULT_WINDOW_SECONDS: f32 = 30.0;

/// Ticks per second the ring is sized for. Deliberately well above any display
/// rate: the window is a promise about the *least* history kept, and a few
/// hundred spare rows cost a few tens of kilobytes.
const ASSUMED_TICK_RATE: f32 = 144.0;

/// Column header for recording CSVs. Shared with the replay parser so the
/// two cannot drift apart.
pub const RECORDING_HEADER: &str = "dt,suspended,pose,\
    pelvis_x,pelvis_y,pelvis_z,vel_x,vel_y,vel_z,intent_x,intent_y,intent_z,\
    yaw,yaw_rate,hip_width,leg_length,standing_height,foot_y_fallback,\
    step_height,stride_gain,\
    lgn_x,lgn_y,lgn_z,rgn_x,rgn_y,rgn_z,\
    lg_valid,lg_x,lg_y,lg_z,rg_valid,rg_x,rg_y,rg_z,\
    out_l_x,out_l_y,out_l_z,out_l_planted,\
    out_r_x,out_r_y,out_r_z,out_r_planted,out_gait_phase";

/// One tick, held in memory until the recording is written.
#[derive(Clone, Copy)]
struct RecordedTick {
    dt: f32,
    suspended: bool,
    pose: &'static str,
    pelvis: Point3<f32>,
    velocity: Vector3<f32>,
    intent: Vector3<f32>,
    yaw: f32,
    yaw_rate: f32,
    hip_width: f32,
    leg_length: f32,
    standing_height: f32,
    foot_y_fallback: f32,
    step_height: f32,
    stride_gain: f32,
    left_normal: Vector3<f32>,
    right_normal: Vector3<f32>,
    left_ground: Option<Point3<f32>>,
    right_ground: Option<Point3<f32>>,
    left_out: Point3<f32>,
    left_planted: bool,
    right_out: Point3<f32>,
    right_planted: bool,
    gait_phase: f32,
}

/// The `FootPlacer::new` arguments, replayed from the `# init` line.
#[derive(Clone, Copy)]
struct Init {
    pelvis: Point3<f32>,
    yaw: f32,
    hip_width: f32,
    foot_y: f32,
}

/// Keeps the last `window` seconds of placer inputs and outputs in memory,
/// and writes them out on demand.
pub struct PlacerRecorder {
    path: String,
    init: Init,
    ticks: VecDeque<RecordedTick>,
    capacity: usize,
    /// Whether ticks are being kept. Off until asked for: a recorder that
    /// starts with the process records the walk to the artefact and charges
    /// the whole session for it.
    armed: bool,
    /// Wall-clock nanoseconds spent inside [`PlacerRecorder::record`] since
    /// arming, and the number of ticks that took. Reported on screen, so a
    /// claim that recording costs frames can be checked rather than argued.
    spent_nanos: u128,
    recorded: u64,
}

impl PlacerRecorder {
    /// Create a recorder when [`RECORDER_ENV_VAR`] is set. Returns `None`
    /// (recording disabled) when the variable is unset.
    ///
    /// The path is only checked here, by creating its parent directory: a
    /// recording that cannot be written is worth knowing about at startup
    /// rather than after the run it was meant to capture.
    pub fn from_env(
        init_pelvis: Point3<f32>,
        init_yaw: f32,
        hip_width: f32,
        init_foot_y: f32,
    ) -> Option<Self> {
        let path = env::var(RECORDER_ENV_VAR).ok()?;
        let window = env::var(WINDOW_ENV_VAR)
            .ok()
            .and_then(|text| text.parse::<f32>().ok())
            .unwrap_or(DEFAULT_WINDOW_SECONDS);

        if let Some(parent) = Path::new(&path).parent() {
            if !parent.as_os_str().is_empty() {
                if let Err(e) = fs::create_dir_all(parent) {
                    eprintln!("PlacerRecorder: cannot create {}: {e}", parent.display());
                    return None;
                }
            }
        }
        eprintln!(
            "PlacerRecorder: press F4 to start keeping the last {window:.0} s \
             of placer input, F4 again to write {path}"
        );
        let mut recorder = Self::new(path, window, init_pelvis, init_yaw, hip_width, init_foot_y);
        recorder.set_armed(false);
        Some(recorder)
    }

    /// A recorder writing to `path`, keeping `window` seconds of history.
    /// Armed: constructing one directly is a decision to record. The in-game
    /// recorder starts disarmed, because there the decision is a keypress.
    pub fn new(
        path: impl Into<String>,
        window: f32,
        init_pelvis: Point3<f32>,
        init_yaw: f32,
        hip_width: f32,
        init_foot_y: f32,
    ) -> Self {
        let capacity = ((window.max(0.1) * ASSUMED_TICK_RATE) as usize).max(1);
        Self {
            path: path.into(),
            init: Init {
                pelvis: init_pelvis,
                yaw: init_yaw,
                hip_width,
                foot_y: init_foot_y,
            },
            ticks: VecDeque::with_capacity(capacity),
            capacity,
            armed: true,
            spent_nanos: 0,
            recorded: 0,
        }
    }

    /// Start or stop keeping ticks. Arming clears the ring and the cost
    /// counters, so each recording is its own measurement.
    pub fn set_armed(&mut self, armed: bool) {
        if armed {
            self.ticks.clear();
            self.spent_nanos = 0;
            self.recorded = 0;
        }
        self.armed = armed;
    }

    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// One line for the debug overlay: what the recorder is doing, and what
    /// it costs per frame.
    pub fn status(&self) -> String {
        if !self.armed {
            return format!("idle (F4 to record {})", self.path);
        }
        let micros_per_tick = if self.recorded == 0 {
            0.0
        } else {
            self.spent_nanos as f64 / self.recorded as f64 / 1000.0
        };
        format!(
            "recording {:.1} s, {} ticks, {micros_per_tick:.1} us/tick",
            self.recorded_seconds(),
            self.recorded
        )
    }

    /// Append one tick, dropping the oldest once the window is full. Call
    /// after `FootPlacer::tick` with the same `ctx`, so the output columns
    /// hold this tick's results. `pose` is a short human-readable tag (must
    /// not contain commas); `suspended` is the flag passed to `set_suspended`
    /// this tick.
    pub fn record(
        &mut self,
        suspended: bool,
        pose: &'static str,
        ctx: &PlacerCtx<'_>,
        placer: &FootPlacer,
    ) {
        if !self.armed {
            return;
        }
        let started = Instant::now();
        if self.ticks.len() == self.capacity {
            self.ticks.pop_front();
        }
        self.ticks.push_back(RecordedTick {
            dt: ctx.dt,
            suspended,
            pose,
            pelvis: ctx.pelvis,
            velocity: ctx.velocity,
            intent: ctx.intent_direction,
            yaw: ctx.yaw,
            yaw_rate: ctx.yaw_rate,
            hip_width: ctx.hip_width,
            leg_length: ctx.leg_length,
            standing_height: ctx.standing_height,
            foot_y_fallback: ctx.foot_y_fallback,
            step_height: ctx.step_height,
            stride_gain: ctx.stride_gain,
            left_normal: ctx.left_ground_normal,
            right_normal: ctx.right_ground_normal,
            left_ground: ctx.left_ground,
            right_ground: ctx.right_ground,
            left_out: placer.left.position,
            left_planted: placer.left.is_planted(),
            right_out: placer.right.position,
            right_planted: placer.right.is_planted(),
            gait_phase: placer.gait_phase(),
        });
        self.spent_nanos += started.elapsed().as_nanos();
        self.recorded += 1;
    }

    /// Seconds of play currently held.
    pub fn recorded_seconds(&self) -> f32 {
        self.ticks.iter().map(|tick| tick.dt).sum()
    }

    /// Write everything held to the recorder's path. The ring is left alone,
    /// so pressing the key twice writes two supersets rather than a fragment.
    pub fn dump(&self) -> std::io::Result<usize> {
        let mut out = String::with_capacity(self.ticks.len() * 400);
        let _ = writeln!(
            out,
            "# init,{},{},{},{},{},{}",
            self.init.pelvis.x,
            self.init.pelvis.y,
            self.init.pelvis.z,
            self.init.yaw,
            self.init.hip_width,
            self.init.foot_y
        );
        let _ = writeln!(out, "{RECORDING_HEADER}");
        for tick in &self.ticks {
            write_tick(&mut out, tick);
        }
        fs::write(&self.path, out)?;
        Ok(self.ticks.len())
    }

    /// Write the recording and say so, for callers with nowhere to put an
    /// error. Used by the debug key and on shutdown.
    pub fn dump_and_report(&self) {
        match self.dump() {
            Ok(rows) => eprintln!(
                "PlacerRecorder: wrote {rows} ticks ({:.1} s) to {}",
                self.recorded_seconds(),
                self.path
            ),
            Err(e) => eprintln!("PlacerRecorder: cannot write {}: {e}", self.path),
        }
    }
}

/// A recording that was never asked for is still worth having: a run that ends
/// in a crash or a quit is exactly when nobody remembered to press the key.
impl Drop for PlacerRecorder {
    fn drop(&mut self) {
        if !self.ticks.is_empty() {
            self.dump_and_report();
        }
    }
}

fn write_tick(out: &mut String, tick: &RecordedTick) {
    let _ = write!(
        out,
        "{},{},{}",
        tick.dt,
        u8::from(tick.suspended),
        tick.pose
    );
    write_point(out, tick.pelvis);
    write_vector(out, tick.velocity);
    write_vector(out, tick.intent);
    let _ = write!(
        out,
        ",{},{},{},{},{},{},{},{}",
        tick.yaw,
        tick.yaw_rate,
        tick.hip_width,
        tick.leg_length,
        tick.standing_height,
        tick.foot_y_fallback,
        tick.step_height,
        tick.stride_gain
    );
    write_vector(out, tick.left_normal);
    write_vector(out, tick.right_normal);
    write_opt_point(out, tick.left_ground);
    write_opt_point(out, tick.right_ground);
    write_point(out, tick.left_out);
    let _ = write!(out, ",{}", u8::from(tick.left_planted));
    write_point(out, tick.right_out);
    let _ = writeln!(out, ",{},{}", u8::from(tick.right_planted), tick.gait_phase);
}

fn write_point(out: &mut String, p: Point3<f32>) {
    let _ = write!(out, ",{},{},{}", p.x, p.y, p.z);
}

fn write_vector(out: &mut String, v: Vector3<f32>) {
    let _ = write!(out, ",{},{},{}", v.x, v.y, v.z);
}

/// `Option<Point3>` encodes as a validity flag plus three coordinates
/// (zeros when absent).
fn write_opt_point(out: &mut String, p: Option<Point3<f32>>) {
    match p {
        Some(p) => {
            let _ = write!(out, ",1,{},{},{}", p.x, p.y, p.z);
        }
        None => {
            let _ = write!(out, ",0,0,0,0");
        }
    }
}
