//! In-game recorder for the foot placer's input stream.
//!
//! Captures every `PlacerCtx` field (plus the suspend flag and the active
//! pose tag) once per tick, alongside the placer's outputs for the same
//! tick. The `replay` test module feeds a recording back through a fresh
//! `FootPlacer`, reproducing in-game behaviour deterministically offline —
//! the bridge between "looks wrong in game" and a debuggable trace.
//!
//! Enable by launching the game with the output path in `PLACER_REC`:
//!
//! ```bash
//! PLACER_REC=scratch/recordings/run_ne.csv cargo run
//! ```
//!
//! Float fields are written with `Display`, which round-trips `f32`
//! exactly, so a replay sees bit-identical inputs.

use std::env;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use nalgebra::{Point3, Vector3};

use super::placer::{FootPlacer, PlacerCtx};

/// Environment variable holding the recording output path.
pub const RECORDER_ENV_VAR: &str = "PLACER_REC";

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

/// Streams placer inputs and outputs to a CSV file, one row per tick.
pub struct PlacerRecorder {
    writer: BufWriter<File>,
    rows_since_flush: u32,
}

/// Flush cadence in rows, so a crash loses at most ~1 s of recording.
const FLUSH_EVERY: u32 = 30;

impl PlacerRecorder {
    /// Create a recorder when [`RECORDER_ENV_VAR`] is set, writing the
    /// `FootPlacer::new` construction parameters as a `# init` comment
    /// line so a replay can rebuild the exact same initial state. Returns
    /// `None` (recording disabled) when the variable is unset; logs and
    /// returns `None` if the file cannot be created.
    pub fn from_env(
        init_pelvis: Point3<f32>,
        init_yaw: f32,
        hip_width: f32,
        init_foot_y: f32,
    ) -> Option<Self> {
        let path = env::var(RECORDER_ENV_VAR).ok()?;
        match Self::create(&path, init_pelvis, init_yaw, hip_width, init_foot_y) {
            Ok(recorder) => {
                eprintln!("PlacerRecorder: recording placer inputs to {path}");
                Some(recorder)
            }
            Err(e) => {
                eprintln!("PlacerRecorder: cannot create {path}: {e}");
                None
            }
        }
    }

    /// Open a recording at `path`, creating parent directories as needed.
    pub fn create(
        path: &str,
        init_pelvis: Point3<f32>,
        init_yaw: f32,
        hip_width: f32,
        init_foot_y: f32,
    ) -> std::io::Result<Self> {
        if let Some(parent) = Path::new(path).parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut writer = BufWriter::new(File::create(path)?);
        writeln!(
            writer,
            "# init,{},{},{},{},{},{}",
            init_pelvis.x, init_pelvis.y, init_pelvis.z, init_yaw, hip_width, init_foot_y
        )?;
        writeln!(writer, "{RECORDING_HEADER}")?;
        Ok(Self {
            writer,
            rows_since_flush: 0,
        })
    }

    /// Append one tick. Call after `FootPlacer::tick` with the same `ctx`,
    /// so the output columns hold this tick's results. `pose` is a short
    /// human-readable tag (must not contain commas); `suspended` is the
    /// flag passed to `set_suspended` this tick.
    pub fn record(
        &mut self,
        suspended: bool,
        pose: &str,
        ctx: &PlacerCtx<'_>,
        placer: &FootPlacer,
    ) {
        let w = &mut self.writer;
        let _ = write!(w, "{},{},{pose}", ctx.dt, u8::from(suspended));
        write_point(w, ctx.pelvis);
        write_vector(w, ctx.velocity);
        write_vector(w, ctx.intent_direction);
        let _ = write!(
            w,
            ",{},{},{},{},{},{},{},{}",
            ctx.yaw,
            ctx.yaw_rate,
            ctx.hip_width,
            ctx.leg_length,
            ctx.standing_height,
            ctx.foot_y_fallback,
            ctx.step_height,
            ctx.stride_gain
        );
        write_vector(w, ctx.left_ground_normal);
        write_vector(w, ctx.right_ground_normal);
        write_opt_point(w, ctx.left_ground);
        write_opt_point(w, ctx.right_ground);
        write_point(w, placer.left.position);
        let _ = write!(w, ",{}", u8::from(placer.left.is_planted()));
        write_point(w, placer.right.position);
        let _ = writeln!(
            w,
            ",{},{}",
            u8::from(placer.right.is_planted()),
            placer.gait_phase()
        );

        self.rows_since_flush += 1;
        if self.rows_since_flush >= FLUSH_EVERY {
            self.rows_since_flush = 0;
            let _ = w.flush();
        }
    }
}

impl Drop for PlacerRecorder {
    fn drop(&mut self) {
        let _ = self.writer.flush();
    }
}

fn write_point(w: &mut impl Write, p: Point3<f32>) {
    let _ = write!(w, ",{},{},{}", p.x, p.y, p.z);
}

fn write_vector(w: &mut impl Write, v: Vector3<f32>) {
    let _ = write!(w, ",{},{},{}", v.x, v.y, v.z);
}

/// `Option<Point3>` encodes as a validity flag plus three coordinates
/// (zeros when absent).
fn write_opt_point(w: &mut impl Write, p: Option<Point3<f32>>) {
    match p {
        Some(p) => {
            let _ = write!(w, ",1,{},{},{}", p.x, p.y, p.z);
        }
        None => {
            let _ = write!(w, ",0,0,0,0");
        }
    }
}
