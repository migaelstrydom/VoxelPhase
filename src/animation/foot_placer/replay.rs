//! Replay an in-game `PlacerRecorder` recording through a fresh placer.
//!
//! ```bash
//! PLACER_REC=/tmp/run_ne.csv cargo run                          # play, then F4
//! PLACER_REC_CSV=/tmp/run_ne.csv \
//!     cargo test --lib foot_placer::replay -- --ignored --nocapture
//! ```
//!
//! The replay rebuilds the placer from the recorded `# init` line, feeds
//! it the exact recorded input stream, and:
//! - reports the max divergence between in-game and replayed foot
//!   positions (should be ~0: nonzero means the recording misses an
//!   input, i.e. the placer has hidden state),
//! - writes `scratch/foot_traces/replay_<name>.csv` in the same format
//!   as `trace::export_csv`, so `scratch/plot_traces.py` plots it.
//!
//! Test-only module (`#[cfg(test)]` in `mod.rs`).

use std::fmt::Write as _;

use nalgebra::{Point3, Vector3};

use super::config::FootPlacerConfig;
use super::placer::{facing_from_yaw, planar_distance, FootPlacer, PlacerCtx};

/// `FootPlacer::new` parameters captured by the recorder's `# init` line.
struct InitLine {
    pelvis: Point3<f32>,
    yaw: f32,
    hip_width: f32,
    foot_y: f32,
}

/// One recorded tick: everything `PlacerCtx` holds (minus the config,
/// which is assumed default), plus the suspend flag and the in-game
/// outputs for divergence checking.
struct RecFrame {
    dt: f32,
    suspended: bool,
    pose: String,
    pelvis: Point3<f32>,
    velocity: Vector3<f32>,
    intent_direction: Vector3<f32>,
    yaw: f32,
    yaw_rate: f32,
    hip_width: f32,
    leg_length: f32,
    standing_height: f32,
    foot_y_fallback: f32,
    step_height: f32,
    stride_gain: f32,
    left_ground_normal: Vector3<f32>,
    right_ground_normal: Vector3<f32>,
    left_ground: Option<Point3<f32>>,
    right_ground: Option<Point3<f32>>,
    out_left: Point3<f32>,
    out_right: Point3<f32>,
}

/// Sequential field reader over one CSV row.
struct Fields<'a> {
    iter: std::str::Split<'a, char>,
    line: usize,
}

impl<'a> Fields<'a> {
    fn new(row: &'a str, line: usize) -> Self {
        Self {
            iter: row.split(','),
            line,
        }
    }

    fn str(&mut self) -> &'a str {
        self.iter
            .next()
            .unwrap_or_else(|| panic!("line {}: row too short", self.line))
    }

    fn f32(&mut self) -> f32 {
        let s = self.str();
        s.parse()
            .unwrap_or_else(|e| panic!("line {}: bad float {s:?}: {e}", self.line))
    }

    fn flag(&mut self) -> bool {
        self.str() == "1"
    }

    fn point(&mut self) -> Point3<f32> {
        Point3::new(self.f32(), self.f32(), self.f32())
    }

    fn vector(&mut self) -> Vector3<f32> {
        Vector3::new(self.f32(), self.f32(), self.f32())
    }

    fn opt_point(&mut self) -> Option<Point3<f32>> {
        let valid = self.flag();
        let p = self.point();
        valid.then_some(p)
    }
}

fn parse_recording(text: &str) -> (InitLine, Vec<RecFrame>) {
    let mut lines = text.lines().enumerate();
    let (_, init_line) = lines.next().expect("empty recording");
    let init_body = init_line
        .strip_prefix("# init,")
        .expect("first line must be the `# init,...` comment");
    let mut f = Fields::new(init_body, 1);
    let init = InitLine {
        pelvis: f.point(),
        yaw: f.f32(),
        hip_width: f.f32(),
        foot_y: f.f32(),
    };
    let (_, header) = lines.next().expect("missing header line");
    assert!(
        header.starts_with("dt,suspended,pose"),
        "unexpected header: {header}"
    );

    let mut frames = Vec::new();
    for (i, row) in lines {
        if row.trim().is_empty() {
            continue;
        }
        let mut f = Fields::new(row, i + 1);
        frames.push(RecFrame {
            dt: f.f32(),
            suspended: f.flag(),
            pose: f.str().to_owned(),
            pelvis: f.point(),
            velocity: f.vector(),
            intent_direction: f.vector(),
            yaw: f.f32(),
            yaw_rate: f.f32(),
            hip_width: f.f32(),
            leg_length: f.f32(),
            standing_height: f.f32(),
            foot_y_fallback: f.f32(),
            step_height: f.f32(),
            stride_gain: f.f32(),
            left_ground_normal: f.vector(),
            right_ground_normal: f.vector(),
            left_ground: f.opt_point(),
            right_ground: f.opt_point(),
            out_left: {
                let p = f.point();
                f.flag();
                p
            },
            out_right: {
                let p = f.point();
                f.flag();
                p
            },
        });
    }
    (init, frames)
}

/// Feed the recorded input stream through a fresh placer. Calls
/// `per_frame` after each tick and returns the max planar divergence
/// between replayed and recorded foot positions, with its frame index.
fn replay_frames(
    init: &InitLine,
    frames: &[RecFrame],
    config: &FootPlacerConfig,
    mut per_frame: impl FnMut(usize, &FootPlacer),
) -> (f32, usize) {
    let mut placer = FootPlacer::new(
        init.pelvis,
        facing_from_yaw(init.yaw),
        init.hip_width,
        init.foot_y,
    );
    let mut max_div = 0.0f32;
    let mut max_div_frame = 0usize;
    for (i, fr) in frames.iter().enumerate() {
        placer.set_suspended(fr.suspended);
        let ctx = PlacerCtx {
            dt: fr.dt,
            pelvis: fr.pelvis,
            velocity: fr.velocity,
            // Recordings predate moving supports; a replayed frame stands on
            // ground that is going nowhere.
            support_velocity: Vector3::zeros(),
            intent_direction: fr.intent_direction,
            yaw: fr.yaw,
            yaw_rate: fr.yaw_rate,
            hip_width: fr.hip_width,
            leg_length: fr.leg_length,
            standing_height: fr.standing_height,
            foot_y_fallback: fr.foot_y_fallback,
            step_height: fr.step_height,
            stride_gain: fr.stride_gain,
            left_ground_normal: fr.left_ground_normal,
            right_ground_normal: fr.right_ground_normal,
            left_ground: fr.left_ground,
            right_ground: fr.right_ground,
            config,
        };
        placer.tick(&ctx);

        let div = planar_distance(placer.left.position, fr.out_left)
            .max(planar_distance(placer.right.position, fr.out_right));
        if div > max_div {
            max_div = div;
            max_div_frame = i;
        }
        per_frame(i, &placer);
    }
    (max_div, max_div_frame)
}

/// Replay the recording at `PLACER_REC_CSV` and export a plot-ready trace.
///
/// Assumes the game ran with the default `FootPlacerConfig`; if you have
/// tuned it, mirror the changes here before trusting the divergence number.
#[test]
#[ignore]
fn replay_recording() {
    let path = std::env::var("PLACER_REC_CSV")
        .expect("set PLACER_REC_CSV to the recording produced via PLACER_REC in game");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let (init, frames) = parse_recording(&text);
    assert!(!frames.is_empty(), "recording has no data rows");
    println!("replaying {} frames from {path}", frames.len());

    let config = FootPlacerConfig::default();

    let mut csv = String::new();
    csv.push_str(
        "time,pelvis_x,pelvis_y,pelvis_z,vel_x,vel_z,yaw,gait_phase,\
         l_step,l_x,l_y,l_z,l_terrain,l_planted_x,l_planted_y,l_planted_z,l_ideal_x,l_ideal_z,\
         r_step,r_x,r_y,r_z,r_terrain,r_planted_x,r_planted_y,r_planted_z,r_ideal_x,r_ideal_z\n",
    );

    let mut time = 0.0f32;
    let (max_div, max_div_frame) = replay_frames(&init, &frames, &config, |i, placer| {
        let fr = &frames[i];
        time += fr.dt;
        let l = &placer.left;
        let r = &placer.right;
        // Terrain-under-foot columns come from the recorded probe
        // contacts; NaN (a plot gap) when the probe missed.
        let l_terrain = fr.left_ground.map_or(f32::NAN, |g| g.y);
        let r_terrain = fr.right_ground.map_or(f32::NAN, |g| g.y);
        writeln!(
            csv,
            "{},{},{},{},{},{},{},{},\
             {},{},{},{},{},{},{},{},{},{},\
             {},{},{},{},{},{},{},{},{},{}",
            time,
            fr.pelvis.x,
            fr.pelvis.y,
            fr.pelvis.z,
            fr.velocity.x,
            fr.velocity.z,
            fr.yaw,
            placer.gait_phase(),
            u8::from(l.is_stepping()),
            l.position.x,
            l.position.y,
            l.position.z,
            l_terrain,
            l.planted_position.x,
            l.planted_position.y,
            l.planted_position.z,
            l.ideal_xz.x,
            l.ideal_xz.z,
            u8::from(r.is_stepping()),
            r.position.x,
            r.position.y,
            r.position.z,
            r_terrain,
            r.planted_position.x,
            r.planted_position.y,
            r.planted_position.z,
            r.ideal_xz.x,
            r.ideal_xz.z,
        )
        .unwrap();
    });

    let suspended_frames = frames.iter().filter(|fr| fr.suspended).count();
    let name = std::path::Path::new(&path)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("recording");
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/scratch/foot_traces");
    std::fs::create_dir_all(dir).unwrap();
    let out = format!("{dir}/replay_{name}.csv");
    std::fs::write(&out, csv).unwrap();

    let pose_counts = pose_histogram(&frames);
    println!("wrote {out}");
    println!("duration: {time:.2} s, suspended frames: {suspended_frames}");
    println!("poses: {pose_counts}");
    println!(
        "max divergence vs in-game outputs: {max_div:.6} m at frame {max_div_frame} \
         (>1e-3 means the recording misses an input)"
    );
}

/// End-to-end proof of the record→parse→replay loop without the game:
/// drive a placer with synthetic inputs while recording through the real
/// `PlacerRecorder`, then parse the file back and replay it. The replay
/// must reproduce the original foot positions *bit-exactly* — `Display`
/// round-trips `f32` — so any divergence means the recording format
/// misses an input the placer consumed.
#[test]
fn record_replay_round_trip() {
    use super::recorder::PlacerRecorder;

    const HIP_WIDTH: f32 = 0.12;
    const LEG_LENGTH: f32 = 0.5;
    const STANDING_HEIGHT: f32 = 0.425;
    const DT: f32 = 1.0 / 30.0;

    let init_pelvis = Point3::new(0.3, STANDING_HEIGHT, -0.2);
    let path = std::env::temp_dir().join("voxel_phase_placer_roundtrip.csv");
    let path = path.to_str().unwrap().to_owned();

    let config = FootPlacerConfig::default();
    let mut placer = FootPlacer::new(init_pelvis, facing_from_yaw(0.0), HIP_WIDTH, 0.0);
    // A window comfortably longer than the run, so the ring keeps all of it:
    // a round trip that silently dropped its first frames would prove nothing.
    let mut recorder = PlacerRecorder::new(&path, 60.0, init_pelvis, 0.0, HIP_WIDTH, 0.0);

    // Accelerating straight run with a gentle weave, a mid-run suspension
    // window (probes lost), and per-frame dt wobble — exercises the
    // optional fields, the suspend flag and float round-tripping.
    let mut pelvis = init_pelvis;
    for i in 0..240usize {
        let t = i as f32 * DT;
        let dt = DT + (i % 7) as f32 * 1.3e-4;
        let speed = (4.0 * t).min(3.0);
        let velocity = Vector3::new(0.4 * (1.7 * t).sin(), 0.0, speed);
        pelvis += velocity * dt;
        let suspended = (100..106).contains(&i);

        let contact =
            |anchor: Point3<f32>| (!suspended).then_some(Point3::new(anchor.x, 0.0, anchor.z));
        let left_ground = contact(placer.left.probe_anchor());
        let right_ground = contact(placer.right.probe_anchor());

        placer.set_suspended(suspended);
        let ctx = PlacerCtx {
            dt,
            pelvis,
            velocity,
            support_velocity: Vector3::zeros(),
            intent_direction: Vector3::new(0.0, 0.0, 1.0),
            yaw: 0.1 * (1.7 * t).sin(),
            yaw_rate: 0.17 * (1.7 * t).cos(),
            hip_width: HIP_WIDTH,
            leg_length: LEG_LENGTH,
            standing_height: STANDING_HEIGHT,
            foot_y_fallback: pelvis.y - STANDING_HEIGHT,
            step_height: 0.15,
            stride_gain: 0.4,
            left_ground_normal: Vector3::y(),
            right_ground_normal: Vector3::y(),
            left_ground,
            right_ground,
            config: &config,
        };
        placer.tick(&ctx);
        recorder.record(suspended, "walk", &ctx, &placer);
    }
    drop(recorder);

    let text = std::fs::read_to_string(&path).unwrap();
    let (init, frames) = parse_recording(&text);
    assert_eq!(frames.len(), 240);
    let (max_div, frame) = replay_frames(&init, &frames, &config, |_, _| {});
    assert_eq!(
        max_div, 0.0,
        "replay diverged {max_div} m at frame {frame}: recording misses a placer input"
    );
    let _ = std::fs::remove_file(&path);
}

/// A disarmed recorder keeps nothing, and the ring keeps the *last* window of
/// play, which is the whole point of it:
/// the artefact is at the end of the run, and the walk to it is not worth
/// writing to disk sixty times a second on the chance that it might be.
#[test]
fn a_recording_holds_the_last_window_and_forgets_the_rest() {
    use super::recorder::PlacerRecorder;

    const HIP_WIDTH: f32 = 0.12;
    let dt = 1.0 / 60.0;
    let pelvis = Point3::new(0.0, 0.425, 0.0);
    let path = std::env::temp_dir().join("voxel_phase_placer_window.csv");
    let path = path.to_str().unwrap().to_owned();

    let config = FootPlacerConfig::default();
    let mut placer = FootPlacer::new(pelvis, facing_from_yaw(0.0), HIP_WIDTH, 0.0);
    // One second of window against ten seconds of play.
    let mut recorder = PlacerRecorder::new(&path, 1.0, pelvis, 0.0, HIP_WIDTH, 0.0);
    // The first second is played with the recorder offered but not armed,
    // which is how the game starts.
    recorder.set_armed(false);

    let mut walked = pelvis;
    for i in 0..600usize {
        walked.z += dt;
        let ctx = PlacerCtx {
            dt,
            pelvis: walked,
            velocity: Vector3::new(0.0, 0.0, 1.0),
            support_velocity: Vector3::zeros(),
            intent_direction: Vector3::new(0.0, 0.0, 1.0),
            yaw: 0.0,
            yaw_rate: 0.0,
            hip_width: HIP_WIDTH,
            leg_length: 0.5,
            standing_height: 0.425,
            foot_y_fallback: 0.0,
            step_height: 0.15,
            stride_gain: 0.4,
            left_ground_normal: Vector3::y(),
            right_ground_normal: Vector3::y(),
            left_ground: Some(Point3::new(walked.x, 0.0, walked.z)),
            right_ground: Some(Point3::new(walked.x, 0.0, walked.z)),
            config: &config,
        };
        placer.tick(&ctx);
        recorder.record(false, "walk", &ctx, &placer);
        if i == 59 {
            assert_eq!(
                recorder.recorded_seconds(),
                0.0,
                "a disarmed recorder kept ticks"
            );
            recorder.set_armed(true);
        }
    }

    // The window is a promise about the *least* history kept — the ring is
    // sized in ticks against a rate no display exceeds — so a 60 Hz run keeps
    // more than the second asked for, and nothing like the ten it ran for.
    let seconds = recorder.recorded_seconds();
    assert!(
        (1.0..4.0).contains(&seconds),
        "ten seconds of play left {seconds:.1} s in a one-second window"
    );
    recorder.dump().expect("the recording must write");

    let text = std::fs::read_to_string(&path).unwrap();
    let (_, frames) = parse_recording(&text);
    assert!(!frames.is_empty(), "the window kept nothing");
    let last = frames.last().unwrap();
    assert!(
        (last.pelvis.z - walked.z).abs() < 1e-4,
        "the window must end at the last tick, not the first"
    );
    let _ = std::fs::remove_file(&path);
}

fn pose_histogram(frames: &[RecFrame]) -> String {
    let mut counts: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
    for fr in frames {
        *counts.entry(fr.pose.as_str()).or_default() += 1;
    }
    counts
        .iter()
        .map(|(k, v)| format!("{k}={v}"))
        .collect::<Vec<_>>()
        .join(" ")
}
