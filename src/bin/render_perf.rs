//! Rendering performance bench: what a frame of the real game costs while a
//! grenade goes off.
//!
//! Runs the game headlessly — the real level, systems, spawnables and
//! renderer, drawing into an offscreen image — with a fixed camera, drops a
//! grenade at a site, and reports each frame's simulate and render wall clock,
//! the renderer's CPU stages, its GPU spans from timestamp queries, and what
//! it drew. The quiet stretch before the drop is the baseline the blast is
//! compared against.
//!
//! ```text
//! cargo run --release --bin render_perf
//! cargo run --release --bin render_perf -- --timeline --csv /tmp/render.csv
//! cargo run --release --bin render_perf -- --snapshot /tmp/shot.png   # check the framing
//! cargo run --release --bin render_perf -- --eye 12,6,17 --look 12,1,29
//! ```
//!
//! Always measure a release build. There is no vsync offscreen, so the fence
//! wait is time the CPU spent on a GPU that was still busy.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use nalgebra::Point3;

use voxel_phase::level::load_level;
use voxel_phase::render_perf::{
    run, summary, timeline, worst_frames, write_csv, GrenadeBlast, RunConfig, DEFAULT_WINDOW,
};

const USAGE: &str = "usage: render_perf [--level <level.ron>] [--at X,Z] [--drop-at S]
                   [--eye X,Y,Z] [--look X,Y,Z] [--width W] [--height H]
                   [--seconds S] [--fps F] [--worst N] [--timeline] [--window S]
                   [--csv <out.csv>] [--snapshot <out.png>]";

const DEFAULT_LEVEL: &str = "levels/test_arena.level.ron";
const DEFAULT_WORST: usize = 8;

/// Parsed command line.
struct Args {
    level: PathBuf,
    scenario: GrenadeBlast,
    config: RunConfig,
    /// How many of the slowest frames to list.
    worst: usize,
    /// Print the per-window timeline as well as the summary.
    timeline: bool,
    window: f32,
    /// Per-frame CSV.
    csv: Option<PathBuf>,
    /// Image of the last frame, for checking what the camera saw.
    snapshot: Option<PathBuf>,
}

fn main() -> ExitCode {
    env_logger::init();
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(message) => {
            eprintln!("{message}\n{USAGE}");
            return ExitCode::FAILURE;
        }
    };

    match execute(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("error: {message}");
            ExitCode::FAILURE
        }
    }
}

fn execute(args: &Args) -> Result<(), String> {
    if cfg!(debug_assertions) {
        eprintln!("warning: debug build — timings are not representative; use --release\n");
    }
    let level =
        load_level(&args.level).map_err(|e| format!("loading {}: {e}", args.level.display()))?;
    let level_name = level_name(&args.level);

    let (result, image) = run(
        &level,
        &level_name,
        &args.scenario,
        args.config,
        args.snapshot.is_some(),
    )
    .map_err(|e| e.to_string())?;

    println!("{}", summary(&result));
    println!("{}", worst_frames(&result, args.worst));
    if args.timeline {
        println!("{}", timeline(&result, args.window));
    }
    if let Some(path) = &args.csv {
        write_csv(&result, path).map_err(|e| format!("writing {}: {e}", path.display()))?;
        println!("wrote {}", path.display());
    }
    if let (Some(path), Some(image)) = (&args.snapshot, image) {
        image
            .save(path)
            .map_err(|e| format!("writing {}: {e}", path.display()))?;
        println!("wrote {}", path.display());
    }
    Ok(())
}

fn level_name(path: &Path) -> String {
    path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

fn parse_args(args: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut parsed = Args {
        level: PathBuf::from(DEFAULT_LEVEL),
        scenario: GrenadeBlast::default(),
        config: RunConfig::default(),
        worst: DEFAULT_WORST,
        timeline: false,
        window: DEFAULT_WINDOW,
        csv: None,
        snapshot: None,
    };
    let mut args = args.peekable();

    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or(format!("{name} needs a value"));
        match arg.as_str() {
            "--level" => parsed.level = PathBuf::from(value("--level")?),
            "--at" => parsed.scenario.site = parse_pair(&value("--at")?)?,
            "--drop-at" => parsed.scenario.drop_at = parse_number(&value("--drop-at")?)?,
            "--eye" => parsed.scenario.eye = Some(parse_point(&value("--eye")?)?),
            "--look" => parsed.scenario.look = Some(parse_point(&value("--look")?)?),
            "--width" => parsed.config.width = parse_number(&value("--width")?)?,
            "--height" => parsed.config.height = parse_number(&value("--height")?)?,
            "--seconds" => parsed.config.duration = parse_number(&value("--seconds")?)?,
            "--fps" => parsed.config.frame_dt = 1.0 / parse_number::<f32>(&value("--fps")?)?,
            "--worst" => parsed.worst = parse_number(&value("--worst")?)?,
            "--timeline" => parsed.timeline = true,
            "--window" => parsed.window = parse_number(&value("--window")?)?,
            "--csv" => parsed.csv = Some(PathBuf::from(value("--csv")?)),
            "--snapshot" => parsed.snapshot = Some(PathBuf::from(value("--snapshot")?)),
            "-h" | "--help" => return Err(USAGE.to_string()),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(parsed)
}

fn parse_number<T: std::str::FromStr>(text: &str) -> Result<T, String> {
    text.trim()
        .parse()
        .map_err(|_| format!("not a number: {text}"))
}

fn parse_pair(text: &str) -> Result<(f32, f32), String> {
    let (x, z) = text
        .split_once(',')
        .ok_or(format!("expected X,Z, got {text}"))?;
    Ok((parse_number(x)?, parse_number(z)?))
}

fn parse_point(text: &str) -> Result<Point3<f32>, String> {
    let parts: Vec<&str> = text.split(',').collect();
    match parts.as_slice() {
        [x, y, z] => Ok(Point3::new(
            parse_number(x)?,
            parse_number(y)?,
            parse_number(z)?,
        )),
        _ => Err(format!("expected X,Y,Z, got {text}")),
    }
}
