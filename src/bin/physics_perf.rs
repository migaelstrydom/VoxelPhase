//! Physics performance bench: where a frame's physics time goes.
//!
//! Builds a scenario on real level terrain, steps it headlessly at the game's
//! cadence (1/240 s fixed step, 60 Hz frames), and reports the world's own
//! stage-by-stage profile: for the whole run, over time, and — with several
//! dome counts — as the body count grows.
//!
//! ```text
//! cargo run --release --bin physics_perf
//! cargo run --release --bin physics_perf -- --domes 1,2,4,8
//! cargo run --release --bin physics_perf -- --timeline --csv /tmp/blast.csv
//! ```
//!
//! No Vulkan, no window, no ECS. Always measure a release build.

use std::path::PathBuf;
use std::process::ExitCode;

use voxel_phase::physics_perf::{
    run, scaling_table, summary, timeline, write_csv, Ground, IglooBlast, PerfRun, RunConfig,
    DEFAULT_WINDOW,
};

const USAGE: &str = "usage: physics_perf [--domes N[,N…]] [--level <level.ron>] [--at X,Z]
                    [--seconds S] [--repeats R] [--fps F] [--speed M/S] [--seed N]
                    [--timeline] [--window S] [--csv <out.csv>]";

const DEFAULT_LEVEL: &str = "levels/test_arena.level.ron";
/// Where the test arena's own igloo stands.
const DEFAULT_SITE: (f32, f32) = (12.0, 29.0);

/// Parsed command line.
struct Args {
    /// Dome counts to run, one run each.
    domes: Vec<usize>,
    level: PathBuf,
    site: (f32, f32),
    config: RunConfig,
    /// Mean outward speed of a blasted block, in m/s.
    blast_speed: Option<f32>,
    seed: Option<u64>,
    /// Print the per-window timeline as well as the summary.
    timeline: bool,
    window: f32,
    /// Per-frame CSV of the last run.
    csv: Option<PathBuf>,
}

fn main() -> ExitCode {
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
    let ground = Ground::load(&args.level, args.site)?;

    let mut runs: Vec<PerfRun> = Vec::new();
    for &domes in &args.domes {
        let defaults = IglooBlast::default();
        let scenario = IglooBlast {
            domes,
            blast_speed: args.blast_speed.unwrap_or(defaults.blast_speed),
            seed: args.seed.unwrap_or(defaults.seed),
            ..defaults
        };
        let result = run(&scenario, &ground, args.config);
        println!("{}", summary(&result));
        if args.timeline {
            println!("{}", timeline(&result, args.window));
        }
        runs.push(result);
    }

    if runs.len() > 1 {
        println!("{}", scaling_table(&runs, args.window));
    }
    if let (Some(path), Some(last)) = (&args.csv, runs.last()) {
        write_csv(last, path).map_err(|e| format!("writing {}: {e}", path.display()))?;
        println!("wrote {}", path.display());
    }
    Ok(())
}

fn parse_args(args: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut parsed = Args {
        domes: vec![1],
        level: PathBuf::from(DEFAULT_LEVEL),
        site: DEFAULT_SITE,
        config: RunConfig::default(),
        blast_speed: None,
        seed: None,
        timeline: false,
        window: DEFAULT_WINDOW,
        csv: None,
    };
    let mut args = args.peekable();

    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or(format!("{name} needs a value"));
        match arg.as_str() {
            "--domes" => {
                parsed.domes = value("--domes")?
                    .split(',')
                    .map(|n| n.trim().parse().map_err(|_| format!("bad dome count {n}")))
                    .collect::<Result<_, _>>()?;
            }
            "--level" => parsed.level = PathBuf::from(value("--level")?),
            "--at" => parsed.site = parse_pair(&value("--at")?)?,
            "--seconds" => parsed.config.duration = parse_number(&value("--seconds")?)?,
            "--repeats" => parsed.config.repeats = parse_number(&value("--repeats")?)?,
            "--fps" => parsed.config.frame_dt = 1.0 / parse_number::<f32>(&value("--fps")?)?,
            "--speed" => parsed.blast_speed = Some(parse_number(&value("--speed")?)?),
            "--seed" => parsed.seed = Some(parse_number(&value("--seed")?)?),
            "--timeline" => parsed.timeline = true,
            "--window" => parsed.window = parse_number(&value("--window")?)?,
            "--csv" => parsed.csv = Some(PathBuf::from(value("--csv")?)),
            "-h" | "--help" => return Err(USAGE.to_string()),
            other => return Err(format!("unknown argument {other}")),
        }
    }

    if parsed.domes.is_empty() || parsed.domes.contains(&0) {
        return Err("dome counts must be positive".into());
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
