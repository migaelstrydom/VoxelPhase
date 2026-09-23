//! Terrain destruction bench: where a blast's rebuild time goes.
//!
//! Sets off the game's grenade charge at a sweep of surface points across real
//! level terrain, one after another, each followed by the frame's terrain
//! update, and reports the terrain's own stage-by-stage timings: blast by
//! blast, and summarised over the sweep.
//!
//! ```text
//! cargo run --release --bin terrain_perf
//! cargo run --release --bin terrain_perf -- --blasts 40 --spacing 17
//! cargo run --release --bin terrain_perf -- --radius 2.5 --csv /tmp/blasts.csv
//! ```
//!
//! No Vulkan, no window, no ECS, no physics. Always measure a release build.

use std::path::PathBuf;
use std::process::ExitCode;

use voxel_phase::perf::Ground;
use voxel_phase::terrain::BlastConfig;
use voxel_phase::terrain_perf::{blast_table, run, summary, write_csv, BlastSweep, RunConfig};

const USAGE: &str = "usage: terrain_perf [--level <level.ron>] [--blasts N] [--spacing M]
                    [--margin M] [--radius R] [--repeats R] [--csv <out.csv>]";

const DEFAULT_LEVEL: &str = "levels/test_arena.level.ron";
const DEFAULT_REPEATS: usize = 3;

/// Parsed command line.
struct Args {
    level: PathBuf,
    sweep: BlastSweep,
    /// A charge that cuts exactly this radius, instead of the grenade's
    /// budgeted one.
    radius: Option<f32>,
    repeats: usize,
    /// Per-blast CSV.
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
    let ground = Ground::load(&args.level, (0.0, 0.0))?;
    let sites = args.sweep.sites(&ground);
    if sites.is_empty() {
        return Err("the sweep found no surface to blast; try a smaller --margin".into());
    }

    let config = match args.radius {
        Some(radius) => RunConfig {
            blast: BlastConfig::fixed_radius(radius),
            charge: format!("fixed {radius} m crater"),
            repeats: args.repeats,
        },
        None => RunConfig {
            blast: BlastConfig::default(),
            charge: "grenade (BlastConfig::default, budgeted)".into(),
            repeats: args.repeats,
        },
    };

    let result = run(&ground, &sites, &config);
    println!("{}", blast_table(&result));
    println!("{}", summary(&result));

    if let Some(path) = &args.csv {
        write_csv(&result, path).map_err(|e| format!("writing {}: {e}", path.display()))?;
        println!("wrote {}", path.display());
    }
    Ok(())
}

fn parse_args(args: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut parsed = Args {
        level: PathBuf::from(DEFAULT_LEVEL),
        sweep: BlastSweep::default(),
        radius: None,
        repeats: DEFAULT_REPEATS,
        csv: None,
    };
    let mut args = args.peekable();

    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or(format!("{name} needs a value"));
        match arg.as_str() {
            "--level" => parsed.level = PathBuf::from(value("--level")?),
            "--blasts" => parsed.sweep.limit = parse_number(&value("--blasts")?)?,
            "--spacing" => parsed.sweep.spacing = parse_number(&value("--spacing")?)?,
            "--margin" => parsed.sweep.margin = parse_number(&value("--margin")?)?,
            "--radius" => parsed.radius = Some(parse_number(&value("--radius")?)?),
            "--repeats" => parsed.repeats = parse_number(&value("--repeats")?)?,
            "--csv" => parsed.csv = Some(PathBuf::from(value("--csv")?)),
            "-h" | "--help" => return Err(USAGE.to_string()),
            other => return Err(format!("unknown argument {other}")),
        }
    }

    if parsed.sweep.spacing <= 0.0 {
        return Err("--spacing must be positive".into());
    }
    Ok(parsed)
}

fn parse_number<T: std::str::FromStr>(text: &str) -> Result<T, String> {
    text.trim()
        .parse()
        .map_err(|_| format!("not a number: {text}"))
}
