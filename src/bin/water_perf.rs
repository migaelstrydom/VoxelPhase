//! Water budget bench.
//!
//! Runs each water level's terrain and water headlessly at the game's cadence
//! (1/60 s ticks), then sets off a grenade on the shore of its water and keeps
//! running while it responds. Reports the field's CPU time per frame, quiet,
//! on the blast frame, and through the transient: stepping, carrying the
//! water over the edit, and rebuilding the drawn surface; and its opening.
//!
//! ```text
//! cargo run --release --bin water_perf
//! cargo run --release --bin water_perf -- --level levels/thin_ice.level.ron
//! cargo run --release --bin water_perf -- --quiet 4 --transient 20
//! ```
//!
//! No Vulkan, no window, no ECS. Always measure a release build.

use std::path::PathBuf;
use std::process::ExitCode;

use voxel_phase::water_perf::{field_table, run_field_level, Durations};

const USAGE: &str = "usage: water_perf [--level <level.ron>]... [--quiet S] [--transient S]";

/// Every level that has water.
const DEFAULT_LEVELS: [&str; 8] = [
    "levels/island_sea.level.ron",
    "levels/test_arena.level.ron",
    "levels/skyway.level.ron",
    "levels/subsidence.level.ron",
    "levels/thin_ice.level.ron",
    "levels/wrecking_yard.level.ron",
    "levels/water_park.level.ron",
    "levels/swim_test.level.ron",
];

/// What to measure.
struct Options {
    levels: Vec<PathBuf>,
    durations: Durations,
}

fn main() -> ExitCode {
    env_logger::init();
    let options = match parse_args(std::env::args().skip(1)) {
        Ok(parsed) => parsed,
        Err(message) => {
            eprintln!("{message}\n{USAGE}");
            return ExitCode::FAILURE;
        }
    };
    if cfg!(debug_assertions) {
        eprintln!("warning: debug build — timings are not representative; use --release\n");
    }
    let Options { levels, durations } = options;

    for level in &levels {
        match run_field_level(level, durations) {
            Ok(subject) => println!("{}", field_table(&subject)),
            Err(message) => {
                eprintln!("error: {message}");
                return ExitCode::FAILURE;
            }
        }
    }
    ExitCode::SUCCESS
}

fn parse_args(args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut levels = Vec::new();
    let mut durations = Durations::default();
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or(format!("{name} needs a value"));
        match arg.as_str() {
            "--level" => levels.push(PathBuf::from(value(&arg)?)),
            "--quiet" => durations.quiet = parse_number(&value(&arg)?)?,
            "--transient" => durations.transient = parse_number(&value(&arg)?)?,
            "-h" | "--help" => return Err(String::new()),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if levels.is_empty() {
        levels = DEFAULT_LEVELS.iter().map(PathBuf::from).collect();
    }
    Ok(Options { levels, durations })
}

fn parse_number<T: std::str::FromStr>(text: &str) -> Result<T, String> {
    text.trim()
        .parse()
        .map_err(|_| format!("not a number: {text}"))
}
