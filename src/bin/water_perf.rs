//! Water budget bench.
//!
//! Runs each water level's terrain and water headlessly at the game's cadence
//! (1/60 s ticks), then sets off a grenade on the shore of its water and keeps
//! running while it responds. Reports the water's CPU time per frame, quiet,
//! on the blast frame, and through the transient, plus the harness's scripted
//! pond breach.
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

use voxel_phase::water_perf::{
    basin_table, ripple_cost, ripple_line, run_breach, run_level, subject_table, worst_case,
    worst_case_line, Durations,
};

const USAGE: &str = "usage: water_perf [--level <level.ron>]... [--quiet S] [--transient S]
                  [--basins] [--repeats N]";

/// Frames of ripple stepping measured per level.
const RIPPLE_FRAMES: usize = 240;

/// Re-floods of each level's largest basin, for the worst-case figure.
const WORST_CASE_REPEATS: usize = 20;

/// Every level that has water.
const DEFAULT_LEVELS: [&str; 5] = [
    "levels/test_arena.level.ron",
    "levels/skyway.level.ron",
    "levels/subsidence.level.ron",
    "levels/thin_ice.level.ron",
    "levels/wrecking_yard.level.ron",
];

fn main() -> ExitCode {
    env_logger::init();
    let (levels, durations, show_basins, repeats) = match parse_args(std::env::args().skip(1)) {
        Ok(parsed) => parsed,
        Err(message) => {
            eprintln!("{message}\n{USAGE}");
            return ExitCode::FAILURE;
        }
    };
    if cfg!(debug_assertions) {
        eprintln!("warning: debug build — timings are not representative; use --release\n");
    }

    for level in &levels {
        match run_level(level, durations) {
            Ok(subject) => {
                println!("{}", subject_table(&subject));
                if show_basins {
                    println!("{}", basin_table(&subject));
                }
            }
            Err(message) => {
                eprintln!("error: {message}");
                return ExitCode::FAILURE;
            }
        }
    }
    println!("Worst case (§9.2): the largest basin re-flooded in place");
    for level in &levels {
        match worst_case(level, repeats) {
            Ok(w) => println!("  {}", worst_case_line(&w)),
            Err(message) => {
                eprintln!("error: {message}");
                return ExitCode::FAILURE;
            }
        }
    }
    println!();

    println!("Ripples at the budget: the largest basin stirred on every tile it has");
    for level in &levels {
        match ripple_cost(level, RIPPLE_FRAMES) {
            Ok(r) => println!("  {}", ripple_line(&r)),
            Err(message) => {
                eprintln!("error: {message}");
                return ExitCode::FAILURE;
            }
        }
    }
    println!();

    match run_breach(durations) {
        Ok(subject) => println!("{}", subject_table(&subject)),
        Err(message) => {
            eprintln!("error: {message}");
            return ExitCode::FAILURE;
        }
    }
    ExitCode::SUCCESS
}

fn parse_args(
    args: impl Iterator<Item = String>,
) -> Result<(Vec<PathBuf>, Durations, bool, usize), String> {
    let mut levels = Vec::new();
    let mut durations = Durations::default();
    let mut show_basins = false;
    let mut repeats = WORST_CASE_REPEATS;
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or(format!("{name} needs a value"));
        match arg.as_str() {
            "--level" => levels.push(PathBuf::from(value(&arg)?)),
            "--quiet" => durations.quiet = parse_number(&value(&arg)?)?,
            "--transient" => durations.transient = parse_number(&value(&arg)?)?,
            "--basins" => show_basins = true,
            "--repeats" => repeats = parse_number(&value(&arg)?)?,
            "-h" | "--help" => return Err(String::new()),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if levels.is_empty() {
        levels = DEFAULT_LEVELS.iter().map(PathBuf::from).collect();
    }
    Ok((levels, durations, show_basins, repeats))
}

fn parse_number<T: std::str::FromStr>(text: &str) -> Result<T, String> {
    text.trim()
        .parse()
        .map_err(|_| format!("not a number: {text}"))
}
