//! Watch water do something, and measure what it did.
//!
//! Scripted scenarios on small synthetic terrain: a pond breached, an island
//! pool over a pond. Each runs headlessly at the true tick rate and reports
//! the level at named probes, the volume held and the drift from the start.
//!
//! # Usage
//!
//! ```bash
//! cargo run --bin water_viewer -- --list
//! cargo run --bin water_viewer -- breach --no-render --fast-forward 60
//! cargo run --bin water_viewer -- all --no-render            # one line per scenario
//! cargo run --bin water_viewer -- breach --no-render --every 5 --csv /tmp/breach.csv
//! ```

use std::path::PathBuf;
use std::process::ExitCode;

use voxel_phase::water_viewer::{
    catalogue, report, run, select, summary_line, write_csv, RunConfig,
};

const USAGE: &str = "usage: water_viewer <scenario|all> [--no-render] [--fast-forward N]
                    [--every SECONDS] [--csv <out.csv>]
       water_viewer --list";

/// Seconds between report rows when `--every` is not given.
const DEFAULT_EVERY: f32 = 2.0;

struct Options {
    scenario: String,
    config: RunConfig,
    every: f32,
    csv: Option<PathBuf>,
    render: bool,
}

fn main() -> ExitCode {
    env_logger::init();
    let options = match parse_args(std::env::args().skip(1)) {
        Ok(Some(options)) => options,
        Ok(None) => return ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("{message}\n{USAGE}");
            return ExitCode::FAILURE;
        }
    };

    let scenarios = select(&options.scenario);
    if scenarios.is_empty() {
        eprintln!("no scenario matches {}; try --list", options.scenario);
        return ExitCode::FAILURE;
    }
    if options.render {
        eprintln!("note: filmstrips arrive with the new meshers; reporting only");
    }

    let single = scenarios.len() == 1;
    for scenario in &scenarios {
        let recorded = match run(scenario, options.config) {
            Ok(recorded) => recorded,
            Err(message) => {
                eprintln!("error: {message}");
                return ExitCode::FAILURE;
            }
        };
        if single {
            println!("{}", report(&recorded, options.every));
        } else {
            println!("{}", summary_line(&recorded));
        }
        if let Some(path) = &options.csv {
            if let Err(e) = write_csv(&recorded, path) {
                eprintln!("error: writing {}: {e}", path.display());
                return ExitCode::FAILURE;
            }
            println!("wrote {}", path.display());
        }
    }
    ExitCode::SUCCESS
}

fn parse_args(args: impl Iterator<Item = String>) -> Result<Option<Options>, String> {
    let mut scenario = None;
    let mut options = Options {
        scenario: String::new(),
        config: RunConfig::default(),
        every: DEFAULT_EVERY,
        csv: None,
        render: true,
    };
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or(format!("{name} needs a value"));
        match arg.as_str() {
            "--list" => {
                for s in catalogue() {
                    println!("{:<14} {}", s.name, s.description);
                }
                return Ok(None);
            }
            "--no-render" => options.render = false,
            "--fast-forward" => options.config.fast_forward = parse_number(&value(&arg)?)?,
            "--every" => options.every = parse_number(&value(&arg)?)?,
            "--csv" => options.csv = Some(PathBuf::from(value(&arg)?)),
            "-h" | "--help" => return Err(String::new()),
            other if other.starts_with("--") => return Err(format!("unknown argument {other}")),
            other => scenario = Some(other.to_string()),
        }
    }
    options.scenario = scenario.ok_or("which scenario? try --list")?;
    Ok(Some(options))
}

fn parse_number<T: std::str::FromStr>(text: &str) -> Result<T, String> {
    text.trim()
        .parse()
        .map_err(|_| format!("not a number: {text}"))
}
