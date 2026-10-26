//! Watch the character in the real game: scripted intent, real physics, real
//! water, drawn by the real renderer.
//!
//! Runs a level headlessly with the player driven by a script instead of the
//! keyboard, prints each frame's motion and animation state against the water
//! it is in, and writes a filmstrip of the frames the game drew.
//!
//! ```text
//! cargo run --release --bin character_viewer -- --list
//! cargo run --release --bin character_viewer -- wade_in
//! cargo run --release --bin character_viewer -- swim --tiles 16 --from 0.2 --to 0.6
//! cargo run --release --bin character_viewer -- all --tiles 0      # reports only
//! cargo run --release --bin character_viewer -- traversal --tiles 0 --summary
//! cargo run --release --bin character_viewer -- contract --tiles 0
//! ```
//!
//! `--plan` draws the traversal planner's plan in the frames filmed: the
//! body's path, flights dashed, and the footfalls, left blue and right
//! orange, faded where tentative. `--track` has the plan drive the character
//! (as F8 does in game): along its path at its speed, hopping where it hops.
//!
//! `traversal` runs every scenario that crosses a station of the Traversal
//! Garden; `--summary` prints only each one's traversal summary as a table
//! (speed lost, snags, bounce, foot slip, whether it got across).
//! `--starts N` crosses each station N times, from starts staggered back by
//! 0.11 m, and prints the worst crossing, how many got across and the best
//! slowest speed: one crossing turns on where the frames and the stride fall
//! at the station. `--back M` starts every scenario M metres further back,
//! to follow one of those crossings frame by frame.
//!
//! `contract` measures the traction drive against what the traversal
//! planner's `GaitParams` claim it does (slope, takeoff, ride; see
//! `character_viewer/contract.rs`), prints a row per check and fails if any
//! does not hold.
//!
//! The anim_viewer is the fast, GPU-free tool for gait on analytic ground;
//! this one is for anything the physics body itself does — floating, pitching
//! over to swim, coming ashore — which a kinematic stand-in cannot show.

use std::path::PathBuf;
use std::process::ExitCode;

use voxel_phase::character_viewer::{
    catalogue, check_contract, find, report, run, traversal, ContractCheck, RunConfig, Scenario,
    TraversalSpread,
};
use voxel_phase::rendering::visual_bench::contact_sheet;

/// How much further back each staggered start is, in metres: no divisor of
/// a frame's travel or a step at the garden's speeds.
const START_STAGGER: f32 = 0.11;

const USAGE: &str =
    "usage: character_viewer <scenario|all|traversal|contract> [--list] [--summary] [--starts N] [--back M] [--plan] [--track] [--tiles N]
                        [--from F] [--to F]
                        [--fps HZ] [--width W] [--height H] [--columns C]
                        [--every N] [--out <sheet.png>]
                        [--azimuth DEG] [--elevation DEG] [--distance M]";

/// Camera settings given on the command line, overriding the scenario's.
#[derive(Default)]
struct CameraOverride {
    azimuth: Option<f32>,
    elevation: Option<f32>,
    distance: Option<f32>,
}

struct Args {
    scenarios: Vec<Scenario>,
    config: RunConfig,
    columns: u32,
    /// Print a report row every this many frames.
    every: usize,
    /// Print only the traversal summary table, not the frame-by-frame report.
    summary: bool,
    /// Crossings of each station the summary judges, from staggered starts.
    starts: usize,
    /// Run the drive contract checks instead of scenarios.
    contract: bool,
    out: Option<PathBuf>,
}

fn main() -> ExitCode {
    env_logger::init();
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(Some(args)) => args,
        Ok(None) => return ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("{message}\n{USAGE}");
            return ExitCode::FAILURE;
        }
    };

    if args.contract {
        return contract(&args.config);
    }
    if args.summary {
        println!("{}", TraversalSpread::header());
        for scenario in &args.scenarios {
            let mut summaries = Vec::new();
            for start in 0..args.starts {
                let staggered = scenario.started_back(start as f32 * START_STAGGER);
                match run(&staggered, &args.config) {
                    Ok(take) => summaries.extend(traversal(&take)),
                    Err(e) => {
                        eprintln!("{}: {e}", scenario.name);
                        return ExitCode::FAILURE;
                    }
                }
            }
            if let Some(spread) = TraversalSpread::of(&summaries) {
                println!("{}", spread.row(scenario.name));
            }
        }
        return ExitCode::SUCCESS;
    }
    for scenario in &args.scenarios {
        let take = match run(scenario, &args.config) {
            Ok(take) => take,
            Err(e) => {
                eprintln!("{}: {e}", scenario.name);
                return ExitCode::FAILURE;
            }
        };
        print!("{}", report(&take, args.every));
        if take.tiles.is_empty() {
            continue;
        }
        let tiles: Vec<_> = take
            .tiles
            .into_iter()
            .map(|t| (t.caption, t.image))
            .collect();
        let sheet = contact_sheet(&tiles, args.columns);
        let path = match (&args.out, args.scenarios.len()) {
            (Some(path), 1) => path.clone(),
            _ => std::env::temp_dir().join(format!("character_viewer_{}.png", scenario.name)),
        };
        match sheet.save(&path) {
            Ok(()) => println!("wrote {}", path.display()),
            Err(e) => {
                eprintln!("{}: {e}", path.display());
                return ExitCode::FAILURE;
            }
        }
    }
    ExitCode::SUCCESS
}

fn parse_args(mut args: impl Iterator<Item = String>) -> Result<Option<Args>, String> {
    let mut which = None;
    let mut config = RunConfig::default();
    let mut columns = 4;
    let mut every = 3;
    let mut summary = false;
    let mut starts = 1;
    let mut back = 0.0;
    let mut out = None;
    let mut camera = CameraOverride::default();

    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or(format!("{name} needs a value"));
        match arg.as_str() {
            "--list" => {
                for s in catalogue() {
                    println!("{:16} {}", s.name, s.summary);
                }
                return Ok(None);
            }
            "--summary" => summary = true,
            "--starts" => starts = parse::<usize>(&value("--starts")?)?.max(1),
            "--back" => back = parse(&value("--back")?)?,
            "--plan" => config.plan = true,
            "--track" => config.track = true,
            "--tiles" => config.tiles = parse(&value("--tiles")?)?,
            "--from" => config.from = parse(&value("--from")?)?,
            "--to" => config.to = parse(&value("--to")?)?,
            "--fps" => config.frame_rate = parse(&value("--fps")?)?,
            "--width" => config.width = parse(&value("--width")?)?,
            "--height" => config.height = parse(&value("--height")?)?,
            "--columns" => columns = parse(&value("--columns")?)?,
            "--every" => every = parse(&value("--every")?)?,
            "--out" => out = Some(PathBuf::from(value("--out")?)),
            "--azimuth" => camera.azimuth = Some(parse(&value("--azimuth")?)?),
            "--elevation" => camera.elevation = Some(parse(&value("--elevation")?)?),
            "--distance" => camera.distance = Some(parse(&value("--distance")?)?),
            name if !name.starts_with("--") && which.is_none() => which = Some(name.to_string()),
            other => return Err(format!("unexpected argument '{other}'")),
        }
    }

    let which = which.ok_or("name a scenario, or 'all'")?;
    let contract = which == "contract";
    let mut scenarios = if contract {
        Vec::new()
    } else if which == "all" {
        catalogue()
    } else if which == "traversal" {
        catalogue()
            .into_iter()
            .filter(|s| s.station.is_some())
            .collect()
    } else {
        vec![find(&which).ok_or(format!("no scenario '{which}' (try --list)"))?]
    };
    for scenario in &mut scenarios {
        *scenario = scenario.started_back(back);
        let rig = &mut scenario.camera;
        rig.azimuth = camera.azimuth.unwrap_or(rig.azimuth);
        rig.elevation = camera.elevation.unwrap_or(rig.elevation);
        rig.distance = camera.distance.unwrap_or(rig.distance);
    }
    Ok(Some(Args {
        scenarios,
        config,
        columns,
        every,
        summary,
        starts,
        contract,
        out,
    }))
}

/// Runs the drive contract checks, a row each; fails if any does not hold.
fn contract(config: &RunConfig) -> ExitCode {
    match check_contract(config) {
        Ok(checks) => {
            println!("{}", ContractCheck::header());
            for check in &checks {
                println!("{}", check.row());
            }
            if checks.iter().all(|check| check.holds) {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        Err(error) => {
            eprintln!("contract: {error:?}");
            ExitCode::FAILURE
        }
    }
}

fn parse<T: std::str::FromStr>(text: &str) -> Result<T, String> {
    text.parse()
        .map_err(|_| format!("'{text}' is not a valid value"))
}
