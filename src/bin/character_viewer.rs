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
//! cargo run --release --bin character_viewer -- turning --tiles 0 --summary --track --starts 5
//! cargo run --release --bin character_viewer -- contract --tiles 0
//! cargo run --release --bin character_viewer -- all --tiles 0 --track --audit
//! cargo run --release --bin character_viewer -- soak --track --seeds 0..40
//! cargo run --release --bin character_viewer -- soak:ledge_garden:17 --track --feet
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
//! `turning` runs the turning runs on the garden's open ground; `--summary`
//! prints how far the plan's path lagged the body's travel through the turn
//! (mean and worst, in degrees), how often the path was laid afresh, the
//! body's speed against what was asked, and the feet against the plan's
//! footfalls, over every start.
//!
//! `contract` measures the traction drive against what the traversal
//! planner's `GaitParams` claim it does (slope, takeoff, ride; see
//! `character_viewer/contract.rs`), prints a row per check and fails if any
//! does not hold.
//!
//! `--audit` prints, in place of the report, whether each run's legs were
//! drawn as legs (`character_viewer/pose_audit.rs`): per oddity (a straight
//! leg held out level, a foot over its hip, a foot reined in from where it
//! was asked for, a foot snapping against the pelvis or asked to jump,
//! crossed feet, a folded leg, legs that stopped stepping), how many times,
//! over how many frames, and the longest; then the stride's shape (stances
//! counted, their mean length, and how far ahead of its hip a foot was set
//! down and picked up), to hold against the same run before a change. One
//! scenario alone also lists each finding with its time.
//! `--feet` ends each report row with where each foot was drawn against the
//! pelvis.
//!
//! `soak` audits seeded random play (`character_viewer/soak.rs`): for each
//! level (`--levels a,b`, by default every soak level) and each seed of
//! `--seeds A..B`, a run from a random start under a random script of the
//! stick, jump and crouch, a row each and the totals. A run is
//! `soak:<level>:<seed>`, a scenario like any other: name it to see its
//! report or film it.
//!
//! The anim_viewer is the fast, GPU-free tool for gait on analytic ground;
//! this one is for anything the physics body itself does — floating, pitching
//! over to swim, coming ashore — which a kinematic stand-in cannot show.

use std::path::PathBuf;
use std::process::ExitCode;

use voxel_phase::character_viewer::{
    catalogue, check_contract, find, find_soak, report, run, soak_level, soak_scenario, traversal,
    ContractCheck, Judged, Oddity, PoseAudit, RunConfig, Scenario, SoakLevel, StrideShape, Take,
    TraversalSpread, TurnSummary, SOAK_LEVELS,
};
use voxel_phase::rendering::visual_bench::contact_sheet;

/// How much further back each staggered start is, in metres: no divisor of
/// a frame's travel or a step at the garden's speeds.
const START_STAGGER: f32 = 0.11;

const USAGE: &str =
    "usage: character_viewer <scenario|all|traversal|turning|contract|soak> [--list] [--summary] [--audit] [--feet] [--starts N] [--back M] [--plan] [--track] [--tiles N]
                        [--seeds A..B] [--levels a,b]
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
    /// Print each run's pose audit in place of its report.
    audit: bool,
    /// End each report row with the feet against the pelvis.
    feet: bool,
    /// The seeded random runs to audit instead of scenarios.
    soak: Option<Soak>,
    out: Option<PathBuf>,
}

/// Which seeded random runs to play.
struct Soak {
    levels: Vec<SoakLevel>,
    seeds: std::ops::Range<u64>,
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
    if let Some(soak) = &args.soak {
        return run_soak(soak, &args.config);
    }
    if args.audit {
        println!("{}", PoseAudit::header());
        for scenario in &args.scenarios {
            let take = match run(scenario, &args.config) {
                Ok(take) => take,
                Err(e) => {
                    eprintln!("{}: {e}", scenario.name);
                    return ExitCode::FAILURE;
                }
            };
            let audit = PoseAudit::of(&take.frames);
            println!("{}", audit.row(scenario.name));
            if args.scenarios.len() == 1 {
                print!("{}", audit.listing());
            }
        }
        return ExitCode::SUCCESS;
    }
    if args.summary {
        let (turns, crossings): (Vec<_>, Vec<_>) = args
            .scenarios
            .iter()
            .partition(|scenario| matches!(scenario.judged, Judged::Turn { .. }));
        if !crossings.is_empty() {
            println!("{}", TraversalSpread::header());
        }
        for scenario in crossings {
            let Some(takes) = runs(scenario, &args) else {
                return ExitCode::FAILURE;
            };
            let summaries: Vec<_> = takes.iter().filter_map(traversal).collect();
            if let Some(spread) = TraversalSpread::of(&summaries) {
                println!("{}", spread.row(scenario.name));
            }
        }
        if !turns.is_empty() {
            println!("{}", TurnSummary::header());
        }
        for scenario in turns {
            let Some(takes) = runs(scenario, &args) else {
                return ExitCode::FAILURE;
            };
            let summaries: Vec<_> = takes
                .iter()
                .map(|take| TurnSummary::of(&take.frames))
                .collect();
            println!("{}", TurnSummary::over(&summaries).row(scenario.name));
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
        print!("{}", report(&take, args.every, args.feet));
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

/// `scenario` run from each of the staggered starts, `None` (and the error
/// printed) if a run failed.
fn runs(scenario: &Scenario, args: &Args) -> Option<Vec<Take>> {
    (0..args.starts)
        .map(|start| {
            let staggered = scenario.started_back(start as f32 * START_STAGGER);
            run(&staggered, &args.config)
                .inspect_err(|e| eprintln!("{}: {e}", scenario.name))
                .ok()
        })
        .collect()
}

fn parse_args(mut args: impl Iterator<Item = String>) -> Result<Option<Args>, String> {
    let mut which = None;
    let mut config = RunConfig::default();
    let mut columns = 4;
    let mut every = 3;
    let mut summary = false;
    let mut starts = 1;
    let mut back = 0.0;
    let mut audit = false;
    let mut feet = false;
    let mut seeds = 0..20;
    let mut levels = SOAK_LEVELS.to_vec();
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
            "--audit" => audit = true,
            "--feet" => feet = true,
            "--seeds" => seeds = parse_seeds(&value("--seeds")?)?,
            "--levels" => {
                levels = value("--levels")?
                    .split(',')
                    .map(|name| soak_level(name).ok_or(format!("no soak level '{name}'")))
                    .collect::<Result<_, _>>()?
            }
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
    let soak = (which == "soak").then_some(Soak { levels, seeds });
    let mut scenarios = if contract || soak.is_some() {
        Vec::new()
    } else if which == "all" {
        catalogue()
    } else if which == "turning" {
        catalogue()
            .into_iter()
            .filter(|s| matches!(s.judged, Judged::Turn { .. }))
            .collect()
    } else if which == "traversal" {
        catalogue()
            .into_iter()
            .filter(|s| s.judged.station().is_some())
            .collect()
    } else {
        vec![find(&which)
            .or_else(|| find_soak(&which))
            .ok_or(format!("no scenario '{which}' (try --list)"))?]
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
        audit,
        feet,
        soak,
        out,
    }))
}

/// A range of seeds written `A..B`, or one seed `A`.
fn parse_seeds(text: &str) -> Result<std::ops::Range<u64>, String> {
    match text.split_once("..") {
        Some((from, to)) => Ok(parse(from)?..parse(to)?),
        None => {
            let seed: u64 = parse(text)?;
            Ok(seed..seed + 1)
        }
    }
}

/// Plays and audits every seeded run of `soak`, a row each, then the totals
/// over all of them and the runs drawn oddly for longest.
fn run_soak(soak: &Soak, config: &RunConfig) -> ExitCode {
    let config = RunConfig {
        tiles: 0,
        ..*config
    };
    let mut audits: Vec<(&'static str, PoseAudit)> = Vec::new();
    println!("{}", PoseAudit::header());
    for level in &soak.levels {
        for seed in soak.seeds.clone() {
            let scenario = soak_scenario(level, seed);
            match run(&scenario, &config) {
                Ok(take) => {
                    let audit = PoseAudit::of(&take.frames);
                    println!("{}", audit.row(scenario.name));
                    audits.push((scenario.name, audit));
                }
                // A start over no ground is no run.
                Err(e) => eprintln!("{}: skipped: {e}", scenario.name),
            }
        }
    }
    let total = PoseAudit {
        judged: audits.iter().map(|(_, audit)| audit.judged).sum(),
        findings: audits
            .iter()
            .flat_map(|(_, audit)| audit.findings.iter().copied())
            .collect(),
        stride: audits
            .iter()
            .fold(StrideShape::default(), |all, (_, audit)| {
                all.with(audit.stride)
            }),
    };
    println!("{}", total.row(&format!("{} runs", audits.len())));
    let held = |audit: &PoseAudit| -> usize {
        [
            Oddity::Level,
            Oddity::OverHip,
            Oddity::Crossed,
            Oddity::Folded,
        ]
        .into_iter()
        .map(|oddity| audit.tally(oddity).frames)
        .sum()
    };
    audits.sort_by_key(|(_, audit)| std::cmp::Reverse((held(audit), audit.odd_frames())));
    println!("-- drawn oddly for longest");
    for (name, audit) in audits.iter().take(10).filter(|(_, a)| a.odd_frames() > 0) {
        println!("{}", audit.row(name));
    }
    ExitCode::SUCCESS
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
