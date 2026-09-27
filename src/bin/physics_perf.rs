//! Physics performance bench: where a frame's physics time goes.
//!
//! Builds a scenario on real level terrain, steps it headlessly at the game's
//! cadence (1/240 s fixed step, 60 Hz frames), and reports the world's own
//! stage-by-stage profile: for the whole run, over time, and — with several
//! dome counts or stack heights — as the body count grows.
//!
//! Three scenarios: an igloo blown apart (`igloo`, the default), a pyramid
//! of crates at rest (`stack`), and window panes shattered into hull shards
//! (`glass`). Any of them can be disturbed: `--grenade-at` sets one of the
//! game's grenades off partway through, and `--volley` throws real grenade
//! bodies into it at the game's speed. A stack is kept awake when left
//! alone, and left asleep when something is coming to wake it.
//!
//! ```text
//! cargo run --release --bin physics_perf
//! cargo run --release --bin physics_perf -- --domes 1,2,4,8
//! cargo run --release --bin physics_perf -- --scenario stack --layers 5,10,15,20
//! cargo run --release --bin physics_perf -- --scenario stack --grenade-at 3 --timeline
//! cargo run --release --bin physics_perf -- --scenario glass --panes 1,2,4,8
//! cargo run --release --bin physics_perf -- --scenario stack --volley 8 --timeline
//! cargo run --release --bin physics_perf -- --timeline --csv /tmp/blast.csv
//! ```
//!
//! No Vulkan, no window, no ECS. Always measure a release build.

use std::path::PathBuf;

use nalgebra::Vector3;
use std::process::ExitCode;

use voxel_phase::physics_perf::{
    run, scaling_table, summary, timeline, write_csv, BoxStack, GlassShatter, Ground, IglooBlast,
    PerfRun, PerfScenario, RunConfig, WithGrenade, WithVolley, DEFAULT_WINDOW,
};

const USAGE: &str = "usage: physics_perf [--scenario igloo|stack|glass] [--domes N[,N…]]
                    [--layers N[,N…]] [--sleep] [--panes N[,N…]]
                    [--level <level.ron>] [--at X,Z]
                    [--grenade-at S] [--grenade-offset X,Y,Z]
                    [--volley N] [--volley-range M]
                    [--seconds S] [--repeats R] [--fps F] [--speed M/S] [--seed N]
                    [--timeline] [--window S] [--csv <out.csv>]";

const DEFAULT_LEVEL: &str = "levels/test_arena.level.ron";
/// Where the test arena's own igloo stands.
const DEFAULT_SITE: (f32, f32) = (12.0, 29.0);
/// Where a grenade goes off, from the ground at the site: just in front of a
/// stack's base, near the middle of an igloo.
const DEFAULT_GRENADE_OFFSET: [f32; 3] = [0.0, 0.3, -1.0];
/// Simulated time a run goes on for after its grenade, unless `--seconds` says.
const AFTER_GRENADE: f32 = 3.0;
/// How far from the site a volley is thrown from, in metres.
const DEFAULT_VOLLEY_RANGE: f32 = 12.0;

/// Which population of bodies to measure.
#[derive(Clone, Copy, PartialEq)]
enum ScenarioKind {
    Igloo,
    Stack,
    Glass,
}

/// Parsed command line.
struct Args {
    scenario: ScenarioKind,
    /// Dome counts to run for the igloo, one run each.
    domes: Vec<usize>,
    /// Pyramid heights to run for the stack, one run each.
    layers: Vec<usize>,
    /// Let the stack fall asleep once it settles.
    sleep: bool,
    /// Pane counts to run for the glass, one run each.
    panes: Vec<usize>,
    /// Grenades to throw into the scenario, if any.
    volley: Option<usize>,
    volley_range: f32,
    /// When a grenade goes off, if one does.
    grenade_at: Option<f32>,
    grenade_offset: Vector3<f32>,
    /// `--seconds` as given, before a grenade can lengthen the run.
    seconds: Option<f32>,
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
    for scenario in scenarios(args) {
        let result = run(scenario.as_ref(), &ground, run_config(args));
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

/// The run's length: as given, or long enough to watch a grenade's aftermath.
fn run_config(args: &Args) -> RunConfig {
    let duration = match (args.seconds, args.grenade_at) {
        (Some(seconds), _) => seconds,
        (None, Some(at)) => at + AFTER_GRENADE,
        (None, None) => args.config.duration,
    };
    RunConfig {
        duration,
        ..args.config
    }
}

/// One scenario per requested dome count, stack height or pane count, with
/// the volley and the grenade if there are any.
fn scenarios(args: &Args) -> Vec<Box<dyn PerfScenario>> {
    base_scenarios(args)
        .into_iter()
        .map(|scenario| match args.volley {
            Some(count) => Box::new(WithVolley::new(
                scenario,
                count,
                args.volley_range,
                args.seed.unwrap_or(0),
            )) as Box<dyn PerfScenario>,
            None => scenario,
        })
        .map(|scenario| match args.grenade_at {
            Some(at) => Box::new(WithGrenade::new(scenario, at, args.grenade_offset))
                as Box<dyn PerfScenario>,
            None => scenario,
        })
        .collect()
}

/// Whether anything is coming to disturb the scenario after it starts.
fn disturbed(args: &Args) -> bool {
    args.grenade_at.is_some() || args.volley.is_some()
}

fn base_scenarios(args: &Args) -> Vec<Box<dyn PerfScenario>> {
    match args.scenario {
        ScenarioKind::Igloo => args
            .domes
            .iter()
            .map(|&domes| {
                let defaults = IglooBlast::default();
                Box::new(IglooBlast {
                    domes,
                    blast_speed: args.blast_speed.unwrap_or(defaults.blast_speed),
                    seed: args.seed.unwrap_or(defaults.seed),
                    ..defaults
                }) as Box<dyn PerfScenario>
            })
            .collect(),
        ScenarioKind::Stack => args
            .layers
            .iter()
            .map(|&layers| {
                Box::new(BoxStack {
                    layers,
                    sleep: args.sleep || disturbed(args),
                    ..BoxStack::default()
                }) as Box<dyn PerfScenario>
            })
            .collect(),
        ScenarioKind::Glass => args
            .panes
            .iter()
            .map(|&panes| {
                let defaults = GlassShatter::default();
                Box::new(GlassShatter {
                    panes,
                    blast_speed: args.blast_speed.unwrap_or(defaults.blast_speed),
                    seed: args.seed.unwrap_or(defaults.seed),
                    ..defaults
                }) as Box<dyn PerfScenario>
            })
            .collect(),
    }
}

fn parse_args(args: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut parsed = Args {
        scenario: ScenarioKind::Igloo,
        domes: vec![1],
        layers: vec![BoxStack::default().layers],
        sleep: false,
        panes: vec![GlassShatter::default().panes],
        volley: None,
        volley_range: DEFAULT_VOLLEY_RANGE,
        grenade_at: None,
        grenade_offset: Vector3::from(DEFAULT_GRENADE_OFFSET),
        seconds: None,
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
            "--scenario" => {
                parsed.scenario = match value("--scenario")?.as_str() {
                    "igloo" => ScenarioKind::Igloo,
                    "stack" => ScenarioKind::Stack,
                    "glass" => ScenarioKind::Glass,
                    other => return Err(format!("unknown scenario {other}")),
                }
            }
            "--domes" => parsed.domes = parse_counts(&value("--domes")?, "dome count")?,
            "--layers" => parsed.layers = parse_counts(&value("--layers")?, "layer count")?,
            "--sleep" => parsed.sleep = true,
            "--panes" => parsed.panes = parse_counts(&value("--panes")?, "pane count")?,
            "--volley" => parsed.volley = Some(parse_number(&value("--volley")?)?),
            "--volley-range" => parsed.volley_range = parse_number(&value("--volley-range")?)?,
            "--level" => parsed.level = PathBuf::from(value("--level")?),
            "--at" => parsed.site = parse_pair(&value("--at")?)?,
            "--seconds" => parsed.seconds = Some(parse_number(&value("--seconds")?)?),
            "--grenade-at" => parsed.grenade_at = Some(parse_number(&value("--grenade-at")?)?),
            "--grenade-offset" => {
                parsed.grenade_offset = parse_vector(&value("--grenade-offset")?)?
            }
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
    if parsed.layers.is_empty() || parsed.layers.contains(&0) {
        return Err("layer counts must be positive".into());
    }
    if parsed.panes.is_empty() || parsed.panes.contains(&0) {
        return Err("pane counts must be positive".into());
    }
    Ok(parsed)
}

fn parse_counts(text: &str, what: &str) -> Result<Vec<usize>, String> {
    text.split(',')
        .map(|n| n.trim().parse().map_err(|_| format!("bad {what} {n}")))
        .collect()
}

fn parse_number<T: std::str::FromStr>(text: &str) -> Result<T, String> {
    text.trim()
        .parse()
        .map_err(|_| format!("not a number: {text}"))
}

fn parse_vector(text: &str) -> Result<Vector3<f32>, String> {
    let parts: Vec<&str> = text.split(',').collect();
    let [x, y, z] = parts[..] else {
        return Err(format!("expected X,Y,Z, got {text}"));
    };
    Ok(Vector3::new(
        parse_number(x)?,
        parse_number(y)?,
        parse_number(z)?,
    ))
}

fn parse_pair(text: &str) -> Result<(f32, f32), String> {
    let (x, z) = text
        .split_once(',')
        .ok_or(format!("expected X,Z, got {text}"))?;
    Ok((parse_number(x)?, parse_number(z)?))
}
