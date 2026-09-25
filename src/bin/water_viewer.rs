//! Watch water do something, and measure what it did.
//!
//! Scripted scenarios on small synthetic terrain: a dam breached, an island
//! pool over a pond, a staircase. Each runs headlessly at the true tick rate
//! and reports the level at named probes, the volume held and the ledger,
//! then renders a filmstrip of the run through the game's own renderer.
//!
//! # Usage
//!
//! ```bash
//! cargo run --bin water_viewer -- --list
//! cargo run --bin water_viewer -- breach --no-render --fast-forward 60
//! cargo run --bin water_viewer -- all --no-render            # one line per scenario
//! cargo run --bin water_viewer -- breach --no-render --every 5 --csv /tmp/breach.csv
//! cargo run --bin water_viewer -- island_pool --tiles 8 --from 0 --to 1
//! cargo run --bin water_viewer -- sea_wall --eye 2,2,-4 --look 0,-1,-8   # a closer look
//! ```

use std::path::PathBuf;
use std::process::ExitCode;

use nalgebra::Point3;

use voxel_phase::level::Settle;
use voxel_phase::level_viewer::{custom_shot, LevelViewer};
use voxel_phase::rendering::visual_bench::contact_sheet;
use voxel_phase::water_viewer::{
    catalogue, report, run, run_with_captures, select, summary_line, write_csv, RunConfig, Scenario,
};

const USAGE: &str = "usage: water_viewer <scenario|all> [--no-render] [--fast-forward N]
                    [--settle steady|as-authored] [--every SECONDS] [--csv <out.csv>] [--reaches]
                    [--tiles N] [--from F] [--to F] [--columns N]
                    [--width W] [--height H] [--out <sheet.png>]
       water_viewer --list";

/// Seconds between report rows when `--every` is not given.
const DEFAULT_EVERY: f32 = 2.0;

struct Options {
    scenario: String,
    config: RunConfig,
    every: f32,
    csv: Option<PathBuf>,
    render: bool,
    /// List every reach laid.
    reaches: bool,
    film: Film,
}

/// What the filmstrip shows.
struct Film {
    tiles: usize,
    /// Start and end of the strip, as fractions of the scenario's duration.
    from: f32,
    to: f32,
    columns: u32,
    width: u32,
    height: u32,
    out: Option<PathBuf>,
    /// Where the camera stands and what it looks at, in place of the
    /// scenario's own.
    camera: Option<(Point3<f32>, Point3<f32>)>,
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

    let single = scenarios.len() == 1;
    for scenario in &scenarios {
        let recorded = if options.render && single {
            film(scenario, &options)
        } else {
            run(scenario, options.config)
        };
        let recorded = match recorded {
            Ok(recorded) => recorded,
            Err(message) => {
                eprintln!("error: {message}");
                return ExitCode::FAILURE;
            }
        };
        if single {
            println!("{}", report(&recorded, options.every));
            if options.reaches {
                for line in &recorded.reaches {
                    println!("{line}");
                }
            }
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

/// Run a scenario and photograph it at evenly spaced moments.
fn film(scenario: &Scenario, options: &Options) -> Result<voxel_phase::water_viewer::Run, String> {
    let film = &options.film;
    let level = scenario.level()?;
    let mut viewer =
        LevelViewer::open(&level, film.width, film.height).map_err(|e| e.to_string())?;
    let tiles = film.tiles.max(1);
    let span = (film.to - film.from).max(0.0) * scenario.duration;
    let captures: Vec<f32> = (0..tiles)
        .map(|i| {
            let f = if tiles == 1 {
                0.0
            } else {
                i as f32 / (tiles - 1) as f32
            };
            film.from * scenario.duration + f * span
        })
        .collect();

    let (eye, look) = film.camera.unwrap_or(scenario.camera);
    let mut frames = Vec::new();
    let mut failure = None;
    let recorded = run_with_captures(scenario, options.config, &captures, |at, terrain, water| {
        viewer.swap_state(terrain, water);
        let mut shot = custom_shot(eye, look);
        shot.label = format!("t = {at:.1} s");
        match viewer.render(&shot) {
            Ok(image) => frames.push((shot.label, image)),
            Err(e) => failure = Some(e.to_string()),
        }
        viewer.swap_state(terrain, water);
    })?;
    if let Some(message) = failure {
        return Err(message);
    }

    let sheet = contact_sheet(&frames, film.columns);
    let out = film
        .out
        .clone()
        .unwrap_or_else(|| PathBuf::from(format!("/tmp/water_{}.png", scenario.name)));
    sheet
        .save(&out)
        .map_err(|e| format!("writing {}: {e}", out.display()))?;
    println!("wrote {}", out.display());
    Ok(recorded)
}

fn parse_args(args: impl Iterator<Item = String>) -> Result<Option<Options>, String> {
    let mut scenario = None;
    let mut options = Options {
        scenario: String::new(),
        config: RunConfig::default(),
        every: DEFAULT_EVERY,
        csv: None,
        render: true,
        reaches: false,
        film: Film {
            tiles: 8,
            from: 0.0,
            to: 1.0,
            columns: 4,
            width: 480,
            height: 360,
            out: None,
            camera: None,
        },
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
            "--reaches" => options.reaches = true,
            "--fast-forward" => options.config.fast_forward = parse_number(&value(&arg)?)?,
            "--settle" => {
                options.config.settle = Some(match value(&arg)?.as_str() {
                    "steady" => Settle::Steady,
                    "as-authored" => Settle::AsAuthored,
                    other => return Err(format!("unknown settle {other}")),
                })
            }
            "--every" => options.every = parse_number(&value(&arg)?)?,
            "--csv" => options.csv = Some(PathBuf::from(value(&arg)?)),
            "--tiles" => options.film.tiles = parse_number(&value(&arg)?)?,
            "--from" => options.film.from = parse_number(&value(&arg)?)?,
            "--to" => options.film.to = parse_number(&value(&arg)?)?,
            "--columns" => options.film.columns = parse_number(&value(&arg)?)?,
            "--width" => options.film.width = parse_number(&value(&arg)?)?,
            "--height" => options.film.height = parse_number(&value(&arg)?)?,
            "--out" => options.film.out = Some(PathBuf::from(value(&arg)?)),
            "--eye" => {
                let eye = parse_point(&value(&arg)?)?;
                let look = options.film.camera.map_or(Point3::origin(), |(_, l)| l);
                options.film.camera = Some((eye, look));
            }
            "--look" => {
                let look = parse_point(&value(&arg)?)?;
                let eye = options.film.camera.map_or(Point3::origin(), |(e, _)| e);
                options.film.camera = Some((eye, look));
            }
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
