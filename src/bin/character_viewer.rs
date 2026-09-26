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
//! ```
//!
//! The anim_viewer is the fast, GPU-free tool for gait on analytic ground;
//! this one is for anything the physics body itself does — floating, pitching
//! over to swim, coming ashore — which a kinematic stand-in cannot show.

use std::path::PathBuf;
use std::process::ExitCode;

use voxel_phase::character_viewer::{catalogue, find, report, run, RunConfig, Scenario};
use voxel_phase::rendering::visual_bench::contact_sheet;

const USAGE: &str =
    "usage: character_viewer <scenario|all> [--list] [--tiles N] [--from F] [--to F]
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
    let mut out = None;
    let mut camera = CameraOverride::default();

    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or(format!("{name} needs a value"));
        match arg.as_str() {
            "--list" => {
                for s in catalogue() {
                    println!("{:12} {}", s.name, s.summary);
                }
                return Ok(None);
            }
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
    let mut scenarios = if which == "all" {
        catalogue()
    } else {
        vec![find(&which).ok_or(format!("no scenario '{which}' (try --list)"))?]
    };
    for scenario in &mut scenarios {
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
        out,
    }))
}

fn parse<T: std::str::FromStr>(text: &str) -> Result<T, String> {
    text.parse()
        .map_err(|_| format!("'{text}' is not a valid value"))
}
