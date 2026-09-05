//! Watch the character animate, and measure what it did.
//!
//! `level_viewer` exists because a level can pass every numeric check and still
//! be visibly wrong. Animation has the opposite problem as well as that one: a
//! gait can look wrong in a way nobody can put a number to from a still frame,
//! and it can be measurably wrong in a way no still frame shows. So this tool
//! does both from a single run — a report of what the feet did, and a filmstrip
//! of them doing it, cut from the same recording.
//!
//! Headless, like `visual_bench` and `level_viewer`: it renders offscreen and
//! writes a PNG, so it runs from a plain shell with no display attached. The
//! report needs no GPU at all, which is why `--no-render` is the fast path and
//! the one to use while tuning.
//!
//! # Usage
//!
//! ```bash
//! cargo run --bin anim_viewer -- --list
//! cargo run --bin anim_viewer -- walk                  # report + filmstrip
//! cargo run --bin anim_viewer -- walk --no-render      # report only, no Vulkan
//! cargo run --bin anim_viewer -- all --no-render       # the whole catalogue
//! cargo run --bin anim_viewer -- walk_to_run --tiles 12 --out /tmp/w2r.png
//! cargo run --bin anim_viewer -- walk --csv /tmp/walk.csv
//! cargo run --bin anim_viewer -- stairs --angle three-quarter --from 0.4 --to 0.8
//! ```

use std::path::PathBuf;

use voxel_phase::anim_viewer::{
    analyse_take, catalogue, report, run, select, strip, summary_line, write_csv, Angle, Body,
    FilmConfig, MeshCapture, Run, Scenario, Take,
};
use voxel_phase::animation::CharacterRigConfig;
use voxel_phase::character::LocomotionConfig;
use voxel_phase::core::error::{EngineError, EngineResult};
use voxel_phase::rendering::visual_bench::{contact_sheet, VisualBench};

const DEFAULT_WIDTH: u32 = 480;
const DEFAULT_HEIGHT: u32 = 360;
const DEFAULT_TILES: usize = 8;
const DEFAULT_COLUMNS: u32 = 4;

/// How often the driver keeps a drawable mesh. Every third frame at 60 Hz is
/// 20 poses a second — finer than any strip needs, cheap enough not to think
/// about.
const CAPTURE_STRIDE: usize = 3;

struct Options {
    scenario: String,
    out: Option<PathBuf>,
    csv: Option<PathBuf>,
    width: u32,
    height: u32,
    tiles: usize,
    columns: u32,
    render: bool,
    markers: bool,
    angle: Angle,
    from: f32,
    to: f32,
    /// Print the per-beat breakdown as well as the whole take.
    detail: bool,
    /// Override the pelvis ride height, in metres. Defaults to the game's own.
    ride: Option<f32>,
    /// Override the character's walk speed, in m/s. Defaults to the game's own.
    /// Sprint and crouch multipliers still apply on top.
    speed: Option<f32>,
}

fn main() {
    env_logger::init();

    let options = match parse_args() {
        Ok(Some(options)) => options,
        Ok(None) => return,
        Err(message) => {
            eprintln!("{message}");
            std::process::exit(2);
        }
    };

    if let Err(e) = execute(&options) {
        eprintln!("anim_viewer: {e}");
        std::process::exit(1);
    }
}

fn execute(options: &Options) -> EngineResult<()> {
    let scenarios = select(&options.scenario);
    if scenarios.is_empty() {
        return Err(EngineError::InvalidState(format!(
            "no scenario matches '{}' (try --list)",
            options.scenario
        )));
    }

    // A catalogue run is read as a table, a single run as a report. Rendering a
    // sheet per scenario for the whole catalogue would take minutes and be read
    // by nobody, so the sweep is numbers only.
    let sweep = scenarios.len() > 1;
    let mut bench = match (options.render, sweep) {
        (true, false) => Some(VisualBench::new(options.width, options.height)?),
        _ => None,
    };

    let mut worst = 0;
    for scenario in &scenarios {
        let take = record(scenario, options, options.render && !sweep);

        let windows = analyse_take(&take);
        if sweep {
            println!("{}", summary_line(&windows[0]));
        } else if options.detail {
            print!("{}", report(&windows));
        } else {
            print!("{}", report(&windows[..1]));
        }
        worst = worst.max(windows[0].verdict() as usize);

        if let Some(path) = &options.csv {
            let path = if sweep {
                path.with_file_name(format!(
                    "{}_{}.csv",
                    path.file_stem().and_then(|s| s.to_str()).unwrap_or("take"),
                    scenario.name
                ))
            } else {
                path.clone()
            };
            write_csv(&take, &path)?;
            println!("wrote {}", path.display());
        }

        if let Some(bench) = bench.as_mut() {
            render_strip(bench, scenario, &take, options)?;
        }
    }

    if sweep {
        println!();
        println!(
            "{} scenarios; run one by name for the full report",
            scenarios.len()
        );
    }

    Ok(())
}

/// Drive one scenario with the real animator.
fn record(scenario: &Scenario, options: &Options, capture_meshes: bool) -> Take {
    let mut locomotion = LocomotionConfig::player();
    if let Some(speed) = options.speed.or(scenario.speed) {
        locomotion.walk_speed = speed;
    }

    run(&Run {
        name: scenario.name.to_string(),
        ground: scenario.ground.as_ref(),
        script: &scenario.script,
        rig: CharacterRigConfig::default(),
        ride_height: options
            .ride
            .unwrap_or_else(|| Body::ride_height(&locomotion)),
        locomotion,
        start: scenario.start,
        support: scenario.support,
        capture: if capture_meshes {
            MeshCapture::Every(CAPTURE_STRIDE)
        } else {
            MeshCapture::None
        },
    })
}

fn render_strip(
    bench: &mut VisualBench,
    scenario: &Scenario,
    take: &Take,
    options: &Options,
) -> EngineResult<()> {
    let config = FilmConfig {
        tiles: options.tiles,
        markers: options.markers,
        angle: options.angle,
        from: options.from,
        to: options.to,
    };

    let shots = strip(take, scenario.ground.as_ref(), &config);
    if shots.is_empty() {
        return Err(EngineError::InvalidState(
            "the take captured no drawable frames".to_string(),
        ));
    }

    let mut tiles = Vec::with_capacity(shots.len());
    for shot in &shots {
        tiles.push((shot.label.clone(), bench.render(shot)?));
    }

    let path = options
        .out
        .clone()
        .unwrap_or_else(|| PathBuf::from(format!("/tmp/anim_{}.png", scenario.name)));
    let sheet = contact_sheet(&tiles, options.columns);
    sheet
        .save(&path)
        .map_err(|e| EngineError::InvalidState(format!("writing {}: {e}", path.display())))?;
    println!("wrote {} ({} frames)", path.display(), tiles.len());

    Ok(())
}

fn parse_args() -> Result<Option<Options>, String> {
    let mut scenario = None;
    let mut out = None;
    let mut csv = None;
    let mut width = DEFAULT_WIDTH;
    let mut height = DEFAULT_HEIGHT;
    let mut tiles = DEFAULT_TILES;
    let mut columns = DEFAULT_COLUMNS;
    let mut render = true;
    let mut markers = true;
    let mut angle = Angle::Side;
    let mut from = 0.0;
    let mut to = 1.0;
    let mut detail = false;
    let mut ride = None;
    let mut speed = None;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => {
                print_usage();
                return Ok(None);
            }
            "--list" => {
                print_catalogue();
                return Ok(None);
            }
            "--no-render" => render = false,
            "--no-markers" => markers = false,
            "--detail" => detail = true,
            "--ride" => ride = Some(parse_f32(&mut args, "--ride")?),
            "--speed" => speed = Some(parse_f32(&mut args, "--speed")?),
            "--out" => out = Some(PathBuf::from(next(&mut args, "--out")?)),
            "--csv" => csv = Some(PathBuf::from(next(&mut args, "--csv")?)),
            "--angle" => {
                let text = next(&mut args, "--angle")?;
                angle = Angle::parse(&text).ok_or_else(|| {
                    format!("unknown angle '{text}' (side, behind, three-quarter)")
                })?;
            }
            "--width" => width = parse_u32(&mut args, "--width")?,
            "--height" => height = parse_u32(&mut args, "--height")?,
            "--tiles" => tiles = parse_u32(&mut args, "--tiles")? as usize,
            "--columns" => columns = parse_u32(&mut args, "--columns")?,
            "--from" => from = parse_f32(&mut args, "--from")?,
            "--to" => to = parse_f32(&mut args, "--to")?,
            other if other.starts_with('-') => {
                return Err(format!("unknown option '{other}' (try --help)"));
            }
            other => scenario = Some(other.to_string()),
        }
    }

    let Some(scenario) = scenario else {
        print_usage();
        return Err("no scenario given (try --list)".to_string());
    };

    if !(0.0..=1.0).contains(&from) || !(0.0..=1.0).contains(&to) || from >= to {
        return Err("--from and --to are fractions of the take, with from < to".to_string());
    }

    Ok(Some(Options {
        scenario,
        out,
        csv,
        width,
        height,
        tiles,
        columns,
        render,
        markers,
        angle,
        from,
        to,
        detail,
        ride,
        speed,
    }))
}

fn next(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    args.next().ok_or_else(|| format!("{flag} needs a value"))
}

fn parse_u32(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<u32, String> {
    next(args, flag)?
        .parse()
        .map_err(|_| format!("{flag} needs a number"))
}

fn parse_f32(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<f32, String> {
    next(args, flag)?
        .parse()
        .map_err(|_| format!("{flag} needs a number"))
}

fn print_catalogue() {
    println!("Scenarios:");
    for scenario in catalogue() {
        println!("  {:<16} {}", scenario.name, scenario.description);
    }
    println!();
    println!("'all' runs every scenario as a numbers-only sweep.");
}

fn print_usage() {
    println!("Usage: anim_viewer <scenario|all> [options]");
    println!();
    println!("Runs the real character animator over a scripted scenario, reports what");
    println!("the feet did, and writes a filmstrip contact sheet of them doing it.");
    println!();
    println!("Options:");
    println!("  --list             list the scenarios and stop");
    println!("  --no-render        report only; needs no GPU and is much faster");
    println!("  --detail           also report each beat of the script separately");
    println!(
        "  --speed <m/s>      walk speed (default: the game's {})",
        LocomotionConfig::player().walk_speed
    );
    println!("  --ride <m>         pelvis height above ground while supported");
    println!("                     (default: the game's, i.e. the capsule's resting centre)");
    println!("  --csv <path>       write every recorded frame as CSV");
    println!("  --out <path>       output image (default /tmp/anim_<scenario>.png)");
    println!("  --tiles <n>        frames in the strip (default {DEFAULT_TILES})");
    println!("  --columns <n>      tiles per sheet row (default {DEFAULT_COLUMNS})");
    println!("  --width <px>       per-tile width (default {DEFAULT_WIDTH})");
    println!("  --height <px>      per-tile height (default {DEFAULT_HEIGHT})");
    println!("  --angle <a>        side (default), behind, or three-quarter");
    println!("  --no-markers       hide the placer's anchor / target / probe spheres");
    println!("  --from <0..1>      start of the take to film, as a fraction");
    println!("  --to <0..1>        end of the take to film");
}
