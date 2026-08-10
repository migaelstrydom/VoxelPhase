//! Look at a level.
//!
//! `level_check` answers everything about a level that can be put as a number.
//! This answers the rest. An object floating a quarter of a metre off the
//! ground passes every numeric check there is — deliberate drops are ordinary
//! authoring — and is unmistakable the moment anyone looks at a frame. So are a
//! wall facing the wrong way, a landmark hidden behind a bench, and a jump that
//! reads as impossible however the arithmetic comes out.
//!
//! Headless, like `visual_bench` and unlike the game: it renders offscreen and
//! writes a PNG, so it runs from a plain shell with no display attached.
//!
//! # Usage
//!
//! ```bash
//! cargo run --bin level_viewer -- levels/subsidence.level.ron
//! cargo run --bin level_viewer -- levels/subsidence.level.ron --tiles --columns 2
//! cargo run --bin level_viewer -- levels/subsidence.level.ron --shot pit_rim --width 1200 --height 800
//! cargo run --bin level_viewer -- levels/subsidence.level.ron --eye 60,20,40 --look 74,10,62
//! ```

use std::path::{Path, PathBuf};

use nalgebra::Point3;

use voxel_phase::core::error::{EngineError, EngineResult};
use voxel_phase::level::load_level;
use voxel_phase::level_viewer::{custom_shot, standard_shots, LevelViewer};
use voxel_phase::rendering::visual_bench::contact_sheet;

/// Per-tile render resolution. Modest by default because the standard set is
/// read as a sheet; go bigger with `--shot` when one view is worth the pixels.
const DEFAULT_WIDTH: u32 = 640;
const DEFAULT_HEIGHT: u32 = 420;
const DEFAULT_COLUMNS: u32 = 2;

struct Options {
    level: PathBuf,
    out: PathBuf,
    width: u32,
    height: u32,
    columns: u32,
    /// Also write each shot as its own PNG beside the sheet.
    tiles: bool,
    /// Render only shots whose label contains this.
    shot: Option<String>,
    /// An explicit camera, replacing the standard set entirely.
    view: Option<(Point3<f32>, Point3<f32>)>,
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

    if let Err(e) = run(&options) {
        eprintln!("level_viewer: {e}");
        std::process::exit(1);
    }
}

fn run(options: &Options) -> EngineResult<()> {
    let level = load_level(&options.level).map_err(|e| {
        EngineError::InvalidState(format!("loading {}: {e}", options.level.display()))
    })?;

    println!(
        "opening '{}' ({} segments, {} objects)",
        level.name,
        level.segments.len(),
        level.object_count()
    );

    let mut viewer = LevelViewer::open(&level, options.width, options.height)?;

    let shots = match options.view {
        Some((eye, target)) => vec![custom_shot(eye, target)],
        None => {
            let all = standard_shots(
                &level,
                &viewer.terrain(),
                options.width as f32 / options.height as f32,
            );
            match &options.shot {
                Some(filter) => all
                    .into_iter()
                    .filter(|s| s.label.contains(filter.as_str()))
                    .collect(),
                None => all,
            }
        }
    };

    if shots.is_empty() {
        return Err(EngineError::InvalidState(
            "no shots matched (try --shot with part of a segment name, or drop it)".to_string(),
        ));
    }

    println!(
        "rendering {} shot(s) at {}x{}",
        shots.len(),
        options.width,
        options.height
    );

    let mut tiles = Vec::with_capacity(shots.len());
    for shot in &shots {
        let image = viewer.render(shot)?;
        tiles.push((shot.label.clone(), image));
    }

    if options.tiles {
        let directory = options
            .out
            .parent()
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("."));
        let stem = level_stem(&options.level);

        for (label, image) in &tiles {
            let path = directory.join(format!("{stem}_{}.png", slug(label)));
            save(image, &path)?;
            println!("  {}", path.display());
        }
    }

    // A single shot is the shot, not a one-tile sheet with a caption strip.
    if tiles.len() == 1 {
        save(&tiles[0].1, &options.out)?;
    } else {
        save(&contact_sheet(&tiles, options.columns), &options.out)?;
    }
    println!("wrote {}", options.out.display());

    Ok(())
}

/// A level's name without its `.level.ron` tail, for naming output files.
fn level_stem(path: &Path) -> String {
    path.file_name()
        .and_then(|n| n.to_str())
        .map(|n| n.trim_end_matches(".ron").trim_end_matches(".level"))
        .unwrap_or("level")
        .to_string()
}

fn save(image: &image::RgbaImage, path: &Path) -> EngineResult<()> {
    image
        .save(path)
        .map_err(|e| EngineError::InvalidState(format!("writing {}: {e}", path.display())))
}

/// Turn a shot label into something safe for a filename.
fn slug(label: &str) -> String {
    label
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '_'
            }
        })
        .collect::<String>()
        .trim_matches('_')
        .replace("__", "_")
}

/// Returns `Ok(None)` when the arguments asked for something already done, such
/// as `--help`.
fn parse_args() -> Result<Option<Options>, String> {
    let mut level: Option<PathBuf> = None;
    let mut out: Option<PathBuf> = None;
    let mut width = DEFAULT_WIDTH;
    let mut height = DEFAULT_HEIGHT;
    let mut columns = DEFAULT_COLUMNS;
    let mut tiles = false;
    let mut shot = None;
    let mut eye = None;
    let mut look = None;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => {
                print_usage();
                return Ok(None);
            }
            "--tiles" => tiles = true,
            "--out" => out = Some(PathBuf::from(next(&mut args, "--out")?)),
            "--shot" => shot = Some(next(&mut args, "--shot")?),
            "--eye" => eye = Some(parse_point(&mut args, "--eye")?),
            "--look" => look = Some(parse_point(&mut args, "--look")?),
            "--width" => width = parse_u32(&mut args, "--width")?,
            "--height" => height = parse_u32(&mut args, "--height")?,
            "--columns" => columns = parse_u32(&mut args, "--columns")?,
            other if other.starts_with('-') => {
                return Err(format!("unknown option '{other}' (try --help)"));
            }
            other => level = Some(PathBuf::from(other)),
        }
    }

    let Some(level) = level else {
        print_usage();
        return Err("no level file given".to_string());
    };

    let view = match (eye, look) {
        (Some(eye), Some(look)) => Some((eye, look)),
        (None, None) => None,
        _ => return Err("--eye and --look go together".to_string()),
    };

    let out = out.unwrap_or_else(|| PathBuf::from(format!("/tmp/{}.png", level_stem(&level))));

    Ok(Some(Options {
        level,
        out,
        width,
        height,
        columns,
        tiles,
        shot,
        view,
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

fn parse_point(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<Point3<f32>, String> {
    let text = next(args, flag)?;
    let parts: Vec<f32> = text
        .split(',')
        .map(|p| p.trim().parse::<f32>())
        .collect::<Result<_, _>>()
        .map_err(|_| format!("{flag} wants three numbers, as x,y,z"))?;

    match parts[..] {
        [x, y, z] => Ok(Point3::new(x, y, z)),
        _ => Err(format!("{flag} wants three numbers, as x,y,z")),
    }
}

fn print_usage() {
    println!("Usage: level_viewer <level.ron> [options]");
    println!();
    println!("Renders a level headlessly and writes a PNG contact sheet.");
    println!();
    println!("Options:");
    println!("  --out <path>       output image (default /tmp/<level>.png)");
    println!("  --shot <text>      render only shots whose label contains this");
    println!("  --eye x,y,z        an explicit camera position, replacing the standard set");
    println!("  --look x,y,z       what that camera points at (required with --eye)");
    println!("  --width <px>       per-tile width (default {DEFAULT_WIDTH})");
    println!("  --height <px>      per-tile height (default {DEFAULT_HEIGHT})");
    println!("  --columns <n>      tiles per sheet row (default {DEFAULT_COLUMNS})");
    println!("  --tiles            also write each shot as its own PNG");
    println!();
    println!("The standard set is one overview from each of two opposite corners,");
    println!("a three-quarter view of every segment, and the player's own view");
    println!("from the spawn point.");
}
