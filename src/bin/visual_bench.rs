//! Headless visual scenario renderer.
//!
//! Renders a [`VisualScene`] through the real Vulkan pipeline — the real
//! shaders, the real post chain — and writes the result to a PNG contact sheet.
//! The visual counterpart to `bench_viewer`, and unlike that one it needs no
//! window, so it runs from a plain shell.
//!
//! # Usage
//!
//! ```bash
//! cargo run --bin visual_bench -- --list
//! cargo run --bin visual_bench -- material_grid
//! cargo run --bin visual_bench -- props --out /tmp/props.png --width 800 --height 600
//! cargo run --bin visual_bench -- sun_sweep --columns 3 --tiles
//! ```

use std::path::PathBuf;

use voxel_phase::core::error::{EngineError, EngineResult};
use voxel_phase::rendering::visual_bench::scenes::{all_scenes, find_scene};
use voxel_phase::rendering::visual_bench::{contact_sheet, VisualBench};

/// Per-tile render resolution. Small by default because a sweep is read as a
/// grid, and twelve full-resolution tiles is a slow, unwieldy image.
const DEFAULT_WIDTH: u32 = 480;
const DEFAULT_HEIGHT: u32 = 360;
const DEFAULT_COLUMNS: u32 = 3;

struct Options {
    scene: String,
    out: PathBuf,
    width: u32,
    height: u32,
    columns: u32,
    /// Also write each shot as its own PNG beside the sheet.
    tiles: bool,
}

fn main() {
    env_logger::init();

    let options = match parse_args() {
        Ok(Some(options)) => options,
        Ok(None) => return,
        Err(message) => {
            eprintln!("{}", message);
            std::process::exit(2);
        }
    };

    if let Err(e) = run(&options) {
        eprintln!("visual_bench: {}", e);
        std::process::exit(1);
    }
}

fn run(options: &Options) -> EngineResult<()> {
    let scene = find_scene(&options.scene).ok_or_else(|| {
        EngineError::InvalidState(format!("unknown scene '{}' (try --list)", options.scene))
    })?;

    let mut bench = VisualBench::new(options.width, options.height)?;
    let shots = scene.shots(&bench.context())?;

    if shots.is_empty() {
        return Err(EngineError::InvalidState(format!(
            "scene '{}' produced no shots",
            scene.name()
        )));
    }

    println!(
        "rendering {} ({} shots at {}x{})",
        scene.name(),
        shots.len(),
        options.width,
        options.height
    );

    let mut tiles = Vec::with_capacity(shots.len());
    for shot in &shots {
        let image = bench.render(shot)?;
        tiles.push((shot.label.clone(), image));
    }

    if options.tiles {
        let directory = options
            .out
            .parent()
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("."));

        for (label, image) in &tiles {
            let path = directory.join(format!("{}_{}.png", scene.name(), slug(label)));
            save(image, &path)?;
            println!("  {}", path.display());
        }
    }

    let sheet = contact_sheet(&tiles, options.columns);
    save(&sheet, &options.out)?;
    println!("wrote {}", options.out.display());

    Ok(())
}

fn save(image: &image::RgbaImage, path: &PathBuf) -> EngineResult<()> {
    image
        .save(path)
        .map_err(|e| EngineError::InvalidState(format!("writing {}: {}", path.display(), e)))
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

/// Returns `Ok(None)` when the arguments asked for something that is already
/// done, such as `--list`.
fn parse_args() -> Result<Option<Options>, String> {
    let mut scene: Option<String> = None;
    let mut out: Option<PathBuf> = None;
    let mut width = DEFAULT_WIDTH;
    let mut height = DEFAULT_HEIGHT;
    let mut columns = DEFAULT_COLUMNS;
    let mut tiles = false;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--list" => {
                print_scenes();
                return Ok(None);
            }
            "--help" | "-h" => {
                print_usage();
                return Ok(None);
            }
            "--tiles" => tiles = true,
            "--out" => out = Some(PathBuf::from(next(&mut args, "--out")?)),
            "--width" => width = parse_u32(&mut args, "--width")?,
            "--height" => height = parse_u32(&mut args, "--height")?,
            "--columns" => columns = parse_u32(&mut args, "--columns")?,
            other if other.starts_with('-') => {
                return Err(format!("unknown option '{}' (try --help)", other));
            }
            other => scene = Some(other.to_string()),
        }
    }

    let Some(scene) = scene else {
        print_usage();
        return Err("no scene given".to_string());
    };

    let out = out.unwrap_or_else(|| PathBuf::from(format!("/tmp/{}.png", scene)));

    Ok(Some(Options {
        scene,
        out,
        width,
        height,
        columns,
        tiles,
    }))
}

fn next(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    args.next().ok_or_else(|| format!("{} needs a value", flag))
}

fn parse_u32(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<u32, String> {
    next(args, flag)?
        .parse()
        .map_err(|_| format!("{} needs a number", flag))
}

fn print_scenes() {
    println!("Available scenes:");
    for scene in all_scenes() {
        println!("  {:<16} {}", scene.name(), scene.description());
    }
}

fn print_usage() {
    println!("Usage: visual_bench <scene> [options]");
    println!();
    println!("Options:");
    println!("  --list             list available scenes");
    println!("  --out <path>       output contact sheet (default /tmp/<scene>.png)");
    println!(
        "  --width <px>       per-tile width (default {})",
        DEFAULT_WIDTH
    );
    println!(
        "  --height <px>      per-tile height (default {})",
        DEFAULT_HEIGHT
    );
    println!(
        "  --columns <n>      tiles per sheet row (default {})",
        DEFAULT_COLUMNS
    );
    println!("  --tiles            also write each shot as its own PNG");
    println!();
    print_scenes();
}
