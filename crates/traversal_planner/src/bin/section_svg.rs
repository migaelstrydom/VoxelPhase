//! Draw a traversal scene's section as an SVG: a side view over a top-down
//! strip of the lanes, sharing the `s` axis, at true aspect.
//!
//! ```text
//! cargo run --release -p traversal_planner --bin section_svg -- fixtures/zigzag_stones.ron -o out.svg
//! cargo run --release -p traversal_planner --bin section_svg -- fixtures/*.ron --out-dir scratch/sections
//! ```
//!
//! With `--out-dir`, each scene is written to `<dir>/<scene name>.svg`.
//!
//! To look at one as a PNG at its own size (Quick Look crops to a square):
//!
//! ```text
//! "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --hide-scrollbars \
//!     --window-size=<width>,<height> --screenshot=$PWD/out.png file://$PWD/out.svg
//! ```
//!
//! The width and height are on the SVG's first line.

use std::path::{Path, PathBuf};
use std::{env, fs, process};

use traversal_planner::svg::render_scene;
use traversal_planner::TraversalScene;

fn main() {
    let mut scenes = Vec::new();
    let mut out: Option<PathBuf> = None;
    let mut out_dir: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-o" => out = args.next().map(PathBuf::from),
            "--out-dir" => out_dir = args.next().map(PathBuf::from),
            _ => scenes.push(PathBuf::from(arg)),
        }
    }
    if scenes.is_empty()
        || (out.is_some() && scenes.len() > 1)
        || (out.is_none() && out_dir.is_none())
    {
        eprintln!("usage: section_svg <scene.ron> -o <out.svg> | section_svg <scene.ron>... --out-dir <dir>");
        process::exit(2);
    }
    for scene_path in &scenes {
        let target = match (&out, &out_dir) {
            (Some(out), _) => out.clone(),
            (None, Some(dir)) => dir
                .join(scene_path.file_stem().unwrap_or_default())
                .with_extension("svg"),
            (None, None) => unreachable!(),
        };
        if let Err(error) = draw(scene_path, &target) {
            eprintln!("{}: {error}", scene_path.display());
            process::exit(1);
        }
        println!("{}", target.display());
    }
}

fn draw(scene_path: &Path, target: &Path) -> Result<(), String> {
    let text = fs::read_to_string(scene_path).map_err(|error| error.to_string())?;
    let scene = TraversalScene::from_ron(&text).map_err(|error| error.to_string())?;
    let title = scene_path.file_stem().unwrap_or_default().to_string_lossy();
    let svg = render_scene(&title, &scene, &scene.section());
    if let Some(dir) = target.parent() {
        fs::create_dir_all(dir).map_err(|error| error.to_string())?;
    }
    fs::write(target, svg).map_err(|error| error.to_string())
}
