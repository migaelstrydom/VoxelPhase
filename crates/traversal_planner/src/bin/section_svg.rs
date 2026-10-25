//! Plan a traversal scene and draw it as an SVG: the section in a side view
//! over a top-down strip of the lanes, sharing the `s` axis, at true aspect,
//! with the plan over it (or, for a refusal, how far it got, in red). Prints
//! each scene's outcome, and the outcome it expects.
//!
//! ```text
//! cargo run --release -p traversal_planner --bin section_svg -- fixtures/zigzag_stones.ron -o out.svg
//! cargo run --release -p traversal_planner --bin section_svg -- fixtures/*.ron --out-dir scratch/sections
//! ```
//!
//! With `--out-dir`, each scene is written to `<dir>/<scene name>.svg`. With
//! `--time`, each scene is planned repeatedly and its mean plan time printed
//! (build with `--release`). With `--speed v`, each scene is planned at `v`
//! m/s instead of its own speed (the player crouches at 2, walks at 5 and
//! sprints at 8). With `--phases`, each plan's phases are printed: rides,
//! hops (pushed off by a foot) and roll-offs (riding off ground that falls
//! away), where and how fast.
//!
//! To look at one as a PNG at its own size (Quick Look crops to a square):
//!
//! ```text
//! "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --hide-scrollbars \
//!     --window-size=<width>,<height> --screenshot=$PWD/out.png file://$PWD/out.svg
//! ```
//!
//! The width and height are on the SVG's first line.

use std::hint::black_box;
use std::path::{Path, PathBuf};
use std::time::Instant;
use std::{env, fs, process};

use traversal_planner::plan::Phase;
use traversal_planner::section::StripQuery;
use traversal_planner::svg::render_scene;
use traversal_planner::{Motion, Planner, PlannerSettings, TraversalScene};

/// How many times `--time` plans each scene.
const TIMED_PLANS: usize = 20;

fn main() {
    let mut scenes = Vec::new();
    let mut out: Option<PathBuf> = None;
    let mut out_dir: Option<PathBuf> = None;
    let mut time = false;
    let mut speed: Option<f32> = None;
    let mut phases = false;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-o" => out = args.next().map(PathBuf::from),
            "--out-dir" => out_dir = args.next().map(PathBuf::from),
            "--time" => time = true,
            "--phases" => phases = true,
            "--speed" => speed = args.next().and_then(|v| v.parse().ok()),
            _ => scenes.push(PathBuf::from(arg)),
        }
    }
    if scenes.is_empty()
        || (out.is_some() && scenes.len() > 1)
        || (out.is_none() && out_dir.is_none())
    {
        eprintln!("usage: section_svg <scene.ron> -o <out.svg> | section_svg <scene.ron>... --out-dir <dir> [--time] [--speed v] [--phases]");
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
        if let Err(error) = draw(scene_path, &target, time, speed, phases) {
            eprintln!("{}: {error}", scene_path.display());
            process::exit(1);
        }
        println!("{}", target.display());
    }
}

fn draw(
    scene_path: &Path,
    target: &Path,
    time: bool,
    speed: Option<f32>,
    phases: bool,
) -> Result<(), String> {
    let text = fs::read_to_string(scene_path).map_err(|error| error.to_string())?;
    let mut scene = TraversalScene::from_ron(&text).map_err(|error| error.to_string())?;
    if let Some(speed) = speed {
        scene.body.speed = speed;
        scene.intent.speed = speed;
    }
    let name = scene_path.file_stem().unwrap_or_default().to_string_lossy();
    let section = scene.section();
    let planner = Planner::new(scene.params.clone(), PlannerSettings::default());
    let query = StripQuery::new(&section, &scene.params);
    let outcome = planner.plan(&query, &scene.request());
    let timing = if time {
        let start = Instant::now();
        for _ in 0..TIMED_PLANS {
            let _ = black_box(planner.plan(&query, &scene.request()));
        }
        let mean = start.elapsed().as_secs_f64() / TIMED_PLANS as f64;
        format!(" [{:.2} ms]", mean * 1.0e3)
    } else {
        String::new()
    };
    let verdict = match &outcome {
        Ok(planned) => format!("crosses, cost {:.2}", planned.cost),
        Err(refusal) => format!(
            "refuses, the way runs out at s = {:.2}",
            refusal
                .partial
                .phases
                .last()
                .map_or(scene.body.s, |phase| phase.end_s())
        ),
    };
    let expected = scene.expect.map_or(String::new(), |expected| {
        format!(" (expected: {expected:?})")
    });
    eprintln!("{name}: {verdict}{expected}{timing}");
    if phases {
        let plan = match &outcome {
            Ok(planned) => &planned.plan,
            Err(refusal) => &refusal.partial,
        };
        for phase in &plan.phases {
            eprintln!("  {}", describe(phase));
        }
    }
    let svg = render_scene(
        &format!("{name}: {verdict}"),
        &scene,
        &section,
        Some(&outcome),
    );
    if let Some(dir) = target.parent() {
        fs::create_dir_all(dir).map_err(|error| error.to_string())?;
    }
    fs::write(target, svg).map_err(|error| error.to_string())
}

/// One line for a phase: what it is, where along the path, how fast.
fn describe(phase: &Phase) -> String {
    let at = format!("t {:.3}+{:.3}", phase.start, phase.duration);
    match phase.motion {
        Motion::Grounded {
            from,
            to,
            start_speed,
            end_speed,
        } => format!("{at} ride     s {from:.2}..{to:.2}  {start_speed:.2} -> {end_speed:.2} m/s"),
        Motion::Flight { takeoff, .. } => format!(
            "{at} {} s {:.2}..{:.2}  {:.2} m/s, climb {:.2}",
            if phase.contacts.is_empty() {
                "hop     "
            } else {
                "roll-off"
            },
            takeoff.s,
            phase.end_s(),
            takeoff.speed,
            takeoff.climb
        ),
    }
}
