//! What traversal planning costs a frame in the game's own physics world (no
//! GPU): a level spawned headless, a body walked along a heading following
//! its plan exactly, each frame's casts, sampling and planning timed.
//!
//! ```text
//! cargo run --release --bin traversal_perf -- levels/traversal_garden.level.ron --at 2.3,10 --heading 90
//! cargo run --release --bin traversal_perf -- levels/stack_yard.level.ron --at 104,100 --heading 90 \
//!     --blast 108,2,100,350,4 --frames 120 --verify
//! ```
//!
//! - `--at x,z` or `--at x,y,z` (the floor first below `y`), `--heading deg` (turned about `+Y` from `+Z`; 90 runs along
//!   `+X`), `--speed v` (default 5), `--frames n` (default 180).
//! - `--blast x,y,z,strength,radius` (repeatable): shoves after settling; the
//!   world is then stepped every frame, so the rubble moves while planned
//!   over.
//! - `--verify`: compares each frame's kept strip with one sampled afresh,
//!   failing past 3 cm of height or 1 % of stations in one only.
//!
//! Prints the first frame (the path laid, every station cast) apart from the
//! rest, and the median, 99th percentile and worst of each cost.

use std::path::PathBuf;
use std::process::ExitCode;

use nalgebra::Point3;
use voxel_phase::section_capture::Blast;
use voxel_phase::traversal_perf::{run, PerfSpec};

/// How far a kept height may be from a fresh one before the run fails, in
/// metres: a body's movement allowance, and some.
const STALE_HEIGHT: f32 = 0.03;
/// Share of stations that may be in one strip and not the other.
const STALE_SHARE: f32 = 0.01;

const USAGE: &str =
    "usage: traversal_perf <level.ron> --at x[,y],z --heading deg [--speed v] [--frames n] \
                     [--blast x,y,z,strength,radius]... [--verify]";

fn numbers(text: &str, count: usize) -> Result<Vec<f32>, String> {
    let values: Vec<f32> = text
        .split(',')
        .map(|part| {
            part.trim()
                .parse::<f32>()
                .map_err(|error| format!("{text}: {error}"))
        })
        .collect::<Result<_, _>>()?;
    if values.len() != count {
        return Err(format!("{text}: expected {count} numbers"));
    }
    Ok(values)
}

fn parse(args: &[String]) -> Result<PerfSpec, String> {
    let (mut level, mut at, mut heading) = (None, None, None);
    let (mut speed, mut frames, mut blasts, mut verify) = (5.0, 180, Vec::new(), false);
    let mut args = args.iter();
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "--at" => {
                let text = value()?;
                let v = numbers(text, 2).or_else(|_| numbers(text, 3))?;
                at = Some(match v[..] {
                    [x, y, z] => ((x, z), Some(y)),
                    _ => ((v[0], v[1]), None),
                });
            }
            "--heading" => heading = Some(numbers(value()?, 1)?[0]),
            "--speed" => speed = numbers(value()?, 1)?[0],
            "--frames" => frames = value()?.parse().map_err(|_| "--frames: a count")?,
            "--blast" => {
                let v = numbers(value()?, 5)?;
                blasts.push(Blast {
                    at: Point3::new(v[0], v[1], v[2]),
                    strength: v[3],
                    radius: v[4],
                });
            }
            "--verify" => verify = true,
            _ if level.is_none() && !arg.starts_with('-') => level = Some(PathBuf::from(arg)),
            _ => return Err(format!("unexpected {arg}")),
        }
    }
    Ok(PerfSpec {
        level: level.ok_or("no level")?,
        at: at.ok_or("no --at")?.0,
        below: at.and_then(|at| at.1),
        heading_degrees: heading.ok_or("no --heading")?,
        speed,
        frames,
        blasts,
        verify,
    })
}

/// Median, 99th percentile and worst of `values`.
fn spread(mut values: Vec<f64>) -> (f64, f64, f64) {
    values.sort_by(f64::total_cmp);
    let at = |fraction: f64| values[((values.len() - 1) as f64 * fraction) as usize];
    (at(0.5), at(0.99), at(1.0))
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let spec = match parse(&args) {
        Ok(spec) => spec,
        Err(error) => {
            eprintln!("{error}\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    let result = match run(&spec) {
        Ok(result) => result,
        Err(error) => {
            eprintln!("{}: {error}", spec.level.display());
            return ExitCode::FAILURE;
        }
    };
    let Some((first, rest)) = result.frames.split_first() else {
        eprintln!("no frames planned");
        return ExitCode::FAILURE;
    };
    println!(
        "first frame (path laid): {} casts, section {:.2} ms, plan {:.2} ms",
        first.cost.casts,
        first.cost.section_seconds * 1.0e3,
        first.cost.plan_seconds * 1.0e3
    );
    if rest.is_empty() {
        return ExitCode::SUCCESS;
    }
    let ms = |pick: fn(&voxel_phase::traversal_perf::FrameRecord) -> f64| {
        spread(rest.iter().map(pick).collect())
    };
    for (name, (median, high, worst)) in [
        ("casts", ms(|f| f.cost.casts as f64)),
        ("section ms", ms(|f| f.cost.section_seconds * 1.0e3)),
        ("plan ms", ms(|f| f.cost.plan_seconds * 1.0e3)),
        (
            "total ms",
            ms(|f| (f.cost.section_seconds + f.cost.plan_seconds) * 1.0e3),
        ),
    ] {
        println!("{name:>10}: median {median:.2}, 99th {high:.2}, worst {worst:.2}");
    }
    let laid = rest.iter().filter(|f| f.cost.laid_afresh).count();
    let diverged = rest.iter().filter(|f| f.diverged).count();
    let refused = rest.iter().filter(|f| !f.crosses).count();
    println!(
        "{} frames: path laid again {laid}, commitment dropped {diverged}, refused {refused}",
        rest.len()
    );
    let checked: Vec<_> = result
        .frames
        .iter()
        .filter_map(|f| f.against_fresh)
        .collect();
    if !checked.is_empty() {
        let height = checked.iter().map(|d| d.height).fold(0.0, f32::max);
        let missing = checked.iter().map(|d| d.missing).sum::<usize>();
        let shared = checked.iter().map(|d| d.shared).sum::<usize>();
        println!(
            "kept strip against a fresh one over {} frames: heights within {:.1} mm, {missing} of {} stations in one only",
            checked.len(),
            height * 1.0e3,
            missing + shared
        );
        if height > STALE_HEIGHT || missing as f32 > STALE_SHARE * (missing + shared) as f32 {
            return ExitCode::FAILURE;
        }
    }
    ExitCode::SUCCESS
}
