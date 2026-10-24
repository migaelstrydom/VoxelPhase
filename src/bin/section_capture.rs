//! Capture a settled physics scene as a traversal planner fixture: spawn a
//! level headless (no GPU), optionally blast it, let it come to rest, and
//! sample the strip from a start and heading.
//!
//! ```text
//! cargo run --release --bin section_capture -- levels/stack_yard.level.ron \
//!     --at 104,100 --heading 90 --blast 108,1,100,3000,6 --expect refuses \
//!     -o crates/traversal_planner/fixtures/crate_pile.ron
//! ```
//!
//! - `--at x,z`: world point the body stands at; its height is the floor's.
//! - `--heading deg`: the path's heading, turned about `+Y` from `+Z` like
//!   every yaw in the level format (90 runs along `+X`).
//! - `--blast x,y,z,strength,radius` (repeatable): a grenade's shove, N·s at
//!   the centre falling off to the radius in metres, before settling.
//! - `--speed v` (default 3), `--lead left|right` (default left),
//!   `--expect crosses|refuses`.
//!
//! Draw the result with `section_svg` (`-p traversal_planner`).

use std::path::PathBuf;
use std::process::ExitCode;
use std::{env, fs};

use nalgebra::Point3;
use traversal_planner::plan::Limb;
use traversal_planner::scene::Outcome;
use voxel_phase::section_capture::{capture, Blast, Capture};

const USAGE: &str = "usage: section_capture <level.ron> --at x,z --heading deg \
                     [--blast x,y,z,strength,radius]... [--speed v] [--lead left|right] \
                     [--expect crosses|refuses] -o <scene.ron>";

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

fn parse(args: &[String]) -> Result<(Capture, PathBuf), String> {
    let mut level = None;
    let mut at = None;
    let mut heading = None;
    let mut out = None;
    let mut blasts = Vec::new();
    let (mut speed, mut lead, mut expect) = (3.0, Limb::LeftFoot, None);
    let mut args = args.iter();
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "--at" => {
                let v = numbers(value()?, 2)?;
                at = Some((v[0], v[1]));
            }
            "--heading" => heading = Some(numbers(value()?, 1)?[0]),
            "--blast" => {
                let v = numbers(value()?, 5)?;
                blasts.push(Blast {
                    at: Point3::new(v[0], v[1], v[2]),
                    strength: v[3],
                    radius: v[4],
                });
            }
            "--speed" => speed = numbers(value()?, 1)?[0],
            "--lead" => {
                lead = match value()?.as_str() {
                    "left" => Limb::LeftFoot,
                    "right" => Limb::RightFoot,
                    other => return Err(format!("--lead {other}: left or right")),
                }
            }
            "--expect" => {
                expect = Some(match value()?.as_str() {
                    "crosses" => Outcome::Crosses,
                    "refuses" => Outcome::Refuses,
                    other => return Err(format!("--expect {other}: crosses or refuses")),
                })
            }
            "-o" => out = Some(PathBuf::from(value()?)),
            _ if level.is_none() && !arg.starts_with('-') => level = Some(PathBuf::from(arg)),
            _ => return Err(format!("unexpected {arg}")),
        }
    }
    let spec = Capture {
        level: level.ok_or("no level")?,
        at: at.ok_or("no --at")?,
        heading_degrees: heading.ok_or("no --heading")?,
        blasts,
        speed,
        lead,
        expect,
    };
    Ok((spec, out.ok_or("no -o")?))
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();
    let (spec, out) = match parse(&args) {
        Ok(parsed) => parsed,
        Err(error) => {
            eprintln!("{error}\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    let captured = match capture(&spec) {
        Ok(captured) => captured,
        Err(error) => {
            eprintln!("{}: {error}", spec.level.display());
            return ExitCode::FAILURE;
        }
    };
    let ron = match captured.scene.to_ron() {
        Ok(ron) => ron,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::FAILURE;
        }
    };
    let settled = match captured.settled_after {
        Some(seconds) => format!("at rest after {seconds:.1} s"),
        None => format!(
            "still moving after the settle: {} bodies, at {}",
            captured.restless.len(),
            captured
                .restless
                .iter()
                .map(|p| format!("({:.1}, {:.1}, {:.1})", p.x, p.y, p.z))
                .collect::<Vec<_>>()
                .join(", ")
        ),
    };
    let header = format!(
        "// Captured by section_capture {}\n// The scene was {settled}.\n",
        args.join(" ")
    );
    if let Err(error) = fs::write(&out, header + &ron) {
        eprintln!("{}: {error}", out.display());
        return ExitCode::FAILURE;
    }
    println!(
        "{} ({settled}; sampling the strip took {:.2} ms)",
        out.display(),
        captured.sample_seconds * 1.0e3
    );
    ExitCode::SUCCESS
}
