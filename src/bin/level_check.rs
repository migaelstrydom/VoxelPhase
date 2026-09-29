//! Offline level validator and schematic exporter.
//!
//! ```bash
//! cargo run --bin level_check -- levels/test_arena.level.ron
//! cargo run --bin level_check -- levels/test_arena.level.ron --svg out/arena.svg
//! cargo run --bin level_check -- levels/test_arena.level.ron --mesh-edges
//! cargo run --release --bin level_check        # every level in levels/, findings only
//! ```
//!
//! Loads a level, generates and meshes its terrain, and reports statistics,
//! mesh integrity, placement problems, objects not at rest where they were
//! authored, and the player's derived jump envelope. Exits non-zero if any
//! error was found; warnings do not fail the check.
//!
//! No Vulkan and no window — it runs anywhere the crate builds.

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use voxel_phase::level::{load_level, Level};
use voxel_phase::level_check::{
    build_terrain, check_disturbance, check_level, check_rest, write_schematic, Report, RestTrial,
    Severity,
};
use voxel_phase::terrain::TerrainWorld;

const USAGE: &str = "usage: level_check [<level.ron> [--svg <out.svg>] [--mesh-edges]]\n\
                     With no level, checks every level in levels/ and prints only findings.";

/// Where the shipped levels live, relative to the crate root.
const LEVELS_DIR: &str = "levels";

/// Parsed command line.
struct Args {
    /// The one level to report on in full; every shipped level if absent.
    level: Option<PathBuf>,
    svg: Option<PathBuf>,
    /// List every edge behind the open-edge count, with its position.
    mesh_edges: bool,
}

fn main() -> ExitCode {
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(message) => {
            eprintln!("{message}\n{USAGE}");
            return ExitCode::FAILURE;
        }
    };

    let outcome = match &args.level {
        Some(level) => run(level, &args),
        None => run_all(Path::new(env!("CARGO_MANIFEST_DIR")).join(LEVELS_DIR)),
    };
    match outcome {
        Ok(passed) if passed => ExitCode::SUCCESS,
        Ok(_) => ExitCode::FAILURE,
        Err(message) => {
            eprintln!("error: {message}");
            ExitCode::FAILURE
        }
    }
}

fn parse_args(args: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut level = None;
    let mut svg = None;
    let mut mesh_edges = false;
    let mut args = args.peekable();

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--svg" => {
                svg = Some(PathBuf::from(
                    args.next().ok_or("--svg needs an output path")?,
                ));
            }
            "--mesh-edges" => mesh_edges = true,
            "-h" | "--help" => return Err(USAGE.to_string()),
            other if other.starts_with('-') => return Err(format!("unknown option {other}")),
            other => {
                if level.replace(PathBuf::from(other)).is_some() {
                    return Err("only one level file may be given".into());
                }
            }
        }
    }

    if level.is_none() && (svg.is_some() || mesh_edges) {
        return Err("--svg and --mesh-edges need a level file".into());
    }

    Ok(Args {
        level,
        svg,
        mesh_edges,
    })
}

/// One level, checked.
struct Checked {
    level: Level,
    /// Holds the level's terrain, which the per-level options read.
    trial: RestTrial,
    report: Report,
    build_time: Duration,
}

/// Every check over one level.
fn check(level_path: &Path) -> Result<Checked, String> {
    let level = load_level(level_path).map_err(|e| e.to_string())?;

    let started = Instant::now();
    let terrain = build_terrain(&level);
    let build_time = started.elapsed();

    let mut trial = RestTrial::spawn(&level, terrain);
    let mut report = check_level(&level, level_path, &trial.terrain());
    let rest = check_rest(&mut trial, &mut report);
    report.push_section(rest);
    let disturbance = check_disturbance(&mut trial, &mut report);
    report.push_section(disturbance);
    Ok(Checked {
        level,
        trial,
        report,
        build_time,
    })
}

/// Report on one level in full. Returns whether it passed.
fn run(level_path: &Path, args: &Args) -> Result<bool, String> {
    let checked = check(level_path)?;
    print_report(level_path, &checked.report, checked.build_time);

    let terrain = checked.trial.terrain();
    if args.mesh_edges {
        print_defective_edges(&terrain);
    }

    if let Some(path) = &args.svg {
        write_schematic(&checked.level, &terrain, path)
            .map_err(|e| format!("could not write {}: {e}", path.display()))?;
        println!("Schematic written to {}", path.display());
    }

    Ok(checked.report.passed())
}

/// Check every level in `dir`, printing only findings. Returns whether all
/// of them passed; a level that fails to load counts as failing.
fn run_all(dir: PathBuf) -> Result<bool, String> {
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .map_err(|e| format!("could not read {}: {e}", dir.display()))?
        .filter_map(|entry| entry.ok().map(|e| e.path()))
        .filter(|path| path.to_string_lossy().ends_with(".level.ron"))
        .collect();
    paths.sort();

    let (mut errors, mut warnings, mut passed) = (0, 0, true);
    for path in &paths {
        let name = path.file_name().unwrap_or_default().to_string_lossy();
        match check(path) {
            Ok(Checked { report, .. }) => {
                println!(
                    "=== {name}: {} error(s), {} warning(s)",
                    report.error_count(),
                    report.warning_count()
                );
                print_findings(&report);
                errors += report.error_count();
                warnings += report.warning_count();
                passed &= report.passed();
            }
            Err(message) => {
                println!("=== {name}: could not be checked: {message}");
                errors += 1;
                passed = false;
            }
        }
    }

    println!(
        "\n{} level(s): {errors} error(s), {warnings} warning(s)",
        paths.len()
    );
    Ok(passed)
}

/// List the edges behind the open-edge count.
///
/// The count alone cannot distinguish a hole in the surface from a place where
/// the surface passes through itself, and cannot say where either is. Grouping
/// by position makes clusters — the shape a real crack takes — obvious.
fn print_defective_edges(terrain: &TerrainWorld) {
    let edges = terrain.defective_edges();

    println!("\nDefective edges ({})", edges.len());
    if edges.is_empty() {
        return;
    }

    let holes = edges.iter().filter(|(_, e)| e.triangles == 1).count();
    println!(
        "  {holes} hole edge(s) (1 triangle), {} non-manifold (3+)",
        edges.len() - holes
    );

    let mut sorted = edges;
    sorted.sort_by(|(sa, a), (sb, b)| {
        sa.cmp(sb).then_with(|| {
            (a.from.x, a.from.y, a.from.z)
                .partial_cmp(&(b.from.x, b.from.y, b.from.z))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    });
    for (segment, edge) in &sorted {
        println!(
            "  {segment:<14} {:>3} tri  ({:8.3},{:8.3},{:8.3}) -> ({:8.3},{:8.3},{:8.3})",
            edge.triangles, edge.from.x, edge.from.y, edge.from.z, edge.to.x, edge.to.y, edge.to.z
        );
    }
}

/// Every finding, errors first: they are the reason for a non-zero exit.
fn print_findings(report: &Report) {
    let mut findings: Vec<_> = report.findings.iter().collect();
    findings.sort_by(|a, b| b.severity.cmp(&a.severity));
    for finding in findings {
        let marker = match finding.severity {
            Severity::Error => "ERROR",
            Severity::Warning => "warn ",
        };
        println!("  {marker} [{}] {}", finding.category, finding.message);
    }
}

fn print_report(level_path: &Path, report: &Report, build_time: Duration) {
    println!("=== level_check: {} ===", level_path.display());

    for section in &report.sections {
        println!("\n{}", section.title);
        let width = section
            .rows
            .iter()
            .map(|(label, _)| label.len())
            .max()
            .unwrap_or(0);
        for (label, value) in &section.rows {
            println!("  {label:<width$}  {value}");
        }
        for note in &section.notes {
            println!("  note: {note}");
        }
    }

    println!("\nFindings");
    if report.findings.is_empty() {
        println!("  none");
    } else {
        print_findings(report);
    }

    println!(
        "\n{} error(s), {} warning(s) — terrain built in {:.2} s",
        report.error_count(),
        report.warning_count(),
        build_time.as_secs_f32()
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_level_path_and_svg_output() {
        let args = parse_args(
            ["levels/a.ron", "--svg", "out/a.svg"]
                .into_iter()
                .map(String::from),
        )
        .expect("should parse");
        assert_eq!(args.level, Some(PathBuf::from("levels/a.ron")));
        assert_eq!(args.svg, Some(PathBuf::from("out/a.svg")));
    }

    #[test]
    fn no_level_path_means_every_level() {
        let args = parse_args(std::iter::empty()).expect("should parse");
        assert_eq!(args.level, None);
    }

    #[test]
    fn per_level_options_need_a_level_path() {
        assert!(parse_args(["--svg".to_string()].into_iter()).is_err());
        assert!(parse_args(["--mesh-edges".to_string()].into_iter()).is_err());
    }
}
