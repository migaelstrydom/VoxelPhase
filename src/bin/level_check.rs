//! Offline level validator and schematic exporter.
//!
//! ```bash
//! cargo run --bin level_check -- levels/test_arena.level.ron
//! cargo run --bin level_check -- levels/test_arena.level.ron --svg out/arena.svg
//! cargo run --bin level_check -- levels/test_arena.level.ron --mesh-edges
//! ```
//!
//! Loads a level, generates and meshes its terrain, and reports statistics,
//! mesh integrity, placement problems and the player's derived jump envelope.
//! Exits non-zero if any error was found; warnings do not fail the check.
//!
//! No Vulkan, no window, no ECS — it runs anywhere the crate builds.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use voxel_phase::level::load_level;
use voxel_phase::level_check::{build_terrain, check_level, write_schematic, Report, Severity};
use voxel_phase::terrain::TerrainWorld;

const USAGE: &str = "usage: level_check <level.ron> [--svg <out.svg>] [--mesh-edges]";

/// Parsed command line.
struct Args {
    level: PathBuf,
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

    match run(&args) {
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

    Ok(Args {
        level: level.ok_or("no level file given")?,
        svg,
        mesh_edges,
    })
}

/// Returns whether the level passed.
fn run(args: &Args) -> Result<bool, String> {
    let level = load_level(&args.level).map_err(|e| e.to_string())?;

    let started = std::time::Instant::now();
    let terrain = build_terrain(&level);
    let build_time = started.elapsed();

    let report = check_level(&level, &args.level, &terrain);
    print_report(&args.level, &report, build_time);

    if args.mesh_edges {
        print_defective_edges(&terrain);
    }

    if let Some(path) = &args.svg {
        write_schematic(&level, &terrain, path)
            .map_err(|e| format!("could not write {}: {e}", path.display()))?;
        println!("Schematic written to {}", path.display());
    }

    Ok(report.passed())
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

fn print_report(level_path: &Path, report: &Report, build_time: std::time::Duration) {
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
        // Errors first: they are the reason for a non-zero exit.
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
        assert_eq!(args.level, PathBuf::from("levels/a.ron"));
        assert_eq!(args.svg, Some(PathBuf::from("out/a.svg")));
    }

    #[test]
    fn rejects_a_missing_level_path() {
        assert!(parse_args(std::iter::empty()).is_err());
        assert!(parse_args(["--svg".to_string()].into_iter()).is_err());
    }
}
