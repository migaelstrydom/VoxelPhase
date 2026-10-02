use std::fmt::Write as _;
use std::io;
use std::path::Path;
use std::time::Duration;

use crate::perf::stats::{as_ms, max_ms, mean_ms, percentile_ms};
use crate::terrain::UpdateTimings;

use super::record::{BlastRecord, TerrainRun};
use super::stage::{BuildPhase, TerrainStage};

/// Frame budget the header compares a blast against, in ms.
const FRAME_BUDGET_MS: f64 = 1000.0 / 60.0;

/// The run as a whole: header, then where a blast's time goes stage by stage.
///
/// Only blasts that rebuilt something are summarised; a charge that removed
/// nothing costs a `detonate` and is listed in the blast table instead.
pub fn summary(run: &TerrainRun) -> String {
    let mut out = String::new();
    let rebuilt: Vec<&UpdateTimings> = run
        .blasts
        .iter()
        .filter_map(|b| b.timings.as_ref())
        .collect();
    let chunks: usize = rebuilt.iter().map(|t| t.chunks_dirtied).sum();

    let _ = writeln!(out, "terrain_perf  —  {}", run.charge);
    let _ = writeln!(
        out,
        "level {}  |  {} blasts, {} rebuilt {} chunks  |  median of {} run(s)",
        run.level,
        run.blasts.len(),
        rebuilt.len(),
        chunks,
        run.repeats
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "{:<24} {:>9} {:>9} {:>9} {:>11} {:>7}",
        "stage", "mean ms", "p95 ms", "max ms", "ms/chunk", "share"
    );

    let whole: Duration = rebuilt.iter().map(|t| t.total()).sum();
    for stage in TerrainStage::ALL {
        let times: Vec<Duration> = rebuilt.iter().map(|t| stage.of(t)).collect();
        let _ = writeln!(
            out,
            "{:<24} {:>9.3} {:>9.3} {:>9.3} {:>11.3} {:>6.1}%",
            stage.label(),
            mean_ms(&times),
            percentile_ms(&times, 0.95),
            max_ms(&times),
            per_chunk_ms(times.iter().sum(), chunks),
            share(times.iter().sum(), whole)
        );
    }

    let totals: Vec<Duration> = rebuilt.iter().map(|t| t.total()).collect();
    let walls: Vec<Duration> = run
        .blasts
        .iter()
        .filter(|b| b.timings.is_some())
        .map(|b| b.wall)
        .collect();
    let _ = writeln!(
        out,
        "{:<24} {:>9.3} {:>9.3} {:>9.3} {:>11.3}",
        "── stages total",
        mean_ms(&totals),
        percentile_ms(&totals, 0.95),
        max_ms(&totals),
        per_chunk_ms(whole, chunks)
    );
    let _ = writeln!(
        out,
        "{:<24} {:>9.3} {:>9.3} {:>9.3}   (frame budget {:.1} ms)",
        "── wall clock",
        mean_ms(&walls),
        percentile_ms(&walls, 0.95),
        max_ms(&walls),
        FRAME_BUDGET_MS
    );
    let _ = writeln!(out);
    out.push_str(&build_breakdown(&rebuilt, chunks));
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "triangles {} → {}  |  open edges after: {}",
        run.triangles_before, run.triangles_after, run.open_edges_after
    );
    let _ = writeln!(
        out,
        "terrain fingerprint: {:016x}{}",
        run.fingerprint,
        if run.deterministic {
            ""
        } else {
            "  (NOT deterministic: repeats disagreed)"
        }
    );
    out
}

/// One row per blast: where it went off, how much it rebuilt, what it cost.
pub fn blast_table(run: &TerrainRun) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{:>3} {:>22} {:>6} {:>5} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}",
        "#",
        "site",
        "chunks",
        "frags",
        "detonate",
        "cut loose",
        "build",
        "commit",
        "adjacency",
        "concat",
        "total",
        "wall"
    );
    for (index, blast) in run.blasts.iter().enumerate() {
        let t = blast.timings.unwrap_or_default();
        let _ = writeln!(
            out,
            "{:>3} {:>22} {:>6} {:>5} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>9.2}",
            index,
            format!(
                "({:.1}, {:.1}, {:.1})",
                blast.site.x, blast.site.y, blast.site.z
            ),
            blast.chunks_dirtied(),
            t.fragments,
            as_ms(t.detonate),
            as_ms(t.cut_loose),
            as_ms(t.build),
            as_ms(t.commit),
            as_ms(t.adjacency),
            as_ms(t.concat),
            as_ms(t.total()),
            as_ms(blast.wall)
        );
    }
    out
}

/// Every blast as a CSV row: site, chunk count, wall clock, and each stage.
pub fn write_csv(run: &TerrainRun, path: &Path) -> io::Result<()> {
    let mut out = String::from("index,x,y,z,chunks,fragments,wall_ms,unaccounted_ms");
    for stage in TerrainStage::ALL {
        let _ = write!(out, ",{}_ms", column(stage.label()));
    }
    for phase in BuildPhase::ALL {
        let _ = write!(out, ",cpu_{}_ms", column(phase.label()));
    }
    out.push('\n');
    for (index, blast) in run.blasts.iter().enumerate() {
        write_row(&mut out, index, blast);
    }
    std::fs::write(path, out)
}

fn write_row(out: &mut String, index: usize, blast: &BlastRecord) {
    let t = blast.timings.unwrap_or_default();
    let _ = write!(
        out,
        "{},{:.3},{:.3},{:.3},{},{},{:.4},{:.4}",
        index,
        blast.site.x,
        blast.site.y,
        blast.site.z,
        blast.chunks_dirtied(),
        t.fragments,
        as_ms(blast.wall),
        as_ms(blast.unaccounted())
    );
    for stage in TerrainStage::ALL {
        let _ = write!(out, ",{:.4}", as_ms(stage.of(&t)));
    }
    for phase in BuildPhase::ALL {
        let _ = write!(out, ",{:.4}", as_ms(phase.of(&t.build_cpu)));
    }
    out.push('\n');
}

/// A label as a CSV column name.
fn column(label: &str) -> String {
    label.replace(" (parallel)", "").replace(['/', ' '], "_")
}

/// Where the parallel build's work goes, in CPU time summed over chunks, and
/// how much of it the threads hid.
fn build_breakdown(rebuilt: &[&UpdateTimings], chunks: usize) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{:<24} {:>9} {:>9} {:>9} {:>11} {:>7}",
        "build phase (CPU)", "mean ms", "p95 ms", "max ms", "ms/chunk", "share"
    );
    let cpu: Duration = rebuilt.iter().map(|t| t.build_cpu.total()).sum();
    for phase in BuildPhase::ALL {
        let times: Vec<Duration> = rebuilt.iter().map(|t| phase.of(&t.build_cpu)).collect();
        let _ = writeln!(
            out,
            "{:<24} {:>9.3} {:>9.3} {:>9.3} {:>11.3} {:>6.1}%",
            phase.label(),
            mean_ms(&times),
            percentile_ms(&times, 0.95),
            max_ms(&times),
            per_chunk_ms(times.iter().sum(), chunks),
            share(times.iter().sum(), cpu)
        );
    }
    let wall: Duration = rebuilt.iter().map(|t| t.build).sum();
    let _ = writeln!(
        out,
        "── build CPU {:.3} ms/blast over {:.3} ms wall: {:.1}x from {} threads",
        as_ms(cpu) / rebuilt.len().max(1) as f64,
        as_ms(wall) / rebuilt.len().max(1) as f64,
        cpu.as_secs_f64() / wall.as_secs_f64().max(1e-12),
        rayon::current_num_threads()
    );
    out
}

fn per_chunk_ms(time: Duration, chunks: usize) -> f64 {
    as_ms(time) / chunks.max(1) as f64
}

fn share(part: Duration, whole: Duration) -> f64 {
    part.as_secs_f64() / whole.as_secs_f64().max(1e-12) * 100.0
}
