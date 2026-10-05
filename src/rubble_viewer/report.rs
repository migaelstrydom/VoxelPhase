//! A run, as text.

use std::fmt::Write;
use std::time::Duration;

use super::driver::{BlastTiming, Fate, FragmentRecord, Run};

/// A table of what each blast cut loose, then the verdict.
pub fn report(run: &Run, verdict: &Result<(), Vec<String>>) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{}  |  as authored: {} samples standing free, {} paper-thin",
        run.scenario, run.loose_before, run.paper_thin_before
    );
    let _ = writeln!(
        out,
        "{:>3} {:>24} {:>6} {:>8} {:>10} {:>8} {:>8} {:>5} {:>6} {:>7} {:>8} {:>8} {:>8} {:>8}",
        "#",
        "blast at",
        "frags",
        "samples",
        "volume m³",
        "free",
        "thin",
        "dust",
        "landed",
        "expired",
        "boulders",
        "carve ms",
        "mesh ms",
        "plan ms"
    );
    for (index, record) in run.blasts.iter().enumerate() {
        let c = record.blast.centre;
        let count = |which: fn(&FragmentRecord) -> bool| {
            record.fragments.iter().filter(|f| which(f)).count()
        };
        let _ = writeln!(
            out,
            "{:>3} {:>24} {:>6} {:>8} {:>10.3} {:>8} {:>8} {:>5} {:>6} {:>7} {:>8} {:>8.2} {:>8.2} {:>8.2}",
            index,
            format!("({:.2}, {:.2}, {:.2})", c.x, c.y, c.z),
            record.fragments.len(),
            record.fragments.iter().map(|f| f.samples).sum::<usize>(),
            record.fragments.iter().map(|f| f.volume).sum::<f32>(),
            record.loose,
            record.paper_thin,
            count(|f| f.fate == Fate::Dust),
            count(|f| matches!(f.fate, Fate::Landed { .. })),
            count(|f| matches!(f.fate, Fate::Expired { .. })),
            count(|f| f.fate.is_boulder()),
            ms(record.timing.detonate),
            ms(record.timing.remesh),
            ms(record.timing.plan),
        );
    }
    let _ = writeln!(
        out,
        "{} fragments, largest {} samples  |  open edges after: {}",
        run.fragments().count(),
        run.fragments().map(|f| f.samples).max().unwrap_or(0),
        run.open_edges
    );
    let flights: Vec<(usize, f32)> = run
        .fragments()
        .filter_map(|f| match f.fate {
            Fate::Landed { frames, drop } => Some((frames, drop)),
            _ => None,
        })
        .collect();
    if !flights.is_empty() {
        let mut frames: Vec<usize> = flights.iter().map(|&(n, _)| n).collect();
        frames.sort_unstable();
        let _ =
            writeln!(
            out,
            "scree landed: {}, frames to land min {} / median {} / max {}, on the first frame {}, \
             drop min {:.2} m / max {:.2} m",
            flights.len(),
            frames[0],
            frames[frames.len() / 2],
            frames[frames.len() - 1],
            frames.iter().filter(|&&n| n == 1).count(),
            flights.iter().map(|&(_, d)| d).fold(f32::INFINITY, f32::min),
            flights.iter().map(|&(_, d)| d).fold(f32::NEG_INFINITY, f32::max),
        );
    }
    let rests: Vec<(usize, f32)> = run
        .fragments()
        .filter_map(|f| match f.fate {
            Fate::Rested { frames, drop, .. } => Some((frames, drop)),
            _ => None,
        })
        .collect();
    let boulders = run.fragments().filter(|f| f.fate.is_boulder()).count();
    if boulders > 0 {
        let mut frames: Vec<usize> = rests.iter().map(|&(n, _)| n).collect();
        frames.sort_unstable();
        let _ = writeln!(
            out,
            "boulders: {boulders}, at rest {}, seconds to rest median {:.1} / max {:.1}, \
             drop min {:.2} m / max {:.2} m",
            rests.len(),
            frames
                .get(frames.len() / 2)
                .map_or(0.0, |&n| n as f32 / 60.0),
            frames.last().map_or(0.0, |&n| n as f32 / 60.0),
            rests.iter().map(|&(_, d)| d).fold(f32::INFINITY, f32::min),
            rests
                .iter()
                .map(|&(_, d)| d)
                .fold(f32::NEG_INFINITY, f32::max),
        );
    }
    let worst = |stage: fn(&BlastTiming) -> Duration| {
        run.blasts
            .iter()
            .map(|b| ms(stage(&b.timing)))
            .fold(0.0, f64::max)
    };
    let _ = writeln!(
        out,
        "slowest blast: carve {:.2} ms, mesh {:.2} ms, plan {:.2} ms",
        worst(|t| t.detonate),
        worst(|t| t.remesh),
        worst(|t| t.plan),
    );
    for problem in run.violations() {
        let _ = writeln!(out, "  invariant broken: {problem}");
    }
    match verdict {
        Ok(()) => out.push_str("PASS\n"),
        Err(problems) => {
            for problem in problems {
                let _ = writeln!(out, "FAIL: {problem}");
            }
        }
    }
    out
}

/// A duration in milliseconds.
fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}
