use std::fmt::Write as _;
use std::io;
use std::path::Path;
use std::time::Duration;

use crate::physics::PhysicsStage;

use super::record::{FrameRecord, PerfRun};

/// Length of one timeline row, in simulated seconds.
pub const DEFAULT_WINDOW: f32 = 0.25;

/// The run as a whole: header, then where the time went stage by stage.
pub fn summary(run: &PerfRun) -> String {
    let mut out = String::new();
    let budget_ms = run.frame_dt * 1000.0;
    let _ = writeln!(out, "{}  —  {}", run.scenario, run.description);
    let _ = writeln!(
        out,
        "terrain {}  |  {} dynamic bodies  |  {} frames of {:.1} ms, median of {} run(s)",
        run.ground,
        run.bodies,
        run.frames.len(),
        budget_ms,
        run.repeats
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "{:<22} {:>9} {:>9} {:>9} {:>7}",
        "stage", "mean ms", "p95 ms", "max ms", "share"
    );

    let whole: Duration = run.frames.iter().map(|f| f.profile.total()).sum();
    for stage in PhysicsStage::ALL {
        let times: Vec<Duration> = run.frames.iter().map(|f| f.profile.stage(stage)).collect();
        let share = times.iter().sum::<Duration>().as_secs_f64() / whole.as_secs_f64().max(1e-12);
        let suffix = if stage.per_substep() { " ×N" } else { "" };
        let _ = writeln!(
            out,
            "{:<22} {:>9.3} {:>9.3} {:>9.3} {:>6.1}%",
            format!("{}{}", stage.label(), suffix),
            mean_ms(&times),
            percentile_ms(&times, 0.95),
            max_ms(&times),
            share * 100.0
        );
    }
    let totals: Vec<Duration> = run.frames.iter().map(|f| f.profile.total()).collect();
    let walls: Vec<Duration> = run.frames.iter().map(|f| f.wall).collect();
    let gaps: Vec<Duration> = run.frames.iter().map(FrameRecord::unaccounted).collect();
    let _ = writeln!(
        out,
        "{:<22} {:>9.3} {:>9.3} {:>9.3}",
        "── stages total",
        mean_ms(&totals),
        percentile_ms(&totals, 0.95),
        max_ms(&totals)
    );
    let _ = writeln!(
        out,
        "{:<22} {:>9.3} {:>9.3} {:>9.3}",
        "── outside stages",
        mean_ms(&gaps),
        percentile_ms(&gaps, 0.95),
        max_ms(&gaps)
    );
    let _ = writeln!(
        out,
        "{:<22} {:>9.3} {:>9.3} {:>9.3}   (budget {:.1} ms)",
        "── frame wall clock",
        mean_ms(&walls),
        percentile_ms(&walls, 0.95),
        max_ms(&walls),
        budget_ms
    );
    let ccd: u32 = run.frames.iter().map(|f| f.profile.ccd_corrections).sum();
    let _ = writeln!(out, "CCD corrections over the run: {}", ccd);
    let _ = writeln!(out, "final state fingerprint: {:016x}", run.fingerprint);
    out
}

/// The run over time: one row per `window` seconds, each the mean frame in it.
///
/// A blast is a sequence of different loads — flight, landing, sliding,
/// sleep — and a whole-run average hides which of them is expensive.
pub fn timeline(run: &PerfRun, window: f32) -> String {
    let mut out = String::new();
    let _ = write!(
        out,
        "{:>6} {:>6} {:>8} {:>8} |",
        "t", "awake", "contacts", "ms"
    );
    for stage in PhysicsStage::ALL {
        let _ = write!(out, " {:>6}", abbreviation(stage));
    }
    let _ = writeln!(out);

    for frames in windows(&run.frames, window) {
        let count = frames.len() as f64;
        let awake = frames.iter().map(|f| f.awake_bodies).sum::<usize>() as f64 / count;
        let contacts = frames.iter().map(|f| f.contacts).sum::<usize>() as f64 / count;
        let totals: Vec<Duration> = frames.iter().map(|f| f.wall).collect();
        let _ = write!(
            out,
            "{:>6.2} {:>6.0} {:>8.0} {:>8.3} |",
            frames[0].sim_time,
            awake,
            contacts,
            mean_ms(&totals)
        );
        for stage in PhysicsStage::ALL {
            let times: Vec<Duration> = frames.iter().map(|f| f.profile.stage(stage)).collect();
            let _ = write!(out, " {:>6.3}", mean_ms(&times));
        }
        let _ = writeln!(out);
    }
    out
}

/// One row per run, to show how cost grows with the number of bodies.
pub fn scaling_table(runs: &[PerfRun], window: f32) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{:>7} {:>9} {:>9} {:>11} {:>12}  {}",
        "bodies", "mean ms", "p95 ms", "peak ms", "peak µs/body", "peak window's top stages"
    );
    for run in runs {
        let walls: Vec<Duration> = run.frames.iter().map(|f| f.wall).collect();
        let peak = windows(&run.frames, window)
            .max_by(|a, b| mean_wall(a).total_cmp(&mean_wall(b)))
            .unwrap_or(&[]);
        let peak_ms = mean_wall(peak);
        let _ = writeln!(
            out,
            "{:>7} {:>9.3} {:>9.3} {:>11.3} {:>12.1}  {}",
            run.bodies,
            mean_ms(&walls),
            percentile_ms(&walls, 0.95),
            peak_ms,
            peak_ms * 1000.0 / run.bodies.max(1) as f64,
            top_stages(peak, 3)
        );
    }
    out
}

/// Every frame as a CSV row: counts, wall clock, and each stage in ms.
pub fn write_csv(run: &PerfRun, path: &Path) -> io::Result<()> {
    let mut out = String::from("sim_time,awake_bodies,contacts,substeps,ccd_corrections,wall_ms");
    for stage in PhysicsStage::ALL {
        let _ = write!(out, ",{}_ms", stage.label());
    }
    out.push('\n');
    for frame in &run.frames {
        let _ = write!(
            out,
            "{:.4},{},{},{},{},{:.4}",
            frame.sim_time,
            frame.awake_bodies,
            frame.contacts,
            frame.profile.substeps,
            frame.profile.ccd_corrections,
            as_ms(frame.wall)
        );
        for stage in PhysicsStage::ALL {
            let _ = write!(out, ",{:.4}", as_ms(frame.profile.stage(stage)));
        }
        out.push('\n');
    }
    std::fs::write(path, out)
}

fn top_stages(frames: &[FrameRecord], count: usize) -> String {
    let mut stages: Vec<(PhysicsStage, f64)> = PhysicsStage::ALL
        .iter()
        .map(|&stage| {
            let times: Vec<Duration> = frames.iter().map(|f| f.profile.stage(stage)).collect();
            (stage, mean_ms(&times))
        })
        .collect();
    stages.sort_by(|a, b| b.1.total_cmp(&a.1));
    stages
        .iter()
        .take(count)
        .map(|(stage, ms)| format!("{} {:.2}", stage.label(), ms))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Consecutive runs of frames each spanning `window` simulated seconds.
fn windows(frames: &[FrameRecord], window: f32) -> impl Iterator<Item = &[FrameRecord]> {
    let mut rest = frames;
    std::iter::from_fn(move || {
        let first = rest.first()?;
        let end = first.sim_time + window - 1e-5;
        let split = rest
            .iter()
            .position(|f| f.sim_time > end)
            .unwrap_or(rest.len())
            .max(1);
        let (head, tail) = rest.split_at(split);
        rest = tail;
        Some(head)
    })
}

fn abbreviation(stage: PhysicsStage) -> &'static str {
    match stage {
        PhysicsStage::StaticNarrowphase => "static",
        PhysicsStage::DynamicNarrowphase => "dyn",
        PhysicsStage::ManifoldMerge => "merge",
        PhysicsStage::Bookkeeping => "book",
        PhysicsStage::Conditioning => "cond",
        PhysicsStage::Integrate => "integ",
        PhysicsStage::Solve => "solve",
        PhysicsStage::CcdSetup => "ccd0",
        PhysicsStage::Ccd => "ccd",
        PhysicsStage::SleepUpdate => "sleep",
    }
}

fn mean_wall(frames: &[FrameRecord]) -> f64 {
    let walls: Vec<Duration> = frames.iter().map(|f| f.wall).collect();
    mean_ms(&walls)
}

fn as_ms(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1000.0
}

fn mean_ms(times: &[Duration]) -> f64 {
    if times.is_empty() {
        return 0.0;
    }
    times.iter().map(|&t| as_ms(t)).sum::<f64>() / times.len() as f64
}

fn max_ms(times: &[Duration]) -> f64 {
    times.iter().map(|&t| as_ms(t)).fold(0.0, f64::max)
}

fn percentile_ms(times: &[Duration], quantile: f64) -> f64 {
    if times.is_empty() {
        return 0.0;
    }
    let mut sorted = times.to_vec();
    sorted.sort_unstable();
    let index = ((sorted.len() - 1) as f64 * quantile).round() as usize;
    as_ms(sorted[index])
}
