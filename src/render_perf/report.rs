use std::fmt::Write as _;
use std::io;
use std::path::Path;
use std::time::Duration;

use crate::perf::stats::as_ms;
use crate::rendering::profile::{GpuSpan, RenderStage};

use super::record::{RenderFrameRecord, RenderRun};

/// Length of one timeline row, in simulated seconds.
pub const DEFAULT_WINDOW: f32 = 0.25;

/// Frames at the start of a run left out of the quiet baseline: the first
/// frames build pipelines, descriptor sets and buffers the rest reuse.
const WARMUP: f32 = 0.25;

/// How many of its slowest systems each worst frame lists.
const SYSTEMS_PER_WORST_FRAME: usize = 5;

/// How long after the detonation counts as the blast.
const BLAST_WINDOW: f32 = 1.0;

/// One number read off every frame.
struct Metric {
    label: String,
    /// `None` where the frame has no value, e.g. GPU times that never landed.
    read: Box<dyn Fn(&RenderFrameRecord) -> Option<f64>>,
    /// Whether the value is a count rather than milliseconds.
    count: bool,
}

impl Metric {
    fn millis(
        label: impl Into<String>,
        read: impl Fn(&RenderFrameRecord) -> Option<Duration> + 'static,
    ) -> Self {
        Self {
            label: label.into(),
            read: Box::new(move |frame| read(frame).map(as_ms)),
            count: false,
        }
    }

    fn count(label: impl Into<String>, read: impl Fn(&RenderFrameRecord) -> f64 + 'static) -> Self {
        Self {
            label: label.into(),
            read: Box::new(move |frame| Some(read(frame))),
            count: true,
        }
    }

    fn values(&self, frames: &[RenderFrameRecord]) -> Vec<f64> {
        frames
            .iter()
            .filter_map(|frame| (self.read)(frame))
            .collect()
    }
}

/// Every metric the report knows, in the order it prints them.
///
/// ```text
///   frame wall ─┬─ simulate ── physics
///               └─ render ──── cpu/<stage> …   gpu/<span> …   counts …
/// ```
fn metrics() -> Vec<Metric> {
    let mut metrics = vec![
        Metric::millis("frame wall", |f| Some(f.wall())),
        Metric::millis("simulate", |f| Some(f.timing.simulate)),
        Metric::millis("  physics", |f| Some(f.physics)),
        Metric::millis("render", |f| Some(f.timing.render)),
    ];
    for stage in RenderStage::ALL {
        metrics.push(Metric::millis(
            format!("  cpu/{}", stage.label()),
            move |f| Some(f.profile.stage(stage)),
        ));
    }
    metrics.push(Metric::millis("  cpu work", |f| Some(f.profile.cpu_work())));
    for span in GpuSpan::ALL {
        metrics.push(Metric::millis(format!("gpu/{}", span.label()), move |f| {
            f.profile.gpu.and_then(|gpu| gpu.span(span))
        }));
    }
    metrics.push(Metric::millis("gpu total", RenderFrameRecord::gpu_total));
    metrics.extend([
        Metric::count("draws", |f| f.profile.counters.mesh_draws() as f64),
        Metric::count("  blended", |f| f.profile.counters.blended_draws as f64),
        Metric::count("shadow casters", |f| {
            f.profile.counters.shadow_casters as f64
        }),
        Metric::count("triangles (k)", |f| {
            f.profile.counters.triangles as f64 / 1000.0
        }),
        Metric::count("particles", |f| f.profile.counters.particles as f64),
        Metric::count("uploaded MB", |f| {
            f.profile.counters.uploaded_mesh_bytes as f64 / (1024.0 * 1024.0)
        }),
        Metric::count("buffer growths", |f| {
            f.profile.counters.buffer_growths as f64
        }),
    ]);
    metrics
}

/// The quiet stretch before the grenade, and the second after it went off.
fn phases(run: &RenderRun) -> (&[RenderFrameRecord], &[RenderFrameRecord]) {
    let between = |from: f32, to: f32| {
        let start = run.frames.partition_point(|f| f.sim_time < from);
        let end = run.frames.partition_point(|f| f.sim_time < to);
        &run.frames[start..end.max(start)]
    };
    let quiet = between(WARMUP, run.dropped_at);
    let blast = match run.detonated_at {
        Some(at) => between(at, at + BLAST_WINDOW),
        None => &[],
    };
    (quiet, blast)
}

/// The run as a whole: header, then every metric quiet against blast.
pub fn summary(run: &RenderRun) -> String {
    let mut out = String::new();
    let budget_ms = run.frame_dt * 1000.0;
    let (quiet, blast) = phases(run);

    let _ = writeln!(
        out,
        "{} at {}×{}  |  {} frames of {:.1} ms  |  grenade down at {:.2} s, {}",
        run.level,
        run.width,
        run.height,
        run.frames.len(),
        budget_ms,
        run.dropped_at,
        match run.detonated_at {
            Some(at) => format!("went off at {at:.2} s"),
            None => "never went off".to_string(),
        }
    );
    let _ = writeln!(
        out,
        "quiet: {} frames before the drop  |  blast: {} frames in the {:.1} s after it went off",
        quiet.len(),
        blast.len(),
        BLAST_WINDOW
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "{:<26} {:>10} {:>10} {:>10} {:>10}",
        "", "quiet mean", "blast mean", "blast p95", "blast max"
    );

    for metric in metrics() {
        let quiet_values = metric.values(quiet);
        let blast_values = metric.values(blast);
        let precision = if metric.count { 1 } else { 3 };
        let cell =
            |value: Option<f64>| value.map_or("-".to_string(), |v| format!("{v:.precision$}"));
        let _ = writeln!(
            out,
            "{:<26} {:>10} {:>10} {:>10} {:>10}",
            metric.label,
            cell(mean(&quiet_values)),
            cell(mean(&blast_values)),
            cell(percentile(&blast_values, 0.95)),
            cell(max(&blast_values)),
        );
    }
    let _ = writeln!(out, "(times in ms; budget {budget_ms:.1} ms)");
    out
}

/// The slowest frames of the run, whole-frame wall clock first.
pub fn worst_frames(run: &RenderRun, count: usize) -> String {
    let mut out = String::new();
    let mut frames: Vec<&RenderFrameRecord> = run.frames.iter().collect();
    frames.sort_by(|a, b| b.wall().cmp(&a.wall()));

    let _ = writeln!(out, "worst {count} frames");
    let _ = writeln!(
        out,
        "{:>7} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>6} {:>6} {:>6}",
        "t s", "wall", "sim", "physics", "render", "fence", "gpu", "draws", "blend", "parts"
    );
    for frame in frames.into_iter().take(count) {
        let counters = &frame.profile.counters;
        let _ = writeln!(
            out,
            "{:>7.3} {:>8.3} {:>8.3} {:>8.3} {:>8.3} {:>8.3} {:>8} {:>6} {:>6} {:>6}",
            frame.sim_time,
            as_ms(frame.wall()),
            as_ms(frame.timing.simulate),
            as_ms(frame.physics),
            as_ms(frame.timing.render),
            as_ms(frame.profile.stage(RenderStage::FenceWait)),
            frame
                .gpu_total()
                .map_or("-".to_string(), |t| format!("{:.3}", as_ms(t))),
            counters.mesh_draws(),
            counters.blended_draws,
            counters.particles,
        );
        let slowest: Vec<String> = frame
            .timing
            .systems
            .iter()
            .take(SYSTEMS_PER_WORST_FRAME)
            .map(|system| format!("{} {:.3}", system.name, as_ms(system.elapsed)))
            .collect();
        let _ = writeln!(out, "        systems: {}", slowest.join("  "));
    }
    out
}

/// The run in `window`-second rows, so a spike can be placed in time.
/// The row the grenade went off in is starred.
pub fn timeline(run: &RenderRun, window: f32) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{:>8} {:>9} {:>9} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>6} {:>6} {:>6}",
        "t s",
        "wall avg",
        "wall max",
        "sim",
        "physics",
        "cpu work",
        "fence",
        "gpu avg",
        "gpu max",
        "draws",
        "blend",
        "parts"
    );

    let mut start = 0;
    while start < run.frames.len() {
        let from = run.frames[start].sim_time;
        let end = start + run.frames[start..].partition_point(|f| f.sim_time < from + window);
        let rows = &run.frames[start..end.max(start + 1)];
        start = end.max(start + 1);

        let ms = |read: fn(&RenderFrameRecord) -> Duration| -> Vec<f64> {
            rows.iter().map(|f| as_ms(read(f))).collect()
        };
        let gpu: Vec<f64> = rows
            .iter()
            .filter_map(|f| f.gpu_total().map(as_ms))
            .collect();
        let most = |read: fn(&RenderFrameRecord) -> u32| rows.iter().map(read).max().unwrap_or(0);
        let went_off = run
            .detonated_at
            .is_some_and(|at| at >= from && at < from + window);
        let cell = |v: Option<f64>| v.map_or("-".to_string(), |v| format!("{v:.3}"));

        let _ = writeln!(
            out,
            "{:>7.2}{} {:>9.3} {:>9.3} {:>8.3} {:>8.3} {:>8.3} {:>8.3} {:>8} {:>8} {:>6} {:>6} {:>6}",
            from,
            if went_off { "*" } else { " " },
            mean(&ms(RenderFrameRecord::wall)).unwrap_or(0.0),
            max(&ms(RenderFrameRecord::wall)).unwrap_or(0.0),
            mean(&ms(|f| f.timing.simulate)).unwrap_or(0.0),
            mean(&ms(|f| f.physics)).unwrap_or(0.0),
            mean(&ms(|f| f.profile.cpu_work())).unwrap_or(0.0),
            mean(&ms(|f| f.profile.stage(RenderStage::FenceWait))).unwrap_or(0.0),
            cell(mean(&gpu)),
            cell(max(&gpu)),
            most(|f| f.profile.counters.mesh_draws()),
            most(|f| f.profile.counters.blended_draws),
            most(|f| f.profile.counters.particles),
        );
    }
    out
}

/// Every frame, every metric, one row each.
pub fn write_csv(run: &RenderRun, path: &Path) -> io::Result<()> {
    let metrics = metrics();
    let mut out = String::from("sim_time");
    for metric in &metrics {
        let name = metric.label.trim().replace(['/', ' '], "_");
        let _ = write!(out, ",{name}");
    }
    out.push('\n');

    for frame in &run.frames {
        let _ = write!(out, "{:.4}", frame.sim_time);
        for metric in &metrics {
            match (metric.read)(frame) {
                Some(value) => {
                    let _ = write!(out, ",{value:.4}");
                }
                None => out.push(','),
            }
        }
        out.push('\n');
    }
    std::fs::write(path, out)
}

fn mean(values: &[f64]) -> Option<f64> {
    (!values.is_empty()).then(|| values.iter().sum::<f64>() / values.len() as f64)
}

fn max(values: &[f64]) -> Option<f64> {
    values.iter().copied().reduce(f64::max)
}

fn percentile(values: &[f64], quantile: f64) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let rank = ((sorted.len() - 1) as f64 * quantile).round() as usize;
    Some(sorted[rank])
}
