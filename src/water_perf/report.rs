//! The water bench's tables.

use std::fmt::Write as _;
use std::time::Duration;

use crate::perf::stats::{as_ms, max_ms, mean_ms, percentile_ms};

use super::runner::{FrameCost, RippleCost, SteadyLoad, Subject, WorstCase};

/// One subject: quiet frames, the blast frame, and the transient after it.
pub fn subject_table(subject: &Subject) -> String {
    let mut out = String::new();
    let p = subject.blast_at;
    let _ = writeln!(
        out,
        "{}  (blast at {:.1}, {:.1}, {:.1}; mesh {} indices; basins {} -> {}; load {:.1} ms)",
        subject.label,
        p.x,
        p.y,
        p.z,
        subject.mesh_indices,
        subject.basins.0,
        subject.basins.1,
        as_ms(subject.load)
    );
    let _ = writeln!(
        out,
        "  {:<18} {:>16} {:>16} {:>16} {:>16} {:>16} {:>24}",
        "phase (ms)", "geometry", "rebuild", "settle", "solve", "mesh", "water mean/p99/max"
    );
    out.push_str(&phase_row("quiet", &subject.quiet));
    out.push_str(&phase_row(
        "blast frame",
        std::slice::from_ref(&subject.blast),
    ));
    out.push_str(&phase_row("transient", &subject.transient));
    let _ = writeln!(
        out,
        "  terrain rebuild on the blast frame: {:.2} ms (not water)",
        as_ms(subject.blast.terrain)
    );
    out
}

/// Each basin before the blast and after the transient.
pub fn basin_table(subject: &Subject) -> String {
    let mut out = String::from("  before:\n");
    for line in &subject.basin_lines.0 {
        let _ = writeln!(out, "    {line}");
    }
    out.push_str("  after:\n");
    for line in &subject.basin_lines.1 {
        let _ = writeln!(out, "    {line}");
    }
    out
}

fn phase_row(name: &str, frames: &[FrameCost]) -> String {
    let pair = |f: fn(&FrameCost) -> Duration| {
        let times: Vec<Duration> = frames.iter().map(f).collect();
        format!("{:.3}/{:.3}", mean_ms(&times), percentile_ms(&times, 0.99))
    };
    let water: Vec<Duration> = frames.iter().map(FrameCost::water).collect();
    format!(
        "  {:<18} {:>16} {:>16} {:>16} {:>16} {:>16} {:>24}\n",
        format!("{name} ({})", frames.len()),
        pair(|f| f.geometry),
        pair(|f| f.rebuild),
        pair(|f| f.settle),
        pair(|f| f.solve),
        pair(|f| f.mesh),
        format!(
            "{:.3} / {:.3} / {:.3}",
            mean_ms(&water),
            percentile_ms(&water, 0.99),
            max_ms(&water)
        ),
    )
}

/// The largest basin's re-flood and mesh rebuild.
pub fn worst_case_line(w: &WorstCase) -> String {
    format!(
        "{:<24} largest basin {:>6} spans: re-region {:.3} ms mean, {:.3} max; mesh {:.3} ms mean",
        w.label,
        w.spans,
        mean_ms(&w.reregion),
        max_ms(&w.reregion),
        mean_ms(&w.mesh)
    )
}

/// Ripple stepping at the budget.
pub fn ripple_line(r: &RippleCost) -> String {
    format!(
        "{:<24} {:>2} tiles awake: step {:.3} ms mean, p99 {:.3}; upload {} KB per frame",
        r.label,
        r.tiles,
        mean_ms(&r.steps),
        percentile_ms(&r.steps, 0.99),
        r.upload_bytes / 1024
    )
}

/// A steady open's cost.
pub fn steady_line(s: &SteadyLoad) -> String {
    format!(
        "{:<24} load {:.1} ms; {} sweeps{}; {:.0} m³ filled",
        s.label,
        as_ms(s.load),
        s.sweeps,
        if s.converged {
            ""
        } else {
            " (did not converge)"
        },
        s.filled
    )
}
