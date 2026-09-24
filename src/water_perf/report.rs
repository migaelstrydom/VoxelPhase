//! The water bench's tables.

use std::fmt::Write as _;
use std::time::Duration;

use crate::perf::stats::{as_ms, max_ms, mean_ms, percentile_ms};

use super::runner::{FrameCost, Subject};

/// One subject: quiet frames, the blast frame, and the transient after it.
pub fn subject_table(subject: &Subject) -> String {
    let mut out = String::new();
    let p = subject.blast_at;
    let _ = writeln!(
        out,
        "{}  (blast at {:.1}, {:.1}, {:.1}; quiet mesh {} indices)",
        subject.label, p.x, p.y, p.z, subject.mesh_indices
    );
    let _ = writeln!(
        out,
        "  {:<18} {:>20} {:>20} {:>20} {:>24}",
        "phase (ms)", "flow mean/p99", "wave mean/p99", "mesh mean/p99", "water mean/p99/max"
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

fn phase_row(name: &str, frames: &[FrameCost]) -> String {
    let pair = |f: fn(&FrameCost) -> Duration| {
        let times: Vec<Duration> = frames.iter().map(f).collect();
        format!(
            "{:.3} / {:.3}",
            mean_ms(&times),
            percentile_ms(&times, 0.99)
        )
    };
    let water: Vec<Duration> = frames.iter().map(FrameCost::water).collect();
    format!(
        "  {:<18} {:>20} {:>20} {:>20} {:>24}\n",
        format!("{name} ({})", frames.len()),
        pair(|f| f.flow),
        pair(|f| f.wave),
        pair(|f| f.mesh),
        format!(
            "{:.3} / {:.3} / {:.3}",
            mean_ms(&water),
            percentile_ms(&water, 0.99),
            max_ms(&water)
        ),
    )
}
