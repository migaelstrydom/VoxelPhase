//! Printing a run: a table of the water over time, and a CSV of every frame.

use std::fmt::Write as _;
use std::path::Path;

use super::driver::{Run, Sample};

/// A table with one row per `every` simulated seconds, plus the events.
pub fn report(run: &Run, every: f32) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "{}", run.scenario);
    for event in &run.events {
        let _ = writeln!(out, "  {:>7.2}s  {}", event.time, event.text);
    }

    let _ = write!(out, "  {:>8}", "t (s)");
    for name in &run.probe_names {
        let _ = write!(out, " {:>10}", name);
    }
    let _ = writeln!(
        out,
        " {:>12} {:>10} {:>10} {:>7} {:>6}",
        "volume m³", "sunk m³", "ledger", "basins", "links"
    );

    let mut next = 0.0f32;
    let last = run.samples.len().saturating_sub(1);
    for (index, sample) in run.samples.iter().enumerate() {
        if sample.time + 1e-4 < next && index != last {
            continue;
        }
        next = sample.time + every;
        out.push_str(&row(sample));
    }
    out
}

fn row(sample: &Sample) -> String {
    let mut out = String::new();
    let _ = write!(out, "  {:>8.2}", sample.time);
    for level in &sample.probes {
        match level {
            Some(y) => {
                let _ = write!(out, " {:>10.3}", y);
            }
            None => {
                let _ = write!(out, " {:>10}", "dry");
            }
        }
    }
    let _ = writeln!(
        out,
        " {:>12.2} {:>10.3} {:>10.1e} {:>7} {:>6}",
        sample.volume, sample.sunk, sample.ledger_error, sample.basins, sample.links
    );
    out
}

/// One line per run: the final state against the start.
pub fn summary_line(run: &Run) -> String {
    let Some(last) = run.samples.last() else {
        return format!("{:<14} no samples", run.scenario);
    };
    let probes: Vec<String> = run
        .probe_names
        .iter()
        .zip(&last.probes)
        .map(|(name, level)| match level {
            Some(y) => format!("{name} {y:.2}"),
            None => format!("{name} dry"),
        })
        .collect();
    format!(
        "{:<14} t={:.0}s  {}  volume {:.1} m³, sunk {:.2}, ledger {:+.1e}, {} basins",
        run.scenario,
        last.time,
        probes.join("  "),
        last.volume,
        last.sunk,
        last.ledger_error,
        last.basins
    )
}

/// Every recorded frame as CSV.
pub fn write_csv(run: &Run, path: &Path) -> std::io::Result<()> {
    let mut out = String::from("time");
    for name in &run.probe_names {
        let _ = write!(out, ",{name}");
    }
    out.push_str(",volume,sunk,discarded,ledger_error,basins,links\n");
    for sample in &run.samples {
        let _ = write!(out, "{}", sample.time);
        for level in &sample.probes {
            match level {
                Some(y) => {
                    let _ = write!(out, ",{y}");
                }
                None => out.push(','),
            }
        }
        let _ = writeln!(
            out,
            ",{},{},{},{},{},{}",
            sample.volume,
            sample.sunk,
            sample.discarded,
            sample.ledger_error,
            sample.basins,
            sample.links
        );
    }
    std::fs::write(path, out)
}
