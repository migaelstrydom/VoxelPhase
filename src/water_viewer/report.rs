//! Printing a run: a table of the water over time, a line per scenario, and
//! a CSV of every frame.

use std::fmt::Write as _;
use std::path::Path;

use super::driver::Run;

/// The run as a table, one row every `every` seconds.
pub fn report(run: &Run, every: f32) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{} (settled for {:.1} s)",
        run.scenario, run.settled_for
    );
    for (time, text) in &run.events {
        let _ = writeln!(out, "  {time:>8.2}s  {text}");
    }
    let _ = write!(out, "{:>8}", "t (s)");
    for name in &run.probe_names {
        let _ = write!(out, " {name:>10}");
    }
    let _ = writeln!(
        out,
        " {:>11} {:>10} {:>10} {:>6} {:>4} {:>8} {:>6}",
        "volume m³", "sunk m³", "ledger", "awake", "sub", "step ms", "u m/s"
    );
    let mut next = 0.0;
    for s in &run.samples {
        if s.time + 1e-4 < next {
            continue;
        }
        next = s.time + every;
        let _ = write!(out, "{:>8.2}", s.time);
        for probe in &s.probes {
            match probe {
                Some(level) => {
                    let _ = write!(out, " {level:>10.3}");
                }
                None => {
                    let _ = write!(out, " {:>10}", "dry");
                }
            }
        }
        let _ = writeln!(
            out,
            " {:>11.2} {:>10.3} {:>10.1e} {:>6} {:>4} {:>8.3} {:>6.2}",
            s.volume,
            s.sunk,
            s.ledger_error,
            s.awake,
            s.substeps,
            s.step.as_secs_f64() * 1e3,
            s.max_speed
        );
    }
    let _ = writeln!(out, "{}", timing_line(run));
    out
}

/// Mean, p99 and max stepping time, and the slowest edit.
pub fn timing_line(run: &Run) -> String {
    let mut times: Vec<f64> = run
        .step_times
        .iter()
        .map(|d| d.as_secs_f64() * 1e3)
        .collect();
    times.sort_by(f64::total_cmp);
    let mean = times.iter().sum::<f64>() / times.len().max(1) as f64;
    let at = |q: f64| {
        times
            .get(((times.len() as f64 - 1.0) * q) as usize)
            .copied()
            .unwrap_or(0.0)
    };
    let edit = run
        .edit_times
        .iter()
        .max()
        .map_or(0.0, |d| d.as_secs_f64() * 1e3);
    format!(
        "step ms: mean {mean:.3}, p99 {:.3}, max {:.3}; slowest edit {edit:.3} ms",
        at(0.99),
        at(1.0)
    )
}

/// One line for a run among many: its name and its stepping time.
pub fn summary_line(run: &Run) -> String {
    format!("{:<16} {}", run.scenario, timing_line(run))
}

/// Every recorded frame: time, each probe's level (empty where dry), and the
/// volume, what has drained away, the ledger's error and the fastest water.
pub fn write_csv(run: &Run, path: &Path) -> std::io::Result<()> {
    let mut out = String::from("time");
    for name in &run.probe_names {
        let _ = write!(out, ",{name}");
    }
    out.push_str(",volume,sunk,ledger_error,awake,max_speed\n");
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
            ",{},{},{},{},{}",
            sample.volume, sample.sunk, sample.ledger_error, sample.awake, sample.max_speed
        );
    }
    std::fs::write(path, out)
}
