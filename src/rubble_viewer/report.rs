//! A run, as text.

use std::fmt::Write;

use super::driver::Run;

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
        "{:>3} {:>24} {:>6} {:>8} {:>10} {:>8} {:>8}",
        "#", "blast at", "frags", "samples", "volume m³", "free", "thin"
    );
    for (index, record) in run.blasts.iter().enumerate() {
        let c = record.blast.centre;
        let _ = writeln!(
            out,
            "{:>3} {:>24} {:>6} {:>8} {:>10.3} {:>8} {:>8}",
            index,
            format!("({:.2}, {:.2}, {:.2})", c.x, c.y, c.z),
            record.fragments.len(),
            record.fragments.iter().map(|f| f.samples).sum::<usize>(),
            record.fragments.iter().map(|f| f.volume).sum::<f32>(),
            record.loose,
            record.paper_thin
        );
    }
    let _ = writeln!(
        out,
        "{} fragments, largest {} samples  |  open edges after: {}",
        run.fragments().count(),
        run.fragments().map(|f| f.samples).max().unwrap_or(0),
        run.open_edges
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
