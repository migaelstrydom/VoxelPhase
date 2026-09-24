//! Water budget bench: what water costs the CPU per frame, quiet, on the frame
//! of a blast, and while it drains afterwards. Backs `src/bin/water_perf.rs`.

mod report;
mod runner;

pub use report::subject_table;
pub use runner::{run_breach, run_level, Durations, FrameCost, Subject, WARM_UP};
