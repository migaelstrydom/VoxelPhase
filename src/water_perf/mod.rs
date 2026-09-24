//! Water budget bench: what water costs the CPU per frame, quiet, on the frame
//! of a blast, and while it drains afterwards. Backs `src/bin/water_perf.rs`.

mod report;
mod runner;

pub use report::{basin_table, subject_table, worst_case_line};
pub use runner::{
    run_breach, run_level, worst_case, Durations, FrameCost, Subject, WorstCase, WARM_UP,
};
