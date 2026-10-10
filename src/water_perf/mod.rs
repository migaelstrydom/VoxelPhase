//! Water budget bench: what the field costs the CPU per frame on a real
//! level, quiet, on the frame of a blast, and while it answers the blast;
//! and what it costs to open. Backs `src/bin/water_perf.rs`.

mod runner;

pub use runner::{field_table, run_field_level, Durations, FieldFrameCost, FieldSubject, WARM_UP};
