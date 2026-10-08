//! Offline harness for water: scripted scenarios on small synthetic terrain,
//! run headlessly and reported as levels, volumes and events over time.
//!
//! Backs `src/bin/water_viewer.rs`.

mod driver;
mod field_driver;
#[cfg(test)]
mod field_tests;
mod hunt;
mod report;
mod scenario;
mod scenarios;
#[cfg(test)]
mod tests;

pub use driver::{run, run_with_captures, Event, Run, RunConfig, Sample, TICK_RATE};
pub use field_driver::{field_report, run_field, timing_line, FieldRun, FieldSample, SETTLE_LIMIT};
pub use report::{report, summary_line, write_csv};
pub use scenario::{Action, Beat, Probe, Scenario};
pub use scenarios::{catalogue, find, select};
