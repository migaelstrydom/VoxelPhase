//! Offline harness for water: scripted scenarios on small synthetic terrain,
//! run headlessly on the field and reported as levels, volumes and events
//! over time.
//!
//! Backs `src/bin/water_viewer.rs`.

mod driver;
mod hunt;
mod report;
mod scenario;
mod scenarios;
#[cfg(test)]
mod tests;

pub use driver::{run, Run, RunConfig, Sample, SETTLE_LIMIT, TICK_RATE};
pub use report::{report, summary_line, timing_line, write_csv};
pub use scenario::{Action, Beat, Probe, Scenario};
pub use scenarios::{catalogue, find, select};
