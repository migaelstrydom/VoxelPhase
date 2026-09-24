//! Offline harness for water: scripted scenarios on small synthetic terrain,
//! run headlessly and reported as levels, volumes and events over time.
//!
//! Backs `src/bin/water_viewer.rs`.

mod driver;
mod legacy;
mod report;
mod scenario;
mod scenarios;

pub use driver::{run, Event, Run, RunConfig, Sample, TICK_RATE};
pub use legacy::{LegacyTickTimings, LegacyWater};
pub use report::{report, summary_line, write_csv};
pub use scenario::{Action, Beat, Probe, Scenario};
pub use scenarios::{catalogue, find, select};
