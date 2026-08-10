//! Offline validation and schematic export for level files.
//!
//! Drives the `level_check` binary, and runs headlessly — no Vulkan, no window,
//! no ECS — so levels can be checked in tests and in any shell.

pub mod baseline;
pub mod placement;
pub mod reach;
pub mod report;
pub mod routes;
pub mod runner;
pub mod segments;
pub mod svg;

pub use baseline::{BaselineVerdict, Baselines};
pub use reach::{JumpArc, JumpEnvelope, Stance};
pub use report::{Finding, Report, Section, Severity};
pub use routes::RouteMap;
pub use runner::{build_terrain, check_level};
pub use segments::Crossing;
pub use svg::write_schematic;
