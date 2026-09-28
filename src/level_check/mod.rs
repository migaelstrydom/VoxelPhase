//! Offline validation and schematic export for level files.
//!
//! Drives the `level_check` binary, and runs headlessly — no Vulkan, no window —
//! so levels can be checked in tests and in any shell. Only the rest trial
//! touches the ECS, because spawnables build their bodies through it.

pub mod baseline;
pub mod placement;
pub mod reach;
pub mod report;
pub mod rest;
pub mod routes;
pub mod runner;
pub mod segments;
pub mod svg;
pub mod water;
pub mod water_plan;

pub use baseline::{BaselineVerdict, Baselines};
pub use reach::{JumpArc, JumpEnvelope, Stance};
pub use report::{Finding, Report, Section, Severity};
pub use rest::{check_rest, RestTrial};
pub use routes::RouteMap;
pub use runner::{build_terrain, check_level};
pub use segments::Crossing;
pub use svg::write_schematic;
pub use water::check_water;
pub use water_plan::{PlanCell, WaterPlan};
