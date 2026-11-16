//! The real game run headlessly with a scripted pilot in place of the keyboard, reporting the character's motion and animation against the water, how it crossed traversal stations and whether its legs were drawn as legs (`pose_audit`), rendered as a filmstrip; `soak` plays seeded random scripts.

mod contract;
mod driver;
mod ledge_scenarios;
mod pilot;
mod pose_audit;
mod record;
mod report;
mod scenario;
mod scenarios;
mod shrub_scenarios;
mod soak;
mod traversal;
mod traversal_scenarios;
mod turning;

pub use contract::{check_contract, ContractCheck};
pub use driver::{run, RunConfig};
pub use pilot::Pilot;
pub use pose_audit::{Finding, Leg, Oddity, PoseAudit, StrideShape, Tally};
pub use record::{FootRecord, FrameRecord, Take, Tile};
pub use report::{report, traversal};
pub use scenario::{CameraRig, Judged, Scenario};
pub use scenarios::{catalogue, find};
pub use soak::{find_soak, soak_level, soak_scenario, SoakLevel, SOAK_LEVELS};
pub use traversal::{summarise, Station, TraversalSpread, TraversalSummary};
pub use turning::TurnSummary;
