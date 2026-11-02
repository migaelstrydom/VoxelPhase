//! The real game run headlessly with a scripted pilot in place of the keyboard, reporting the character's motion and animation against the water and how it crossed traversal stations, rendered as a filmstrip.

mod contract;
mod driver;
mod ledge_scenarios;
mod pilot;
mod record;
mod report;
mod scenario;
mod scenarios;
mod shrub_scenarios;
mod traversal;
mod traversal_scenarios;
mod turning;

pub use contract::{check_contract, ContractCheck};
pub use driver::{run, RunConfig};
pub use pilot::Pilot;
pub use record::{FootRecord, FrameRecord, Take, Tile};
pub use report::{report, traversal};
pub use scenario::{CameraRig, Judged, Scenario};
pub use scenarios::{catalogue, find};
pub use traversal::{summarise, Station, TraversalSpread, TraversalSummary};
pub use turning::TurnSummary;
