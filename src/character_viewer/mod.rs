//! The real game run headlessly with a scripted pilot in place of the keyboard, reporting the character's motion and animation against the water and how it crossed traversal stations, rendered as a filmstrip.

mod driver;
mod pilot;
mod record;
mod report;
mod scenario;
mod scenarios;
mod traversal;
mod traversal_scenarios;

pub use driver::{run, RunConfig};
pub use pilot::Pilot;
pub use record::{FootRecord, FrameRecord, Take, Tile};
pub use report::{report, traversal};
pub use scenario::{CameraRig, Scenario};
pub use scenarios::{catalogue, find};
pub use traversal::{summarise, Station, TraversalSummary};
