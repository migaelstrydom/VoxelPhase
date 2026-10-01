//! The real game run headlessly with a scripted pilot in place of the keyboard, reporting the character's motion and animation against the water, rendered as a filmstrip.

mod driver;
mod pilot;
mod record;
mod report;
mod scenario;
mod scenarios;

pub use driver::{run, RunConfig};
pub use pilot::Pilot;
pub use record::{FrameRecord, Take, Tile};
pub use report::report;
pub use scenario::{CameraRig, Scenario};
pub use scenarios::{catalogue, find};
