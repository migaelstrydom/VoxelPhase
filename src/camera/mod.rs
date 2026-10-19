//! The follow-target camera and its configuration.

mod clearance;
mod config;
mod follow_target;

pub use clearance::{CameraClearance, ClearanceState, Scenery};
pub use config::CameraConfig;
pub use follow_target::FollowTarget;
