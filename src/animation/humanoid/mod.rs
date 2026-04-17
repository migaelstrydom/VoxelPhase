//! Humanoid-specific character animation: skeleton, stride wheel, gait.

pub mod gait;
pub mod pose_state;
pub mod skeleton;
pub mod stride_wheel;
pub mod upper_state;

pub use gait::GaitCycle;
pub use pose_state::{AirKind, Gait, PoseState, SampleCtx, Takeoff, TickCtx};
pub use skeleton::{generate_character_mesh, Skeleton};
pub use upper_state::{UpperSampleCtx, UpperState, UpperTickCtx};
