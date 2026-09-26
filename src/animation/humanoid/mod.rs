//! Humanoid-specific character animation: skeleton, stride phase, gait.

pub mod body_frame;
pub mod gait;
pub mod pose_state;
pub mod skeleton;
pub mod stride_sync;
pub mod upper_state;

pub use body_frame::BodyFrame;
pub use gait::GaitCycle;
pub use pose_state::{
    AirKind, Gait, GaitKey, PoseKey, PoseState, SampleCtx, Stroke, Takeoff, TickCtx,
};
pub use skeleton::{generate_character_mesh, Skeleton};
pub use upper_state::{UpperKey, UpperSampleCtx, UpperState, UpperTickCtx};
