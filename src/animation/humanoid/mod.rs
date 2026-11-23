//! Humanoid-specific character animation: skeleton, stride phase, gait,
//! hanging from and climbing onto ledges, and lying knocked down.

pub mod body_frame;
pub mod down;
pub mod foot_slew;
pub mod gait;
pub mod hang;
pub mod pose_state;
pub mod skeleton;
pub mod stride_sync;
pub mod throw;
pub mod upper_state;

pub use body_frame::BodyFrame;
pub use foot_slew::FootSlew;
pub use gait::GaitCycle;
pub use hang::{FigureNow, Hang, HangPhase, LedgeReading};
pub use pose_state::{
    AirKind, Gait, GaitKey, PoseKey, PoseState, SampleCtx, Stroke, Takeoff, TickCtx,
};
pub use skeleton::{generate_character_mesh, Skeleton};
pub use upper_state::{UpperKey, UpperSampleCtx, UpperState, UpperTickCtx};
