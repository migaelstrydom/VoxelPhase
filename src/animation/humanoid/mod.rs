//! Humanoid-specific character animation: skeleton, stride wheel, gait.

pub mod gait;
pub mod skeleton;
pub mod stride_wheel;

pub use gait::GaitCycle;
pub use skeleton::{generate_character_mesh, Skeleton};
