//! A small heart-shaped creature that runs on two legs.

mod animator;
mod config;
mod ears;
mod mesh;
mod skeleton;
mod systems;

pub use animator::CritterAnimator;
pub use config::{CritterRigConfig, EarConfig};
pub use ears::{EarSide, Ears};
pub use mesh::generate_critter_mesh;
pub use skeleton::{CritterSkeleton, FootPose};
pub use systems::{CritterAnimationSystem, CritterProbeConfigSystem};
