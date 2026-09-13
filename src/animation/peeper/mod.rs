//! A one-eyed stalker that walks on stilts.

mod animator;
mod config;
mod mesh;
mod skeleton;
mod systems;

pub use animator::{Mood, PeeperAnimator};
pub use config::PeeperRigConfig;
pub use mesh::generate_peeper_mesh;
pub use skeleton::{FootPose, PeeperSkeleton};
pub use systems::{PeeperAnimationSystem, PeeperProbeConfigSystem};
