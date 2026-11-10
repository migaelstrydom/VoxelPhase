//! Stilt walkers: two long legs under a haunch and a neck, with a peeper's eye on top or a costume in its place.

mod animator;
mod config;
mod costume;
mod mesh;
mod skeleton;
mod systems;

pub use animator::{Mood, StiltAnimator};
pub use config::StiltRigConfig;
pub use costume::{Costume, Garment};
pub use mesh::generate_stilt_mesh;
pub use skeleton::{FootPose, StiltSkeleton};
pub use systems::{StiltAnimationSystem, StiltProbeConfigSystem};
