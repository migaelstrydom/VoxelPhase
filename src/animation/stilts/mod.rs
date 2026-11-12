//! Stilt walkers: two long legs under a haunch and a neck, dressed in a model whose parts ride the rig's joints.

mod animator;
mod carriage;
mod config;
mod drape;
mod dress;
mod skeleton;
mod systems;

pub use animator::{Mood, StiltAnimator};
pub use carriage::{Carriage, CarriageMotion, Gait};
pub use config::StiltRigConfig;
pub use drape::{Drape, DrapeMotion};
pub use dress::{Dress, Drive, Joint, Worn};
pub use skeleton::{FootPose, StiltSkeleton};
pub use systems::{StiltAnimationSystem, StiltProbeConfigSystem};
