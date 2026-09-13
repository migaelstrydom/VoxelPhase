//! Two-legged locomotion, independent of the rig drawn around it.

mod dims;
mod ground;
mod locomotion;

pub use dims::LegRigDims;
pub use ground::{probe_tags, FootGround};
pub use locomotion::{LeggedLocomotion, LocomotionCtx};
