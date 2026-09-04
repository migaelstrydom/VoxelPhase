//! The drive channel: how gameplay asks for momentum, and what it declares
//! about the way that momentum is produced.
//!
//! See `docs/TRACTION_DRIVE_DESIGN.md`. This module owns the ECS side of the
//! seam — the three components and the translation that turns them into a
//! command for the physics engine. The engine side arrives at later stages.

pub mod components;
pub mod translate;

pub use components::{
    Actuator, BodyMotion, DriveIntent, NormalProjection, NormalVerbs, ReactionAnchor,
};
pub use translate::{resolve_drive, DriveTarget};
