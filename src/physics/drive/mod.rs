//! The engine side of the drive seam.
//!
//! See `docs/TRACTION_DRIVE_DESIGN.md`. Gameplay's half of the seam — the
//! `DriveIntent`/`Actuator`/`BodyMotion` components — lives in `crate::drive`.
//! This module owns what the engine does with a command once it has crossed:
//! the command that crosses, the Support Set that says which contacts a body
//! may push against, the grip it is allowed at the contacts that are not among
//! them, and the world-anchored rows a medium anchor is delivered by.

pub mod command;
pub mod grip;
pub mod medium;
pub mod support;

pub use command::{DriveCommand, ReactionAnchor};
pub use grip::stamp_non_support_grip;
pub use support::{
    ContactSite, SupportConfig, SupportContact, SupportResolver, SupportSet, SupportSets,
};
