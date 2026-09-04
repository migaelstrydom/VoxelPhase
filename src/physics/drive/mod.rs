//! The engine side of the drive seam.
//!
//! See `docs/TRACTION_DRIVE_DESIGN.md`. Gameplay's half of the seam — the
//! `DriveIntent`/`Actuator`/`BodyMotion` components — lives in `crate::drive`.
//! This module owns what the engine does with a command once it has crossed:
//! today, the Support Set that says which contacts a body may push against.

pub mod support;

pub use support::{SupportConfig, SupportContact, SupportResolver, SupportSet, SupportSets};
