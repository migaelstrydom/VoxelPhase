//! The engine side of the drive seam.
//!
//! See `docs/TRACTION_DRIVE_DESIGN.md`. Gameplay's half of the seam — the
//! `DriveIntent`/`Actuator`/`BodyMotion` components — lives in `crate::drive`.
//! This module owns what the engine does with a command once it has crossed:
//! the command that crosses, the Support Set that says which contacts a body
//! may push against, the grip it is allowed at the contacts that are not among
//! them, the per-contact targets a support anchor is delivered by, the
//! world-anchored rows a medium anchor is delivered by, and the allowance —
//! the one place a body is permitted momentum no contact could have given it.

pub mod allowance;
pub mod command;
pub mod grip;
pub mod ledger;
pub mod medium;
pub mod plan;
pub mod support;

pub use allowance::{apply_allowances, Allowance, AllowanceCommand};
pub use command::{DriveCommand, NormalProjection, NormalVerbs, ReactionAnchor};
pub use grip::stamp_non_support_grip;
pub use ledger::{AllowanceLedger, AllowanceUsage, TractionLedger, TractionUsage};
pub use plan::{TractionPlanner, TractionRow};
pub use support::{
    ContactSite, SupportConfig, SupportContact, SupportResolver, SupportSet, SupportSets,
};
