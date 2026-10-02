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
//!
//! # The drive-aware surface (R11)
//!
//! R11 budgets the places in `src/physics/` that can tell a driven body from an
//! undriven one. This module is the mechanism and is drive-aware throughout by
//! construction; what the budget counts is the general engine code — body,
//! world, solver, pipeline — that still has to know. Three sites do, and each is
//! commented where it lives:
//!
//! - `RigidBody::drive` — the state itself, one `Option<BodyDrive>` replacing
//!   the two `velocity_drive` fields. Nothing in force integration, body
//!   integration or CCD reads it.
//! - `PhysicsWorld::set_body_drive`'s wake — a saturated row is not motion, so
//!   `EnergyTracker` would sleep a character leaning on a wall it cannot move.
//! - `PhysicsWorld`'s medium-drive lifetime — `set_medium_drive` /
//!   `retire_medium_drive` / `clear_body_drive`, which own the constraint
//!   handle a `BodyDrive::Medium` names.
//!
//! Three more sites were budgeted and are gone: the target shift and the
//! `integrate_forces` application went at Stage 5, and the solver's warm-start
//! persistence rule and restitution suppression went at Stage 7 — both existed
//! because a pre-solve drive re-asserted approach velocity every substep, and
//! a body whose velocity changes only through solved impulses does not.
//!
//! What the drive additionally puts into general engine code is **parameters**,
//! not tests: `RigidBody::non_support_grip` and `RigidBody::allowance`,
//! `SolverContact::traction` and `accumulated_torsional_impulse`, and
//! `PhysicsWorld::substeps_taken`. Every body and every contact carries them,
//! the solver reads them unconditionally, and their defaults are inert — so
//! none of them can answer "is this body driven?" without being told.
//!
//! See `docs/TRACTION_DRIVE_DESIGN.md` §8 (R11) for the full audit.

pub mod allowance;
pub mod command;
pub mod grip;
pub mod ledger;
pub mod medium;
pub mod plan;
pub mod support;

pub use allowance::{apply_allowances, Allowance, AllowanceCommand};
pub use command::{DriveCommand, ReactionAnchor, VerticalProjection, VerticalVerbs};
pub use grip::stamp_non_support_grip;
pub use ledger::{AllowanceLedger, AllowanceUsage, TractionLedger, TractionUsage};
pub use plan::{TractionPlanner, TractionRow};
pub use support::{
    ContactSite, SupportConfig, SupportContact, SupportResolver, SupportSet, SupportSets,
};
