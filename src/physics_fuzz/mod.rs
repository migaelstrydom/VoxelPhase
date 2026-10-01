//! Physics fuzzing: structures built from a seed, disturbed from a seed, and
//! judged against what physics allows rather than against an expected result.
//!
//! ```text
//!   seed ─▶ Case::generate ─▶ Case { structure, knocks, ball, walker }
//!                                   │
//!                                   ▼
//!                               run(case) ─▶ Verdict { violations }
//! ```
//!
//! The bench harness asks whether a scenario someone thought of behaves as
//! expected. This asks whether any of thousands nobody thought of breaks a
//! rule that holds for all of them: bodies nothing drives never gain energy
//! nothing gave them, never pass under the floor, and never go non-finite.
//! A failing seed replays exactly; once understood, it becomes
//! a bench test.

mod case;
mod run;
mod structure;
mod walker;

pub use case::{Case, Knock, Projectile};
pub use run::{run, RunOptions, TraceFrame, Verdict, Violation, ENERGY_GAIN_LIMIT, RUN_SECONDS};
pub use structure::{Structure, StructureKind};
pub use walker::{Walker, WalkerScript};
