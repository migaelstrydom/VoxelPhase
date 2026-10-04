//! Offline harness for rubble: scripted blasts on small synthetic segments, judged by invariants.
//!
//! Backs `src/bin/rubble_viewer.rs`. Each scenario is a level, a script of
//! blasts and what it expects to come loose. After every blast the whole
//! terrain is audited: a blast may cut terrain loose, but it must never leave
//! more terrain standing free than there was before it. What is cut loose is
//! graded and launched by the game's own `RubblePlanner`, and every piece of
//! scree is flown against the remeshed terrain until it lands: none may land
//! on its first frame (it started inside the ground) or never land.
//!
//! ```text
//!   Scenario { level RON, blasts, expect }
//!       │
//!       ▼
//!   run: per blast  TerrainWorld::detonate ──▶ fragments
//!                   RubblePlanner::plan ──▶ dust, or scree flown to landing
//!                   TerrainWorld::loose_samples ──▶ the audit
//!        at the end TerrainWorld::update ──▶ open mesh edges
//!       │
//!       ▼
//!   Run ──▶ violations + the scenario's own expectations ──▶ report
//! ```
//!
//! Scree flies on its own integrator, so the harness calls the same
//! `detonate` the explosion system does and flies `Flight` directly. Once
//! fragments become bodies (Phase 3 of `docs/terrain_rubble/DESIGN.md`) it
//! has to run the game's own systems instead.

mod driver;
mod garden;
mod report;
mod scenario;
mod scenarios;
#[cfg(test)]
mod tests;

pub use driver::{run, BlastRecord, FragmentRecord, Run};
pub use report::report;
pub use scenario::{Blast, Scenario};
pub use scenarios::{catalogue, find};
