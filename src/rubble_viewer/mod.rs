//! Offline harness for rubble: scripted blasts on small synthetic segments, judged by invariants.
//!
//! Backs `src/bin/rubble_viewer.rs`. Each scenario is a level, a script of
//! blasts and what it expects to come loose. After every blast the whole
//! terrain is audited: a blast may cut terrain loose, but it must never leave
//! more terrain standing free than there was before it.
//!
//! ```text
//!   Scenario { level RON, blasts, expect }
//!       │
//!       ▼
//!   run: per blast  TerrainWorld::detonate ──▶ fragments
//!                   TerrainWorld::loose_samples ──▶ the audit
//!        at the end TerrainWorld::update ──▶ open mesh edges
//!       │
//!       ▼
//!   Run ──▶ violations + the scenario's own expectations ──▶ report
//! ```
//!
//! Fragments only crumble into dust so far, so the terrain is all there is to
//! judge, and the harness calls the same `detonate` the explosion system does.
//! Once fragments become bodies (Phase 3 of `docs/TERRAIN_RUBBLE_DESIGN.md`)
//! it has to run the game's own systems instead.

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
