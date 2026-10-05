//! Offline harness for rubble: scripted blasts on small synthetic segments, judged by invariants.
//!
//! Backs `src/bin/rubble_viewer.rs`. Each scenario is a level, a script of
//! blasts and what it expects to come loose. After every blast the whole
//! terrain is audited: a blast may cut terrain loose, but it must never leave
//! more terrain standing free than there was before it. What is cut loose is
//! graded and launched by the game's own `RubblePlanner`, and every piece of
//! scree is flown against the remeshed terrain until it lands: none may land
//! on its first frame (it started inside the ground) or never land. Every
//! boulder is stepped as a body until it sleeps: none may leave free flight
//! in its first frame (the solver pushed it out of the ground), fall out of
//! the world or never come to rest.
//!
//! ```text
//!   Scenario { level RON, blasts, expect }
//!       │
//!       ▼
//!   run: per blast  TerrainWorld::detonate ──▶ fragments
//!                   RubblePlanner::plan ──▶ dust, scree flown to landing,
//!                                           or boulders stepped to sleep
//!                   TerrainWorld::loose_samples ──▶ the audit
//!        at the end TerrainWorld::update ──▶ open mesh edges
//!       │
//!       ▼
//!   Run ──▶ violations + the scenario's own expectations ──▶ report
//! ```
//!
//! No ECS: the harness calls the same `detonate` the explosion system does,
//! flies `Flight` directly, and places each blast's boulders with the game's
//! own `Boulder::place` in a `PhysicsWorld` of their own, stepped on the
//! terrain with the blast's shove on the first frame, as the game throws it.

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
