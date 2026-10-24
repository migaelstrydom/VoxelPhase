//! What traversal planning costs a frame in the game's own physics world: a
//! level spawned headless, a body walked along a heading following its plan
//! exactly, and each frame's casts, sampling and planning timed.
//!
//! ```text
//!   level ──RestTrial──▶ PhysicsWorld ◀── blasts, then stepped every frame
//!                             │
//!   body ──▶ TraversalPlanning::update ──▶ PlanFrame (plan, section, cost)
//!     ▲                                         │
//!     └──────── the plan a frame on ◀───────────┘
//! ```

mod run;

pub use run::{run, FrameRecord, PerfRun, PerfSpec, StripDifference};
