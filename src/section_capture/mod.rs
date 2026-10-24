//! Captures of settled physics scenes as traversal planner fixtures: a level
//! spawned headless, optionally blasted, left to come to rest, and sampled
//! into a strip from a start and heading.
//!
//! ```text
//!   level.ron ──RestTrial──▶ PhysicsWorld + TerrainWorld
//!                 blasts (radial impulses), then step until at rest
//!                                │
//!            PathFrame (start, heading) ──▶ PhysicsCaster ──sample_section──▶ Section
//!                                                                              │
//!                                     TraversalScene { ground: Captured(..) } ◀┘
//! ```

mod capture;

pub use capture::{capture, Blast, Capture, Captured};
