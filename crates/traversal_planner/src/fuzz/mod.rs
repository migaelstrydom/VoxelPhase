//! The fuzzer: random authored scenes planned and checked against every
//! property; a failing scene shrunk to the fewest shapes that still fail.
//!
//! ```text
//!   seed ──► generate ──► TraversalScene ──► sweep: plan, check_plan, check_replan
//!                                                 │ a violation
//!                                                 ▼
//!                                      shrink ──► minimal scene ──► fixture
//! ```

mod generate;
mod rng;
mod shrink;
mod sweep;

pub use generate::generate;
pub use rng::Rng;
pub use shrink::shrink;
pub use sweep::{try_scene, Trial};
