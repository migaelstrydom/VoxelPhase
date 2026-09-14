//! What a level asks the player to do.
//!
//! ```text
//!   Gem (Collectable) ──► CollectionSystem ──► LevelProgress ──► ObjectiveHudSystem
//!        │                                          ▲                   │
//!   GemMotionSystem                                 │              DebugLines
//!   (bob + spin)                              GoalSystem ◄── Goal (radius,
//!                                                             required_gems)
//! ```
//!
//! [`LevelProgress`] is the single piece of state: how many gems exist, how
//! many are caught, how long it has taken, and whether the goal has been
//! reached. Everything else either writes one of those fields or draws them.

mod gem;
mod goal;
mod progress;

pub use gem::{GemMotion, GemMotionSystem};
pub use goal::{Goal, GoalSystem};
pub use progress::{LevelProgress, ObjectiveHudSystem, ProgressSystem};
