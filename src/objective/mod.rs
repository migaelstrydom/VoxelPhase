//! What a level asks the player to do: take every Platonic solid it holds,
//! then reach its goal.
//!
//! ```text
//!   Solid (Collectable) ──► CollectionSystem ──► LevelProgress ──► ObjectiveHudSystem
//!        │                                          ▲                   │
//!   HoverSystem                                     │              DebugLines
//!   (bob, turn, breath)                       GoalSystem ◄── Goal (radius,
//!                                                   │         reached, ruined)
//!                                                   ▼               ▲
//!                                             BrazierSystem ────────┘
//!                                             lights it when reached;
//!                                             ruins it when knocked over
//! ```
//!
//! [`LevelProgress`] is the single piece of state for the run: how many
//! solids exist, how many are taken, how long it has taken, and whether it
//! is won or lost. The objective is the game's score, kept outside the
//! room's physics (`docs/WORLD_AND_STORY.md`): taking a solid moves
//! nothing, and arriving early does nothing. The brazier itself is an
//! ordinary heavy body, so it can be knocked over, and then the level is
//! lost.

mod brazier;
mod goal;
mod hover;
mod progress;

pub use brazier::{Brazier, BrazierSystem};
pub use goal::{Goal, GoalSystem};
pub use hover::{Hover, HoverSystem};
pub use progress::{LevelProgress, ObjectiveHudSystem, ProgressSystem};
