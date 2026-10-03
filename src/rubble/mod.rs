//! Terrain cut loose by a blast, and what becomes of it.
//!
//! `docs/terrain_rubble/DESIGN.md` is the plan. Every fragment crumbles into
//! dust for now; scree and boulders are later phases.

mod dust;
mod systems;

pub use dust::Crumble;
pub use systems::{RubbleQueue, RubbleSpawnSystem};
