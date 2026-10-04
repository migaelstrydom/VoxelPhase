//! Terrain cut loose by a blast: graded, then crumbled to dust or sent falling as scree.
//!
//! `docs/terrain_rubble/DESIGN.md` is the plan. Boulders fly as scree until
//! they become rigid bodies (Phase 3), and nothing settles back into the
//! ground yet (Phase 4).

mod dust;
mod grade;
mod scree;
mod spawn;

pub use dust::{Crumble, CrumbleSize};
pub use grade::{Grade, GradeRules, Measure};
pub use scree::{FallingScree, Flight, Outcome, ScreeRules, ScreeSystem};
pub use spawn::{Cut, Launch, Plan, RubblePlanner, RubbleQueue, RubbleSpawnSystem};
