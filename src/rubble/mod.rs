//! Terrain cut loose by a blast: graded, then crumbled to dust, sent falling as scree, or made a boulder.
//!
//! `docs/terrain_rubble/DESIGN.md` is the plan. Nothing settles back into the
//! ground yet (Phase 4): a boulder rests as a body until the debris budget
//! takes it.

mod boulder;
mod brick_shaper;
mod crack;
mod dust;
mod grade;
mod scree;
mod spawn;

pub use boulder::{Boulder, PlacedBoulder};
pub use brick_shaper::{Brick, BrickShaper, Bricks, Shape};
pub use crack::Cracker;
pub use dust::{Crumble, CrumbleSize};
pub use grade::{Grade, GradeRules, Measure};
pub use scree::{FallingScree, Flight, Outcome, ScreeRules, ScreeSystem};
pub use spawn::{Cut, Launch, Piece, Plan, RubblePlanner, RubbleQueue, RubbleSpawnSystem};
