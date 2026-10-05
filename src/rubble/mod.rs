//! Terrain cut loose by a blast: graded, then crumbled to dust, sent falling as scree, or made a boulder.
//!
//! `docs/terrain_rubble/DESIGN.md` is the plan. A boulder rests as a body until
//! the boulder budget crumbles it; it is never stamped back into the ground.

mod boulder;
mod brick_shaper;
mod budget;
mod crack;
mod dust;
mod grade;
mod scree;
mod spawn;

pub use boulder::{Boulder, PlacedBoulder};
pub use brick_shaper::{Brick, BrickShaper, Bricks, Shape};
pub use budget::{BoulderCullSystem, ResidentBoulder};
pub use crack::Cracker;
pub use dust::{Crumble, CrumbleSize};
pub use grade::{Grade, GradeRules, Measure};
pub use scree::{FallingScree, Flight, Outcome, ScreeRules, ScreeSystem};
pub use spawn::{Cut, Launch, Piece, Plan, RubblePlanner, RubbleQueue, RubbleSpawnSystem};
