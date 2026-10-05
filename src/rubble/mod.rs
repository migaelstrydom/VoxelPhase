//! Terrain cut loose by a blast: graded, then crumbled to dust, sent falling as scree, or made a boulder.
//!
//! `docs/terrain_rubble/DESIGN.md` is the plan. A boulder that comes to rest
//! is stamped back into the terrain (`settle`); one that never may is left to
//! the boulder budget, which crumbles it.

mod boulder;
mod brick_shaper;
mod budget;
mod crack;
mod dust;
mod grade;
mod scree;
mod settle;
mod spawn;

pub use boulder::{Boulder, PlacedBoulder};
pub use brick_shaper::{Brick, BrickShaper, Bricks, Shape};
pub use budget::{BoulderCullSystem, ResidentBoulder};
pub use crack::Cracker;
pub use dust::{Crumble, CrumbleSize};
pub use grade::{Grade, GradeRules, Measure};
pub use scree::{FallingScree, Flight, Outcome, ScreeRules, ScreeSystem};
pub use settle::{SettleSystem, Settler, Settling, Verdict};
pub use spawn::{Cut, Launch, Piece, Plan, RubblePlanner, RubbleQueue, RubbleSpawnSystem};
