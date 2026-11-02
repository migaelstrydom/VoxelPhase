//! The game's side of traversal planning: the physics world read into the
//! planner's path coordinates (`crates/traversal_planner`), planned over
//! frame by frame for the player, drawn and captured. Its world-space casts
//! (`WorldCaster`) also serve the hands' ledge sensor. The character's
//! locomotion state sets the planner's role (`PlanRole`); the plan stands
//! aside while the player rides a carrier (a moving platform).

mod carriers;
mod gait;
mod legs;
mod marker;
mod path_frame;
mod physics_caster;
mod plan_capture;
mod planning;
mod role;
mod section_cache;
mod strip_scenery;
mod system;
mod tracking;
mod wall_slide;
mod world_caster;

pub use carriers::{carriers, Riding, StandsOn};
pub use gait::player_gait;
pub use legs::Legs;
pub use marker::PlansTraversal;
pub use path_frame::PathFrame;
pub use physics_caster::{strip_reach, PhysicsCaster};
pub use planning::{PlanFrame, PlanningCost, PlanningInput, TraversalPlanning};
pub use role::PlanRole;
pub use section_cache::SectionCache;
pub use strip_scenery::{SceneryView, StripScenery};
pub use system::{TraversalPlanState, TraversalPlanSystem};
pub use wall_slide::{LaidPath, WallSlide};
pub use world_caster::{WorldCaster, WorldHit};
