//! The game's side of traversal planning: the physics world read into the
//! planner's path coordinates (`crates/traversal_planner`).

mod path_frame;
mod physics_caster;

pub use path_frame::PathFrame;
pub use physics_caster::PhysicsCaster;
