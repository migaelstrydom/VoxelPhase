//! ECS systems that glue subsystems together: input, character control, camera, physics sync, terrain, water, drawing the parts of breakable bodies, and rendering.

mod camera;
mod character_control;
mod fall_spray;
mod front_spray;
mod part_draw;
mod physics_sync;
mod player_input;
mod render;
mod spray_budget;
mod terrain_anchor;
mod terrain_update;
mod water;

pub use camera::CameraControlSystem;
pub use character_control::{resolve_ground_speed, CharacterControlSystem};
pub use part_draw::PartDrawSystem;
pub use physics_sync::{PhysicsResource, PhysicsSyncSystem};
pub use player_input::PlayerInputSystem;
pub use render::{entity_rotation, probe_owner, FrameStart, RenderSystem};
pub use terrain_anchor::{AnchorShareSystem, TerrainAnchorSystem};
pub use terrain_update::TerrainUpdateSystem;
pub use water::WaterSystem;
