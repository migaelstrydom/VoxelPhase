//! ECS systems that glue subsystems together: input, character control, camera, physics sync, terrain, water and rendering.

mod camera;
mod character_control;
mod physics_sync;
mod player_input;
mod render;
mod terrain_anchor;
mod terrain_update;
mod water;

pub use camera::CameraControlSystem;
pub use character_control::{resolve_ground_speed, CharacterControlSystem};
pub use physics_sync::{PhysicsResource, PhysicsSyncSystem};
pub use player_input::PlayerInputSystem;
pub use render::{probe_owner, FrameStart, RenderSystem};
pub use terrain_anchor::TerrainAnchorSystem;
pub use terrain_update::TerrainUpdateSystem;
pub use water::WaterSystem;
