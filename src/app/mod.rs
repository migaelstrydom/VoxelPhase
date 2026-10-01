//! Application wiring: the window and frame loop, ECS dispatcher setup, world assembly, the spawnable library and the camera/player spawners.

mod app;
pub(crate) mod creatures;
mod dispatcher_builder;
mod event_handler;
mod frame;
mod game_world;
mod placer_recording;
pub(crate) mod spawnables;
pub(crate) mod spawners;
mod system_timing;
pub(crate) mod world_builder;

pub use app::App;
pub use frame::{run_frame, FrameTiming};
pub use game_world::GameWorld;
pub use system_timing::{SystemTime, SystemTimings};
