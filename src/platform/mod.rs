pub mod components;
pub mod system;

pub use components::{MovingPlatform, DEFAULT_ARRIVAL_RADIUS, DEFAULT_ROUTE_GAIN};
pub use system::MovingPlatformSystem;
