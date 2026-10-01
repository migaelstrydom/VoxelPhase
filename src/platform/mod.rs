//! Moving platforms: waypoint routes, thrust-and-drag seek motion, and decks that tip under a load.

pub mod components;
pub mod route;
pub mod seek;
pub mod suspension;
pub mod system;

pub use components::{MovingPlatform, DEFAULT_ROUTE_GAIN};
pub use route::{Route, RouteLoop};
pub use seek::{SeekMotion, SeekState};
pub use suspension::{DeckSuspension, DeckTuning, REFERENCE_LOAD_KG};
pub use system::MovingPlatformSystem;
