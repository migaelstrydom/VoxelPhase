//! Ledge grabs: the edges the hands find, held in the frame of the body they
//! are on. Design: `docs/LEDGE_GRABS.md`.

mod config;
mod edge;
mod probe;
mod sensor;
mod system;

#[cfg(all(test, feature = "traversal_scenarios"))]
mod garden_tests;
#[cfg(test)]
mod tests;

pub use config::LedgeConfig;
pub use edge::{Edge, WorldEdge};
pub use probe::{edge_at_hands, Reach};
pub use sensor::LedgeSensor;
pub use system::LedgeSensorSystem;
