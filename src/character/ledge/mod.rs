//! Ledge grabs: the edges the hands find, held in the frame of the body they
//! are on, and the joint the hands hold one by. The `Ledge` locomotion state
//! decides; this module senses and holds. Design: `docs/LEDGE_GRABS.md`.

mod config;
mod edge;
mod grip;
mod probe;
mod sensor;
mod system;

#[cfg(all(test, feature = "traversal_scenarios"))]
mod garden_tests;
#[cfg(test)]
mod tests;

pub use config::LedgeConfig;
pub use edge::{Edge, SensedEdge, WorldEdge};
pub use grip::{Grip, GripFrame};
pub use probe::{edge_at_hands, Reach};
pub use sensor::{EdgeAhead, LedgeSensor};
pub use system::LedgeSensorSystem;
