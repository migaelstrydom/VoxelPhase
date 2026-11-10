//! Pumps as things in the world: a body that lifts the water it stands in and throws it out, wherever it is carried.
//!
//! ```text
//!   WaterPump (on a body) ──PumpSystem, each frame──▶ WaterWorld::set_pump
//!     intake, spouts in the body's frame               a field pump per spout,
//!                                                      where the body stands now
//! ```
//!
//! The field does the pumping (`water::field::pumps`); this module only
//! keeps each pump where its body is, and takes it out when the body goes.
//! A pump built into a body that can break is described by its `Plumbing`,
//! from which `PlumbingShareSystem` works its `WaterPump` out again after
//! each break.

mod component;
mod plumbing;
mod plumbing_system;
mod system;

pub use component::{Spout, WaterPump};
pub use plumbing::{PipeSection, Plumbing};
pub use plumbing_system::PlumbingShareSystem;
pub use system::PumpSystem;
