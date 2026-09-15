//! Compound body fracture system.
//!
//! Monitors per-child-collider impulses on compound bodies and detaches
//! children whose impulse exceeds a break threshold, creating independent
//! rigid bodies for each freed piece.

mod components;
mod contact_load;
mod debris;
mod load;
pub mod systems;

pub use components::{CompoundFracture, FractureJoint};
pub use contact_load::{ContactLoadTracker, ContactSpike, Deadband};
pub use debris::{Debris, DebrisBudget, DebrisCullSystem};
pub use load::{ChildLoad, ChildLoads};
pub use systems::FractureSystem;
