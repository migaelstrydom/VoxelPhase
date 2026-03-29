//! Compound body fracture system.
//!
//! Monitors per-child-collider impulses on compound bodies and detaches
//! children whose impulse exceeds a break threshold, creating independent
//! rigid bodies for each freed piece.

mod components;
pub mod systems;

pub use components::{CompoundFracture, FractureJoint};
pub use systems::FractureSystem;
