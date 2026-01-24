//! Procedural skeletal animation system.

mod biped;
mod components;
mod fabrik;
mod mesh;
mod verlet;

// Re-export main types used by the ECS
pub use biped::SpringBipedSkeleton;
pub use components::{SpringBipedCharacter, SpringBipedCharacterConfig};
