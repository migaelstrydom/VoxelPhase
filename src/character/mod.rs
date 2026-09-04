//! Source-agnostic character locomotion.
//!
//! Everything here is shared by the player and by AI creatures. The pipeline is
//!
//! ```text
//!   PlayerInputSystem ─┐
//!                      ├─► CharacterIntent ─► CharacterControlSystem ─► physics
//!   BrainSystem ───────┘                                             └─► CharacterState
//! ```
//!
//! Nothing in this module knows whether a keyboard or a brain filled the
//! intent, which is what lets creatures inherit coyote time, air steering,
//! jump buffering and grabbing for free.

mod components;
mod config;
mod forgiveness;
pub mod grab;
mod grounding;
mod grounding_system;

pub use components::{
    AirSteering, ArmState, CharacterIntent, CharacterState, LocomotionInput, LocomotionOutcome,
    LocomotionState, MovementRule, Timer,
};
pub use config::LocomotionConfig;
pub use forgiveness::GroundForgiveness;
pub use grounding::Grounding;
pub use grounding_system::ContactGroundingSystem;
