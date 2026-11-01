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

mod attitude;
mod components;
mod config;
mod facing;
mod forgiveness;
pub mod grab;
mod grounding;
mod grounding_system;
mod immersion;
mod immersion_system;
pub mod locomotion;
mod swim;
mod throws;

pub use attitude::{approach, AttitudeControl};
pub use components::{ArmState, CharacterIntent, CharacterState};
pub use config::LocomotionConfig;
pub use facing::facing_from_rotation;
pub use forgiveness::GroundForgiveness;
pub use grounding::Grounding;
pub use grounding_system::ContactGroundingSystem;
pub use immersion::{Immersion, WaterAtBody};
pub use immersion_system::ImmersionSystem;
pub use locomotion::{
    AirSteering, Conditioning, EntryEffects, Facts, Foot, LocomotionInput, LocomotionState,
    MovementRule, PlannedLanding, PlannedTakeoff, Requests, RuleSpeeds,
};
pub use swim::SwimConfig;
pub use throws::{ThrowStroke, ThrowingHand, Throws};
