//! The character's locomotion state machine: what the character is doing
//! (`OnFoot`, `Hop`, `Jump`, `Fall`, `Swim`), when that changes, and what
//! changing it does. Design: `docs/LOCOMOTION_STATES.md`.
//!
//! ```text
//!   CharacterIntent, Grounding ─► Conditioning ─► Facts, Requests
//!                                      ▲                │
//!                                      │                ▼
//!                                      │     LocomotionState::tick ─► next state
//!                                      │     LocomotionState::enter ─► EntryEffects
//!                                      └── consume, cancel, arm ◄─┘      │
//!                                                      driver ◄─ impulse, air speed, cut
//! ```
//!
//! One file per state: each holds that state's exits, its movement rule and
//! its answers to the questions that cut across states.

mod conditioning;
mod effects;
mod fall;
mod hop;
mod input;
mod jump;
mod on_foot;
mod requests;
mod rule;
mod state;
mod swim;
mod takeoff;

#[cfg(test)]
mod tests;

pub use conditioning::Conditioning;
pub use effects::EntryEffects;
pub use fall::Fall;
pub use hop::Hop;
pub use input::{Facts, LocomotionInput, Requests};
pub use jump::{Jump, JumpKind};
pub use on_foot::OnFoot;
pub use requests::{Foot, PlannedLanding, PlannedTakeoff};
pub use rule::{AirSteering, MovementRule, RuleSpeeds};
pub use state::LocomotionState;
pub use swim::Swim;
pub use takeoff::TakeoffPhase;
