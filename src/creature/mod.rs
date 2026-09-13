//! Creature AI runtime.
//!
//! A creature is a character whose intent comes from a brain instead of a
//! keyboard. Everything below the intent — the locomotion FSM, coyote time,
//! grounding, animation — is [`crate::character`], shared with the player.
//!
//! ```text
//!   PerceptionSystem ─► Perception ─► BrainSystem ─► CharacterIntent
//!         │                              │                 │
//!    sight cone,                     Behaviour FSM     CharacterControlSystem
//!    hearing, LOS,                   + steering              (shared)
//!    memory                          primitives
//! ```
//!
//! Perception answers "what is out there", the brain answers "what to do about
//! it", and steering answers "which way is that". Keeping the three apart is
//! what lets a new creature reuse two of them and rewrite only the third.
//!
//! The definitions of individual creatures — geometry, materials, physics —
//! live in `crate::app::creatures`, mirroring the spawnables library.

mod alert_telegraph;
mod brain;
mod collectable;
mod perception;
mod roller;
pub mod steering;

pub use alert_telegraph::{AlertTelegraph, AlertTelegraphSystem};
pub use brain::{Behaviour, Brain, BrainSystem, Temperament};
pub use collectable::{Collectable, CollectionSystem, Reward};
pub use perception::{PerceivedTarget, Perception, PerceptionSystem};
pub use roller::{Roller, RollerLocomotionSystem};
