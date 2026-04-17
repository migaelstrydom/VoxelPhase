//! Character animation system.
//!
//! This module contains character animation logic in one place:
//! - `CharacterRigConfig`: All tuning parameters
//! - `AnimationState`: All runtime state
//! - `Skeleton`: Joint positions and IK (humanoid)
//! - `CharacterAnimator`: Animation logic and state machine
//!
//! The animator is the single source of truth for character animation.
//! Generic systems (probes, collision) have no knowledge of the rig.

mod animator;
mod config;
pub mod humanoid;
mod state;
mod systems;

pub use animator::CharacterAnimator;
pub use config::CharacterRigConfig;
pub use systems::{AnimationProbeConfigSystem, CharacterAnimationSystem};
