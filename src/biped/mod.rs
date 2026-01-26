//! Biped character animation system.
//!
//! This module contains all biped-specific animation logic in one place:
//! - `BipedConfig`: All tuning parameters
//! - `BipedState`: All runtime state
//! - `BipedSkeleton`: Joint positions and IK
//! - `BipedController`: Animation logic and state machine
//!
//! The controller is the single source of truth for biped animation.
//! Generic systems (probes, collision) have no knowledge of bipeds.

mod config;
mod controller;
mod gait;
mod skeleton;
mod state;
mod stride_wheel;
mod systems;

pub use config::BipedConfig;
pub use controller::BipedController;
pub use systems::{BipedAnimationSystem, BipedProbeConfigSystem};
