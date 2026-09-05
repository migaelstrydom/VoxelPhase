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
mod debug_config;
mod foot_placer;
pub mod humanoid;
mod pose;
mod state;
mod systems;

pub use animator::{probe_tags, CharacterAnimator};
pub use config::CharacterRigConfig;
pub use debug_config::AnimationDebugConfig;
pub use foot_placer::{FootPhase, FootPlacer, FootPlacerConfig, FootSide, GaitTiming, PlacerFoot};
pub use pose::{
    BlendPolicy, Crossfade, Cycle, CycleKind, FeetPose, HandsPose, Linear, PoseFragment,
};
pub use systems::{AnimationProbeConfigSystem, CharacterAnimationSystem};
