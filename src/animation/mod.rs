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
//!
//! Below the animator, `legged` is the part that is not about humanoids at
//! all: feet, probes, and the frame they are measured in. Any two-legged
//! thing can own a `LeggedLocomotion` and draw whatever rig it likes around
//! the feet that come out of it.

mod animator;
mod config;
pub mod critter;
mod debug_config;
mod foot_placer;
pub mod humanoid;
mod legged;
pub mod peeper;
mod pose;
pub mod rig;
mod state;
mod systems;

pub use animator::{BodyReading, CharacterAnimator};
pub use config::CharacterRigConfig;
pub use debug_config::AnimationDebugConfig;
pub use foot_placer::{FootPhase, FootPlacer, FootPlacerConfig, FootSide, GaitTiming, PlacerFoot};
pub use legged::{probe_tags, FootGround, LegRigDims, LeggedLocomotion, LocomotionCtx};
pub use pose::{
    BlendPolicy, Crossfade, Cycle, CycleKind, FeetPose, HandsPose, Linear, PoseFragment,
};
pub use systems::{AnimationProbeConfigSystem, CharacterAnimationSystem};
