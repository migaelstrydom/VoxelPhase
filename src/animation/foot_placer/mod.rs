//! Capture-point–based procedural foot placement.
//!
//! See `docs/FOOT_PLACEMENT_PLAN.md`. Stage 1: module is wired into
//! `CharacterAnimator` and ticks each frame, but foot xz is still
//! produced by `PoseState::Grounded::sample`. This module only visualises
//! where steps *would* fire so the capture-point math can be tuned
//! against real gameplay before it becomes authoritative.

mod capture_point;
mod config;
mod placer;
mod swing;

pub use config::FootPlacerConfig;
pub use placer::{FootPhase, FootPlacer, FootSide, PlacerCtx, PlacerFoot};
