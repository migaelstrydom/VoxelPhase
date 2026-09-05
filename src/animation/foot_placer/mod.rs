//! Capture-point–based procedural foot placement.
//!
//! See `docs/FOOT_PLACEMENT_PLAN.md`. The placer owns foot xz (and y
//! via probes) for all grounded states. It runs internally substepped
//! so step sequencing is frame-rate independent — see `placer.rs` for
//! the component diagram.

mod capture_point;
mod clock;
mod config;
mod placer;
mod recorder;
mod swing;
mod timing;

#[cfg(test)]
mod invariants;
#[cfg(test)]
mod replay;
#[cfg(test)]
mod scenarios;
#[cfg(test)]
mod sim;
#[cfg(test)]
mod trace;

pub use config::FootPlacerConfig;
pub use placer::{FootPhase, FootPlacer, FootSide, PlacerCtx, PlacerFoot};
pub use recorder::PlacerRecorder;
pub use timing::GaitTiming;
