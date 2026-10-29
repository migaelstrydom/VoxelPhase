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
mod planned;
mod recorder;
mod swing;
mod timing;

#[cfg(test)]
mod chase;
#[cfg(test)]
mod invariants;
#[cfg(test)]
mod plan_fuzz;
#[cfg(test)]
mod replay;
#[cfg(test)]
mod scenarios;
#[cfg(test)]
mod sim;
#[cfg(test)]
mod trace;

pub use config::FootPlacerConfig;
pub(crate) use placer::{floor_height_at, is_floor};
pub use placer::{FootPhase, FootPlacer, FootSide, PlacerCtx, PlacerFoot};
pub use planned::{PlannedStep, PlannedSteps};
pub use recorder::PlacerRecorder;
pub use swing::Swing;
pub use timing::GaitTiming;
