//! Capture-point–based procedural foot placement.
//!
//! See `docs/FOOT_PLACEMENT_PLAN.md`. The placer owns foot xz (and y
//! via probes) for all grounded states. It runs internally substepped
//! so step sequencing is frame-rate independent — see `placer.rs` for
//! the component diagram. Its stages are in the files named for what they
//! work on: `pace`, `stance`, `stepping` and `ankle`.

mod ankle;
mod capture_point;
mod clock;
mod config;
mod envelope;
mod floor;
mod geometry;
mod leg;
mod mode;
mod pace;
mod placer;
mod planned;
mod recorder;
mod stance;
mod stepping;
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
pub use envelope::LegEnvelope;
pub(crate) use floor::{floor_height_at, is_floor};
pub use mode::LegMode;
pub use placer::{FootPhase, FootPlacer, FootSide, PlacerCtx, PlacerFoot};
pub use planned::{PlannedStep, PlannedSteps, StepId};
pub use recorder::PlacerRecorder;
pub use swing::Swing;
pub use timing::GaitTiming;
