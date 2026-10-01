//! Offline gait harness: the real CharacterAnimator over a scripted body on analytic or moving ground, judged against the cadence the foot placer planned, and rendered as a filmstrip.

mod body;
mod driver;
mod film;
mod ground;
mod metrics;
mod report;
mod scenarios;
mod script;
mod support;
mod take;

pub use body::Body;
pub use driver::{run, travel_bounds, MeshCapture, Run, FRAME_RATE};
pub use film::{strip, Angle, FilmConfig};
pub use ground::{Drop, Flat, Ground, GroundArea, Ledge, Slope, SlopeOnset, Stairs, Undulating};
pub use metrics::{analyse, analyse_take, Check, FootMetrics, GaitMetrics, Verdict};
pub use report::{report, summary_line, write_csv};
pub use scenarios::{catalogue, find, select, Scenario};
pub use script::{Beat, Script};
pub use support::{Carried, SupportMotion};
pub use take::{CapturedMesh, FootSample, FrameSample, Side, Take, TimingSample};
