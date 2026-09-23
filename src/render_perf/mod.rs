//! Rendering performance bench: what a frame of the real game costs, on both
//! processors, while a grenade goes off.
//!
//! ```text
//!   Level ─▶ GameHarness (the game, on Renderer::offscreen)
//!              │ step × N, grenade dropped at the site
//!              ▼
//!            RenderRun (per frame: simulate / render wall, physics,
//!              │         CPU stages, GPU spans, counts)
//!              ▼
//!            report: quiet vs blast, worst frames, timeline, CSV
//! ```

mod harness;
mod record;
mod report;
mod runner;
mod scenario;

pub use harness::{FrameSample, GameHarness};
pub use record::{RenderFrameRecord, RenderRun};
pub use report::{summary, timeline, worst_frames, write_csv, DEFAULT_WINDOW};
pub use runner::{run, RunConfig};
pub use scenario::GrenadeBlast;
