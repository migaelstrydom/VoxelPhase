//! Where a rendered frame's time goes, on both processors, and how much work
//! it was asked to do.
//!
//! ```text
//!   CPU  RenderStage ── Instant laps ──┐
//!   GPU  GpuSpan ─── GpuTimer (timestamp queries, read after the fence) ──┼─▶ RenderProfile
//!        RenderCounters ── draws, triangles, bytes uploaded ──────────────┘
//! ```

mod counters;
mod frame_profile;
mod gpu_span;
mod gpu_timer;
mod stage;

pub use counters::RenderCounters;
pub use frame_profile::RenderProfile;
pub use gpu_span::GpuSpan;
pub use gpu_timer::{GpuTimer, GpuTimings};
pub use stage::RenderStage;
