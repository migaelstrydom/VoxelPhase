use std::time::Duration;

use crate::app::FrameTiming;
use crate::rendering::profile::RenderProfile;

/// One frame of the game: what it cost on both processors and what it drew.
#[derive(Debug, Clone)]
pub struct RenderFrameRecord {
    /// Simulated time at the end of the frame, in seconds.
    pub sim_time: f32,
    /// Wall-clock time of the frame's simulate and render halves.
    pub timing: FrameTiming,
    /// The physics world's own total for the frame, inside `timing.simulate`.
    pub physics: Duration,
    /// The renderer's account of the frame: CPU stages, GPU spans, counts.
    pub profile: RenderProfile,
}

impl RenderFrameRecord {
    /// The whole frame on the CPU, simulate and render together.
    pub fn wall(&self) -> Duration {
        self.timing.simulate + self.timing.render
    }

    /// The frame's whole GPU time, if the timestamps landed.
    pub fn gpu_total(&self) -> Option<Duration> {
        self.profile.gpu.and_then(|gpu| gpu.total)
    }
}

/// Every frame of one run, and when the blast happened in it.
#[derive(Debug, Clone)]
pub struct RenderRun {
    /// The level file the run was played on.
    pub level: String,
    pub width: u32,
    pub height: u32,
    /// Simulated seconds per frame.
    pub frame_dt: f32,
    /// When the grenade was put down, in simulated seconds.
    pub dropped_at: f32,
    /// When it went off, if it did before the run ended.
    pub detonated_at: Option<f32>,
    pub frames: Vec<RenderFrameRecord>,
}
