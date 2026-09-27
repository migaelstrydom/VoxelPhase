use std::time::Duration;

use crate::physics::FrameProfile;

/// What one rendered frame's physics cost, and the state that explains it.
#[derive(Debug, Clone)]
pub struct FrameRecord {
    /// Simulated time at the end of the frame, in seconds.
    pub sim_time: f32,
    /// Wall-clock time of the whole frame's physics, measured outside the world.
    pub wall: Duration,
    /// The world's own stage-by-stage account of the frame.
    pub profile: FrameProfile,
    /// Dynamic bodies not asleep at the end of the frame.
    pub awake_bodies: usize,
    /// Contacts the narrowphase produced this frame.
    pub contacts: usize,
    /// Scenario disturbances delivered at the start of this frame.
    pub disturbances: usize,
}

impl FrameRecord {
    /// Time the stages do not account for: stepping overhead outside them.
    pub fn unaccounted(&self) -> Duration {
        self.wall.saturating_sub(self.profile.total())
    }
}

/// Every frame of one scenario, run at the game's cadence.
#[derive(Debug, Clone)]
pub struct PerfRun {
    pub scenario: String,
    /// The scenario's own one-line description.
    pub description: String,
    /// Which terrain it ran on, and where.
    pub ground: String,
    /// Dynamic bodies the scenario created.
    pub bodies: usize,
    /// Wall-clock budget of one rendered frame, in seconds.
    pub frame_dt: f32,
    /// How many runs each frame's numbers are the median of.
    pub repeats: usize,
    /// Hash of the final state of every body; see `runner::fingerprint`.
    pub fingerprint: u64,
    pub frames: Vec<FrameRecord>,
}
