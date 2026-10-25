use std::time::{Duration, Instant};

/// Tracks frame timing for consistent game updates.
/// This is an ECS resource that systems can read for delta time.
#[derive(Debug)]
pub struct Time {
    /// Game time elapsed since last frame: the wall clock's, times `scale`.
    delta: Duration,
    /// Delta time as f32 seconds (cached for convenience)
    delta_seconds: f32,
    /// Wall-clock seconds since last frame.
    real_delta_seconds: f32,
    /// How fast game time runs against the wall clock: 1 at normal speed,
    /// below it in slow motion.
    scale: f32,
    /// When the last frame started
    last_frame: Instant,
    /// Total time since game start
    total: Duration,
    /// Frame count since start
    frame_count: u64,
}

impl Time {
    pub fn new() -> Self {
        Self {
            delta: Duration::ZERO,
            delta_seconds: 0.0,
            real_delta_seconds: 0.0,
            scale: 1.0,
            last_frame: Instant::now(),
            total: Duration::ZERO,
            frame_count: 0,
        }
    }

    /// A clock that reports the same step every frame, for offline harnesses
    /// and tests that drive systems without a real frame loop.
    pub fn fixed(delta_seconds: f32) -> Self {
        Self {
            delta: Duration::from_secs_f32(delta_seconds),
            delta_seconds,
            real_delta_seconds: delta_seconds,
            scale: 1.0,
            last_frame: Instant::now(),
            total: Duration::ZERO,
            frame_count: 0,
        }
    }

    /// Advance a clock made by [`Time::fixed`] by its own step, so its
    /// total and frame count move exactly as a real frame loop's would.
    pub fn advance_fixed(&mut self) {
        self.total += self.delta;
        self.frame_count += 1;
    }

    /// Call at the start of each frame to update timing.
    pub fn update(&mut self) {
        let now = Instant::now();
        let real = now.duration_since(self.last_frame);
        self.last_frame = now;
        self.real_delta_seconds = real.as_secs_f32();

        // Cap delta time to avoid physics explosions on frame hitches
        // 100ms = 10 FPS minimum
        const MAX_DELTA: Duration = Duration::from_millis(100);
        self.delta = real.min(MAX_DELTA).mul_f32(self.scale);

        self.delta_seconds = self.delta.as_secs_f32();
        self.total += self.delta;
        self.frame_count += 1;
    }

    /// Time elapsed since last frame in seconds (f32)
    #[inline]
    pub fn delta_seconds(&self) -> f32 {
        self.delta_seconds
    }

    /// Wall-clock seconds since last frame, whatever the game's speed.
    #[inline]
    pub fn real_delta_seconds(&self) -> f32 {
        self.real_delta_seconds
    }

    /// How fast game time runs against the wall clock.
    pub fn scale(&self) -> f32 {
        self.scale
    }

    /// Runs game time at `scale` times the wall clock from the next frame on.
    pub fn set_scale(&mut self, scale: f32) {
        self.scale = scale;
    }

    /// Total elapsed time since game start in seconds (f32).
    #[inline]
    pub fn total_seconds(&self) -> f32 {
        self.total.as_secs_f32()
    }
}

impl Default for Time {
    fn default() -> Self {
        Self::new()
    }
}
