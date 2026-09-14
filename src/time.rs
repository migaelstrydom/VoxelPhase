use std::time::{Duration, Instant};

/// Tracks frame timing for consistent game updates.
/// This is an ECS resource that systems can read for delta time.
#[derive(Debug)]
pub struct Time {
    /// Time elapsed since last frame
    delta: Duration,
    /// Delta time as f32 seconds (cached for convenience)
    delta_seconds: f32,
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
            last_frame: Instant::now(),
            total: Duration::ZERO,
            frame_count: 0,
        }
    }

    /// Call at the start of each frame to update timing
    /// A clock that reports the same step every frame, for offline harnesses
    /// and tests that drive systems without a real frame loop.
    pub fn fixed(delta_seconds: f32) -> Self {
        Self {
            delta: Duration::from_secs_f32(delta_seconds),
            delta_seconds,
            last_frame: Instant::now(),
            total: Duration::ZERO,
            frame_count: 0,
        }
    }

    pub fn update(&mut self) {
        let now = Instant::now();
        self.delta = now.duration_since(self.last_frame);
        self.last_frame = now;

        // Cap delta time to avoid physics explosions on frame hitches
        // 100ms = 10 FPS minimum
        const MAX_DELTA: Duration = Duration::from_millis(100);
        if self.delta > MAX_DELTA {
            self.delta = MAX_DELTA;
        }

        self.delta_seconds = self.delta.as_secs_f32();
        self.total += self.delta;
        self.frame_count += 1;
    }

    /// Time elapsed since last frame in seconds (f32)
    #[inline]
    pub fn delta_seconds(&self) -> f32 {
        self.delta_seconds
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
