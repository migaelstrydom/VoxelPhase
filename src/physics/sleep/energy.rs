use rustc_hash::{FxHashMap, FxHashSet};

use generational_arena::Index;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

pub struct SleepTracker {
    /// Squared linear velocity threshold for sleep candidacy.
    linear_threshold_sq: f32,
    /// Squared angular velocity threshold for sleep candidacy.
    angular_threshold_sq: f32,
    /// Frames below threshold required to qualify for sleep.
    delay_frames: u32,
    /// Per-body counters tracking consecutive below-threshold frames.
    frames_below: FxHashMap<Index, u32>,
}

impl SleepTracker {
    pub fn new(linear_threshold: f32, angular_threshold: f32, delay_frames: u32) -> Self {
        Self {
            linear_threshold_sq: linear_threshold * linear_threshold,
            angular_threshold_sq: angular_threshold * angular_threshold,
            delay_frames,
            frames_below: FxHashMap::default(),
        }
    }

    pub fn update_body(&mut self, handle: RigidBodyHandle, body: &RigidBody) -> bool {
        let lin_sq = body.linear_velocity().norm_squared();
        let ang_sq = body.angular_velocity().norm_squared();
        let below = lin_sq < self.linear_threshold_sq && ang_sq < self.angular_threshold_sq;
        let entry = self.frames_below.entry(handle.0).or_insert(0);
        if below {
            *entry = entry.saturating_add(1);
        } else {
            *entry = 0;
        }
        *entry >= self.delay_frames
    }

    pub fn clear_body(&mut self, handle: RigidBodyHandle) {
        self.frames_below.remove(&handle.0);
    }

    pub fn retain_indices(&mut self, live: &FxHashSet<Index>) {
        self.frames_below.retain(|idx, _| live.contains(idx));
    }
}
