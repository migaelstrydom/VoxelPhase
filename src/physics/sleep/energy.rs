use rustc_hash::{FxHashMap, FxHashSet};

use generational_arena::Index;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

pub struct EnergyTracker {
    /// Kinetic energy threshold for sleep candidacy.
    threshold: f32,
    /// Frames below threshold required to qualify for sleep.
    delay_frames: u32,
    /// Per-body counters tracking consecutive below-threshold frames.
    frames_below: FxHashMap<Index, u32>,
}

impl EnergyTracker {
    pub fn new(threshold: f32, delay_frames: u32) -> Self {
        Self {
            threshold,
            delay_frames,
            frames_below: FxHashMap::default(),
        }
    }

    pub fn update_body(&mut self, handle: RigidBodyHandle, body: &RigidBody) -> bool {
        let energy = body.kinetic_energy();
        let entry = self.frames_below.entry(handle.0).or_insert(0);
        if energy < self.threshold {
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
