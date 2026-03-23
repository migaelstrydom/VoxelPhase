//! Tracks water surface levels at sleeping body positions and wakes bodies
//! when the surface changes significantly (waves, flow redistribution).

use std::collections::HashMap;

use nalgebra::Point3;

use super::buoyancy::sample_water;
use super::{WaterGrid, WaveGrid};
use crate::physics::RigidBodyHandle;

/// Threshold for water level change that triggers a wake (meters).
const WAKE_THRESHOLD: f32 = 0.01;

/// Tracks the last-known water surface level at each buoyant body's position.
///
/// Each frame, awake bodies have their water level recorded. When a body is
/// sleeping, the tracker compares the current water level against the stored
/// value and returns handles that need waking.
pub struct WaterSleepTracker {
    /// Last-known water surface level per body, recorded while the body was awake.
    levels: HashMap<RigidBodyHandle, f32>,
}

impl WaterSleepTracker {
    pub fn new() -> Self {
        Self {
            levels: HashMap::new(),
        }
    }

    /// Record the current water level at an awake body's position.
    /// Call this each frame for all awake buoyant bodies.
    pub fn record(
        &mut self,
        handle: RigidBodyHandle,
        flow_grid: &WaterGrid,
        wave_grid: Option<&WaveGrid>,
        position: Point3<f32>,
    ) {
        if let Some(sample) = sample_water(flow_grid, wave_grid, position.x, position.z) {
            self.levels.insert(handle, sample.surface_level);
        } else {
            self.levels.remove(&handle);
        }
    }

    /// Check a sleeping body and return true if the water level changed enough
    /// to warrant waking it.
    pub fn should_wake(
        &self,
        handle: RigidBodyHandle,
        flow_grid: &WaterGrid,
        wave_grid: Option<&WaveGrid>,
        position: Point3<f32>,
    ) -> bool {
        let Some(&stored_level) = self.levels.get(&handle) else {
            return false;
        };
        match sample_water(flow_grid, wave_grid, position.x, position.z) {
            Some(sample) => (sample.surface_level - stored_level).abs() > WAKE_THRESHOLD,
            // Water disappeared — wake so the body can fall.
            None => true,
        }
    }

    /// Remove tracking for a body that no longer exists.
    pub fn remove(&mut self, handle: RigidBodyHandle) {
        self.levels.remove(&handle);
    }
}
