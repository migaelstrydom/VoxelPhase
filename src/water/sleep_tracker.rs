//! Tracks water surface levels at sleeping body positions and wakes bodies
//! when the surface changes significantly (waves, flow redistribution).

use std::collections::HashMap;

use nalgebra::Point3;

use super::buoyancy::WaterSurface;
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
        water: &dyn WaterSurface,
        position: Point3<f32>,
    ) {
        if let Some(sample) = water.sample(position) {
            self.levels.insert(handle, sample.surface_level);
        } else {
            self.levels.remove(&handle);
        }
    }

    /// Check a sleeping body and return true if the water level changed enough
    /// to warrant waking it.
    ///
    /// A sleeping body in water with no recorded level has never been at rest
    /// in it — it was created asleep, or fell asleep dry before the water
    /// arrived — so it wakes to find where it floats.
    pub fn should_wake(
        &self,
        handle: RigidBodyHandle,
        water: &dyn WaterSurface,
        position: Point3<f32>,
    ) -> bool {
        let stored_level = self.levels.get(&handle).copied();
        match (water.sample(position), stored_level) {
            (Some(sample), Some(stored)) => (sample.surface_level - stored).abs() > WAKE_THRESHOLD,
            (Some(_), None) => true,
            // Water disappeared — wake so the body can fall.
            (None, Some(_)) => true,
            (None, None) => false,
        }
    }

    /// Remove tracking for a body that no longer exists.
    pub fn remove(&mut self, handle: RigidBodyHandle) {
        self.levels.remove(&handle);
    }
}

#[cfg(test)]
mod tests {
    use generational_arena::Index;

    use super::*;
    use crate::water::buoyancy::{StillWater, WaterSample};

    fn handle(raw: usize) -> RigidBodyHandle {
        RigidBodyHandle(Index::from_raw_parts(raw, 0))
    }

    const POOL: StillWater = StillWater {
        surface: 0.0,
        floor: -2.0,
    };
    const RAISED: StillWater = StillWater {
        surface: 0.5,
        floor: -2.0,
    };

    struct Dry;

    impl WaterSurface for Dry {
        fn sample(&self, _point: Point3<f32>) -> Option<WaterSample> {
            None
        }
    }

    #[test]
    fn body_created_asleep_in_water_wakes() {
        let tracker = WaterSleepTracker::new();
        assert!(tracker.should_wake(handle(0), &POOL, Point3::origin()));
    }

    #[test]
    fn body_asleep_on_dry_ground_stays_asleep() {
        let tracker = WaterSleepTracker::new();
        assert!(!tracker.should_wake(handle(0), &Dry, Point3::origin()));
    }

    #[test]
    fn body_at_rest_in_still_water_stays_asleep() {
        let mut tracker = WaterSleepTracker::new();
        tracker.record(handle(0), &POOL, Point3::origin());
        assert!(!tracker.should_wake(handle(0), &POOL, Point3::origin()));
    }

    #[test]
    fn rising_water_wakes_a_body_at_rest_in_it() {
        let mut tracker = WaterSleepTracker::new();
        tracker.record(handle(0), &POOL, Point3::origin());
        assert!(tracker.should_wake(handle(0), &RAISED, Point3::origin()));
    }

    #[test]
    fn water_arriving_under_a_body_asleep_dry_wakes_it() {
        let mut tracker = WaterSleepTracker::new();
        tracker.record(handle(0), &Dry, Point3::origin());
        assert!(tracker.should_wake(handle(0), &POOL, Point3::origin()));
    }

    #[test]
    fn water_draining_from_under_a_body_wakes_it() {
        let mut tracker = WaterSleepTracker::new();
        tracker.record(handle(0), &POOL, Point3::origin());
        assert!(tracker.should_wake(handle(0), &Dry, Point3::origin()));
    }
}
