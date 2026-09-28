use rustc_hash::{FxHashMap, FxHashSet};

use generational_arena::Index;
use nalgebra::{Point3, UnitQuaternion};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

/// Decides which bodies have been still long enough to sleep.
///
/// Still means both slow and not moving. The two differ: position correction
/// moves a body out of whatever it is sunk into without giving it any
/// velocity, so a body pushed out of a pillar it is skewered on reads as
/// motionless from its velocity alone, and would sleep there, still skewered.
pub struct SleepTracker {
    /// Squared linear speed threshold for sleep candidacy.
    linear_threshold_sq: f32,
    /// Squared angular speed threshold for sleep candidacy.
    angular_threshold_sq: f32,
    /// Substeps below threshold required to qualify for sleep.
    delay_frames: u32,
    /// Per-body record of how long it has been still, and where it was.
    bodies: FxHashMap<Index, Stillness>,
}

/// One body's record.
struct Stillness {
    /// Consecutive updates the body has been still for.
    frames_below: u32,
    /// Where the body was at the last update.
    position: Point3<f32>,
    /// How the body was turned at the last update.
    rotation: UnitQuaternion<f32>,
}

impl SleepTracker {
    pub fn new(linear_threshold: f32, angular_threshold: f32, delay_frames: u32) -> Self {
        Self {
            linear_threshold_sq: linear_threshold * linear_threshold,
            angular_threshold_sq: angular_threshold * angular_threshold,
            delay_frames,
            bodies: FxHashMap::default(),
        }
    }

    /// Record the body after a substep `dt` long, and whether it has now
    /// been still for long enough to sleep.
    pub fn update_body(&mut self, handle: RigidBodyHandle, body: &RigidBody, dt: f32) -> bool {
        let (position, rotation) = (body.position(), body.rotation());
        let slow = self.is_below(
            body.linear_velocity().norm_squared(),
            body.angular_velocity().norm_squared(),
        );
        // A body seen for the first time has not been watched moving yet:
        // its speed is all there is to go on.
        let unmoved = self.bodies.get(&handle.0).map_or(true, |record| {
            let travelled = (position - record.position).norm_squared();
            let turned = record.rotation.angle_to(&rotation).powi(2);
            self.is_below(travelled / (dt * dt), turned / (dt * dt))
        });
        let record = self.bodies.entry(handle.0).or_insert(Stillness {
            frames_below: 0,
            position,
            rotation,
        });
        record.frames_below = if slow && unmoved {
            record.frames_below.saturating_add(1)
        } else {
            0
        };
        record.position = position;
        record.rotation = rotation;
        record.frames_below >= self.delay_frames
    }

    pub fn clear_body(&mut self, handle: RigidBodyHandle) {
        self.bodies.remove(&handle.0);
    }

    pub fn retain_indices(&mut self, live: &FxHashSet<Index>) {
        self.bodies.retain(|idx, _| live.contains(idx));
    }

    /// Whether linear and angular rates, squared, are both under threshold.
    fn is_below(&self, linear_sq: f32, angular_sq: f32) -> bool {
        linear_sq < self.linear_threshold_sq && angular_sq < self.angular_threshold_sq
    }
}
