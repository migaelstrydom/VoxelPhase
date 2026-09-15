//! The load a jarred body puts on the pieces it carries.
//!
//! A pane held in a frame is accelerated by its joints whenever the frame is.
//! Steady acceleration — standing on the ground, falling freely, being
//! carried round a corner — is a steady load and cracks nothing. What cracks
//! glass is a *change* in acceleration: the frame stopped dead, or was
//! kicked. That is the jerk, and over one frame it is the change in the
//! body's per-frame velocity change.
//!
//! ```text
//!   body velocity ──▶ JoltTracker::advance ──▶ BodyJolt { delta, jerk }
//!                       (Δv this frame,          .at(r) for the piece at r
//!                        Δv last frame)            └─▶ mass × |jerk| = spike
//! ```

use nalgebra::Vector3;

/// A rigid motion increment: a linear part and an angular part, evaluated at
/// a point by the usual `linear + angular × r`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Motion {
    pub linear: Vector3<f32>,
    pub angular: Vector3<f32>,
}

impl Motion {
    /// The increment felt at body-relative world offset `r`.
    pub fn at(&self, r: Vector3<f32>) -> Vector3<f32> {
        self.linear + self.angular.cross(&r)
    }

    fn minus(&self, other: &Motion) -> Motion {
        Motion {
            linear: self.linear - other.linear,
            angular: self.angular - other.angular,
        }
    }
}

/// What the body did this frame, in terms a piece riding on it can be loaded
/// by.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BodyJolt {
    /// Velocity change over this frame: what a piece that let go this frame
    /// did *not* get, so the impulse that gives it back its old velocity.
    pub delta: Motion,
    /// Change in that velocity change since last frame. Zero for anything
    /// steady, including free fall.
    pub jerk: Motion,
}

/// Remembers a body's velocity across frames to turn it into a [`BodyJolt`].
#[derive(Debug, Default)]
pub struct JoltTracker {
    last_velocity: Option<Motion>,
    last_delta: Motion,
}

impl JoltTracker {
    /// This frame's jolt, given the body's velocity after the step.
    ///
    /// The first frame has nothing to compare with and reports no jolt, as
    /// does any frame after the tracker has been reset.
    pub fn advance(&mut self, linear: Vector3<f32>, angular: Vector3<f32>) -> BodyJolt {
        let velocity = Motion { linear, angular };
        let jolt = match self.last_velocity {
            Some(last) => {
                let delta = velocity.minus(&last);
                let jerk = delta.minus(&self.last_delta);
                self.last_delta = delta;
                BodyJolt { delta, jerk }
            }
            None => {
                self.last_delta = Motion::default();
                BodyJolt::default()
            }
        };
        self.last_velocity = Some(velocity);
        jolt
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn falling(tracker: &mut JoltTracker, frames: usize, g_dt: f32) -> BodyJolt {
        let mut jolt = BodyJolt::default();
        for i in 0..frames {
            jolt = tracker.advance(Vector3::new(0.0, -g_dt * i as f32, 0.0), Vector3::zeros());
        }
        jolt
    }

    #[test]
    fn free_fall_is_not_a_jolt() {
        let mut tracker = JoltTracker::default();
        let jolt = falling(&mut tracker, 5, 0.16);
        assert!(jolt.jerk.linear.magnitude() < 1e-6);
        assert!((jolt.delta.linear.y + 0.16).abs() < 1e-6);
    }

    #[test]
    fn stopping_dead_is() {
        let mut tracker = JoltTracker::default();
        falling(&mut tracker, 5, 0.16);
        let jolt = tracker.advance(Vector3::zeros(), Vector3::zeros());
        // Was gaining 0.16 down per frame; now gained 0.64 up: a jerk of 0.8.
        assert!((jolt.jerk.linear.y - 0.8).abs() < 1e-5);
    }

    #[test]
    fn a_spin_loads_the_rim_more_than_the_hub() {
        let mut tracker = JoltTracker::default();
        tracker.advance(Vector3::zeros(), Vector3::zeros());
        tracker.advance(Vector3::zeros(), Vector3::zeros());
        let jolt = tracker.advance(Vector3::zeros(), Vector3::new(0.0, 0.0, 2.0));
        let hub = jolt.jerk.at(Vector3::zeros()).magnitude();
        let rim = jolt.jerk.at(Vector3::new(1.0, 0.0, 0.0)).magnitude();
        assert_eq!(hub, 0.0);
        assert!((rim - 2.0).abs() < 1e-6);
    }
}
