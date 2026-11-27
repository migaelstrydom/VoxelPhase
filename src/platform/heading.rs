//! A platform that turns to face the way it is going, as a creature does,
//! rather than holding whatever heading it was built with.
//!
//! ```text
//!   commanded velocity ──bearing──► yaw error ──× stiffness, capped──► yaw rate
//!   (the seek's, smooth)            (from the body's nose)              (DriveIntent::angular_target)
//! ```
//!
//! It asks the motor for a rate of turn and nothing else, so it is as
//! honest as the rest of the drive: a passenger on the deck is carried round
//! by the deck, a blast that slews it is answered by turning back, and a
//! motor with no angular authority simply never turns. Slower than
//! [`FacesTravel::moving`] it holds its heading, so a turnaround's pause does
//! not set it spinning after a bearing that is nothing but noise.

use nalgebra::{UnitQuaternion, Vector3};
use specs::{Component, DenseVecStorage};

/// Turns a platform's nose (its body's +Z) toward its travel.
#[derive(Component, Debug, Clone, Copy, PartialEq)]
#[storage(DenseVecStorage)]
pub struct FacesTravel {
    /// The fastest it turns, rad/s.
    pub turn_rate: f32,
    /// How hard it turns toward its travel, in 1/s: the rate it turns at
    /// for each radian it is off.
    pub stiffness: f32,
    /// The speed below which it holds its heading, m/s.
    pub moving: f32,
}

impl FacesTravel {
    /// The rate of turn about the world's up, rad/s (positive turns the nose
    /// from +Z toward +X), that brings `facing` round toward `travel`.
    pub fn yaw_rate(&self, facing: &UnitQuaternion<f32>, travel: Vector3<f32>) -> f32 {
        let level = Vector3::new(travel.x, 0.0, travel.z);
        if level.norm() < self.moving {
            return 0.0;
        }
        let nose = facing * Vector3::z();
        let heading = nose.x.atan2(nose.z);
        let wanted = level.x.atan2(level.z);
        let mut off = wanted - heading;
        off = (off + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
        (off * self.stiffness).clamp(-self.turn_rate, self.turn_rate)
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use super::*;

    const TURNING: FacesTravel = FacesTravel {
        turn_rate: 0.6,
        stiffness: 2.0,
        moving: 0.3,
    };

    #[test]
    fn it_turns_the_short_way_toward_its_travel_and_no_faster_than_it_can() {
        let ahead = UnitQuaternion::identity();
        // Travel off to +X: a positive turn, capped.
        assert_eq!(TURNING.yaw_rate(&ahead, Vector3::new(2.0, 0.0, 0.0)), 0.6);
        assert_eq!(TURNING.yaw_rate(&ahead, Vector3::new(-2.0, 0.0, 0.0)), -0.6);
        // A little off: in proportion.
        let slightly = Vector3::new(0.1f32.sin(), 0.0, 0.1f32.cos()) * 2.0;
        assert!((TURNING.yaw_rate(&ahead, slightly) - 0.2).abs() < 1e-4);
        // Across the back, the short way round: facing −X, travelling +Z
        // is a quarter turn back toward +Z, the negative way.
        let west = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -FRAC_PI_2);
        assert!((west * Vector3::z() - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-6);
        assert_eq!(TURNING.yaw_rate(&west, Vector3::new(0.0, 0.0, 2.0)), 0.6);
    }

    #[test]
    fn it_holds_its_heading_when_it_is_hardly_moving_or_going_straight_up() {
        let ahead = UnitQuaternion::identity();
        assert_eq!(TURNING.yaw_rate(&ahead, Vector3::new(0.2, 0.0, -0.1)), 0.0);
        assert_eq!(TURNING.yaw_rate(&ahead, Vector3::new(0.0, 3.0, 0.0)), 0.0);
        assert_eq!(TURNING.yaw_rate(&ahead, Vector3::new(0.0, 0.0, 2.0)), 0.0);
    }
}
