//! How a hover sled glides: it thrusts the way its rider leans and coasts to
//! a stop when nobody does.
//!
//! A rider leans by where they stand: in the middle of the deck the sled
//! coasts, and toward an edge it thrusts that way, harder the further out,
//! up to its cruise speed.
//! Its pods barely grip, so this and the field's own tilt are what move it
//! across the ground.

use nalgebra::{Vector2, Vector3};

/// How a sled answers its rider's lean.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Glide {
    /// Speed it thrusts toward at full lean, m/s.
    pub cruise_speed: f32,
    /// Most it changes its speed by thrusting, m/s².
    pub thrust: f32,
    /// Time constant of its coasting to a stop, s.
    pub coast_time: f32,
    /// Share of the way from the deck's middle to its edge a rider stands
    /// before they lean at all.
    pub dead_zone: f32,
    /// A ridden sled holds the heading it had when its rider boarded, as
    /// runners would, so a lean ahead carries straight on: this is how long
    /// it takes to turn back to it, s...
    pub heading_return: f32,
    /// ...and how quickly its spin follows, s.
    pub heading_time: f32,
}

impl Glide {
    pub const HOVER: Self = Self {
        cruise_speed: 6.0,
        thrust: 5.0,
        coast_time: 1.6,
        dead_zone: 0.4,
        heading_return: 0.4,
        heading_time: 0.08,
    };

    /// The lean of a rider standing at `offset` from the deck's middle, as a
    /// share of its half extents. Each axis has its own dead zone, so a rider
    /// a little to one side of the middle leans straight ahead and not off at
    /// an angle; past it the lean grows to 1 at the deck's edge, and the
    /// whole is at most 1.
    pub fn lean(&self, offset: Vector2<f32>) -> Vector2<f32> {
        let lean = offset.map(|o| {
            o.signum() * ((o.abs() - self.dead_zone) / (1.0 - self.dead_zone)).clamp(0.0, 1.0)
        });
        let amount = lean.norm();
        if amount > 1.0 {
            lean / amount
        } else {
            lean
        }
    }

    /// The change in level velocity, from `velocity`, over `dt`. Under a
    /// lean, toward the cruise speed the way it leans, the harder the
    /// further the lean: it is a throttle, so a rider need not stand at the
    /// nose, where their weight would dip it over every gap. Without one,
    /// coasting toward a stop. `lean` is level, in world space, of length
    /// 0..1.
    pub fn velocity_change(
        &self,
        velocity: Vector3<f32>,
        lean: Option<Vector3<f32>>,
        dt: f32,
    ) -> Vector3<f32> {
        match lean.filter(|lean| lean.norm() > 0.0) {
            Some(lean) => {
                let wanted = lean.normalize() * self.cruise_speed - velocity;
                let most = self.thrust * lean.norm() * dt;
                if wanted.norm() > most {
                    wanted.normalize() * most
                } else {
                    wanted
                }
            }
            None => -velocity * (1.0 - (-dt / self.coast_time).exp()),
        }
    }
}

impl Glide {
    /// The change in spin about the vertical, from `yaw_rate`, over `dt`,
    /// for a ridden sled `heading_error` radians off its held heading.
    pub fn yaw_rate_change(&self, yaw_rate: f32, heading_error: f32, dt: f32) -> f32 {
        let wanted = -heading_error / self.heading_return;
        (wanted - yaw_rate) * (1.0 - (-dt / self.heading_time).exp())
    }
}

impl Default for Glide {
    fn default() -> Self {
        Self::HOVER
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standing_a_little_to_one_side_leans_straight_ahead() {
        let lean = Glide::HOVER.lean(Vector2::new(0.8, 0.25));
        assert!(lean.x > 0.5 && lean.y == 0.0, "{lean:?}");
    }

    #[test]
    fn standing_in_the_middle_leans_nowhere() {
        let glide = Glide::HOVER;
        assert_eq!(glide.lean(Vector2::new(0.2, 0.1)), Vector2::zeros());
    }

    #[test]
    fn standing_at_the_edge_leans_all_the_way_out() {
        let glide = Glide::HOVER;
        let lean = glide.lean(Vector2::new(1.2, 0.0));
        assert!((lean - Vector2::new(1.0, 0.0)).norm() < 1e-6);
        let halfway_out = glide.dead_zone + 0.5 * (1.0 - glide.dead_zone);
        let half = glide.lean(Vector2::new(0.0, halfway_out));
        assert!((half.y - 0.5).abs() < 1e-6);
    }

    #[test]
    fn a_lean_thrusts_up_to_cruise_and_no_faster() {
        let glide = Glide::HOVER;
        let dt = 1.0 / 60.0;
        let mut velocity = Vector3::zeros();
        for _ in 0..600 {
            velocity += glide.velocity_change(velocity, Some(Vector3::x()), dt);
        }
        assert!((velocity.x - glide.cruise_speed).abs() < 1e-3);
        let first = glide.velocity_change(Vector3::zeros(), Some(Vector3::x()), dt);
        assert!((first.norm() - glide.thrust * dt).abs() < 1e-6);
    }

    #[test]
    fn a_ridden_sled_turns_back_to_its_heading() {
        let glide = Glide::HOVER;
        let dt = 1.0 / 60.0;
        let (mut heading, mut yaw_rate) = (0.3f32, 0.0f32);
        for _ in 0..240 {
            yaw_rate += glide.yaw_rate_change(yaw_rate, heading, dt);
            heading += yaw_rate * dt;
        }
        assert!(heading.abs() < 0.01, "{heading}");
    }

    #[test]
    fn with_nobody_leaning_it_coasts_to_a_stop() {
        let glide = Glide::HOVER;
        let dt = 1.0 / 60.0;
        let mut velocity = Vector3::new(3.0, 0.0, 0.0);
        for _ in 0..(5.0 * glide.coast_time / dt) as usize {
            velocity += glide.velocity_change(velocity, None, dt);
        }
        assert!(velocity.norm() < 0.03, "{velocity:?}");
    }
}
