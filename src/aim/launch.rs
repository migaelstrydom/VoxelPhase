//! A throw, stated.

use nalgebra::{Point3, Vector3};

/// Everything a flight needs: where it starts, how fast it leaves, and the
/// gravity it falls under.
///
/// Built by whatever performs the throw and read by the predictor, so the two
/// can never disagree about the arc.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Launch {
    /// World-space position the projectile starts from.
    pub origin: Point3<f32>,
    /// World-space velocity the moment it is released.
    pub velocity: Vector3<f32>,
    /// Acceleration acting on it for the whole flight, already scaled by
    /// whatever gravity scale the projectile carries.
    pub gravity: Vector3<f32>,
}

impl Launch {
    /// Where the projectile is `t` seconds after release, under gravity alone.
    ///
    /// Drag, damping and any contact before `t` are not modelled: this is the
    /// free-flight arc, which is all a landing prediction needs up to the first
    /// thing it hits.
    pub fn position_at(&self, t: f32) -> Point3<f32> {
        self.origin + self.velocity * t + self.gravity * (0.5 * t * t)
    }
}

/// The gravity scale a body needs to fall at `magnitude` in a world whose own
/// gravity is `world_gravity`.
///
/// Projectiles state their arc as an acceleration they want, not as a fraction
/// of the world's; this converts one to the other. A world with no gravity has
/// no fraction that produces any, so the scale is the neutral 1.
pub fn gravity_scale(world_gravity: Vector3<f32>, magnitude: f32) -> f32 {
    let world_magnitude = world_gravity.magnitude();
    if world_magnitude > 1e-6 {
        magnitude / world_magnitude
    } else {
        1.0
    }
}

/// The gravity vector a projectile falling at `magnitude` actually feels.
///
/// Direction comes from the world, strength from the projectile.
pub fn gravity_along(world_gravity: Vector3<f32>, magnitude: f32) -> Vector3<f32> {
    match world_gravity.try_normalize(1e-6) {
        Some(direction) => direction * magnitude,
        None => Vector3::new(0.0, -magnitude, 0.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_flat_throw_falls_the_textbook_distance() {
        let launch = Launch {
            origin: Point3::origin(),
            velocity: Vector3::new(10.0, 0.0, 0.0),
            gravity: Vector3::new(0.0, -10.0, 0.0),
        };

        let after_one_second = launch.position_at(1.0);

        assert!((after_one_second.x - 10.0).abs() < 1e-5);
        assert!((after_one_second.y + 5.0).abs() < 1e-5);
    }

    #[test]
    fn a_world_without_gravity_leaves_the_scale_neutral() {
        assert_eq!(gravity_scale(Vector3::zeros(), 9.81), 1.0);
        assert_eq!(
            gravity_along(Vector3::zeros(), 9.81),
            Vector3::new(0.0, -9.81, 0.0)
        );
    }

    #[test]
    fn a_lighter_arc_than_the_world_scales_below_one() {
        let world = Vector3::new(0.0, -20.0, 0.0);
        assert!((gravity_scale(world, 10.0) - 0.5).abs() < 1e-6);
        assert!((gravity_along(world, 10.0) - Vector3::new(0.0, -10.0, 0.0)).magnitude() < 1e-6);
    }
}
