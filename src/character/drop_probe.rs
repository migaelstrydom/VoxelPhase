use nalgebra::{Point3, Vector3};

use crate::traversal::WorldCaster;

/// How a character finds it is standing over a drop: held up by the brink of
/// something, its centre past it, its feet over nothing. `ContactGroundingSystem`
/// asks it for every grounded body whose `LocomotionConfig` carries one and
/// publishes the answer as `Grounding::over_drop`.
///
/// Contacts cannot tell this from a slope. Terrain rounds every ledge into a
/// bevel, and a capsule resting on one is held by a face that tilts like any
/// walkable slope; friction and the drive's brake hold it there. What differs
/// is what lies under the body's centre: on a slope the ground goes on
/// beneath it, past a brink it falls away. So the probe looks there, and
/// the body is steered off.
///
/// ```text
///         ┌─┐
///         │ │  centre
///      ▓▓▓│ │╲ support normal, leaning outward
///      ▓▓▓▓╲╯
///      ▓▓▓▓▓▓  probe: nothing within reach below the centre
///      ▓▓▓▓▓▓     ┆
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DropProbe {
    /// How far below the capsule's lowest point the ground under its centre
    /// may lie and still be ground the body stands over, in metres. Under a
    /// body of radius `r` on a slope of angle `θ`, the slope lies
    /// `r·(1/cos θ − 1)` below: 0.25 m on a 60° slope, the steepest that holds
    /// anything up, for the player's 0.25 m capsule.
    pub reach_below: f32,
    /// Radius of the sphere cast down from the centre, in metres: about a
    /// foot's, so a seam in the ground is not a drop.
    pub probe_radius: f32,
    /// Least horizontal part of the unit support normal for the body to be
    /// leaning off its support at all: 0.2 is about 12°. Standing square on
    /// something, there is no way off it to steer.
    pub min_lean: f32,
    /// Speed, in m/s, a body over a drop is steered outward at: no slower
    /// unless it is walking back from the brink.
    pub slide_off_speed: f32,
}

impl Default for DropProbe {
    fn default() -> Self {
        Self {
            reach_below: 0.3,
            probe_radius: 0.05,
            min_lean: 0.2,
            slide_off_speed: 1.0,
        }
    }
}

impl DropProbe {
    /// The way off a drop for a capsule of `half_height` (caps included)
    /// centred at `centre`, held up along `support_normal`: outward, when the
    /// support leans and there is no ground within reach under the centre;
    /// `None` when the body stands over its ground.
    pub fn way_off(
        &self,
        caster: &WorldCaster<'_>,
        centre: Point3<f32>,
        support_normal: Vector3<f32>,
        up: Vector3<f32>,
        half_height: f32,
    ) -> Option<Vector3<f32>> {
        let outward = self.outward(support_normal, up)?;
        let ground_below = caster
            .cast(
                centre,
                -up,
                self.probe_length(half_height),
                self.probe_radius,
            )
            .is_some();
        (!ground_below).then_some(outward)
    }

    /// The way off the support, across `up`: the support normal's lean, or
    /// `None` when the body stands square on it.
    pub fn outward(&self, support_normal: Vector3<f32>, up: Vector3<f32>) -> Option<Vector3<f32>> {
        let lean = support_normal - up * support_normal.dot(&up);
        (lean.norm() >= self.min_lean).then(|| lean.normalize())
    }

    /// How far to cast the probe down from the centre of a capsule of
    /// `half_height` (caps included), so that it touches ground no further
    /// than `reach_below` under the capsule's lowest point.
    pub fn probe_length(&self, half_height: f32) -> f32 {
        half_height + self.reach_below - self.probe_radius
    }

    /// `target` with at least `slide_off_speed` along `outward`, unless it
    /// walks back from the brink, which is left to do so.
    pub fn slide_off(&self, target: Vector3<f32>, outward: Vector3<f32>) -> Vector3<f32> {
        let along = target.dot(&outward);
        if along < 0.0 {
            return target;
        }
        target + outward * (self.slide_off_speed - along).max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tilted(degrees: f32) -> Vector3<f32> {
        let tilt = degrees.to_radians();
        Vector3::new(-tilt.sin(), tilt.cos(), 0.0)
    }

    #[test]
    fn a_body_standing_square_has_no_way_off() {
        let probe = DropProbe::default();
        assert_eq!(probe.outward(tilted(5.0), Vector3::y()), None);
    }

    #[test]
    fn the_way_off_is_where_the_support_leans() {
        let probe = DropProbe::default();
        let outward = probe.outward(tilted(37.0), Vector3::y()).unwrap();
        assert!((outward - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-6);
    }

    /// A 60° slope, the steepest that holds a body up, lies within reach
    /// under the player's centre: it is never a drop.
    #[test]
    fn the_steepest_floor_is_within_reach() {
        let probe = DropProbe::default();
        let (radius, half_height) = (0.25, 0.5);
        let below_lowest = radius * (1.0 / 60f32.to_radians().cos() - 1.0);
        let below_centre = half_height + below_lowest;
        assert!(below_centre - probe.probe_radius < probe.probe_length(half_height));
    }

    #[test]
    fn a_standing_body_is_steered_off() {
        let probe = DropProbe::default();
        let off = probe.slide_off(Vector3::zeros(), Vector3::x());
        assert_eq!(off, Vector3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn walking_off_faster_is_left_alone() {
        let probe = DropProbe::default();
        let walk = Vector3::new(5.0, 0.0, 0.0);
        assert_eq!(probe.slide_off(walk, Vector3::x()), walk);
    }

    #[test]
    fn walking_along_the_brink_is_also_steered_off() {
        let probe = DropProbe::default();
        let along = Vector3::new(0.0, 0.0, 3.0);
        assert_eq!(
            probe.slide_off(along, Vector3::x()),
            Vector3::new(1.0, 0.0, 3.0)
        );
    }

    #[test]
    fn walking_back_from_the_brink_is_left_alone() {
        let probe = DropProbe::default();
        let back = Vector3::new(-2.0, 0.0, 0.0);
        assert_eq!(probe.slide_off(back, Vector3::x()), back);
    }
}
