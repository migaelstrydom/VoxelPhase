//! The pump component: where on its body the water goes in and comes out.

use nalgebra::{Isometry3, Point3, Vector3};
use specs::{Component, DenseVecStorage};

use crate::water::PumpPlacement;

/// One opening a pump throws water from.
#[derive(Debug, Clone, Copy)]
pub struct Spout {
    /// Where its water comes out, in the body's frame.
    pub outlet: Point3<f32>,
    /// The velocity its water leaves at, in the body's frame, m/s.
    pub velocity: Vector3<f32>,
}

/// A pump built into a body. It draws the water standing over its intake
/// and throws it from its spouts, so it runs wherever the body lies with
/// its intake under water and stops when it does not.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct WaterPump {
    /// Where it draws water, in the body's frame.
    pub intake: Point3<f32>,
    /// The openings its water leaves by, which share it evenly.
    pub spouts: Vec<Spout>,
    /// The most it moves through all its spouts together, m³/s.
    pub rate: f32,
}

impl WaterPump {
    /// Where each spout's share of it stands in the world when its body is
    /// at `pose`, in the order of its spouts.
    pub fn placements<'a>(
        &'a self,
        pose: &'a Isometry3<f32>,
    ) -> impl Iterator<Item = PumpPlacement> + 'a {
        let rate = self.rate / self.spouts.len().max(1) as f32;
        self.spouts.iter().map(move |spout| PumpPlacement {
            intake: pose * self.intake,
            outlet: pose * spout.outlet,
            velocity: pose.rotation * spout.velocity,
            rate,
        })
    }

    /// Move everything measured from the body's origin when the origin
    /// moves by `moved`, in the body's frame.
    pub fn recentre(&mut self, moved: Vector3<f32>) {
        self.intake -= moved;
        for spout in &mut self.spouts {
            spout.outlet -= moved;
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use nalgebra::{Translation3, UnitQuaternion};

    use super::*;

    fn pump() -> WaterPump {
        WaterPump {
            intake: Point3::new(0.0, -0.5, 0.0),
            spouts: vec![
                Spout {
                    outlet: Point3::new(0.0, 0.5, 0.0),
                    velocity: Vector3::new(0.0, 4.0, 0.0),
                },
                Spout {
                    outlet: Point3::new(0.0, 0.5, 0.2),
                    velocity: Vector3::new(0.0, 4.0, 1.0),
                },
            ],
            rate: 0.1,
        }
    }

    #[test]
    fn a_pump_on_its_side_throws_sideways() {
        // Tipped over towards +x.
        let pose = Isometry3::from_parts(
            Translation3::new(10.0, 2.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -FRAC_PI_2),
        );
        let placed: Vec<PumpPlacement> = pump().placements(&pose).collect();
        assert!((placed[0].outlet - Point3::new(10.5, 2.0, 0.0)).norm() < 1.0e-5);
        assert!((placed[0].intake - Point3::new(9.5, 2.0, 0.0)).norm() < 1.0e-5);
        assert!((placed[0].velocity - Vector3::new(4.0, 0.0, 0.0)).norm() < 1.0e-5);
    }

    #[test]
    fn its_spouts_share_its_rate() {
        let pose = Isometry3::identity();
        let total: f32 = pump().placements(&pose).map(|placed| placed.rate).sum();
        assert!((total - 0.1).abs() < 1.0e-6);
    }
}
