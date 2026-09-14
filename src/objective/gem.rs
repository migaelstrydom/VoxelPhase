//! The idle motion that tells a gem apart from a prop.
//!
//! A gem has no rigid body: it hangs where it was authored and cannot be
//! knocked about, so its `Position` and `Orientation` are nobody else's to
//! write. That makes the bob and the spin a matter of driving them directly
//! from a phase clock rather than of applying forces.

use std::f32::consts::TAU;

use nalgebra::{UnitQuaternion, Vector3};
use specs::{Component, DenseVecStorage, Join, Read, System, WriteStorage};

use crate::components::{Orientation, Position};
use crate::time::Time;

/// A gem's hover, as a function of time alone.
#[derive(Component, Clone, Copy, Debug)]
#[storage(DenseVecStorage)]
pub struct GemMotion {
    /// The authored position the bob is measured from.
    pub anchor: Vector3<f32>,

    /// Peak vertical excursion either side of `anchor`, in metres.
    pub bob_amplitude: f32,

    /// Bob cycles per second.
    pub bob_rate: f32,

    /// Turns about `+Y` per second.
    pub spin_rate: f32,

    /// Seconds into the cycle. Seeded per gem so that a row of them does not
    /// rise and fall as one.
    pub phase: f32,
}

impl GemMotion {
    /// A gem hovering about `anchor`, `phase` seconds into its cycle.
    pub fn new(anchor: Vector3<f32>, phase: f32) -> Self {
        Self {
            anchor,
            bob_amplitude: DEFAULT_BOB_AMPLITUDE,
            bob_rate: DEFAULT_BOB_RATE,
            spin_rate: DEFAULT_SPIN_RATE,
            phase,
        }
    }

    /// Where the gem sits, and how far round it has turned, at its current
    /// phase.
    pub fn pose(&self) -> (Vector3<f32>, UnitQuaternion<f32>) {
        let bob = (self.phase * self.bob_rate * TAU).sin() * self.bob_amplitude;
        let position = self.anchor + Vector3::new(0.0, bob, 0.0);
        let yaw =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), self.phase * self.spin_rate * TAU);
        (position, yaw)
    }
}

/// A few centimetres: enough to read as floating, little enough that the reach
/// check does not depend on when the player arrives.
const DEFAULT_BOB_AMPLITUDE: f32 = 0.08;

/// Slow enough to be a hover rather than a vibration.
const DEFAULT_BOB_RATE: f32 = 0.45;

/// Just under three seconds a turn, so the facets catch the light in sequence.
const DEFAULT_SPIN_RATE: f32 = 0.35;

/// Advances every gem's hover.
pub struct GemMotionSystem;

impl<'a> System<'a> for GemMotionSystem {
    type SystemData = (
        Read<'a, Time>,
        WriteStorage<'a, GemMotion>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Orientation>,
    );

    fn run(&mut self, (time, mut motions, mut positions, mut orientations): Self::SystemData) {
        let dt = time.delta_seconds();

        for (motion, position, orientation) in
            (&mut motions, &mut positions, &mut orientations).join()
        {
            motion.phase += dt;
            let (p, yaw) = motion.pose();
            position.0 = p;
            orientation.0 = yaw;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_gem_starts_at_its_anchor_and_stays_within_the_bob() {
        let anchor = Vector3::new(1.0, 2.0, 3.0);
        let mut motion = GemMotion::new(anchor, 0.0);

        assert!((motion.pose().0 - anchor).norm() < 1e-6);

        for _ in 0..500 {
            motion.phase += 1.0 / 60.0;
            let offset = motion.pose().0 - anchor;
            assert!(offset.x.abs() < 1e-6 && offset.z.abs() < 1e-6);
            assert!(offset.y.abs() <= motion.bob_amplitude + 1e-6);
        }
    }
}
