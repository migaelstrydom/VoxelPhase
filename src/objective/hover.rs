//! The idle motion that tells a collectable (a solid, a scroll) apart from
//! a prop.
//!
//! A solid has no rigid body: it hangs where it was authored and cannot be
//! knocked about, so its `Position` and `Orientation` are nobody else's to
//! write. That makes the bob, the turn and the breathing of its glow a
//! matter of driving them directly from a phase clock rather than of
//! applying forces.

use std::f32::consts::TAU;

use nalgebra::{UnitQuaternion, Vector3};
use specs::{Component, DenseVecStorage, Join, LendJoin, Read, System, WriteStorage};

use crate::components::{MaterialModulation, Orientation, Position};
use crate::rendering::material::SurfaceModulation;
use crate::time::Time;

/// A hover, as a function of time alone.
#[derive(Component, Clone, Copy, Debug)]
#[storage(DenseVecStorage)]
pub struct Hover {
    /// The authored position the bob is measured from.
    pub anchor: Vector3<f32>,

    /// How the thing is held before it turns: a cube on its corner.
    pub rest: UnitQuaternion<f32>,

    /// Peak vertical excursion either side of `anchor`, in metres.
    pub bob_amplitude: f32,

    /// Bob cycles per second.
    pub bob_rate: f32,

    /// Turns about `+Y` per second.
    pub spin_rate: f32,

    /// How far the turning axis leans off plumb, radians, and how many
    /// times a second the lean goes round: a top's slow wobble, which turns
    /// every face to the light in time.
    pub sway: f32,
    pub sway_rate: f32,

    /// How far the glow swells and fades either side of as authored, as a
    /// share of it, and breaths per second.
    pub breath_depth: f32,
    pub breath_rate: f32,

    /// Seconds into the cycle. Seeded per solid so that a row of them does
    /// not rise and fall as one.
    pub phase: f32,
}

impl Hover {
    /// Hovering about `anchor`, held at `rest`, `phase` seconds into its
    /// cycle.
    pub fn new(anchor: Vector3<f32>, rest: UnitQuaternion<f32>, phase: f32) -> Self {
        Self {
            anchor,
            rest,
            bob_amplitude: BOB_AMPLITUDE,
            bob_rate: BOB_RATE,
            spin_rate: SPIN_RATE,
            sway: SWAY,
            sway_rate: SWAY_RATE,
            breath_depth: BREATH_DEPTH,
            breath_rate: BREATH_RATE,
            phase,
        }
    }

    /// Where it sits, and how it is turned, at its current phase.
    pub fn pose(&self) -> (Vector3<f32>, UnitQuaternion<f32>) {
        let t = self.phase;
        let bob = (t * self.bob_rate * TAU).sin() * self.bob_amplitude;
        let position = self.anchor + Vector3::new(0.0, bob, 0.0);

        let spin = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), t * self.spin_rate * TAU);
        let lean_about = t * self.sway_rate * TAU;
        let lean_axis = Vector3::new(lean_about.cos(), 0.0, lean_about.sin());
        let lean = UnitQuaternion::from_scaled_axis(lean_axis * self.sway);
        (position, lean * spin * self.rest)
    }

    /// How bright its glow is at its current phase, as a share of as
    /// authored.
    pub fn glow(&self) -> f32 {
        1.0 + self.breath_depth * (self.phase * self.breath_rate * TAU).sin()
    }
}

/// A few centimetres: enough to read as floating, little enough that the reach
/// check does not depend on when the player arrives.
const BOB_AMPLITUDE: f32 = 0.08;

/// Slow enough to be a hover rather than a vibration.
const BOB_RATE: f32 = 0.45;

/// Under three seconds a turn, so the facets catch the light in sequence.
const SPIN_RATE: f32 = 0.35;

/// A lean that shows the top faces and the bottom ones in turn, slower than
/// the spin so the two never fall into step.
const SWAY: f32 = 0.16;
const SWAY_RATE: f32 = 0.11;

/// A slow breath, out of step with the bob.
const BREATH_DEPTH: f32 = 0.18;
const BREATH_RATE: f32 = 0.27;

/// Advances every hover.
pub struct HoverSystem;

impl<'a> System<'a> for HoverSystem {
    type SystemData = (
        Read<'a, Time>,
        WriteStorage<'a, Hover>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Orientation>,
        WriteStorage<'a, MaterialModulation>,
    );

    fn run(
        &mut self,
        (time, mut hovers, mut positions, mut orientations, mut modulations): Self::SystemData,
    ) {
        let dt = time.delta_seconds();

        for (hover, position, orientation, modulation) in (
            &mut hovers,
            &mut positions,
            &mut orientations,
            (&mut modulations).maybe(),
        )
            .join()
        {
            hover.phase += dt;
            let (p, turned) = hover.pose();
            position.0 = p;
            orientation.0 = turned;
            if let Some(modulation) = modulation {
                modulation.0 = SurfaceModulation::emissive(hover.glow());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_hover_starts_at_its_anchor_and_stays_within_the_bob() {
        let anchor = Vector3::new(1.0, 2.0, 3.0);
        let mut hover = Hover::new(anchor, UnitQuaternion::identity(), 0.0);

        assert!((hover.pose().0 - anchor).norm() < 1e-6);

        for _ in 0..500 {
            hover.phase += 1.0 / 60.0;
            let offset = hover.pose().0 - anchor;
            assert!(offset.x.abs() < 1e-6 && offset.z.abs() < 1e-6);
            assert!(offset.y.abs() <= hover.bob_amplitude + 1e-6);
        }
    }

    /// The turn never tips the thing further from how it is held than the
    /// sway: a cube on its corner stays on its corner.
    #[test]
    fn the_turning_axis_leans_no_further_than_the_sway() {
        let rest = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.6);
        let mut hover = Hover::new(Vector3::zeros(), rest, 0.0);
        for _ in 0..2000 {
            hover.phase += 1.0 / 30.0;
            let (_, turned) = hover.pose();
            // The axis the rest pose holds upright is what leans.
            let axis = turned * rest.inverse() * Vector3::y();
            assert!(axis.angle(&Vector3::y()) <= hover.sway + 1e-4);
        }
    }

    #[test]
    fn the_glow_breathes_about_as_authored() {
        let mut hover = Hover::new(Vector3::zeros(), UnitQuaternion::identity(), 0.0);
        let (mut low, mut high) = (f32::MAX, f32::MIN);
        for _ in 0..2000 {
            hover.phase += 1.0 / 30.0;
            low = low.min(hover.glow());
            high = high.max(hover.glow());
        }
        assert!(low >= 1.0 - hover.breath_depth - 1e-5);
        assert!(high <= 1.0 + hover.breath_depth + 1e-5);
        assert!(high - low > hover.breath_depth);
    }
}
