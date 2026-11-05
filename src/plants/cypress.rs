//! The cypress: one tall, narrow crown on a short trunk, the flame-shaped
//! tree of the painted gardens.
//!
//! ```text
//!        ( )
//!       (   )
//!       (   )   ← one tall crown, its trunk running up inside it
//!       (   )
//!        ( )
//!         |     ← a short stretch of trunk below the leaves
//!   ‾‾‾‾‾‾|‾‾‾‾‾‾
//! ```

use std::f32::consts::TAU;

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use super::limbs::{Limb, Limbs};
use super::plant::{LeafMass, Plant};
use crate::foliage::ShrubSpec;

/// How far the trunk reaches below the ground, m: it grows out of the soil.
const TRUNK_ROOTING: f32 = 0.3;

/// What a cypress grows to.
#[derive(Debug, Clone, Copy)]
pub struct CypressSpec {
    /// Height from the ground to the top of its leaves, m.
    pub height: f32,
    /// Radius of the crown at its widest, m.
    pub radius: f32,
    /// How much bare trunk shows below the leaves, m.
    pub bare_trunk: f32,
    /// The trunk's radius, m.
    pub trunk_radius: f32,
    /// The most the tree leans from upright, radians.
    pub lean: f32,
}

impl Default for CypressSpec {
    fn default() -> Self {
        Self {
            height: 4.5,
            radius: 0.55,
            bare_trunk: 0.5,
            trunk_radius: 0.08,
            lean: 0.04,
        }
    }
}

impl CypressSpec {
    /// The tree grown from `seed`.
    pub fn grow(&self, seed: u32) -> Plant {
        let mut rng = StdRng::seed_from_u64(seed as u64 ^ 0xc1_9e55);
        let crown_height = (self.height - self.bare_trunk).max(2.0 * self.radius);
        let shape = ShrubSpec::with_radius(self.radius)
            .with_height(crown_height)
            .grow(seed);
        let (low, high) = shape.bounds();

        // The crown's leaves span bare_trunk..height above the ground, its
        // frame leaning a little with the trunk, which ends at its centre.
        let lean = rng.gen_range(0.0..=self.lean);
        let heading = rng.gen_range(0.0..TAU);
        let axis = Vector3::new(
            lean.sin() * heading.cos(),
            lean.cos(),
            lean.sin() * heading.sin(),
        );
        let centre_height = self.height - high.y;
        let centre = Point3::from(axis * (centre_height / axis.y));
        let tilt = UnitQuaternion::rotation_between(&Vector3::y(), &axis)
            .unwrap_or_else(UnitQuaternion::identity);
        let turn = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), rng.gen_range(0.0..TAU));
        debug_assert!(
            centre_height + low.y > 0.0,
            "the crown reaches into the ground"
        );

        Plant {
            limbs: Limbs {
                limbs: vec![Limb {
                    base: Point3::new(0.0, -TRUNK_ROOTING, 0.0),
                    tip: centre,
                    radius: self.trunk_radius,
                    parent: None,
                }],
            },
            masses: vec![LeafMass {
                shape,
                pose: Isometry3::from_parts(Translation3::from(centre.coords), tilt * turn),
                limb: 0,
            }],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The crown's leaves span from the bare trunk's top to the height
    /// asked for, and the trunk ends inside them.
    #[test]
    fn a_cypress_stands_its_height_on_a_short_trunk() {
        let spec = CypressSpec::default();
        for seed in 0..8 {
            let plant = spec.grow(seed);
            let mass = &plant.masses[0];
            let ys = mass.shape.lobes.iter().flat_map(|lobe| {
                let centre = (mass.pose * lobe.centre).y;
                let reach = lobe.extent_along(mass.pose.rotation.inverse() * Vector3::y());
                [centre - reach, centre + reach]
            });
            let (bottom, top) = ys.fold((f32::MAX, f32::MIN), |(lo, hi), y| (lo.min(y), hi.max(y)));
            assert!(
                (top - spec.height).abs() < 0.05,
                "seed {seed}: top at {top}"
            );
            assert!(
                (bottom - spec.bare_trunk).abs() < 0.15,
                "seed {seed}: leaves start at {bottom}"
            );
            let local = mass.pose.inverse() * plant.limbs.limbs[0].tip;
            assert!(mass
                .shape
                .lobes
                .iter()
                .any(|lobe| lobe.contains_scaled(local, 1.0)));
        }
    }
}
