//! The lollipop tree: a slender trunk under one round crown, or a fork of
//! branches each under a crown of its own, as the topiary in a Garden of
//! Earthly Delights stands.
//!
//! ```text
//!        ( main )          ( side ) ( main )
//!           |                  \     /
//!           |                   \   /   ← branches from the fork
//!           |                    \ /
//!           |                     |
//!      ‾‾‾‾‾|‾‾‾‾‾          ‾‾‾‾‾‾|‾‾‾‾‾
//! ```

use std::f32::consts::TAU;

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use super::limbs::{Limb, LimbKind, Limbs};
use super::plant::{LeafMass, Plant};
use super::roots::RootSpec;
use crate::foliage::{FoliageShape, ShrubSpec};

/// How far the trunk reaches below the ground, m: it grows out of the soil.
const TRUNK_ROOTING: f32 = 0.3;
/// Where the fork stands, as a share of the main crown's height.
const FORK_HEIGHT: (f32, f32) = (0.55, 0.65);
/// A side crown's radius as a share of the main crown's.
const SIDE_CROWN: (f32, f32) = (0.6, 0.75);
/// A branch's radius as a share of the trunk's.
const BRANCH_RADIUS: f32 = 0.65;
/// How far a side crown's centre stands from the trunk, as a share of its
/// and the main crown's radii together: far enough that their cores, 80%
/// of their size, keep apart.
const SIDE_REACH: f32 = 0.9;

/// What a lollipop tree grows to.
#[derive(Debug, Clone, Copy)]
pub struct LollipopSpec {
    /// Height from the ground to the top of its leaves, m.
    pub height: f32,
    /// Radius of the main crown, m.
    pub crown_radius: f32,
    /// How many crowns: the main one on the trunk's top, the rest on
    /// branches from a fork below it.
    pub crowns: usize,
    /// The trunk's radius, m.
    pub trunk_radius: f32,
    /// The most the trunk leans from upright, radians.
    pub lean: f32,
    /// How its roots spread.
    pub roots: RootSpec,
}

impl Default for LollipopSpec {
    fn default() -> Self {
        Self {
            height: 3.4,
            crown_radius: 0.8,
            crowns: 1,
            trunk_radius: 0.07,
            lean: 0.08,
            roots: RootSpec::default(),
        }
    }
}

impl LollipopSpec {
    /// The tree grown from `seed`.
    pub fn grow(&self, seed: u32) -> Plant {
        let mut rng = StdRng::seed_from_u64(seed as u64 ^ 0x1011_1909);
        let main = crown(self.crown_radius, seed);
        let main_top = main.bounds().1.y;

        // The trunk runs from below the ground, leaning a little, to the main
        // crown's centre, which sits so its leaves top out at `height`.
        let lean = rng.gen_range(0.0..=self.lean);
        let heading = rng.gen_range(0.0..TAU);
        let axis = Vector3::new(
            lean.sin() * heading.cos(),
            lean.cos(),
            lean.sin() * heading.sin(),
        );
        let main_centre = Point3::from(axis * ((self.height - main_top) / axis.y));
        let base = Point3::new(0.0, -TRUNK_ROOTING, 0.0);
        let mut limbs = vec![Limb {
            base,
            tip: main_centre,
            radius: self.trunk_radius,
            parent: None,
            kind: LimbKind::Trunk,
        }];
        let mut masses = vec![LeafMass {
            shape: main,
            pose: turned(main_centre, &mut rng),
            limb: 0,
        }];

        let sides = self.crowns.saturating_sub(1);
        if sides > 0 {
            let fork_share = rng.gen_range(FORK_HEIGHT.0..FORK_HEIGHT.1);
            let fork = Point3::from(main_centre.coords * fork_share);
            let first = rng.gen_range(0.0..TAU);
            for side in 0..sides {
                let radius = self.crown_radius * rng.gen_range(SIDE_CROWN.0..SIDE_CROWN.1);
                let shape = crown(radius, seed.wrapping_add(side as u32 + 1));
                let around = first + TAU * side as f32 / sides as f32 + rng.gen_range(-0.3..0.3);
                let reach = (self.crown_radius + radius) * SIDE_REACH;
                let low = fork.y + radius * 1.2;
                let high = (main_centre.y - 0.3).max(low);
                let centre = Point3::new(
                    fork.x + reach * around.cos(),
                    rng.gen_range(low..=high),
                    fork.z + reach * around.sin(),
                );
                limbs.push(Limb {
                    base: fork,
                    tip: centre,
                    radius: self.trunk_radius * BRANCH_RADIUS,
                    parent: Some(0),
                    kind: LimbKind::Branch,
                });
                masses.push(LeafMass {
                    shape,
                    pose: turned(centre, &mut rng),
                    limb: limbs.len() - 1,
                });
            }
        }
        let roots = self.roots.grow(&limbs[0], 0, &mut rng);
        limbs.extend(roots);
        Plant {
            limbs: Limbs { limbs },
            masses,
        }
    }
}

/// A round crown of `radius`: a topiary ball, leaves all the way round.
fn crown(radius: f32, seed: u32) -> FoliageShape {
    ShrubSpec::with_radius(radius).grow(seed)
}

/// A pose at `centre`, turned a random way about the vertical, so no two
/// crowns show the same face.
fn turned(centre: Point3<f32>, rng: &mut StdRng) -> Isometry3<f32> {
    Isometry3::from_parts(
        Translation3::from(centre.coords),
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), rng.gen_range(0.0..TAU)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn top_of_leaves(plant: &Plant) -> f32 {
        plant
            .masses
            .iter()
            .flat_map(|mass| {
                mass.shape
                    .lobes
                    .iter()
                    .map(move |lobe| (mass.pose * lobe.centre).y + lobe.extent_along(Vector3::y()))
            })
            .fold(f32::MIN, f32::max)
    }

    fn holds(mass: &LeafMass, point: Point3<f32>) -> bool {
        let local = mass.pose.inverse() * point;
        mass.shape
            .lobes
            .iter()
            .any(|lobe| lobe.contains_scaled(local, 1.0))
    }

    /// One crown: its leaves top out at the height asked for, the trunk
    /// grows from below the ground and ends inside the crown.
    #[test]
    fn a_lollipop_stands_its_height_on_its_trunk() {
        let spec = LollipopSpec::default();
        for seed in 0..8 {
            let plant = spec.grow(seed);
            assert_eq!(plant.masses.len(), 1);
            let top = top_of_leaves(&plant);
            assert!(
                (top - spec.height).abs() < 0.02,
                "seed {seed}: top at {top}"
            );
            let trunk = plant.limbs.limbs[0];
            assert!(trunk.base.y < 0.0);
            assert!(
                holds(&plant.masses[0], trunk.tip),
                "seed {seed}: the trunk ends in the open"
            );
        }
    }

    /// Three crowns: each branch ends inside its crown, every side crown is
    /// below the main one, and the fork is below them all.
    #[test]
    fn a_forked_lollipop_holds_each_crown_on_a_branch() {
        let spec = LollipopSpec {
            crowns: 3,
            ..LollipopSpec::default()
        };
        for seed in 0..8 {
            let plant = spec.grow(seed);
            assert_eq!(plant.masses.len(), 3);
            let main = plant.masses[0].pose.translation.vector.y;
            for mass in &plant.masses[1..] {
                let branch = plant.limbs.limbs[mass.limb];
                assert_eq!(branch.kind, LimbKind::Branch);
                assert!(
                    holds(mass, branch.tip),
                    "seed {seed}: a branch ends in the open"
                );
                assert!(branch.base.y < branch.tip.y);
                assert!(mass.pose.translation.vector.y < main);
            }
        }
    }
}
