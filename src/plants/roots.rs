//! A tree's roots: limbs spreading down and out from the trunk's foot,
//! buried while the tree stands, there to be seen when a blast takes the
//! ground from round them.
//!
//! ```text
//!            │ trunk
//!      ‾‾‾‾‾‾│‾‾‾‾‾‾  ground
//!        ╲__╱│╲__╱    roots leave the trunk just under the ground
//!       ╱    │    ╲   and reach down and out
//! ```

use std::f32::consts::TAU;

use nalgebra::Point3;
use rand::rngs::StdRng;
use rand::Rng;

use super::limbs::{Limb, LimbKind};

/// How a tree's roots spread, in multiples of its trunk's radius so a
/// slender tree and a stout one root alike.
#[derive(Debug, Clone, Copy)]
pub struct RootSpec {
    /// How many roots, drawn from this range.
    pub count: (usize, usize),
    /// How far out from the trunk's axis a root's tip reaches, in trunk
    /// radii, drawn from this range.
    pub reach: (f32, f32),
    /// How far below the ground a root's tip goes, in trunk radii, drawn
    /// from this range.
    pub depth: (f32, f32),
    /// A root's radius where it leaves the trunk, as a share of the trunk's.
    pub radius: f32,
    /// How far below the ground roots leave the trunk, in trunk radii: deep
    /// enough that their tops stay under it as they spread.
    pub collar_depth: f32,
}

impl Default for RootSpec {
    fn default() -> Self {
        Self {
            count: (4, 5),
            reach: (6.0, 9.0),
            depth: (2.5, 4.5),
            radius: 0.6,
            collar_depth: 0.6,
        }
    }
}

impl RootSpec {
    /// Roots for `trunk`, the limb at index `trunk_index`, whose plant's
    /// ground is at height 0. Spread evenly round it, each turned a little.
    pub fn grow(&self, trunk: &Limb, trunk_index: usize, rng: &mut StdRng) -> Vec<Limb> {
        let r = trunk.radius;
        let along = trunk.tip - trunk.base;
        let collar_height = -self.collar_depth * r;
        if along.y <= 0.0 || trunk.base.y >= collar_height {
            return Vec::new();
        }
        let collar = trunk.base + along * ((collar_height - trunk.base.y) / along.y);
        let count = rng.gen_range(self.count.0..=self.count.1);
        let first = rng.gen_range(0.0..TAU);
        (0..count)
            .map(|i| {
                let around = first + TAU * (i as f32 + rng.gen_range(-0.2..0.2)) / count as f32;
                let reach = r * rng.gen_range(self.reach.0..=self.reach.1);
                let depth = r * rng.gen_range(self.depth.0..=self.depth.1);
                Limb {
                    base: collar,
                    tip: Point3::new(
                        collar.x + reach * around.cos(),
                        -depth,
                        collar.z + reach * around.sin(),
                    ),
                    radius: r * self.radius,
                    parent: Some(trunk_index),
                    kind: LimbKind::Root,
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use rand::SeedableRng;

    use super::*;

    /// Every root leaves the trunk below the ground and stays below it all
    /// the way to its tip, its top included.
    #[test]
    fn roots_are_buried() {
        let trunk = Limb {
            base: Point3::new(0.0, -0.3, 0.0),
            tip: Point3::new(0.1, 2.7, 0.0),
            radius: 0.07,
            parent: None,
            kind: LimbKind::Trunk,
        };
        for seed in 0..16 {
            let mut rng = StdRng::seed_from_u64(seed);
            let roots = RootSpec::default().grow(&trunk, 0, &mut rng);
            assert!(roots.len() >= 4);
            for root in roots {
                let top = |p: Point3<f32>| (p + Vector3::y() * root.radius).y;
                assert!(
                    top(root.base) < 0.01,
                    "seed {seed}: a root leaves above ground"
                );
                assert!(
                    root.tip.y + root.radius < 0.0,
                    "seed {seed}: a root tip is out"
                );
            }
        }
    }
}
