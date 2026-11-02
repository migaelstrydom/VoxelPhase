//! What a clump of foliage is made of: a few overlapping round lobes.
//!
//! ```text
//!        ╭──╮  ╭───╮
//!      ╭─┤  ╰──┤   ├╮      satellite lobes round a central one, each a
//!      │ ╰─╮ ● ╰─╮ ││      ball of leaves; the core physics sees is a
//!      ╰───┴─────┴─╯       smaller ball inside each (●)
//! ```
//!
//! Every other part reads this one description: the core's colliders
//! (`core`), the drag volume (`drag`) and the drawn skin (`skin`). Lobe
//! centres are in the clump's own frame, whose origin is the core's centre of
//! mass.

use nalgebra::{Point3, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

/// One ball of leaves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lobe {
    /// Centre, in the clump's frame.
    pub centre: Point3<f32>,
    /// Radius of the leafy surface, to the tops of its nubs.
    pub radius: f32,
}

/// A clump of foliage: its lobes, in its own frame.
#[derive(Debug, Clone, PartialEq)]
pub struct FoliageShape {
    /// The lobes. The first is the central one.
    pub lobes: Vec<Lobe>,
}

/// How to grow a shrub's shape from a seed.
#[derive(Debug, Clone, Copy)]
pub struct ShrubSpec {
    /// Radius of the whole clump: no lobe reaches further from its centre.
    pub radius: f32,
    /// How many lobes sit round the central one.
    pub satellites: usize,
    /// Satellite radius as a share of the clump's, drawn from this range.
    pub satellite_radius: (f32, f32),
    /// Squash of the clump's height against its width: below 1 a mound,
    /// above it a column.
    pub height_ratio: f32,
}

impl Default for ShrubSpec {
    fn default() -> Self {
        Self {
            radius: 0.6,
            satellites: 5,
            satellite_radius: (0.42, 0.55),
            height_ratio: 0.85,
        }
    }
}

impl ShrubSpec {
    /// A shrub of `radius`, otherwise as the defaults.
    pub fn with_radius(radius: f32) -> Self {
        Self {
            radius,
            ..Self::default()
        }
    }

    /// The shape this spec grows from `seed`, re-centred so the lobes'
    /// volume-weighted centre is the origin.
    ///
    /// The central lobe takes what the satellites leave: each satellite sits
    /// so its outer surface just reaches the clump's radius, in a direction
    /// spread round the sphere (a jittered Fibonacci spiral) so no two crowd
    /// together.
    pub fn grow(&self, seed: u32) -> FoliageShape {
        let mut rng = StdRng::seed_from_u64(seed as u64);
        let squash = Vector3::new(1.0, self.height_ratio, 1.0);
        let mut lobes = vec![Lobe {
            centre: Point3::origin(),
            radius: self.radius * 0.62,
        }];
        let golden = std::f32::consts::PI * (3.0 - 5.0f32.sqrt());
        let twist = rng.gen_range(0.0..std::f32::consts::TAU);
        for i in 0..self.satellites {
            let y = 1.0 - 2.0 * (i as f32 + 0.5) / self.satellites as f32;
            let y = (y + rng.gen_range(-0.2..0.2)).clamp(-0.9, 0.9);
            let ring = (1.0 - y * y).sqrt();
            let azimuth = twist + golden * i as f32 + rng.gen_range(-0.4..0.4);
            let direction = Vector3::new(ring * azimuth.cos(), y, ring * azimuth.sin());
            let radius =
                self.radius * rng.gen_range(self.satellite_radius.0..self.satellite_radius.1);
            let reach = (self.radius - radius).max(0.0);
            lobes.push(Lobe {
                centre: Point3::from(direction.component_mul(&squash) * reach),
                radius,
            });
        }
        FoliageShape { lobes }.centred()
    }
}

impl FoliageShape {
    /// The same lobes, moved so their volume-weighted centre is the origin.
    pub fn centred(mut self) -> Self {
        let centre = self.volume_centre();
        for lobe in &mut self.lobes {
            lobe.centre -= centre.coords;
        }
        self
    }

    /// Volume-weighted centre of the lobes (overlaps counted twice).
    pub fn volume_centre(&self) -> Point3<f32> {
        let mut weighted = Vector3::zeros();
        let mut total = 0.0;
        for lobe in &self.lobes {
            let volume = lobe.radius.powi(3);
            weighted += lobe.centre.coords * volume;
            total += volume;
        }
        if total > 0.0 {
            Point3::from(weighted / total)
        } else {
            Point3::origin()
        }
    }

    /// Distance from the origin to the furthest leaf.
    pub fn bounding_radius(&self) -> f32 {
        self.lobes
            .iter()
            .map(|l| l.centre.coords.magnitude() + l.radius)
            .fold(0.0, f32::max)
    }

    /// The lowest point of the leaves, as a height below the origin: how far
    /// down a clump standing upright reaches.
    pub fn depth_below(&self) -> f32 {
        self.lobes
            .iter()
            .map(|l| l.radius - l.centre.y)
            .fold(0.0, f32::max)
    }

    /// The space the leaves fill, in m³, by sampling: overlapping lobes count
    /// once.
    pub fn leaf_volume(&self) -> f32 {
        const STEPS: i32 = 24;
        let r = self.bounding_radius();
        let cell = 2.0 * r / STEPS as f32;
        let mut inside = 0usize;
        for i in 0..STEPS {
            for j in 0..STEPS {
                for k in 0..STEPS {
                    let p = Point3::new(
                        -r + (i as f32 + 0.5) * cell,
                        -r + (j as f32 + 0.5) * cell,
                        -r + (k as f32 + 0.5) * cell,
                    );
                    if self.contains(p) {
                        inside += 1;
                    }
                }
            }
        }
        inside as f32 * cell.powi(3)
    }

    /// Whether `point` (in the clump's frame) is among the leaves.
    pub fn contains(&self, point: Point3<f32>) -> bool {
        self.lobes
            .iter()
            .any(|l| (point - l.centre).magnitude_squared() < l.radius * l.radius)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A grown shrub stays inside its radius, is centred, and is the same
    /// shrub every time it grows from the same seed.
    #[test]
    fn a_shrub_grows_within_its_radius_the_same_each_time() {
        let spec = ShrubSpec::with_radius(0.7);
        let shape = spec.grow(7);
        assert_eq!(shape.lobes.len(), 1 + spec.satellites);
        // Re-centring moves the lobes by up to the satellites' imbalance.
        assert!(
            shape.bounding_radius() < 0.7 * 1.2,
            "{}",
            shape.bounding_radius()
        );
        assert!(shape.volume_centre().coords.magnitude() < 1e-4);
        assert_eq!(shape, spec.grow(7));
        assert_ne!(shape, spec.grow(8));
    }

    /// The sampled volume of one lobe is a sphere's.
    #[test]
    fn one_lobe_fills_a_sphere() {
        let shape = FoliageShape {
            lobes: vec![Lobe {
                centre: Point3::origin(),
                radius: 0.5,
            }],
        };
        let sphere = 4.0 / 3.0 * std::f32::consts::PI * 0.125;
        assert!((shape.leaf_volume() - sphere).abs() < sphere * 0.03);
    }
}
