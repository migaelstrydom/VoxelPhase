//! What a clump of foliage is made of: a few overlapping lobes of leaves,
//! each an ellipsoid: round on a topiary shrub, tall on a cypress.
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

use nalgebra::{Point3, UnitQuaternion, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

/// One mass of leaves: an ellipsoid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lobe {
    /// Centre, in the clump's frame.
    pub centre: Point3<f32>,
    /// Semi-axes along the lobe's own x, y and z, to the tops of its nubs.
    pub radii: Vector3<f32>,
    /// How the lobe's axes are turned in the clump's frame.
    pub rotation: UnitQuaternion<f32>,
}

impl Lobe {
    /// A round lobe of `radius` at `centre`.
    pub fn round(centre: Point3<f32>, radius: f32) -> Self {
        Self {
            centre,
            radii: Vector3::repeat(radius),
            rotation: UnitQuaternion::identity(),
        }
    }

    /// Its shortest semi-axis.
    pub fn least_radius(&self) -> f32 {
        self.radii.min()
    }

    /// Its longest semi-axis.
    pub fn greatest_radius(&self) -> f32 {
        self.radii.max()
    }

    /// The radius of a ball of its volume.
    pub fn mean_radius(&self) -> f32 {
        self.radii.product().cbrt()
    }

    /// Its volume, m³.
    pub fn volume(&self) -> f32 {
        4.0 / 3.0 * std::f32::consts::PI * self.radii.product()
    }

    /// Its surface area, m², by Knud Thomsen's formula (within about 1%).
    pub fn area(&self) -> f32 {
        const P: f32 = 1.6075;
        let [a, b, c] = [self.radii.x, self.radii.y, self.radii.z].map(|r| r.powf(P));
        4.0 * std::f32::consts::PI * ((a * b + a * c + b * c) / 3.0).powf(1.0 / P)
    }

    /// Where `direction`, a unit vector in the lobe's own frame, lands on its
    /// surface scaled by `scale`, in the clump's frame: the unit sphere
    /// stretched to the lobe.
    pub fn surface_point(&self, direction: Vector3<f32>, scale: f32) -> Point3<f32> {
        self.centre + self.rotation * self.radii.component_mul(&direction) * scale
    }

    /// The outward normal of the surface at `direction`, in the clump's
    /// frame.
    pub fn normal_at(&self, direction: Vector3<f32>) -> Vector3<f32> {
        (self.rotation * direction.component_div(&self.radii)).normalize()
    }

    /// `point` in the lobe's unit-sphere coordinates: inside the lobe where
    /// its length is under 1.
    fn unit(&self, point: Point3<f32>) -> Vector3<f32> {
        (self.rotation.inverse() * (point - self.centre)).component_div(&self.radii)
    }

    /// Whether `point` is inside the lobe shrunk by `scale`.
    pub fn contains_scaled(&self, point: Point3<f32>, scale: f32) -> bool {
        self.unit(point).magnitude_squared() < scale * scale
    }

    /// How far `point` is outside the surface, m: negative inside. Exact for
    /// a round lobe, and close for an ellipsoid (Quilez's bound, exact on
    /// its axes).
    pub fn signed_distance(&self, point: Point3<f32>) -> f32 {
        let local = self.rotation.inverse() * (point - self.centre);
        let k0 = local.component_div(&self.radii).magnitude();
        let k1 = local
            .component_div(&self.radii.component_mul(&self.radii))
            .magnitude();
        if k1 < 1e-9 {
            return -self.least_radius();
        }
        k0 * (k0 - 1.0) / k1
    }

    /// How far it reaches from its centre along `axis`, a unit vector in the
    /// clump's frame.
    pub fn extent_along(&self, axis: Vector3<f32>) -> f32 {
        (self.rotation.inverse() * axis)
            .component_mul(&self.radii)
            .magnitude()
    }
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
    /// How low round the central lobe the satellites go, as the sine of
    /// their elevation: -1 all the way round, as a clump that has tumbled;
    /// near 0 only the sides and top, as a shrub growing from the ground.
    pub lowest_satellite: f32,
}

impl Default for ShrubSpec {
    fn default() -> Self {
        Self {
            radius: 0.6,
            satellites: 5,
            satellite_radius: (0.42, 0.55),
            height_ratio: 0.85,
            lowest_satellite: -1.0,
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

    /// The same shrub growing from the ground: its satellites on its sides
    /// and top, so it stands on a broad base rather than perched on a lobe.
    pub fn rooted(self) -> Self {
        Self {
            lowest_satellite: -0.2,
            ..self
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
        let mut lobes = vec![Lobe::round(Point3::origin(), self.radius * 0.62)];
        let golden = std::f32::consts::PI * (3.0 - 5.0f32.sqrt());
        let twist = rng.gen_range(0.0..std::f32::consts::TAU);
        for i in 0..self.satellites {
            let span = 1.0 - self.lowest_satellite;
            let y = 1.0 - span * (i as f32 + 0.5) / self.satellites as f32;
            let y = (y + rng.gen_range(-0.2..0.2)).clamp(self.lowest_satellite.max(-0.9), 0.9);
            let ring = (1.0 - y * y).sqrt();
            let azimuth = twist + golden * i as f32 + rng.gen_range(-0.4..0.4);
            let direction = Vector3::new(ring * azimuth.cos(), y, ring * azimuth.sin());
            let radius =
                self.radius * rng.gen_range(self.satellite_radius.0..self.satellite_radius.1);
            let reach = (self.radius - radius).max(0.0);
            lobes.push(Lobe::round(
                Point3::from(direction.component_mul(&squash) * reach),
                radius,
            ));
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
            let volume = lobe.radii.product();
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
            .map(|l| l.centre.coords.magnitude() + l.greatest_radius())
            .fold(0.0, f32::max)
    }

    /// The lowest point of the leaves, as a height below the origin: how far
    /// down a clump standing upright reaches.
    pub fn depth_below(&self) -> f32 {
        self.lobes
            .iter()
            .map(|l| l.extent_along(Vector3::y()) - l.centre.y)
            .fold(0.0, f32::max)
    }

    /// The space the leaves fill, in m³, by sampling: overlapping lobes count
    /// once.
    pub fn leaf_volume(&self) -> f32 {
        const STEPS: i32 = 24;
        let (low, high) = self.bounds();
        let cell = (high - low) / STEPS as f32;
        let mut inside = 0usize;
        for i in 0..STEPS {
            for j in 0..STEPS {
                for k in 0..STEPS {
                    let step = Vector3::new(i as f32 + 0.5, j as f32 + 0.5, k as f32 + 0.5);
                    if self.contains(low + step.component_mul(&cell)) {
                        inside += 1;
                    }
                }
            }
        }
        inside as f32 * cell.product()
    }

    /// The box the leaves fill, as its lowest and highest corners.
    pub fn bounds(&self) -> (Point3<f32>, Point3<f32>) {
        let extent = |l: &Lobe| {
            Vector3::new(
                l.extent_along(Vector3::x()),
                l.extent_along(Vector3::y()),
                l.extent_along(Vector3::z()),
            )
        };
        let low = self
            .lobes
            .iter()
            .map(|l| l.centre - extent(l))
            .fold(Point3::from(Vector3::repeat(f32::MAX)), |a, b| a.inf(&b));
        let high = self
            .lobes
            .iter()
            .map(|l| l.centre + extent(l))
            .fold(Point3::from(Vector3::repeat(f32::MIN)), |a, b| a.sup(&b));
        (low, high)
    }

    /// Whether `point` (in the clump's frame) is among the leaves.
    pub fn contains(&self, point: Point3<f32>) -> bool {
        self.lobes.iter().any(|l| l.contains_scaled(point, 1.0))
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
            lobes: vec![Lobe::round(Point3::origin(), 0.5)],
        };
        let sphere = 4.0 / 3.0 * std::f32::consts::PI * 0.125;
        assert!((shape.leaf_volume() - sphere).abs() < sphere * 0.03);
    }

    /// A tall lobe turned on its side: its distance is exact on its axes,
    /// its volume is an ellipsoid's, and it reaches down by its width.
    #[test]
    fn a_turned_tall_lobe_measures_as_an_ellipsoid() {
        let lobe = Lobe {
            radii: Vector3::new(0.4, 1.5, 0.3),
            rotation: UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                std::f32::consts::FRAC_PI_2,
            ),
            ..Lobe::round(Point3::new(0.0, 0.2, 0.0), 0.4)
        };
        let along = lobe.signed_distance(Point3::new(2.0, 0.2, 0.0));
        assert!((along - 0.5).abs() < 1e-5, "{along}");
        let across = lobe.signed_distance(Point3::new(0.0, 0.2, 0.1));
        assert!((across + 0.2).abs() < 1e-5, "{across}");
        let shape = FoliageShape { lobes: vec![lobe] };
        assert!((shape.depth_below() - (0.4 - 0.2)).abs() < 1e-5);
        assert!((shape.bounding_radius() - (0.2 + 1.5)).abs() < 1e-5);
        let exact = lobe.volume();
        let sampled = FoliageShape {
            lobes: vec![Lobe {
                centre: Point3::origin(),
                ..lobe
            }],
        }
        .leaf_volume();
        assert!(
            (sampled - exact).abs() < 0.05 * exact,
            "{sampled} vs {exact}"
        );
    }
}
