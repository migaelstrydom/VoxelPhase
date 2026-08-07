//! What an effect looks like, as data.
//!
//! Every particle effect in the engine is one [`ParticleSpec`] — a description
//! of the particles to make, with no code of its own. The spawner reads a spec
//! and produces particles from it, so adding an effect is authoring numbers
//! rather than writing another spawn branch.
//!
//! ```text
//!   ParticleSpec ──▶ sample(position, scale, rng) ──▶ Particle ──▶ ParticlePool
//! ```

use std::f32::consts::TAU;

use nalgebra::{Vector3, Vector4};
use rand::Rng;

use super::particle::Particle;
use super::ramp::ColourRamp;

/// A closed range a spawner draws uniformly from.
///
/// Effects are authored as ranges rather than single values throughout: a burst
/// whose particles all share a lifetime dies as one flat event, which is the
/// most recognisable tell of a cheap particle system.
#[derive(Debug, Clone, Copy)]
pub struct Spread {
    pub min: f32,
    pub max: f32,
}

impl Spread {
    pub const fn new(min: f32, max: f32) -> Self {
        Self { min, max }
    }

    /// A range with no variation.
    pub const fn fixed(value: f32) -> Self {
        Self::new(value, value)
    }

    /// A value drawn uniformly from the range.
    pub fn sample(&self, rng: &mut impl Rng) -> f32 {
        if self.max <= self.min {
            self.min
        } else {
            rng.gen_range(self.min..self.max)
        }
    }

    /// A value drawn from the range mirrored about zero — for quantities like
    /// spin, where the range describes a magnitude but the sign is arbitrary.
    pub fn sample_signed(&self, rng: &mut impl Rng) -> f32 {
        let magnitude = self.sample(rng);
        if rng.gen_bool(0.5) {
            -magnitude
        } else {
            magnitude
        }
    }
}

/// The shape a burst is thrown in.
///
/// This is the difference between effects far more than colour is: a fireball
/// is a sphere, dust from a blast is a ground-hugging ring, and a splash is a
/// crown. Each variant describes a direction distribution only — speed comes
/// from the spec.
#[derive(Debug, Clone, Copy)]
pub enum LaunchPattern {
    /// Every direction equally likely. Fireballs, flashes, sparks.
    Sphere,

    /// Upper hemisphere, pulled towards vertical by `up_bias`. 0 is a bare
    /// hemisphere, large values approach a straight-up jet.
    Upward { up_bias: f32 },

    /// A ring hugging the ground, thrown outward at `elevation` degrees above
    /// horizontal. This is the blast skirt — the low, fast sheet of dust that
    /// runs outward along the surface and reads as a shockwave.
    Ring { elevation: Spread },
}

impl LaunchPattern {
    /// A unit direction drawn from this pattern.
    pub fn sample(&self, rng: &mut impl Rng) -> Vector3<f32> {
        match *self {
            Self::Sphere => random_direction(rng),

            Self::Upward { up_bias } => {
                let mut direction = random_direction(rng);
                direction.y = direction.y.abs() + up_bias;
                direction.normalize()
            }

            Self::Ring { elevation } => {
                let azimuth = rng.gen_range(0.0..TAU);
                let pitch = elevation.sample(rng).to_radians();
                Vector3::new(
                    azimuth.cos() * pitch.cos(),
                    pitch.sin(),
                    azimuth.sin() * pitch.cos(),
                )
            }
        }
    }
}

/// A complete particle effect, as authored data.
///
/// Fields are grouped as the particle itself is: how many and how long, how it
/// moves, and how it is drawn.
#[derive(Debug, Clone)]
pub struct ParticleSpec {
    /// Seconds a particle lives.
    pub lifetime: Spread,
    /// Billboard radius at birth.
    pub size: Spread,
    /// Launch speed in world units per second.
    pub speed: Spread,
    /// Direction distribution for the launch.
    pub launch: LaunchPattern,
    /// Radius of the sphere around the spawn point particles are scattered in.
    ///
    /// A burst spawned from a single point always betrays that point: the
    /// particles visibly radiate from a pinhole. Giving the source a volume
    /// removes the tell for the cost of one random direction.
    pub spawn_radius: f32,

    /// Colour and opacity over life.
    pub ramp: ColourRamp,
    /// Size multiple reached at death. See [`Particle::growth`].
    pub growth: f32,
    /// Magnitude of the mask's rotation rate, in radians per second; the sign
    /// is chosen per particle.
    pub spin: Spread,
    /// Compositing mode, 0 (blend) to 1 (add). See [`Particle::additive`].
    pub additive: f32,
    /// Noise erosion of the particle's silhouette. See [`Particle::billow`].
    pub billow: f32,

    /// Gravity multiplier; negative for anything buoyant.
    pub gravity_scale: f32,
    /// Air drag coefficient.
    pub drag: f32,
    /// Swirl strength. See [`Particle::turbulence`].
    pub turbulence: f32,
    /// Motion smear, in seconds. See [`Particle::stretch`].
    pub stretch: f32,
}

impl Default for ParticleSpec {
    /// A plain white puff. Every field is meant to be overridden; the default
    /// exists so a new effect only has to state what makes it different.
    fn default() -> Self {
        Self {
            lifetime: Spread::new(0.5, 1.0),
            size: Spread::new(0.1, 0.2),
            speed: Spread::new(1.0, 3.0),
            launch: LaunchPattern::Sphere,
            spawn_radius: 0.0,
            ramp: ColourRamp::fade(
                Vector4::new(1.0, 1.0, 1.0, 1.0),
                Vector4::new(1.0, 1.0, 1.0, 0.0),
            ),
            growth: 1.0,
            spin: Spread::fixed(0.0),
            additive: 0.0,
            billow: 0.0,
            gravity_scale: 0.0,
            drag: 0.0,
            turbulence: 0.0,
            stretch: 0.0,
        }
    }
}

impl ParticleSpec {
    /// One particle drawn from this spec, born at `origin`.
    ///
    /// `scale` stretches the effect in space: sizes, speeds and the spawn
    /// volume all grow with it, while timings, colours and drag do not. That
    /// keeps a big explosion reading as the same event seen larger rather than
    /// as a slower one — the alternative, scaling lifetimes too, makes large
    /// blasts feel sluggish.
    pub fn sample(&self, origin: Vector3<f32>, scale: f32, rng: &mut impl Rng) -> Particle {
        let lifetime = self.lifetime.sample(rng);
        let direction = self.launch.sample(rng);
        let offset = random_direction(rng) * self.spawn_radius * scale * rng.gen::<f32>();

        Particle {
            position: origin + offset,
            velocity: direction * self.speed.sample(rng) * scale,
            life: lifetime,
            max_life: lifetime,
            gravity_scale: self.gravity_scale,
            drag: self.drag,
            turbulence: self.turbulence,
            colour: self.ramp.sample(0.0),
            ramp: self.ramp,
            size: self.size.sample(rng) * scale,
            growth: self.growth,
            stretch: self.stretch,
            angle: rng.gen_range(0.0..TAU),
            spin: self.spin.sample_signed(rng),
            additive: self.additive,
            billow: self.billow,
            seed: rng.gen_range(0.0..1.0),
        }
    }
}

/// Generate a random direction on the unit sphere.
pub fn random_direction(rng: &mut impl Rng) -> Vector3<f32> {
    let theta = rng.gen_range(0.0..TAU);
    let phi = rng.gen_range(-1.0f32..1.0).acos();

    Vector3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fixed_spread_always_gives_its_value() {
        let mut rng = rand::thread_rng();
        let spread = Spread::fixed(7.0);
        for _ in 0..16 {
            assert_eq!(spread.sample(&mut rng), 7.0);
        }
    }

    #[test]
    fn a_signed_spread_reaches_both_directions() {
        let mut rng = rand::thread_rng();
        let spread = Spread::new(1.0, 2.0);

        let samples: Vec<_> = (0..64).map(|_| spread.sample_signed(&mut rng)).collect();
        assert!(samples.iter().any(|value| *value < 0.0));
        assert!(samples.iter().any(|value| *value > 0.0));
        assert!(samples
            .iter()
            .all(|value| (1.0..=2.0).contains(&value.abs())));
    }

    #[test]
    fn a_ground_ring_stays_low_and_spreads_all_the_way_round() {
        let mut rng = rand::thread_rng();
        let pattern = LaunchPattern::Ring {
            elevation: Spread::new(0.0, 15.0),
        };

        let mut saw_east = false;
        let mut saw_west = false;
        for _ in 0..256 {
            let direction = pattern.sample(&mut rng);
            assert!((direction.magnitude() - 1.0).abs() < 1e-4);
            // sin(15°) is the steepest the ring may climb.
            assert!(direction.y >= 0.0 && direction.y <= 0.26, "{direction:?}");
            saw_east |= direction.x > 0.5;
            saw_west |= direction.x < -0.5;
        }
        assert!(saw_east && saw_west, "ring did not cover the full circle");
    }

    #[test]
    fn an_upward_pattern_never_throws_anything_downwards() {
        let mut rng = rand::thread_rng();
        let pattern = LaunchPattern::Upward { up_bias: 0.5 };

        for _ in 0..256 {
            assert!(pattern.sample(&mut rng).y > 0.0);
        }
    }

    #[test]
    fn scale_grows_an_effect_in_space_but_not_in_time() {
        let mut rng = rand::thread_rng();
        let spec = ParticleSpec {
            lifetime: Spread::fixed(2.0),
            size: Spread::fixed(0.5),
            speed: Spread::fixed(10.0),
            spawn_radius: 1.0,
            ..Default::default()
        };

        let small = spec.sample(Vector3::zeros(), 1.0, &mut rng);
        let large = spec.sample(Vector3::zeros(), 3.0, &mut rng);

        assert_eq!(large.size, small.size * 3.0);
        assert!((large.velocity.magnitude() - small.velocity.magnitude() * 3.0).abs() < 1e-3);
        assert_eq!(large.max_life, small.max_life);
    }

    #[test]
    fn a_spec_with_no_spawn_radius_puts_everything_on_the_origin() {
        let mut rng = rand::thread_rng();
        let spec = ParticleSpec::default();
        let origin = Vector3::new(3.0, -1.0, 2.0);

        for _ in 0..16 {
            assert_eq!(spec.sample(origin, 2.0, &mut rng).position, origin);
        }
    }
}
