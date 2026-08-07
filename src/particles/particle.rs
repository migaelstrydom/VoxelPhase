//! Core particle data structures.

use nalgebra::{Vector3, Vector4};

use super::ramp::ColourRamp;

/// Spatial frequency of the swirl field, in radians per world unit.
///
/// Low enough that neighbouring particles in one puff are pushed the same way —
/// a swirl finer than the puff scrambles it into noise instead of curling it.
const SWIRL_FREQUENCY: f32 = 0.6;

/// How fast the swirl field itself churns, in radians per second. Without this
/// the field is static and a rising column threads through fixed eddies, which
/// reads as a pattern rather than as air moving.
const SWIRL_CHURN: f32 = 1.3;

/// A single particle in the system.
///
/// Split conceptually into three groups: where it is, how it moves, and how it
/// is drawn. The draw fields are all per-particle rather than per-effect so one
/// pool of particles can hold smoke and fire at once and the renderer never has
/// to ask what kind of thing it is looking at.
#[derive(Debug, Clone)]
pub struct Particle {
    /// World position of the particle.
    pub position: Vector3<f32>,
    /// Velocity in world units per second.
    pub velocity: Vector3<f32>,

    /// Remaining lifetime in seconds.
    pub life: f32,
    /// Initial lifetime, for calculating normalised age.
    pub max_life: f32,

    /// Gravity multiplier (1.0 = normal gravity, 0.0 = none, negative rises).
    pub gravity_scale: f32,
    /// Drag coefficient for air resistance.
    pub drag: f32,
    /// Strength of the swirl pushing the particle off a straight path, in
    /// world units per second. Zero moves ballistically.
    ///
    /// This is what stops a cloud of puffs looking like a firework: real smoke
    /// is dragged sideways by air that is itself moving, so its edges curl.
    pub turbulence: f32,

    /// Colour over life, sampled into `colour` each update.
    pub ramp: ColourRamp,
    /// Current RGBA, sampled from `ramp` at the particle's age.
    pub colour: Vector4<f32>,

    /// Billboard radius at birth.
    pub size: f32,
    /// Multiple of `size` the billboard reaches at death. 1.0 holds it steady;
    /// above that the particle expands as it ages, which is most of what makes
    /// a fireball read as an expanding volume rather than a flying blob.
    pub growth: f32,

    /// Seconds of the particle's own motion its billboard is smeared over.
    ///
    /// 0 draws a round particle. Above that the billboard stretches along the
    /// direction of travel by `velocity * stretch`, which is what turns a fast
    /// stream of embers into streaks instead of a dotted line. Kept as a time
    /// rather than a length so a particle that slows down rounds off by itself.
    pub stretch: f32,

    /// Current rotation of the particle's mask, in radians.
    pub angle: f32,
    /// Rotation rate in radians per second. Two puffs from the same burst
    /// spinning at different rates stop them reading as copies of each other.
    pub spin: f32,

    /// How the particle is composited: 0 blends over the scene like smoke, 1
    /// adds to it like flame. Values between cross-fade, which is how a
    /// fireball puff hands over from glowing to occluding without a seam.
    pub additive: f32,
    /// How much the fragment mask is eaten away by noise: 0 draws a clean soft
    /// disc (sparks, droplets), 1 a torn, billowing edge (fire, smoke, dust).
    pub billow: f32,
    /// Per-particle noise offset, so two overlapping puffs are not the same
    /// shape. Any value works; the shader only needs it to differ.
    pub seed: f32,
}

impl Default for Particle {
    fn default() -> Self {
        Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            life: 1.0,
            max_life: 1.0,
            gravity_scale: 1.0,
            drag: 0.0,
            turbulence: 0.0,
            ramp: ColourRamp::default(),
            colour: Vector4::new(1.0, 1.0, 1.0, 1.0),
            size: 0.1,
            growth: 1.0,
            stretch: 0.0,
            angle: 0.0,
            spin: 0.0,
            additive: 0.0,
            billow: 0.0,
            seed: 0.0,
        }
    }
}

impl Particle {
    /// A particle with the given placement and appearance, and inert dynamics
    /// (full gravity, no drag) for the caller to override.
    pub fn new(
        position: Vector3<f32>,
        velocity: Vector3<f32>,
        size: f32,
        lifetime: f32,
        ramp: ColourRamp,
    ) -> Self {
        Self {
            position,
            velocity,
            life: lifetime,
            max_life: lifetime,
            colour: ramp.sample(0.0),
            ramp,
            size,
            ..Self::default()
        }
    }

    /// Set gravity response and air drag.
    pub fn with_dynamics(mut self, gravity_scale: f32, drag: f32) -> Self {
        self.gravity_scale = gravity_scale;
        self.drag = drag;
        self
    }

    /// Smear the billboard over this many seconds of travel.
    pub fn with_stretch(mut self, stretch: f32) -> Self {
        self.stretch = stretch;
        self
    }

    /// Returns true if this particle has expired.
    pub fn is_dead(&self) -> bool {
        self.life <= 0.0
    }

    /// Returns the normalized age of this particle (0.0 = just born, 1.0 = about to die).
    pub fn normalized_age(&self) -> f32 {
        if self.max_life > 0.0 {
            1.0 - (self.life / self.max_life)
        } else {
            1.0
        }
    }

    /// Billboard radius right now, after growth.
    pub fn drawn_size(&self) -> f32 {
        let t = self.normalized_age();
        self.size * (1.0 + (self.growth - 1.0) * t)
    }

    /// Update the particle for one frame.
    pub fn update(&mut self, dt: f32, gravity: f32) {
        self.velocity.y -= gravity * self.gravity_scale * dt;

        if self.turbulence > 0.0 {
            let age = self.max_life - self.life;
            self.velocity += swirl(self.position, age) * self.turbulence * dt;
        }

        let speed = self.velocity.magnitude();
        if speed > 0.001 {
            let drag_force = speed * speed * self.drag;
            let drag_decel = (drag_force * dt).min(speed);
            self.velocity -= self.velocity.normalize() * drag_decel;
        }

        self.position += self.velocity * dt;
        self.angle += self.spin * dt;
        self.life -= dt;

        self.colour = self.ramp.sample(self.normalized_age());
    }
}

/// A divergence-free-ish swirl field, evaluated analytically.
///
/// Each component is driven by the *other* two axes, so the field curls rather
/// than pushing everything one way — the cheap stand-in for the eddies a fluid
/// solver would give, at a handful of sines per particle per frame instead of a
/// grid solve. It is not truly divergence free, which does not matter: nothing
/// here conserves mass, it only has to look unpredictable.
fn swirl(position: Vector3<f32>, time: f32) -> Vector3<f32> {
    let p = position * SWIRL_FREQUENCY;
    let t = time * SWIRL_CHURN;
    Vector3::new(
        (p.y + t * 1.3).sin() * (p.z - t * 0.7).cos(),
        (p.z + t * 0.9).sin() * (p.x - t * 1.1).cos(),
        (p.x + t * 1.7).sin() * (p.y - t * 0.5).cos(),
    )
}

/// Pool of particles for efficient memory management.
///
/// Uses a fixed-size pool to avoid allocations during gameplay.
pub struct ParticlePool {
    particles: Vec<Particle>,
    max_particles: usize,
}

impl ParticlePool {
    /// Create a new particle pool with the given maximum capacity.
    pub fn new(max_particles: usize) -> Self {
        Self {
            particles: Vec::with_capacity(max_particles),
            max_particles,
        }
    }

    /// Add a new particle to the pool.
    ///
    /// If the pool is full, the oldest particle is replaced.
    pub fn spawn(&mut self, particle: Particle) {
        if self.particles.len() < self.max_particles {
            self.particles.push(particle);
        } else {
            // Find and replace the oldest particle (shortest remaining life)
            if let Some((idx, _)) = self
                .particles
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.life.partial_cmp(&b.life).unwrap())
            {
                self.particles[idx] = particle;
            }
        }
    }

    /// Update all particles and remove dead ones.
    pub fn update(&mut self, dt: f32, gravity: f32) {
        // Update all particles
        for particle in &mut self.particles {
            particle.update(dt, gravity);
        }

        // Remove dead particles
        self.particles.retain(|p| !p.is_dead());
    }

    /// Get an iterator over all active particles.
    pub fn iter(&self) -> impl Iterator<Item = &Particle> {
        self.particles.iter()
    }

    /// All active particles, in no meaningful order.
    ///
    /// The renderer needs random access to sort them by depth without copying
    /// the pool; nothing else should care about the backing storage.
    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }

    /// Get the number of active particles.
    pub fn count(&self) -> usize {
        self.particles.len()
    }

    /// Check if the pool is empty.
    pub fn is_empty(&self) -> bool {
        self.particles.is_empty()
    }
}

impl Default for ParticlePool {
    fn default() -> Self {
        Self::new(10000) // Default to 10k particles max
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_particle(lifetime: f32) -> Particle {
        Particle::new(
            Vector3::zeros(),
            Vector3::zeros(),
            1.0,
            lifetime,
            ColourRamp::default(),
        )
    }

    #[test]
    fn growth_runs_from_the_birth_size_to_its_multiple() {
        let mut particle = test_particle(1.0);
        particle.growth = 4.0;

        assert_eq!(particle.drawn_size(), 1.0);
        particle.life = 0.5;
        assert_eq!(particle.drawn_size(), 2.5);
        particle.life = 0.0;
        assert_eq!(particle.drawn_size(), 4.0);
    }

    #[test]
    fn a_particle_that_does_not_grow_keeps_its_size() {
        let mut particle = test_particle(1.0);
        for step in 0..=10 {
            particle.life = 1.0 - step as f32 / 10.0;
            assert_eq!(particle.drawn_size(), 1.0);
        }
    }

    #[test]
    fn turbulence_pushes_a_particle_off_a_straight_path() {
        let launch = Vector3::new(0.0, 0.0, 4.0);

        let mut straight = test_particle(2.0);
        straight.velocity = launch;
        straight.position = Vector3::new(1.0, 2.0, 3.0);
        let mut swirled = straight.clone();
        swirled.turbulence = 3.0;

        for _ in 0..30 {
            straight.update(1.0 / 60.0, 0.0);
            swirled.update(1.0 / 60.0, 0.0);
        }

        assert!(
            (swirled.position - straight.position).magnitude() > 0.05,
            "swirl left the path unchanged"
        );
    }

    #[test]
    fn neighbouring_particles_are_swirled_together_not_scattered() {
        // A puff must curl as a body. If the field varied faster than the puff
        // is wide, particles a few centimetres apart would be thrown in
        // opposite directions and the puff would boil instead of roll.
        let here = swirl(Vector3::new(2.0, 1.0, -3.0), 0.4);
        let nearby = swirl(Vector3::new(2.05, 1.05, -3.05), 0.4);

        assert!((here - nearby).magnitude() < 0.15, "field varies too fast");
    }
}
