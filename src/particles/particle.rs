//! Core particle data structures.

use nalgebra::{Vector3, Vector4};

/// A single particle in the system.
#[derive(Debug, Clone)]
pub struct Particle {
    /// World position of the particle.
    pub position: Vector3<f32>,
    /// Velocity in world units per second.
    pub velocity: Vector3<f32>,
    /// RGBA color (interpolated between start and end).
    pub color: Vector4<f32>,
    /// Starting color for interpolation.
    pub start_color: Vector4<f32>,
    /// Ending color for interpolation.
    pub end_color: Vector4<f32>,
    /// Particle size (radius for billboards).
    pub size: f32,
    /// Remaining lifetime in seconds.
    pub life: f32,
    /// Initial lifetime for calculating normalized age.
    pub max_life: f32,
    /// Gravity multiplier (1.0 = normal gravity, 0.0 = no gravity).
    pub gravity_scale: f32,
    /// Drag coefficient for air resistance.
    pub drag: f32,
}

impl Particle {
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

    /// Update the particle for one frame.
    pub fn update(&mut self, dt: f32, gravity: f32) {
        // Apply gravity
        self.velocity.y -= gravity * self.gravity_scale * dt;

        // Apply drag
        let speed = self.velocity.magnitude();
        if speed > 0.001 {
            let drag_force = speed * speed * self.drag;
            let drag_decel = (drag_force * dt).min(speed);
            self.velocity -= self.velocity.normalize() * drag_decel;
        }

        // Update position
        self.position += self.velocity * dt;

        // Update lifetime
        self.life -= dt;

        // Interpolate color based on normalized age
        let t = self.normalized_age();
        self.color = self.start_color.lerp(&self.end_color, t);
    }
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
