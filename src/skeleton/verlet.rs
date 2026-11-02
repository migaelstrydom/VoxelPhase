//! Verlet integration physics for skeletal animation.
//!
//! This module provides a position-based physics simulation using Verlet integration.
//! It's particularly well-suited for skeletal animation because:
//! - Position-based (no velocity drift)
//! - Constraints naturally maintain bone lengths
//! - Easy to apply external forces and impulses
//! - Same system works for ropes, chains, and ragdolls

use nalgebra::{Point3, Vector3};

/// A particle in the Verlet simulation.
///
/// Particles are the "joints" of the skeleton. Each has a position and
/// remembers its previous position (velocity is implicit in the difference).
#[derive(Clone, Debug)]
pub struct Particle {
    /// Current position in world space.
    pub position: Point3<f32>,
    /// Previous position (for Verlet integration).
    pub prev_position: Point3<f32>,
    /// Accumulated acceleration for this frame.
    pub acceleration: Vector3<f32>,
    /// Inverse mass (0 = infinite mass / pinned).
    pub inv_mass: f32,
    /// Share of its velocity lost per step at 60 Hz (0-1, higher = more
    /// damping). A step of another length loses the share that compounds to
    /// the same per second.
    pub damping: f32,
}

impl Particle {
    /// Create a new particle at the given position.
    pub fn new(position: Point3<f32>) -> Self {
        Self {
            position,
            prev_position: position,
            acceleration: Vector3::zeros(),
            inv_mass: 1.0,
            damping: 0.02,
        }
    }

    /// Create a pinned (immovable) particle.
    pub fn pinned(position: Point3<f32>) -> Self {
        Self {
            position,
            prev_position: position,
            acceleration: Vector3::zeros(),
            inv_mass: 0.0, // Infinite mass = pinned
            damping: 0.0,
        }
    }

    /// Check if this particle is pinned (immovable).
    #[inline]
    pub fn is_pinned(&self) -> bool {
        self.inv_mass == 0.0
    }

    /// Get the implicit velocity (current - previous position).
    #[inline]
    pub fn velocity(&self) -> Vector3<f32> {
        self.position - self.prev_position
    }

    /// Apply an impulse (instantaneous velocity change).
    pub fn apply_impulse(&mut self, impulse: Vector3<f32>) {
        if !self.is_pinned() {
            // Modify previous position to change implicit velocity
            self.prev_position -= impulse;
        }
    }

    /// Apply a force (will be integrated over time).
    pub fn apply_force(&mut self, force: Vector3<f32>) {
        if !self.is_pinned() {
            self.acceleration += force * self.inv_mass;
        }
    }
}

/// A distance constraint between two particles.
///
/// This is the "bone" of the skeleton - maintains a fixed distance
/// between two particles (joints).
#[derive(Clone, Debug)]
pub struct DistanceConstraint {
    /// Index of the first particle.
    pub particle_a: usize,
    /// Index of the second particle.
    pub particle_b: usize,
    /// Rest length (target distance).
    pub rest_length: f32,
    /// Stiffness (0-1, 1 = completely rigid).
    pub stiffness: f32,
}

impl DistanceConstraint {
    /// Create a new distance constraint.
    pub fn new(particle_a: usize, particle_b: usize, rest_length: f32) -> Self {
        Self {
            particle_a,
            particle_b,
            rest_length,
            stiffness: 1.0, // Fully rigid by default (bones don't stretch)
        }
    }

    /// Create a constraint with a specific stiffness.
    pub fn with_stiffness(mut self, stiffness: f32) -> Self {
        self.stiffness = stiffness.clamp(0.0, 1.0);
        self
    }
}

/// The step `Particle::damping` is stated for.
const DAMPING_REFERENCE_STEP: f32 = 1.0 / 60.0;

/// The Verlet physics simulation.
///
/// Contains particles and constraints, handles integration and solving.
pub struct VerletSystem {
    /// All particles in the simulation.
    pub particles: Vec<Particle>,
    /// Distance constraints (bones).
    pub distance_constraints: Vec<DistanceConstraint>,
    /// Number of constraint solving iterations.
    pub iterations: u32,
    /// Global gravity.
    pub gravity: Vector3<f32>,
    /// Length of the last step, which the positions' difference measures
    /// velocity over; `None` before the first.
    last_dt: Option<f32>,
}

impl VerletSystem {
    /// Create a new Verlet system.
    pub fn new() -> Self {
        Self {
            particles: Vec::new(),
            distance_constraints: Vec::new(),
            iterations: 4, // Good balance of accuracy vs performance
            gravity: Vector3::new(0.0, -9.81, 0.0),
            last_dt: None,
        }
    }

    /// Add a particle and return its index.
    pub fn add_particle(&mut self, particle: Particle) -> usize {
        let idx = self.particles.len();
        self.particles.push(particle);
        idx
    }

    /// Add a distance constraint.
    pub fn add_distance_constraint(&mut self, constraint: DistanceConstraint) {
        self.distance_constraints.push(constraint);
    }

    /// Connect two particles with a distance constraint using their current distance.
    pub fn connect(&mut self, a: usize, b: usize) {
        let rest_length = (self.particles[a].position - self.particles[b].position).magnitude();
        self.add_distance_constraint(DistanceConstraint::new(a, b, rest_length));
    }

    /// Update the simulation by one timestep.
    pub fn update(&mut self, dt: f32) {
        // Apply gravity and integrate
        self.integrate(dt);

        // Solve constraints iteratively
        for _ in 0..self.iterations {
            self.solve_distance_constraints();
        }
    }

    /// Verlet integration step, corrected for a step that differs from the
    /// last: the positions' difference is a velocity over the last step, so
    /// it is rescaled to this one, and damping is applied as a rate. Without
    /// either, a body swung faster or stiffer with the frame rate, and kept
    /// its real-time swing when F9 slowed the step.
    fn integrate(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        let dt2 = dt * dt;
        let rescale = self.last_dt.map_or(1.0, |last| dt / last);
        self.last_dt = Some(dt);

        for particle in &mut self.particles {
            if particle.is_pinned() {
                continue;
            }

            // Apply gravity
            particle.acceleration += self.gravity;

            // Verlet integration: new_pos = pos + (pos - prev_pos) + acc*dt^2
            let velocity = (particle.position - particle.prev_position) * rescale;
            let retained = (1.0 - particle.damping)
                .max(0.0)
                .powf(dt / DAMPING_REFERENCE_STEP);
            let damped_velocity = velocity * retained;

            let new_position = particle.position + damped_velocity + particle.acceleration * dt2;

            particle.prev_position = particle.position;
            particle.position = new_position;
            particle.acceleration = Vector3::zeros();
        }
    }

    /// Solve all distance constraints.
    fn solve_distance_constraints(&mut self) {
        let particles = &mut self.particles;
        for constraint in &self.distance_constraints {
            let (pos_a, pos_b, inv_mass_a, inv_mass_b) = {
                let a = &particles[constraint.particle_a];
                let b = &particles[constraint.particle_b];
                (a.position, b.position, a.inv_mass, b.inv_mass)
            };

            let delta = pos_b - pos_a;
            let current_length = delta.magnitude();

            if current_length < 0.0001 {
                continue; // Avoid division by zero
            }

            let diff = (current_length - constraint.rest_length) / current_length;
            let correction = delta * diff * constraint.stiffness;

            // Distribute correction based on inverse mass
            let total_inv_mass = inv_mass_a + inv_mass_b;
            if total_inv_mass > 0.0 {
                let ratio_a = inv_mass_a / total_inv_mass;
                let ratio_b = inv_mass_b / total_inv_mass;

                particles[constraint.particle_a].position += correction * ratio_a;
                particles[constraint.particle_b].position -= correction * ratio_b;
            }
        }
    }

    /// Move a pinned particle to a new position.
    pub fn set_particle_position(&mut self, index: usize, position: Point3<f32>) {
        if index < self.particles.len() {
            self.particles[index].position = position;
            if self.particles[index].is_pinned() {
                self.particles[index].prev_position = position;
            }
        }
    }

    /// Get a particle's position.
    pub fn get_particle_position(&self, index: usize) -> Option<Point3<f32>> {
        self.particles.get(index).map(|p| p.position)
    }

    /// Apply an external impulse to a particle.
    pub fn apply_impulse(&mut self, index: usize, impulse: Vector3<f32>) {
        if let Some(particle) = self.particles.get_mut(index) {
            particle.apply_impulse(impulse);
        }
    }

    /// Apply an external force to a particle.
    pub fn apply_force(&mut self, index: usize, force: Vector3<f32>) {
        if let Some(particle) = self.particles.get_mut(index) {
            particle.apply_force(force);
        }
    }

    /// Pin a particle in place.
    pub fn pin(&mut self, index: usize) {
        if let Some(particle) = self.particles.get_mut(index) {
            particle.inv_mass = 0.0;
        }
    }

    /// Unpin a particle.
    pub fn unpin(&mut self, index: usize) {
        if let Some(particle) = self.particles.get_mut(index) {
            particle.inv_mass = 1.0;
        }
    }
}

impl Default for VerletSystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A pendulum swung at 144 Hz is where one swung at 60 Hz is after the
    /// same time, and at a tenth of the step it covers a tenth of the swing.
    #[test]
    fn the_swing_follows_time_not_the_frame_rate() {
        fn swing(step: f32, seconds: f32) -> Point3<f32> {
            let mut system = VerletSystem::new();
            let hinge = system.add_particle(Particle::pinned(Point3::new(0.0, 2.0, 0.0)));
            let bob = system.add_particle(Particle::new(Point3::new(1.0, 2.0, 0.0)));
            system.add_distance_constraint(DistanceConstraint::new(hinge, bob, 1.0));
            for _ in 0..(seconds / step).round() as usize {
                system.update(step);
            }
            system.particles[bob].position
        }

        let at_60 = swing(1.0 / 60.0, 0.5);
        let at_144 = swing(1.0 / 144.0, 0.5);
        assert!(
            (at_60 - at_144).norm() < 0.05,
            "60 Hz {at_60}, 144 Hz {at_144}"
        );

        let mut slowed = VerletSystem::new();
        let hinge = slowed.add_particle(Particle::pinned(Point3::new(0.0, 2.0, 0.0)));
        let bob = slowed.add_particle(Particle::new(Point3::new(1.0, 2.0, 0.0)));
        slowed.add_distance_constraint(DistanceConstraint::new(hinge, bob, 1.0));
        for _ in 0..15 {
            slowed.update(1.0 / 60.0);
        }
        // F9 at x0.1: the same frames now step a tenth of the time.
        for _ in 0..150 {
            slowed.update(1.0 / 600.0);
        }
        let real_time = swing(1.0 / 60.0, 0.5);
        assert!(
            (slowed.particles[bob].position - real_time).norm() < 0.05,
            "slowed {}, real time {real_time}",
            slowed.particles[bob].position
        );
    }

    #[test]
    fn test_basic_simulation() {
        let mut system = VerletSystem::new();

        // Create a simple two-particle system
        let p0 = system.add_particle(Particle::pinned(Point3::new(0.0, 2.0, 0.0)));
        let p1 = system.add_particle(Particle::new(Point3::new(0.0, 1.0, 0.0)));

        system.connect(p0, p1);

        // Run a few updates
        for _ in 0..10 {
            system.update(1.0 / 60.0);
        }

        // The pinned particle should not have moved
        assert_eq!(system.particles[p0].position, Point3::new(0.0, 2.0, 0.0));

        // The free particle should have moved down due to gravity
        // but constraint should keep it at distance 1.0 from pinned
        let dist = (system.particles[p1].position - system.particles[p0].position).magnitude();
        assert!(
            (dist - 1.0).abs() < 0.1,
            "Distance should be approximately 1.0"
        );
    }

    #[test]
    fn test_constraint_solving() {
        let mut system = VerletSystem::new();
        system.gravity = Vector3::zeros(); // No gravity for this test

        let p0 = system.add_particle(Particle::pinned(Point3::new(0.0, 0.0, 0.0)));
        let p1 = system.add_particle(Particle::new(Point3::new(2.0, 0.0, 0.0))); // Start too far

        system.add_distance_constraint(DistanceConstraint::new(p0, p1, 1.0));

        // Run constraint solving
        for _ in 0..20 {
            system.update(1.0 / 60.0);
        }

        // Should converge to rest length
        let dist = (system.particles[p1].position - system.particles[p0].position).magnitude();
        assert!(
            (dist - 1.0).abs() < 0.01,
            "Distance should converge to 1.0, got {}",
            dist
        );
    }
}
