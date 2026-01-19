//! Pogo stick skeleton - the simplest possible Verlet skeleton.
//!
//! This is Stage 1 of the iterative skeleton build-up:
//! - 2 particles: Foot (root) and Head (top)
//! - 1 distance constraint: The "pole"
//! - No IK, no locomotion controller
//!
//! Purpose: Validate Verlet integration and constraint solving work correctly
//! before adding more complexity.

use nalgebra::{Point3, Vector3};

use super::verlet::{DistanceConstraint, Particle, VerletSystem};

/// Configuration for the pogo stick skeleton.
#[derive(Clone, Debug)]
pub struct PogoConfig {
    /// Total height of the pogo stick (foot to head).
    pub height: f32,
    /// Damping for the head particle (0-1, higher = more damping).
    pub head_damping: f32,
    /// Stiffness of the pole constraint (0-1, 1 = rigid).
    pub pole_stiffness: f32,
}

impl Default for PogoConfig {
    fn default() -> Self {
        Self {
            height: 1.5,
            head_damping: 0.05,
            pole_stiffness: 1.0,
        }
    }
}

/// Joint indices for the pogo stick.
#[derive(Clone, Debug)]
pub struct PogoJoints {
    /// Foot (root) - pinned when grounded.
    pub foot: usize,
    /// Head (top) - free to move.
    pub head: usize,
}

/// A simple pogo stick skeleton with 2 particles and 1 constraint.
pub struct PogoStickSkeleton {
    /// The Verlet physics system.
    pub verlet: VerletSystem,
    /// Joint indices.
    pub joints: PogoJoints,
    /// Configuration.
    pub config: PogoConfig,
}

impl PogoStickSkeleton {
    /// Create a new pogo stick skeleton at the origin.
    pub fn new(config: PogoConfig) -> Self {
        let mut verlet = VerletSystem::new();

        // Disable gravity on the verlet system - we want the character's
        // physics body to handle gravity, not the skeleton
        verlet.gravity = Vector3::zeros();

        // Create particles
        // Foot starts at origin, pinned (infinite mass)
        let foot = verlet.add_particle(Particle::pinned(Point3::new(0.0, 0.0, 0.0)));

        // Head is above foot
        let mut head_particle = Particle::new(Point3::new(0.0, config.height, 0.0));
        head_particle.damping = config.head_damping;
        let head = verlet.add_particle(head_particle);

        // Connect with distance constraint (the "pole")
        verlet.add_distance_constraint(
            DistanceConstraint::new(foot, head, config.height).with_stiffness(config.pole_stiffness),
        );

        let joints = PogoJoints { foot, head };

        Self {
            verlet,
            joints,
            config,
        }
    }

    /// Update the physics simulation.
    pub fn update(&mut self, dt: f32) {
        self.verlet.update(dt);
    }

    /// Set the root (foot) position.
    ///
    /// The foot is the anchor point that follows the physics body.
    pub fn set_root_position(&mut self, position: Point3<f32>) {
        self.verlet.set_particle_position(self.joints.foot, position);
    }

    /// Get a joint's world position.
    pub fn joint_position(&self, joint: usize) -> Point3<f32> {
        self.verlet
            .get_particle_position(joint)
            .unwrap_or(Point3::origin())
    }

    /// Get the foot position.
    pub fn foot_position(&self) -> Point3<f32> {
        self.joint_position(self.joints.foot)
    }

    /// Get the head position.
    pub fn head_position(&self) -> Point3<f32> {
        self.joint_position(self.joints.head)
    }

    /// Apply an impulse to the head (for bouncing/landing effects).
    pub fn apply_head_impulse(&mut self, impulse: Vector3<f32>) {
        self.verlet.apply_impulse(self.joints.head, impulse);
    }

    /// Get the direction from foot to head (the "up" direction of the pogo).
    pub fn up_direction(&self) -> Vector3<f32> {
        let foot = self.foot_position();
        let head = self.head_position();
        (head - foot).normalize()
    }

    /// Pin the foot (when grounded).
    pub fn pin_foot(&mut self) {
        self.verlet.pin(self.joints.foot);
    }

    /// Unpin the foot (when airborne).
    pub fn unpin_foot(&mut self) {
        self.verlet.unpin(self.joints.foot);
    }

    /// Check if foot is pinned.
    pub fn is_foot_pinned(&self) -> bool {
        self.verlet.particles[self.joints.foot].is_pinned()
    }
}

impl Default for PogoStickSkeleton {
    fn default() -> Self {
        Self::new(PogoConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pogo_creation() {
        let pogo = PogoStickSkeleton::default();

        // Check initial positions
        let foot = pogo.foot_position();
        let head = pogo.head_position();

        assert!((foot.y - 0.0).abs() < 0.001);
        assert!((head.y - 1.5).abs() < 0.001);

        // Check constraint maintains distance
        let dist = (head - foot).magnitude();
        assert!((dist - 1.5).abs() < 0.001);
    }

    #[test]
    fn test_pogo_constraint_maintained() {
        let mut pogo = PogoStickSkeleton::default();

        // Move root position
        pogo.set_root_position(Point3::new(5.0, 10.0, 3.0));

        // Run physics for several frames
        for _ in 0..60 {
            pogo.update(1.0 / 60.0);
        }

        // Distance should still be maintained
        let foot = pogo.foot_position();
        let head = pogo.head_position();
        let dist = (head - foot).magnitude();

        assert!(
            (dist - 1.5).abs() < 0.05,
            "Distance should be ~1.5, got {}",
            dist
        );
    }

    #[test]
    fn test_pogo_impulse() {
        let mut pogo = PogoStickSkeleton::default();

        // Apply sideways impulse to head
        pogo.apply_head_impulse(Vector3::new(0.5, 0.0, 0.0));

        // Run physics
        for _ in 0..10 {
            pogo.update(1.0 / 60.0);
        }

        // Head should have moved sideways but constraint maintains distance
        let foot = pogo.foot_position();
        let head = pogo.head_position();
        let dist = (head - foot).magnitude();

        assert!(
            (dist - 1.5).abs() < 0.05,
            "Distance should be ~1.5, got {}",
            dist
        );
        // Head x position should have changed
        assert!(head.x > 0.01, "Head should have moved in x direction");
    }
}
