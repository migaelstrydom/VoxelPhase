//! Humanoid skeleton definition and builder.
//!
//! Defines the bone structure for a goofy platformer character with:
//! - Oversized head for expressiveness
//! - Long, flexible arms for grabbing and swinging
//! - Strong legs for jumping and landing
//! - Compact torso for agility

use nalgebra::{Point3, Vector3};

use super::fabrik::IKChain;
use super::verlet::{AngleConstraint, Particle, VerletSystem};

/// Named bone/joint indices for the humanoid skeleton.
///
/// These map to particle indices in the Verlet system.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HumanoidJoints {
    // Core (torso)
    pub hips: usize,
    pub spine: usize,
    pub chest: usize,

    // Head
    pub neck: usize,
    pub head: usize,

    // Left arm
    pub left_shoulder: usize,
    pub left_elbow: usize,
    pub left_hand: usize,

    // Right arm
    pub right_shoulder: usize,
    pub right_elbow: usize,
    pub right_hand: usize,

    // Left leg
    pub left_hip: usize,
    pub left_knee: usize,
    pub left_foot: usize,

    // Right leg
    pub right_hip: usize,
    pub right_knee: usize,
    pub right_foot: usize,
}

/// Configuration for building a humanoid skeleton.
#[derive(Clone, Debug)]
pub struct HumanoidConfig {
    // Overall scale
    pub height: f32,

    // Proportions (as fractions of height)
    pub head_size: f32,      // Oversized for cartoon look
    pub torso_length: f32,   // Compact torso
    pub arm_length: f32,     // Long arms for grabbing
    pub leg_length: f32,     // Strong legs
    pub shoulder_width: f32, // How wide the shoulders are
    pub hip_width: f32,      // How wide the hips are

    // Limb segment ratios
    pub upper_arm_ratio: f32, // Upper vs lower arm
    pub forearm_ratio: f32,
    pub thigh_ratio: f32, // Upper vs lower leg
    pub shin_ratio: f32,
}

impl Default for HumanoidConfig {
    fn default() -> Self {
        Self {
            height: 1.0,

            // Goofy platformer proportions
            head_size: 0.25,    // Big head!
            torso_length: 0.25, // Short torso
            arm_length: 0.35,   // Long arms for grabbing
            leg_length: 0.40,   // Good legs for jumping
            shoulder_width: 0.20,
            hip_width: 0.12,

            // Segment ratios
            upper_arm_ratio: 0.45,
            forearm_ratio: 0.55,
            thigh_ratio: 0.55,
            shin_ratio: 0.45,
        }
    }
}

/// A complete humanoid skeleton with physics and IK chains.
pub struct HumanoidSkeleton {
    /// The Verlet physics system containing all joints.
    pub verlet: VerletSystem,
    /// Named joint indices.
    pub joints: HumanoidJoints,
    /// IK chain for left arm.
    pub left_arm_chain: IKChain,
    /// IK chain for right arm.
    pub right_arm_chain: IKChain,
    /// IK chain for left leg.
    pub left_leg_chain: IKChain,
    /// IK chain for right leg.
    pub right_leg_chain: IKChain,
    /// IK chain for spine (hips to head).
    pub spine_chain: IKChain,
    /// Configuration used to build this skeleton.
    pub config: HumanoidConfig,
}

impl HumanoidSkeleton {
    /// Build a new humanoid skeleton from configuration.
    pub fn new(config: HumanoidConfig) -> Self {
        let mut verlet = VerletSystem::new();

        // Calculate actual lengths from proportions
        let h = config.height;
        let head_radius = config.head_size * h * 0.5;
        let torso_len = config.torso_length * h;
        let arm_len = config.arm_length * h;
        let leg_len = config.leg_length * h;
        let shoulder_w = config.shoulder_width * h;
        let hip_w = config.hip_width * h;

        // Calculate positions relative to hips (our root)
        // Hips are at origin, everything else relative to that

        // Core positions
        let hips_pos = Point3::new(0.0, 0.0, 0.0);
        let spine_pos = Point3::new(0.0, torso_len * 0.4, 0.0);
        let chest_pos = Point3::new(0.0, torso_len * 0.8, 0.0);
        let neck_pos = Point3::new(0.0, torso_len, 0.0);
        let head_pos = Point3::new(0.0, torso_len + head_radius * 1.5, 0.0);

        // Arm positions (from shoulders)
        let upper_arm_len = arm_len * config.upper_arm_ratio;
        let forearm_len = arm_len * config.forearm_ratio;

        let left_shoulder_pos = Point3::new(-shoulder_w, torso_len * 0.9, 0.0);
        let left_elbow_pos = Point3::new(-shoulder_w - upper_arm_len * 0.7, torso_len * 0.6, 0.0);
        let left_hand_pos = Point3::new(
            -shoulder_w - upper_arm_len * 0.7 - forearm_len * 0.7,
            torso_len * 0.3,
            0.0,
        );

        let right_shoulder_pos = Point3::new(shoulder_w, torso_len * 0.9, 0.0);
        let right_elbow_pos = Point3::new(shoulder_w + upper_arm_len * 0.7, torso_len * 0.6, 0.0);
        let right_hand_pos = Point3::new(
            shoulder_w + upper_arm_len * 0.7 + forearm_len * 0.7,
            torso_len * 0.3,
            0.0,
        );

        // Leg positions (from hips)
        let thigh_len = leg_len * config.thigh_ratio;
        let shin_len = leg_len * config.shin_ratio;

        let left_hip_pos = Point3::new(-hip_w, 0.0, 0.0);
        let left_knee_pos = Point3::new(-hip_w * 0.8, -thigh_len, 0.0);
        let left_foot_pos = Point3::new(-hip_w * 0.6, -thigh_len - shin_len, 0.0);

        let right_hip_pos = Point3::new(hip_w, 0.0, 0.0);
        let right_knee_pos = Point3::new(hip_w * 0.8, -thigh_len, 0.0);
        let right_foot_pos = Point3::new(hip_w * 0.6, -thigh_len - shin_len, 0.0);

        // Add all particles
        // Hips is pinned (controlled by player position)
        let hips = verlet.add_particle(Particle::pinned(hips_pos));
        let spine = verlet.add_particle(Particle::new(spine_pos));
        let chest = verlet.add_particle(Particle::new(chest_pos));
        let neck = verlet.add_particle(Particle::new(neck_pos));
        let head = verlet.add_particle(Particle::new(head_pos));

        let left_shoulder = verlet.add_particle(Particle::new(left_shoulder_pos));
        let left_elbow = verlet.add_particle(Particle::new(left_elbow_pos));
        let left_hand = verlet.add_particle(Particle::new(left_hand_pos));

        let right_shoulder = verlet.add_particle(Particle::new(right_shoulder_pos));
        let right_elbow = verlet.add_particle(Particle::new(right_elbow_pos));
        let right_hand = verlet.add_particle(Particle::new(right_hand_pos));

        let left_hip = verlet.add_particle(Particle::new(left_hip_pos));
        let left_knee = verlet.add_particle(Particle::new(left_knee_pos));
        let left_foot = verlet.add_particle(Particle::new(left_foot_pos));

        let right_hip = verlet.add_particle(Particle::new(right_hip_pos));
        let right_knee = verlet.add_particle(Particle::new(right_knee_pos));
        let right_foot = verlet.add_particle(Particle::new(right_foot_pos));

        // Connect bones (distance constraints)
        // Spine
        verlet.connect(hips, spine);
        verlet.connect(spine, chest);
        verlet.connect(chest, neck);
        verlet.connect(neck, head);

        // Left arm
        verlet.connect(chest, left_shoulder);
        verlet.connect(left_shoulder, left_elbow);
        verlet.connect(left_elbow, left_hand);

        // Right arm
        verlet.connect(chest, right_shoulder);
        verlet.connect(right_shoulder, right_elbow);
        verlet.connect(right_elbow, right_hand);

        // Left leg
        verlet.connect(hips, left_hip);
        verlet.connect(left_hip, left_knee);
        verlet.connect(left_knee, left_foot);

        // Right leg
        verlet.connect(hips, right_hip);
        verlet.connect(right_hip, right_knee);
        verlet.connect(right_knee, right_foot);

        // Structural constraints (keep torso stable)
        verlet.connect(left_shoulder, right_shoulder); // Shoulder bar
        verlet.connect(left_hip, right_hip); // Hip bar
        verlet.connect(left_shoulder, hips); // Cross bracing
        verlet.connect(right_shoulder, hips);

        // Add angle constraints for natural joint limits
        // Elbow constraints (can't bend backward)
        verlet.add_angle_constraint(
            AngleConstraint::new(left_elbow, left_shoulder, left_hand)
                .with_limits(0.3, std::f32::consts::PI - 0.1),
        );
        verlet.add_angle_constraint(
            AngleConstraint::new(right_elbow, right_shoulder, right_hand)
                .with_limits(0.3, std::f32::consts::PI - 0.1),
        );

        // Knee constraints (can't bend forward)
        verlet.add_angle_constraint(
            AngleConstraint::new(left_knee, left_hip, left_foot)
                .with_limits(0.3, std::f32::consts::PI - 0.1),
        );
        verlet.add_angle_constraint(
            AngleConstraint::new(right_knee, right_hip, right_foot)
                .with_limits(0.3, std::f32::consts::PI - 0.1),
        );

        // Reduce gravity effect on upper body
        verlet.particles[spine].damping = 0.1;
        verlet.particles[chest].damping = 0.1;
        verlet.particles[neck].damping = 0.15;
        verlet.particles[head].damping = 0.2;
        verlet.particles[left_shoulder].damping = 0.1;
        verlet.particles[right_shoulder].damping = 0.1;

        // Build IK chains
        let positions: Vec<Point3<f32>> = verlet.particles.iter().map(|p| p.position).collect();

        let left_arm_chain = IKChain::new(
            vec![chest, left_shoulder, left_elbow, left_hand],
            &positions,
        );
        let right_arm_chain = IKChain::new(
            vec![chest, right_shoulder, right_elbow, right_hand],
            &positions,
        );
        let left_leg_chain = IKChain::new(vec![hips, left_hip, left_knee, left_foot], &positions);
        let right_leg_chain =
            IKChain::new(vec![hips, right_hip, right_knee, right_foot], &positions);
        let spine_chain = IKChain::new(vec![hips, spine, chest, neck, head], &positions);

        let joints = HumanoidJoints {
            hips,
            spine,
            chest,
            neck,
            head,
            left_shoulder,
            left_elbow,
            left_hand,
            right_shoulder,
            right_elbow,
            right_hand,
            left_hip,
            left_knee,
            left_foot,
            right_hip,
            right_knee,
            right_foot,
        };

        Self {
            verlet,
            joints,
            left_arm_chain,
            right_arm_chain,
            left_leg_chain,
            right_leg_chain,
            spine_chain,
            config,
        }
    }

    /// Update the skeleton physics.
    pub fn update(&mut self, dt: f32) {
        self.verlet.update(dt);
    }

    /// Set the world position of the hips (root).
    pub fn set_root_position(&mut self, position: Point3<f32>) {
        self.verlet
            .set_particle_position(self.joints.hips, position);
    }

    /// Get the world position of the hips (root).
    pub fn root_position(&self) -> Point3<f32> {
        self.verlet.particles[self.joints.hips].position
    }

    /// Get a joint's world position.
    pub fn joint_position(&self, joint: usize) -> Point3<f32> {
        self.verlet.particles[joint].position
    }

    /// Get all joint positions as a vector.
    pub fn all_positions(&self) -> Vec<Point3<f32>> {
        self.verlet.particles.iter().map(|p| p.position).collect()
    }

    /// Apply an impulse to a joint (e.g., from collision).
    pub fn apply_impulse(&mut self, joint: usize, impulse: Vector3<f32>) {
        self.verlet.apply_impulse(joint, impulse);
    }

    /// Apply an impulse to the whole body.
    pub fn apply_body_impulse(&mut self, impulse: Vector3<f32>) {
        for i in 0..self.verlet.particles.len() {
            self.verlet.apply_impulse(i, impulse);
        }
    }

    /// Get the center of mass.
    pub fn center_of_mass(&self) -> Point3<f32> {
        let mut total = Vector3::zeros();
        let mut count = 0;

        for particle in &self.verlet.particles {
            if !particle.is_pinned() {
                total += particle.position.coords;
                count += 1;
            }
        }

        if count > 0 {
            Point3::from(total / count as f32)
        } else {
            self.root_position()
        }
    }

    /// Calculate the facing direction based on shoulder orientation.
    pub fn facing_direction(&self) -> Vector3<f32> {
        let left = self.joint_position(self.joints.left_shoulder);
        let right = self.joint_position(self.joints.right_shoulder);
        let chest = self.joint_position(self.joints.chest);

        // Cross product of shoulder vector and up gives forward
        let shoulder_vec = (right - left).normalize();
        let _to_chest = (chest.coords - (left.coords + right.coords) * 0.5).normalize();

        shoulder_vec.cross(&Vector3::y()).normalize()
    }
}

impl Default for HumanoidSkeleton {
    fn default() -> Self {
        Self::new(HumanoidConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_skeleton_creation() {
        let skeleton = HumanoidSkeleton::default();

        // Should have 17 particles (joints)
        assert_eq!(skeleton.verlet.particles.len(), 17);

        // Hips should be at origin
        assert_eq!(
            skeleton.joint_position(skeleton.joints.hips),
            Point3::new(0.0, 0.0, 0.0)
        );

        // Head should be above hips
        assert!(skeleton.joint_position(skeleton.joints.head).y > 0.0);

        // Feet should be below hips
        assert!(skeleton.joint_position(skeleton.joints.left_foot).y < 0.0);
        assert!(skeleton.joint_position(skeleton.joints.right_foot).y < 0.0);
    }

    #[test]
    fn test_skeleton_physics() {
        let mut skeleton = HumanoidSkeleton::default();

        // Move root
        skeleton.set_root_position(Point3::new(5.0, 10.0, 0.0));

        // Update physics
        for _ in 0..60 {
            skeleton.update(1.0 / 60.0);
        }

        // Body should follow (approximately, due to physics)
        assert!(skeleton.joint_position(skeleton.joints.head).x > 0.0);
    }
}
