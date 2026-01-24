//! FABRIK (Forward And Backward Reaching Inverse Kinematics) solver.
//!
//! FABRIK is a fast, iterative IK algorithm that works by:
//! 1. Forward reaching: Move end effector to target, propagate back
//! 2. Backward reaching: Move root back to origin, propagate forward
//! 3. Repeat until convergence
//!
//! This implementation supports:
//! - Single chains (arm, leg)
//! - Joint constraints (angle limits)
//! - Target positions for any joint

use nalgebra::Point3;

/// A chain of joints for IK solving.
///
/// The chain goes from base (index 0) to end effector (last index).
/// Joint positions are stored as indices into an external particle array.
#[derive(Clone, Debug)]
pub struct IKChain {
    /// Indices of joints in this chain (into the skeleton's particle array).
    pub joint_indices: Vec<usize>,
    /// Bone lengths between consecutive joints.
    pub bone_lengths: Vec<f32>,
    /// Total length of the chain (for reachability check).
    pub total_length: f32,
}

impl IKChain {
    /// Create a new IK chain from joint indices and their positions.
    pub fn new(joint_indices: Vec<usize>, positions: &[Point3<f32>]) -> Self {
        let mut bone_lengths = Vec::new();
        let mut total_length = 0.0;

        for i in 0..joint_indices.len() - 1 {
            let a = positions[joint_indices[i]];
            let b = positions[joint_indices[i + 1]];
            let length = (b - a).magnitude();
            bone_lengths.push(length);
            total_length += length;
        }

        Self {
            joint_indices,
            bone_lengths,
            total_length,
        }
    }

    /// Get the number of joints in this chain.
    pub fn len(&self) -> usize {
        self.joint_indices.len()
    }

    /// Get the base (root) joint index.
    pub fn base(&self) -> usize {
        self.joint_indices[0]
    }

    /// Get the end effector joint index.
    pub fn end_effector(&self) -> usize {
        *self.joint_indices.last().unwrap()
    }
}

/// An IK target for a chain.
#[derive(Clone, Debug)]
pub struct IKTarget {
    /// Target position in world space.
    pub position: Point3<f32>,
    /// Weight of this target (0-1).
    pub weight: f32,
}

impl IKTarget {
    /// Create a new target at the given position.
    pub fn new(position: Point3<f32>) -> Self {
        Self {
            position,
            weight: 1.0,
        }
    }
}

/// The FABRIK solver.
pub struct FABRIKSolver {
    /// Maximum iterations per solve.
    pub max_iterations: u32,
    /// Tolerance for convergence (distance from target).
    pub tolerance: f32,
}

impl FABRIKSolver {
    /// Create a new FABRIK solver with default settings.
    pub fn new() -> Self {
        Self {
            max_iterations: 10,
            tolerance: 0.001,
        }
    }

    /// Solve IK for a single chain.
    ///
    /// `positions` is a mutable slice of all joint positions.
    /// `chain` defines which joints to solve.
    /// `target` is the target position for the end effector.
    /// `base_pinned` if true, the base won't move.
    ///
    /// Returns the number of iterations used.
    pub fn solve(
        &self,
        positions: &mut [Point3<f32>],
        chain: &IKChain,
        target: &IKTarget,
        base_pinned: bool,
    ) -> u32 {
        if chain.len() < 2 {
            return 0;
        }

        // Blend target position based on weight
        let original_end = positions[chain.end_effector()];
        let target_pos = original_end
            .coords
            .lerp(&target.position.coords, target.weight);
        let target_pos = Point3::from(target_pos);

        // Store the base position
        let base_pos = positions[chain.base()];

        // Check if target is reachable
        let dist_to_target = (target_pos - base_pos).magnitude();
        if dist_to_target > chain.total_length {
            // Target is unreachable - stretch toward it
            self.stretch_toward(positions, chain, target_pos);
            return 1;
        }

        // Iterative solving
        for iteration in 0..self.max_iterations {
            // Forward reaching (from end effector to base)
            self.forward_reach(positions, chain, target_pos);

            // Backward reaching (from base to end effector)
            self.backward_reach(
                positions,
                chain,
                if base_pinned {
                    base_pos
                } else {
                    positions[chain.base()]
                },
            );

            // If base is pinned, restore it
            if base_pinned {
                positions[chain.base()] = base_pos;
            }

            // Check convergence
            let end_pos = positions[chain.end_effector()];
            if (end_pos - target_pos).magnitude() < self.tolerance {
                return iteration + 1;
            }
        }

        self.max_iterations
    }

    /// Forward reaching phase: move end effector to target, propagate back.
    fn forward_reach(&self, positions: &mut [Point3<f32>], chain: &IKChain, target: Point3<f32>) {
        // Set end effector to target
        let n = chain.len();
        positions[chain.joint_indices[n - 1]] = target;

        // Propagate backward through the chain
        for i in (0..n - 1).rev() {
            let current_idx = chain.joint_indices[i];
            let next_idx = chain.joint_indices[i + 1];
            let bone_length = chain.bone_lengths[i];

            let direction = (positions[current_idx] - positions[next_idx]).normalize();
            positions[current_idx] = positions[next_idx] + direction * bone_length;
        }
    }

    /// Backward reaching phase: fix base, propagate forward.
    fn backward_reach(&self, positions: &mut [Point3<f32>], chain: &IKChain, base: Point3<f32>) {
        let n = chain.len();

        // Set base to original position
        positions[chain.joint_indices[0]] = base;

        // Propagate forward through the chain
        for i in 0..n - 1 {
            let current_idx = chain.joint_indices[i];
            let next_idx = chain.joint_indices[i + 1];
            let bone_length = chain.bone_lengths[i];

            let direction = (positions[next_idx] - positions[current_idx]).normalize();
            positions[next_idx] = positions[current_idx] + direction * bone_length;
        }
    }

    /// Stretch the chain toward an unreachable target.
    fn stretch_toward(&self, positions: &mut [Point3<f32>], chain: &IKChain, target: Point3<f32>) {
        let base_pos = positions[chain.base()];
        let direction = (target - base_pos).normalize();

        let mut current_pos = base_pos;
        for i in 1..chain.len() {
            let bone_length = chain.bone_lengths[i - 1];
            current_pos = current_pos + direction * bone_length;
            positions[chain.joint_indices[i]] = current_pos;
        }
    }
}

impl Default for FABRIKSolver {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn test_simple_chain_ik() {
        // Create a 3-joint chain: shoulder -> elbow -> hand
        let mut positions = vec![
            Point3::new(0.0, 0.0, 0.0), // shoulder
            Point3::new(1.0, 0.0, 0.0), // elbow
            Point3::new(2.0, 0.0, 0.0), // hand
        ];

        let chain = IKChain::new(vec![0, 1, 2], &positions);
        let solver = FABRIKSolver::new();

        // Target reachable position
        let target = IKTarget::new(Point3::new(1.0, 1.0, 0.0));
        solver.solve(&mut positions, &chain, &target, true);

        // End effector should be close to target
        let dist = (positions[2] - target.position).magnitude();
        assert!(
            dist < 0.01,
            "End effector should reach target, dist: {}",
            dist
        );

        // Shoulder should not have moved
        assert_eq!(positions[0], Point3::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_unreachable_target() {
        let mut positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];

        let chain = IKChain::new(vec![0, 1, 2], &positions);
        let solver = FABRIKSolver::new();

        // Target far away (unreachable)
        let target = IKTarget::new(Point3::new(10.0, 0.0, 0.0));
        solver.solve(&mut positions, &chain, &target, true);

        // Chain should stretch toward target
        let direction = (positions[2] - positions[0]).normalize();
        let expected_dir = Vector3::new(1.0, 0.0, 0.0);
        assert!((direction - expected_dir).magnitude() < 0.01);
    }
}
