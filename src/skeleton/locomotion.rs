//! Procedural locomotion controller.
//!
//! Handles procedural walking, foot placement, and body dynamics.
//! The locomotion system:
//! 1. Tracks ideal foot positions based on body movement
//! 2. Triggers step cycles when feet drift too far
//! 3. Uses IK to place feet on terrain
//! 4. Animates the body based on movement state

use nalgebra::{Point3, Vector3};

use super::fabrik::{FABRIKSolver, IKTarget};
use super::humanoid::HumanoidSkeleton;

/// State of a single foot in the step cycle.
#[derive(Clone, Debug)]
pub struct FootState {
    /// Current planted position (where the foot is on ground).
    pub planted_position: Point3<f32>,
    /// Target position for the current step (where we're stepping to).
    pub target_position: Point3<f32>,
    /// Is this foot currently in a step (airborne)?
    pub is_stepping: bool,
    /// Progress through the current step [0, 1].
    pub step_progress: f32,
    /// Height of the step arc.
    pub step_height: f32,
    /// Time since last step.
    pub time_since_step: f32,
}

impl FootState {
    /// Create a new foot state at the given position.
    pub fn new(position: Point3<f32>) -> Self {
        Self {
            planted_position: position,
            target_position: position,
            is_stepping: false,
            step_progress: 0.0,
            step_height: 0.1,
            time_since_step: 0.0,
        }
    }

    /// Get the current foot position (accounting for step animation).
    pub fn current_position(&self) -> Point3<f32> {
        if !self.is_stepping {
            self.planted_position
        } else {
            // Interpolate along step arc
            let t = self.step_progress;

            // Smooth step curve (ease in/out)
            let smooth_t = t * t * (3.0 - 2.0 * t);

            // Horizontal interpolation
            let horizontal = self
                .planted_position
                .coords
                .lerp(&self.target_position.coords, smooth_t);

            // Vertical arc (parabola)
            let arc_height = 4.0 * t * (1.0 - t) * self.step_height;

            Point3::from(horizontal) + Vector3::new(0.0, arc_height, 0.0)
        }
    }
}

/// Configuration for the locomotion controller.
#[derive(Clone, Debug)]
pub struct LocomotionConfig {
    /// Maximum distance a foot can drift from ideal before stepping (relative to height).
    pub step_threshold: f32,
    /// Speed of step animation.
    pub step_speed: f32,
    /// Height of step arc (relative to height).
    pub step_height: f32,
    /// How far ahead to place the foot when stepping (relative to height).
    pub step_overshoot: f32,
    /// Minimum time between steps for each foot.
    pub step_cooldown: f32,
    /// Base stride length (distance between ideal foot positions).
    pub stride_length: f32,
    /// Lateral spacing between feet/hips (relative to height).
    pub step_width: f32,
    /// Body bob amplitude when walking.
    pub body_bob_amount: f32,
    /// Body lean amount when accelerating.
    pub body_lean_amount: f32,
    /// Lateral hip sway amount (relative to height).
    pub hip_sway_amount: f32,
    /// Hip twist amount along the movement axis (relative to height).
    pub hip_twist_amount: f32,
    /// Shoulder counter-twist amount (relative to height).
    pub shoulder_twist_amount: f32,
    /// Arm lift amount during swing (relative to height).
    pub arm_lift_amount: f32,
    /// Arm swing amount when walking.
    pub arm_swing_amount: f32,
    /// Speed at which gait reaches full amplitude.
    pub gait_speed: f32,
    /// Blend toward pelvis position derived from planted feet (0-1).
    pub pelvis_anchor_strength: f32,
    /// Smoothing factor for contact-anchored pelvis (0-1).
    pub pelvis_anchor_smoothing: f32,
}

impl Default for LocomotionConfig {
    fn default() -> Self {
        Self {
            step_threshold: 0.18,
            step_speed: 20.0,
            step_height: 0.11,
            step_overshoot: 0.14,
            step_cooldown: 0.18,
            stride_length: 1.0,
            step_width: 0.16,
            body_bob_amount: 0.02,
            body_lean_amount: 0.05,
            hip_sway_amount: 0.015,
            hip_twist_amount: 0.02,
            shoulder_twist_amount: 0.015,
            arm_lift_amount: 0.015,
            arm_swing_amount: 0.1,
            gait_speed: 5.0,
            pelvis_anchor_strength: 0.5,
            pelvis_anchor_smoothing: 0.25,
        }
    }
}

/// The locomotion controller.
pub struct LocomotionController {
    /// Configuration.
    pub config: LocomotionConfig,
    /// Left foot state.
    pub left_foot: FootState,
    /// Right foot state.
    pub right_foot: FootState,
    /// Current body velocity (for leaning).
    pub velocity: Vector3<f32>,
    /// Previous body position (for velocity calculation).
    prev_position: Point3<f32>,
    /// Walking cycle phase [0, 1).
    pub walk_phase: f32,
    /// Is currently grounded?
    pub is_grounded: bool,
    /// IK solver.
    ik_solver: FABRIKSolver,
    /// Cached rest offsets for hands in local body space.
    left_hand_rest_offset: Vector3<f32>,
    right_hand_rest_offset: Vector3<f32>,
    /// Last known movement direction for stable idle pose.
    last_move_dir: Vector3<f32>,
    /// Smoothed pelvis anchor position when grounded.
    last_contact_root: Option<Point3<f32>>,
}

impl LocomotionController {
    /// Create a new locomotion controller.
    pub fn new(config: LocomotionConfig, skeleton: &HumanoidSkeleton) -> Self {
        let left_pos = skeleton.joint_position(skeleton.joints.left_foot);
        let right_pos = skeleton.joint_position(skeleton.joints.right_foot);
        let left_hand = skeleton.joint_position(skeleton.joints.left_hand);
        let right_hand = skeleton.joint_position(skeleton.joints.right_hand);
        let left_shoulder = skeleton.joint_position(skeleton.joints.left_shoulder);
        let right_shoulder = skeleton.joint_position(skeleton.joints.right_shoulder);

        Self {
            config,
            left_foot: FootState::new(left_pos),
            right_foot: FootState::new(right_pos),
            velocity: Vector3::zeros(),
            prev_position: skeleton.root_position(),
            walk_phase: 0.0,
            is_grounded: true,
            ik_solver: FABRIKSolver::new(),
            left_hand_rest_offset: left_hand - left_shoulder,
            right_hand_rest_offset: right_hand - right_shoulder,
            last_move_dir: Vector3::new(0.0, 0.0, 1.0),
            last_contact_root: None,
        }
    }

    /// Update the locomotion system.
    ///
    /// `skeleton` - The humanoid skeleton to animate.
    /// `root_position` - Current root (hip) position from physics.
    /// `ground_height` - Height of the ground at current position.
    /// `dt` - Delta time.
    pub fn update(
        &mut self,
        skeleton: &mut HumanoidSkeleton,
        root_position: Point3<f32>,
        ground_height: f32,
        dt: f32,
    ) {
        let root_delta = root_position - self.prev_position;
        let height = skeleton.config.height;

        // If the root teleports (spawn/warp), keep feet relative to hips
        if root_delta.magnitude() > height * 0.5 {
            self.left_foot.planted_position += root_delta;
            self.left_foot.target_position += root_delta;
            self.left_foot.is_stepping = false;
            self.left_foot.step_progress = 0.0;

            self.right_foot.planted_position += root_delta;
            self.right_foot.target_position += root_delta;
            self.right_foot.is_stepping = false;
            self.right_foot.step_progress = 0.0;
        }

        // Use externally-set velocity (from ECS Velocity component)
        // rather than calculating from position changes to avoid drift
        self.prev_position = root_position;

        // Horizontal velocity for walking
        let horizontal_velocity = Vector3::new(self.velocity.x, 0.0, self.velocity.z);
        let speed = horizontal_velocity.magnitude();

        let move_dir = if speed > 0.1 {
            let dir = horizontal_velocity.normalize();
            self.last_move_dir = dir;
            dir
        } else {
            self.last_move_dir
        };

        let mut root_position = root_position;
        if self.is_grounded {
            root_position = self.apply_contact_root_anchor(skeleton, root_position);
        } else {
            self.last_contact_root = None;
        }

        // Keep the upper body upright, with gait-based posture offsets
        self.apply_posture(skeleton, root_position, move_dir, speed);

        if self.is_grounded {
            // Update walking phase based on speed (threshold 0.5 to avoid drift)
            if speed > 0.5 {
                let stride_freq = speed / self.config.stride_length;
                self.walk_phase += stride_freq * dt;
                if self.walk_phase >= 1.0 {
                    self.walk_phase -= 1.0;
                }
            }

            // Update foot timers
            self.left_foot.time_since_step += dt;
            self.right_foot.time_since_step += dt;

            // Calculate ideal foot positions based on current body position and velocity
            let (left_ideal, right_ideal) =
                self.calculate_ideal_foot_positions(root_position, ground_height, height);

            // Check if feet need to step
            self.update_stepping(&left_ideal, &right_ideal, ground_height, height, dt);

            // Apply body dynamics (hip sway/twist, shoulder counter-twist)
            self.apply_body_dynamics(skeleton, move_dir, speed);
        } else {
            // Airborne: tuck legs under the body to avoid trailing drag.
            let right = Vector3::y().cross(&move_dir).normalize();
            let hip_width = self.config.step_width * height;
            let leg_len = skeleton.config.leg_length * height;
            let foot_drop = leg_len * 0.85;
            let forward_bias = move_dir * (self.config.stride_length * 0.1);

            let left_air = root_position - right * hip_width
                + Vector3::new(0.0, -foot_drop, 0.0)
                + forward_bias;
            let right_air = root_position
                + right * hip_width
                + Vector3::new(0.0, -foot_drop, 0.0)
                + forward_bias;

            self.left_foot.planted_position = left_air;
            self.left_foot.target_position = left_air;
            self.left_foot.is_stepping = false;
            self.left_foot.step_progress = 0.0;

            self.right_foot.planted_position = right_air;
            self.right_foot.target_position = right_air;
            self.right_foot.is_stepping = false;
            self.right_foot.step_progress = 0.0;
        }

        // Apply IK to skeleton (legs + arms)
        self.apply_ik(skeleton, move_dir, speed);
    }

    /// Calculate ideal foot positions based on body state.
    fn calculate_ideal_foot_positions(
        &self,
        root_position: Point3<f32>,
        ground_height: f32,
        height: f32,
    ) -> (Point3<f32>, Point3<f32>) {
        let half_stride = self.config.stride_length * 0.5;

        // Direction of movement (or last facing direction if stationary)
        let move_dir = if self.velocity.magnitude() > 0.5 {
            Vector3::new(self.velocity.x, 0.0, self.velocity.z).normalize()
        } else {
            self.last_move_dir
        };

        // Perpendicular for foot spacing
        let right = Vector3::y().cross(&move_dir).normalize();

        // Offset feet to sides
        let hip_width = self.config.step_width * height;
        let left_offset = -right * hip_width;
        let right_offset = right * hip_width;

        // Offset based on walk phase (alternating)
        let phase_offset = (self.walk_phase * std::f32::consts::TAU).sin() * half_stride;

        let left_ideal = Point3::new(
            root_position.x + left_offset.x - move_dir.x * phase_offset,
            ground_height,
            root_position.z + left_offset.z - move_dir.z * phase_offset,
        );

        let right_ideal = Point3::new(
            root_position.x + right_offset.x + move_dir.x * phase_offset,
            ground_height,
            root_position.z + right_offset.z + move_dir.z * phase_offset,
        );

        (left_ideal, right_ideal)
    }

    /// Update stepping logic for both feet.
    fn update_stepping(
        &mut self,
        left_ideal: &Point3<f32>,
        right_ideal: &Point3<f32>,
        ground_height: f32,
        height: f32,
        dt: f32,
    ) {
        // Update step animations
        if self.left_foot.is_stepping {
            self.left_foot.step_progress += dt * self.config.step_speed;
            if self.left_foot.step_progress >= 1.0 {
                self.left_foot.is_stepping = false;
                self.left_foot.step_progress = 0.0;
                self.left_foot.planted_position = self.left_foot.target_position;
                self.left_foot.time_since_step = 0.0;
            }
        }

        if self.right_foot.is_stepping {
            self.right_foot.step_progress += dt * self.config.step_speed;
            if self.right_foot.step_progress >= 1.0 {
                self.right_foot.is_stepping = false;
                self.right_foot.step_progress = 0.0;
                self.right_foot.planted_position = self.right_foot.target_position;
                self.right_foot.time_since_step = 0.0;
            }
        }

        // Keep planted feet on the ground when not stepping.
        if !self.left_foot.is_stepping {
            self.left_foot.planted_position.y = ground_height;
            self.left_foot.target_position.y = ground_height;
        }
        if !self.right_foot.is_stepping {
            self.right_foot.planted_position.y = ground_height;
            self.right_foot.target_position.y = ground_height;
        }

        // Check if we need to start new steps
        let left_dist = self.horizontal_distance(&self.left_foot.planted_position, left_ideal);
        let right_dist = self.horizontal_distance(&self.right_foot.planted_position, right_ideal);

        let threshold = self.config.step_threshold * height;
        let cooldown = self.config.step_cooldown;

        // Alternate stepping (don't step both feet at once)
        let can_left_step = !self.left_foot.is_stepping
            && !self.right_foot.is_stepping
            && self.left_foot.time_since_step > cooldown
            && left_dist > threshold;

        let can_right_step = !self.right_foot.is_stepping
            && !self.left_foot.is_stepping
            && self.right_foot.time_since_step > cooldown
            && right_dist > threshold;

        // Prefer stepping the foot that's further from ideal
        if can_left_step && (!can_right_step || left_dist > right_dist) {
            self.left_foot.is_stepping = true;
            self.left_foot.step_progress = 0.0;
            self.left_foot.step_height = self.config.step_height * height;

            // Calculate target with overshoot
            let overshoot = if self.velocity.magnitude() > 0.5 {
                self.velocity.normalize() * (self.config.step_overshoot * height)
            } else {
                Vector3::zeros()
            };
            self.left_foot.target_position = *left_ideal + overshoot;
            self.left_foot.target_position.y = ground_height;
        } else if can_right_step {
            self.right_foot.is_stepping = true;
            self.right_foot.step_progress = 0.0;
            self.right_foot.step_height = self.config.step_height * height;

            let overshoot = if self.velocity.magnitude() > 0.5 {
                self.velocity.normalize() * (self.config.step_overshoot * height)
            } else {
                Vector3::zeros()
            };
            self.right_foot.target_position = *right_ideal + overshoot;
            self.right_foot.target_position.y = ground_height;
        }
    }

    /// Calculate horizontal distance between two points.
    fn horizontal_distance(&self, a: &Point3<f32>, b: &Point3<f32>) -> f32 {
        let dx = b.x - a.x;
        let dz = b.z - a.z;
        (dx * dx + dz * dz).sqrt()
    }

    /// Apply IK to position the skeleton's feet.
    fn apply_ik(&self, skeleton: &mut HumanoidSkeleton, move_dir: Vector3<f32>, speed: f32) {
        let mut positions = skeleton.all_positions();

        // IK for left leg
        let left_target = IKTarget::new(self.left_foot.current_position());
        self.ik_solver
            .solve(&mut positions, &skeleton.left_leg_chain, &left_target, true);

        // IK for right leg
        let right_target = IKTarget::new(self.right_foot.current_position());
        self.ik_solver.solve(
            &mut positions,
            &skeleton.right_leg_chain,
            &right_target,
            true,
        );

        // IK for arms (swing targets)
        let gait_strength = if self.is_grounded {
            (speed / self.config.gait_speed).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (left_target, right_target) = self.arm_swing_targets(skeleton, move_dir, gait_strength);

        self.ik_solver
            .solve(&mut positions, &skeleton.left_arm_chain, &left_target, true);
        self.ik_solver.solve(
            &mut positions,
            &skeleton.right_arm_chain,
            &right_target,
            true,
        );

        // Apply positions back to skeleton
        for (i, pos) in positions.iter().enumerate() {
            if !skeleton.verlet.particles[i].is_pinned() {
                skeleton.verlet.particles[i].position = *pos;
                skeleton.verlet.particles[i].prev_position = *pos; // Reset velocity
            }
        }
    }

    /// Maintain an upright pose for the upper body.
    ///
    /// This counteracts gravity by positioning spine particles relative to the hips.
    fn apply_posture(
        &self,
        skeleton: &mut HumanoidSkeleton,
        root_position: Point3<f32>,
        move_dir: Vector3<f32>,
        speed: f32,
    ) {
        let joints = &skeleton.joints;
        let config = &skeleton.config;

        let gait_strength = if self.is_grounded {
            (speed / self.config.gait_speed).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let right = Vector3::y().cross(&move_dir).normalize();
        let up = Vector3::y();

        // Calculate where the spine should be (upright above hips)
        let torso_len = config.height * config.torso_length;
        let spine_height = torso_len * 0.33;
        let chest_height = torso_len * 0.66;
        let neck_height = torso_len * 0.9;
        let head_height = torso_len + config.head_size * config.height * 0.5;

        // Gait-based offsets
        let phase = self.walk_phase * std::f32::consts::TAU;
        let bob_amount = self.config.body_bob_amount * config.height * gait_strength;
        let bob = phase.cos().abs() * bob_amount;
        let lean_amount = self.config.body_lean_amount * config.height * gait_strength;

        let sway = phase.sin() * self.config.hip_sway_amount * config.height * gait_strength;
        let pelvis_center = root_position + right * sway;

        // Target positions (upright above root with gait offsets)
        let spine_target = pelvis_center + up * (spine_height + bob);
        let chest_target = pelvis_center + up * (chest_height + bob * 0.5) + move_dir * lean_amount;
        let neck_target =
            pelvis_center + up * (neck_height + bob * 0.3) + move_dir * (lean_amount * 0.5);
        let head_target =
            pelvis_center + up * (head_height + bob * 0.2) + move_dir * (lean_amount * 0.4);

        // Blend factor - how strongly to enforce upright pose (1.0 = fully upright)
        let upright_strength = 0.9;

        // Apply upright targets by blending current position toward target
        let spine_pos = skeleton.joint_position(joints.spine);
        let chest_pos = skeleton.joint_position(joints.chest);
        let neck_pos = skeleton.joint_position(joints.neck);
        let head_pos = skeleton.joint_position(joints.head);

        skeleton.verlet.particles[joints.spine].position =
            spine_pos + (spine_target - spine_pos) * upright_strength;
        skeleton.verlet.particles[joints.chest].position =
            chest_pos + (chest_target - chest_pos) * upright_strength;
        skeleton.verlet.particles[joints.neck].position =
            neck_pos + (neck_target - neck_pos) * upright_strength;
        skeleton.verlet.particles[joints.head].position =
            head_pos + (head_target - head_pos) * upright_strength;

        // Also reset their velocities to prevent flopping
        skeleton.verlet.particles[joints.spine].prev_position =
            skeleton.verlet.particles[joints.spine].position;
        skeleton.verlet.particles[joints.chest].prev_position =
            skeleton.verlet.particles[joints.chest].position;
        skeleton.verlet.particles[joints.neck].prev_position =
            skeleton.verlet.particles[joints.neck].position;
        skeleton.verlet.particles[joints.head].prev_position =
            skeleton.verlet.particles[joints.head].position;

        // Position shoulders relative to chest
        let shoulder_width = config.height * config.shoulder_width * 0.5;
        let shoulder_twist =
            phase.sin() * self.config.shoulder_twist_amount * config.height * gait_strength;
        let left_shoulder_target =
            chest_target - right * shoulder_width - move_dir * shoulder_twist;
        let right_shoulder_target =
            chest_target + right * shoulder_width + move_dir * shoulder_twist;

        let left_shoulder_pos = skeleton.joint_position(joints.left_shoulder);
        let right_shoulder_pos = skeleton.joint_position(joints.right_shoulder);

        skeleton.verlet.particles[joints.left_shoulder].position =
            left_shoulder_pos + (left_shoulder_target - left_shoulder_pos) * upright_strength;
        skeleton.verlet.particles[joints.right_shoulder].position =
            right_shoulder_pos + (right_shoulder_target - right_shoulder_pos) * upright_strength;

        skeleton.verlet.particles[joints.left_shoulder].prev_position =
            skeleton.verlet.particles[joints.left_shoulder].position;
        skeleton.verlet.particles[joints.right_shoulder].prev_position =
            skeleton.verlet.particles[joints.right_shoulder].position;
    }

    /// Apply body dynamics (hip sway and twist).
    fn apply_body_dynamics(
        &self,
        skeleton: &mut HumanoidSkeleton,
        move_dir: Vector3<f32>,
        speed: f32,
    ) {
        let joints = &skeleton.joints;
        let config = &skeleton.config;
        let gait_strength = (speed / self.config.gait_speed).clamp(0.0, 1.0);
        if gait_strength <= 0.0 || !self.is_grounded {
            return;
        }

        let right = Vector3::y().cross(&move_dir).normalize();
        let phase = self.walk_phase * std::f32::consts::TAU;

        let hip_width = config.height * config.hip_width * 0.5;
        let sway = phase.sin() * self.config.hip_sway_amount * config.height * gait_strength;
        let twist = phase.sin() * self.config.hip_twist_amount * config.height * gait_strength;
        let pelvis_center = skeleton.root_position() + right * sway;

        let left_hip_target = pelvis_center - right * hip_width + move_dir * twist;
        let right_hip_target = pelvis_center + right * hip_width - move_dir * twist;

        let left_hip_pos = skeleton.joint_position(joints.left_hip);
        let right_hip_pos = skeleton.joint_position(joints.right_hip);

        let hip_strength = 0.6;
        skeleton.verlet.particles[joints.left_hip].position =
            left_hip_pos + (left_hip_target - left_hip_pos) * hip_strength;
        skeleton.verlet.particles[joints.right_hip].position =
            right_hip_pos + (right_hip_target - right_hip_pos) * hip_strength;

        skeleton.verlet.particles[joints.left_hip].prev_position =
            skeleton.verlet.particles[joints.left_hip].position;
        skeleton.verlet.particles[joints.right_hip].prev_position =
            skeleton.verlet.particles[joints.right_hip].position;
    }

    fn arm_swing_targets(
        &self,
        skeleton: &HumanoidSkeleton,
        move_dir: Vector3<f32>,
        gait_strength: f32,
    ) -> (IKTarget, IKTarget) {
        let joints = &skeleton.joints;
        let config = &skeleton.config;
        let right = Vector3::y().cross(&move_dir).normalize();
        let up = Vector3::y();

        let phase = self.walk_phase * std::f32::consts::TAU;
        let swing = phase.sin();

        let swing_scale = if self.is_grounded { gait_strength } else { 0.0 };
        let swing_amount = self.config.arm_swing_amount * config.height * swing_scale;
        let lift_amount = self.config.arm_lift_amount * config.height * swing_scale;
        let back_bias = 0.04 * config.height;

        let left_shoulder = skeleton.joint_position(joints.left_shoulder);
        let right_shoulder = skeleton.joint_position(joints.right_shoulder);

        let left_rest = right * self.left_hand_rest_offset.x
            + up * self.left_hand_rest_offset.y
            + move_dir * self.left_hand_rest_offset.z;
        let right_rest = right * self.right_hand_rest_offset.x
            + up * self.right_hand_rest_offset.y
            + move_dir * self.right_hand_rest_offset.z;

        // Left arm moves opposite to left leg (forward when left leg back)
        let left_offset = move_dir * (swing_amount * -swing - back_bias)
            + up * (lift_amount * (1.0 - swing.abs()));
        let right_offset = move_dir * (swing_amount * swing - back_bias)
            + up * (lift_amount * (1.0 - swing.abs()));

        let left_target_pos = left_shoulder + left_rest + left_offset;
        let right_target_pos = right_shoulder + right_rest + right_offset;

        let arm_weight = 0.85 + 0.15 * gait_strength;
        (
            IKTarget::weighted(left_target_pos, arm_weight),
            IKTarget::weighted(right_target_pos, arm_weight),
        )
    }

    fn apply_contact_root_anchor(
        &mut self,
        skeleton: &mut HumanoidSkeleton,
        current_root: Point3<f32>,
    ) -> Point3<f32> {
        let height = skeleton.config.height;
        let leg_len = skeleton.config.leg_length * height;

        let left = self.left_foot.current_position();
        let right = self.right_foot.current_position();
        let midpoint = Point3::from((left.coords + right.coords) * 0.5);

        // Only anchor the vertical component to avoid pulling the body backward/forward.
        let target_y = midpoint.y + leg_len * 0.9;
        let strength = self.config.pelvis_anchor_strength.clamp(0.0, 1.0);
        let blended_y = current_root.y + (target_y - current_root.y) * strength;
        let blended = Point3::new(current_root.x, blended_y, current_root.z);

        let smoothed = if let Some(prev) = self.last_contact_root {
            let t = self.config.pelvis_anchor_smoothing.clamp(0.0, 1.0);
            Point3::from(prev.coords.lerp(&blended.coords, t))
        } else {
            blended
        };

        // Move the pinned hips to follow the planted feet midpoint.
        skeleton
            .verlet
            .set_particle_position(skeleton.joints.hips, smoothed);

        self.last_contact_root = Some(smoothed);
        smoothed
    }

    /// Check if currently walking.
    pub fn is_walking(&self) -> bool {
        self.velocity.magnitude() > 0.5
    }

    /// Get current foot positions for rendering/debug.
    pub fn foot_positions(&self) -> (Point3<f32>, Point3<f32>) {
        (
            self.left_foot.current_position(),
            self.right_foot.current_position(),
        )
    }
}
