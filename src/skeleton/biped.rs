//! Spring-leg biped skeleton with physics-based locomotion.
//!
//! This is a simplified biped that uses spring forces for natural bouncy walking:
//! - Pelvis is a point mass with velocity
//! - Each leg is a virtual spring from foot to pelvis
//! - When grounded, springs push the pelvis forward and up
//! - FABRIK IK positions knees cosmetically
//! - Sphere collision on joints (feet, knees, pelvis)

use nalgebra::{Point3, Vector3};

use super::fabrik::{FABRIKSolver, IKChain, IKTarget};
use crate::collision::{sphere_triangle_collision, Triangle, AABB};

/// Configuration for the spring biped skeleton.
#[derive(Clone, Debug)]
pub struct SpringBipedConfig {
    /// Upper leg length (hip to knee).
    pub upper_leg_length: f32,
    /// Lower leg length (knee to foot).
    pub lower_leg_length: f32,
    /// Lateral distance from pelvis center to each hip.
    pub hip_width: f32,
    /// Collision radius for foot spheres.
    pub foot_radius: f32,
    /// Collision radius for knee spheres.
    pub knee_radius: f32,
    /// Collision radius for pelvis sphere.
    pub pelvis_radius: f32,

    // Gait parameters
    /// Speed below which the character is considered standing still.
    pub idle_threshold: f32,
    /// Height of the step arc when foot is swinging.
    pub step_height: f32,
    /// How far ahead to target the foot when moving.
    pub step_ahead: f32,
}

impl Default for SpringBipedConfig {
    fn default() -> Self {
        Self {
            upper_leg_length: 0.25,
            lower_leg_length: 0.25,
            hip_width: 0.12,
            foot_radius: 0.06,
            knee_radius: 0.04,
            pelvis_radius: 0.1,
            idle_threshold: 0.1,
            step_height: 0.2,
            step_ahead: 5.5,
        }
    }
}

/// State of a single leg.
#[derive(Clone, Debug)]
pub struct LegState {
    /// Current foot position.
    pub foot_position: Point3<f32>,
    /// Target foot position (where we're stepping to).
    pub foot_target: Point3<f32>,
    /// Is this leg currently grounded (supporting weight)?
    pub is_grounded: bool,
    /// Progress through swing phase [0, 1] when not grounded.
    pub swing_progress: f32,
    /// Position where the swing started.
    pub swing_start: Point3<f32>,
    /// Knee position (calculated by FABRIK).
    pub knee_position: Point3<f32>,
}

impl LegState {
    fn new(foot_pos: Point3<f32>, knee_pos: Point3<f32>) -> Self {
        Self {
            foot_position: foot_pos,
            foot_target: foot_pos,
            is_grounded: true,
            swing_progress: 0.0,
            swing_start: foot_pos,
            knee_position: knee_pos,
        }
    }
}

/// A spring-leg biped skeleton.
///
/// Uses spring forces for natural bouncy locomotion:
/// - Grounded legs act as springs pushing the pelvis
/// - Pelvis integrates forces with gravity
/// - Feet alternate between grounded and swinging
/// - FABRIK positions knees
pub struct SpringBipedSkeleton {
    pub config: SpringBipedConfig,

    // Pelvis state (point mass)
    pelvis_position: Point3<f32>,
    pelvis_velocity: Vector3<f32>,

    // Leg states
    pub left_leg: LegState,
    pub right_leg: LegState,

    // Gait phase [0, 1) - drives which leg is grounded
    gait_phase: f32,

    // Facing direction (for knee bend direction and step targeting)
    facing: Vector3<f32>,

    // FABRIK solver for knees
    ik_solver: FABRIKSolver,
}

impl SpringBipedSkeleton {
    /// Create a new spring biped at the given pelvis position.
    pub fn new(config: SpringBipedConfig, pelvis_position: Point3<f32>) -> Self {
        let total_leg_length = config.upper_leg_length + config.lower_leg_length;
        let facing = Vector3::new(0.0, 0.0, 1.0);
        let right = right_vector(facing);

        // Calculate hip positions using the same logic as left_hip()/right_hip()
        let left_hip_pos = pelvis_position - right * config.hip_width;
        let right_hip_pos = pelvis_position + right * config.hip_width;

        // Initial foot positions directly below hips
        let left_foot = Point3::new(
            left_hip_pos.x,
            pelvis_position.y - total_leg_length,
            left_hip_pos.z,
        );
        let right_foot = Point3::new(
            right_hip_pos.x,
            pelvis_position.y - total_leg_length,
            right_hip_pos.z,
        );

        // Initial knee positions (bent slightly forward)
        let left_knee = Point3::new(
            left_hip_pos.x,
            pelvis_position.y - config.upper_leg_length,
            left_hip_pos.z + 0.05,
        );
        let right_knee = Point3::new(
            right_hip_pos.x,
            pelvis_position.y - config.upper_leg_length,
            right_hip_pos.z + 0.05,
        );

        Self {
            config,
            pelvis_position,
            pelvis_velocity: Vector3::zeros(),
            left_leg: LegState::new(left_foot, left_knee),
            right_leg: LegState::new(right_foot, right_knee),
            gait_phase: 0.0,
            facing: Vector3::new(0.0, 0.0, 1.0),
            ik_solver: FABRIKSolver::new(),
        }
    }

    /// Get the total leg length.
    pub fn leg_length(&self) -> f32 {
        self.config.upper_leg_length + self.config.lower_leg_length
    }

    /// Get hip positions (derived from pelvis).
    #[inline]
    pub fn left_hip(&self) -> Point3<f32> {
        let left = self.left_vector();
        self.pelvis_position + left * self.config.hip_width
    }

    #[inline]
    pub fn right_hip(&self) -> Point3<f32> {
        let right = self.right_vector();
        self.pelvis_position + right * self.config.hip_width
    }

    /// Solve FABRIK IK to position knees.
    fn solve_knee_ik(&mut self) {
        let right = self.right_vector();
        let left_bend = (self.facing - right * 0.2).normalize();
        let right_bend = (self.facing + right * 0.2).normalize();

        // Left leg: hip -> knee -> foot
        let left_hip = self.left_hip();
        let left_foot = self.left_leg.foot_position;
        let mut positions = vec![left_hip, self.left_leg.knee_position, left_foot];
        let chain = IKChain::new(vec![0, 1, 2], &positions);
        let target = IKTarget::new(left_foot);
        self.ik_solver.solve(&mut positions, &chain, &target, true);

        // Apply knee bend bias (knees should bend forward)
        let mut left_knee = positions[1] + left_bend * 0.05;
        Self::constrain_knee_static(
            &mut left_knee,
            &left_hip,
            &left_foot,
            self.config.upper_leg_length,
            self.config.lower_leg_length,
            left_bend,
        );
        self.left_leg.knee_position = left_knee;

        // Right leg: hip -> knee -> foot
        let right_hip = self.right_hip();
        let right_foot = self.right_leg.foot_position;
        positions = vec![right_hip, self.right_leg.knee_position, right_foot];
        let chain = IKChain::new(vec![0, 1, 2], &positions);
        let target = IKTarget::new(right_foot);
        self.ik_solver.solve(&mut positions, &chain, &target, true);

        let mut right_knee = positions[1] + right_bend * 0.05;
        Self::constrain_knee_static(
            &mut right_knee,
            &right_hip,
            &right_foot,
            self.config.upper_leg_length,
            self.config.lower_leg_length,
            right_bend,
        );
        self.right_leg.knee_position = right_knee;
    }

    /// Constrain knee to be at correct bone lengths from hip and foot (static version).
    fn constrain_knee_static(
        knee: &mut Point3<f32>,
        hip: &Point3<f32>,
        foot: &Point3<f32>,
        upper: f32,
        lower: f32,
        bend_dir: Vector3<f32>,
    ) {
        let hip_to_foot = foot - hip;
        let dist = hip_to_foot.magnitude();

        if dist < 0.001 {
            // Foot at hip - just put knee forward
            *knee = *hip + bend_dir * upper;
            return;
        }

        // Clamp distance to valid range
        let dist = dist.clamp(0.01, upper + lower - 0.01);

        // Law of cosines to find knee angle
        let cos_angle =
            ((upper * upper + dist * dist - lower * lower) / (2.0 * upper * dist)).clamp(-1.0, 1.0);
        let angle = cos_angle.acos();

        // Direction from hip to foot
        let forward = hip_to_foot.normalize();

        let bend_axis = forward.cross(&bend_dir);
        let bend_dir_orth = if bend_axis.magnitude() > 0.01 {
            bend_axis.cross(&forward).normalize()
        } else {
            Vector3::y()
        };

        // Position knee in a stable bend plane
        let knee_offset = forward * (angle.cos() * upper) + bend_dir_orth * (angle.sin() * upper);
        *knee = hip + knee_offset;
    }

    /// Update gait and foot targeting.
    ///
    /// `horizontal_velocity` - Current horizontal movement velocity.
    /// `left_target` - IK target for left foot (None if no ground contact).
    /// `right_target` - IK target for right foot (None if no ground contact).
    pub fn update_gait(
        &mut self,
        horizontal_velocity: Vector3<f32>,
        dt: f32,
        left_target: Option<Point3<f32>>,
        right_target: Option<Point3<f32>>,
    ) {
        let speed = horizontal_velocity.magnitude();
        let leg_length = self.leg_length();
        let step_length = (self.config.step_ahead * 2.0).clamp(0.05, leg_length * 0.9);
        const MIN_STRIDE_TIME: f32 = 0.35;
        let stride_time = if speed > 1e-4 {
            (step_length / speed).max(MIN_STRIDE_TIME)
        } else {
            f32::MAX
        };
        let swing_ratio = 0.4;
        let swing_time = stride_time * swing_ratio;

        let left_hip = self.left_hip();
        let right_hip = self.right_hip();
        let left_ground = left_target.map(|t| t.y).unwrap_or(left_hip.y - leg_length);
        let right_ground = right_target
            .map(|t| t.y)
            .unwrap_or(right_hip.y - leg_length);

        if speed <= self.config.idle_threshold || !stride_time.is_finite() {
            let left_desired =
                left_target.unwrap_or(Point3::new(left_hip.x, left_ground, left_hip.z));
            let right_desired =
                right_target.unwrap_or(Point3::new(right_hip.x, right_ground, right_hip.z));
            let snap_threshold = 0.03;
            let left_delta = (left_desired - self.left_leg.foot_target).magnitude();
            let right_delta = (right_desired - self.right_leg.foot_target).magnitude();

            self.left_leg.foot_target = if left_delta <= snap_threshold {
                self.left_leg.foot_target
            } else {
                left_desired
            };
            self.left_leg.foot_position = self.left_leg.foot_target;
            self.left_leg.is_grounded = true;
            self.left_leg.swing_progress = 0.0;
            self.left_leg.swing_start = self.left_leg.foot_position;

            self.right_leg.foot_target = if right_delta <= snap_threshold {
                self.right_leg.foot_target
            } else {
                right_desired
            };
            self.right_leg.foot_position = self.right_leg.foot_target;
            self.right_leg.is_grounded = true;
            self.right_leg.swing_progress = 0.0;
            self.right_leg.swing_start = self.right_leg.foot_position;

            self.gait_phase = 0.0;
            return;
        }

        self.gait_phase += (speed * dt) / step_length;
        self.gait_phase %= 1.0;

        let facing = self.facing;
        let left_phase = self.gait_phase;
        let right_phase = (self.gait_phase + 0.5) % 1.0;

        Self::update_leg_planted(
            &mut self.left_leg,
            left_hip,
            left_phase,
            facing,
            step_length,
            swing_ratio,
            swing_time,
            self.config.step_height,
            dt,
            left_ground,
            left_target,
        );
        Self::update_leg_planted(
            &mut self.right_leg,
            right_hip,
            right_phase,
            facing,
            step_length,
            swing_ratio,
            swing_time,
            self.config.step_height,
            dt,
            right_ground,
            right_target,
        );
    }

    fn update_leg_planted(
        leg: &mut LegState,
        hip: Point3<f32>,
        phase: f32,
        facing: Vector3<f32>,
        step_length: f32,
        swing_ratio: f32,
        swing_time: f32,
        step_height: f32,
        dt: f32,
        ground_y: f32,
        target: Option<Point3<f32>>,
    ) {
        let in_swing = phase < swing_ratio;

        if in_swing {
            if leg.is_grounded {
                leg.is_grounded = false;
                leg.swing_progress = 0.0;
                leg.swing_start = leg.foot_position;
                leg.foot_target = target.unwrap_or_else(|| {
                    Self::compute_step_target(hip, facing, step_length, ground_y)
                });
            }

            let step = if swing_time > 1e-5 {
                dt / swing_time
            } else {
                1.0
            };
            leg.swing_progress = (leg.swing_progress + step).min(1.0);
            let t = leg.swing_progress;
            let smooth_t = t * t * (3.0 - 2.0 * t);

            let horizontal = leg
                .swing_start
                .coords
                .lerp(&leg.foot_target.coords, smooth_t);
            let arc = (std::f32::consts::PI * t).sin() * step_height;

            leg.foot_position = Point3::new(horizontal.x, horizontal.y + arc, horizontal.z);
            if leg.swing_progress >= 1.0 {
                leg.is_grounded = true;
                leg.foot_position = leg.foot_target;
            }
        } else {
            if !leg.is_grounded {
                leg.is_grounded = true;
                leg.foot_position = leg.foot_target;
            }
            leg.swing_progress = 0.0;
            leg.swing_start = leg.foot_position;
        }
    }

    fn compute_step_target(
        hip: Point3<f32>,
        facing: Vector3<f32>,
        step_length: f32,
        ground_y: f32,
    ) -> Point3<f32> {
        let forward = facing * (step_length * 0.5);
        Point3::new(hip.x + forward.x, ground_y, hip.z + forward.z)
    }

    /// Solve IK only - updates foot positions and knee positions.
    ///
    /// Call this after setting pelvis position to update the leg geometry.
    /// Does NOT run spring physics or move the pelvis.
    pub fn solve_ik_only(&mut self) {
        let max_leg_length = self.config.upper_leg_length + self.config.lower_leg_length;

        // Get hip positions (copies, to avoid borrow issues)
        let left_hip = self.left_hip();
        let right_hip = self.right_hip();

        // Clamp feet to be within leg reach of their respective hips
        Self::clamp_foot_to_hip(&left_hip, &mut self.left_leg.foot_position, max_leg_length);
        Self::clamp_foot_to_hip(
            &right_hip,
            &mut self.right_leg.foot_position,
            max_leg_length,
        );

        // Solve IK for knees
        self.solve_knee_ik();
    }

    /// Clamp a foot position to be within max distance from hip.
    fn clamp_foot_to_hip(hip: &Point3<f32>, foot: &mut Point3<f32>, max_length: f32) {
        let to_foot = *foot - hip;
        let dist = to_foot.magnitude();

        if dist > max_length {
            // Foot is too far - pull it back toward hip
            let dir = to_foot / dist;
            *foot = hip + dir * max_length * 0.98; // Slight margin for IK
        }
    }

    /// Resolve collisions with terrain triangles.
    ///
    /// Checks sphere collision for feet, knees, and pelvis against provided triangles.
    /// Returns true if any collision was resolved.
    pub fn resolve_collisions(&mut self, triangles: &[Triangle]) -> bool {
        let mut had_collision = false;

        // Pelvis collision
        for tri in triangles {
            if let Some(contact) =
                sphere_triangle_collision(self.pelvis_position, self.config.pelvis_radius, tri)
            {
                // Push pelvis out of collision
                self.pelvis_position += contact.normal * contact.depth;

                // Cancel velocity into the collision surface
                let vel_into_surface = self.pelvis_velocity.dot(&contact.normal);
                if vel_into_surface < 0.0 {
                    self.pelvis_velocity -= contact.normal * vel_into_surface;
                }

                had_collision = true;
            }
        }

        // Left knee collision
        for tri in triangles {
            if let Some(contact) =
                sphere_triangle_collision(self.left_leg.knee_position, self.config.knee_radius, tri)
            {
                self.left_leg.knee_position += contact.normal * contact.depth;
                had_collision = true;
            }
        }

        // Right knee collision
        for tri in triangles {
            if let Some(contact) = sphere_triangle_collision(
                self.right_leg.knee_position,
                self.config.knee_radius,
                tri,
            ) {
                self.right_leg.knee_position += contact.normal * contact.depth;
                had_collision = true;
            }
        }

        // Left foot collision (only when grounded - swinging feet don't collide)
        if self.left_leg.is_grounded {
            for tri in triangles {
                if let Some(contact) = sphere_triangle_collision(
                    self.left_leg.foot_position,
                    self.config.foot_radius,
                    tri,
                ) {
                    self.left_leg.foot_position += contact.normal * contact.depth;
                    had_collision = true;
                }
            }
        }

        // Right foot collision
        if self.right_leg.is_grounded {
            for tri in triangles {
                if let Some(contact) = sphere_triangle_collision(
                    self.right_leg.foot_position,
                    self.config.foot_radius,
                    tri,
                ) {
                    self.right_leg.foot_position += contact.normal * contact.depth;
                    had_collision = true;
                }
            }
        }

        had_collision
    }

    /// Get the AABB containing all joints for terrain queries.
    pub fn get_collision_aabb(&self) -> AABB {
        let margin = self.config.pelvis_radius.max(self.config.foot_radius) + 0.1;

        let mut min = self.pelvis_position;
        let mut max = self.pelvis_position;

        // Expand to include all joints
        for pos in [
            self.left_leg.foot_position,
            self.right_leg.foot_position,
            self.left_leg.knee_position,
            self.right_leg.knee_position,
            self.left_hip(),
            self.right_hip(),
        ] {
            min.x = min.x.min(pos.x);
            min.y = min.y.min(pos.y);
            min.z = min.z.min(pos.z);
            max.x = max.x.max(pos.x);
            max.y = max.y.max(pos.y);
            max.z = max.z.max(pos.z);
        }

        AABB::new(
            Point3::new(min.x - margin, min.y - margin, min.z - margin),
            Point3::new(max.x + margin, max.y + margin, max.z + margin),
        )
    }

    #[inline]
    pub fn left_vector(&self) -> Vector3<f32> {
        left_vector(self.facing)
    }

    #[inline]
    pub fn right_vector(&self) -> Vector3<f32> {
        right_vector(self.facing)
    }

    // === Getters ===

    pub fn pelvis_position(&self) -> Point3<f32> {
        self.pelvis_position
    }

    pub fn left_foot_position(&self) -> Point3<f32> {
        self.left_leg.foot_position
    }

    pub fn right_foot_position(&self) -> Point3<f32> {
        self.right_leg.foot_position
    }

    pub fn left_knee_position(&self) -> Point3<f32> {
        self.left_leg.knee_position
    }

    pub fn right_knee_position(&self) -> Point3<f32> {
        self.right_leg.knee_position
    }

    pub fn facing_direction(&self) -> Vector3<f32> {
        self.facing
    }

    // === Setters (for external physics integration) ===

    /// Set pelvis position directly (e.g., from external physics body).
    pub fn set_pelvis_position(&mut self, position: Point3<f32>) {
        self.pelvis_position = position;
    }

    /// Set facing direction.
    pub fn set_facing(&mut self, facing: Vector3<f32>) {
        let horizontal = Vector3::new(facing.x, 0.0, facing.z);
        if horizontal.magnitude() > 0.01 {
            self.facing = horizontal.normalize();
        }
    }
}

#[inline]
fn right_vector(facing: Vector3<f32>) -> Vector3<f32> {
    facing.cross(&Vector3::y()).normalize()
}

#[inline]
fn left_vector(facing: Vector3<f32>) -> Vector3<f32> {
    Vector3::y().cross(&facing).normalize()
}

impl Default for SpringBipedSkeleton {
    fn default() -> Self {
        Self::new(SpringBipedConfig::default(), Point3::new(0.0, 1.0, 0.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spring_biped_creation() {
        let biped = SpringBipedSkeleton::default();

        let pelvis = biped.pelvis_position();
        assert!((pelvis.y - 1.0).abs() < 0.001);

        // Feet should be below pelvis at leg length
        let left_foot = biped.left_foot_position();
        let right_foot = biped.right_foot_position();

        assert!(left_foot.y < pelvis.y);
        assert!(right_foot.y < pelvis.y);
    }

    #[test]
    fn test_collision_aabb() {
        let biped = SpringBipedSkeleton::default();
        let aabb = biped.get_collision_aabb();

        // AABB should contain all joints
        assert!(aabb.contains_point(biped.pelvis_position()));
        assert!(aabb.contains_point(biped.left_foot_position()));
        assert!(aabb.contains_point(biped.right_foot_position()));
    }
}
