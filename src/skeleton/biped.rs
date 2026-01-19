//! Simple biped skeleton - pelvis + two legs.
//!
//! This is a simplified version for testing the step controller:
//! - 3 positions: Pelvis, left foot, right foot
//! - No physics simulation (Verlet removed)
//! - Pelvis position set directly from physics body
//! - Foot positions set directly from step controller
//!
//! Later stages will add FABRIK IK for knee positioning.

use nalgebra::{Point3, Vector3};

/// Configuration for the biped skeleton.
#[derive(Clone, Debug)]
pub struct BipedConfig {
    /// Distance from pelvis to knee (upper leg length).
    pub upper_leg_length: f32,
    /// Distance from knee to foot (lower leg length).
    pub lower_leg_length: f32,
    /// Lateral distance from pelvis center to each hip.
    pub hip_width: f32,
}

impl Default for BipedConfig {
    fn default() -> Self {
        Self {
            upper_leg_length: 0.3,
            lower_leg_length: 0.3,
            hip_width: 0.2,
        }
    }
}

/// A simple biped skeleton: pelvis + two feet.
///
/// This is a minimal skeleton for testing the step controller.
/// Positions are set directly - no physics simulation.
pub struct SimpleBipedSkeleton {
    /// Configuration.
    pub config: BipedConfig,
    /// Pelvis position (set directly from physics body).
    pelvis: Point3<f32>,
    /// Left foot position (set from step controller).
    left_foot: Point3<f32>,
    /// Right foot position (set from step controller).
    right_foot: Point3<f32>,
}

impl SimpleBipedSkeleton {
    /// Create a new biped skeleton at the given pelvis position.
    pub fn new(config: BipedConfig, pelvis_position: Point3<f32>) -> Self {
        // Calculate initial foot positions below pelvis
        let total_leg_length = config.upper_leg_length + config.lower_leg_length;
        let left_foot = Point3::new(
            pelvis_position.x - config.hip_width,
            pelvis_position.y - total_leg_length,
            pelvis_position.z,
        );
        let right_foot = Point3::new(
            pelvis_position.x + config.hip_width,
            pelvis_position.y - total_leg_length,
            pelvis_position.z,
        );

        Self {
            config,
            pelvis: pelvis_position,
            left_foot,
            right_foot,
        }
    }

    /// Set the pelvis position directly.
    pub fn set_pelvis_position(&mut self, position: Point3<f32>) {
        self.pelvis = position;
    }

    /// Set the left foot position directly.
    pub fn set_left_foot_position(&mut self, position: Point3<f32>) {
        self.left_foot = position;
    }

    /// Set the right foot position directly.
    pub fn set_right_foot_position(&mut self, position: Point3<f32>) {
        self.right_foot = position;
    }

    /// Get the pelvis position.
    pub fn pelvis_position(&self) -> Point3<f32> {
        self.pelvis
    }

    /// Get the left foot position.
    pub fn left_foot_position(&self) -> Point3<f32> {
        self.left_foot
    }

    /// Get the right foot position.
    pub fn right_foot_position(&self) -> Point3<f32> {
        self.right_foot
    }
}

impl Default for SimpleBipedSkeleton {
    fn default() -> Self {
        Self::new(BipedConfig::default(), Point3::new(0.0, 1.0, 0.0))
    }
}

// ============================================================================
// Simple Stepping Controller
// ============================================================================

/// State of a single foot.
#[derive(Clone, Debug)]
pub struct FootState {
    /// Where the foot is planted (target when grounded).
    pub planted_position: Point3<f32>,
    /// Is this foot currently stepping (in the air)?
    pub is_stepping: bool,
    /// Progress through the step animation [0, 1].
    pub step_progress: f32,
    /// Where we're stepping to.
    pub step_target: Point3<f32>,
    /// Where we started the step from.
    pub step_start: Point3<f32>,
}

impl FootState {
    pub fn new(position: Point3<f32>) -> Self {
        Self {
            planted_position: position,
            is_stepping: false,
            step_progress: 0.0,
            step_target: position,
            step_start: position,
        }
    }

    /// Get the current target position (interpolated if stepping).
    pub fn current_target(&self, step_height: f32) -> Point3<f32> {
        if !self.is_stepping {
            return self.planted_position;
        }

        // Interpolate from start to target with an arc
        let t = self.step_progress;
        // Smooth step function for nicer motion
        let smooth_t = t * t * (3.0 - 2.0 * t);

        // Horizontal interpolation
        let horizontal = self
            .step_start
            .coords
            .lerp(&self.step_target.coords, smooth_t);

        // Vertical arc (parabola peaking at t=0.5)
        let arc_height = 4.0 * t * (1.0 - t) * step_height;

        Point3::new(horizontal.x, horizontal.y + arc_height, horizontal.z)
    }
}

/// Configuration for the stepping controller.
#[derive(Clone, Debug)]
pub struct StepConfig {
    /// How far a foot can drift from ideal before stepping.
    pub step_threshold: f32,
    /// How fast steps complete (1.0 = 1 second per step).
    pub step_speed: f32,
    /// Height of the step arc.
    pub step_height: f32,
    /// How far ahead to place the foot when stepping.
    pub step_overshoot: f32,
    /// Lateral offset from pelvis center to ideal foot position.
    pub foot_spread: f32,
    /// Forward/backward offset for feet (0 = directly below pelvis).
    pub foot_offset_z: f32,
}

impl Default for StepConfig {
    fn default() -> Self {
        Self {
            step_threshold: 0.12, // Trigger steps sooner (was 0.3)
            step_speed: 8.0,      // Faster step animation (was 5.0)
            step_height: 0.1,     // Slightly lower arc (was 0.15)
            step_overshoot: 0.15, // Step further ahead (was 0.1)
            foot_spread: 0.15,
            foot_offset_z: 0.0,
        }
    }
}

/// Simple stepping controller for the biped.
pub struct SimpleStepController {
    pub config: StepConfig,
    pub left_foot: FootState,
    pub right_foot: FootState,
    /// Which foot stepped most recently (to alternate).
    last_step_was_left: bool,
}

impl SimpleStepController {
    pub fn new(config: StepConfig, initial_position: Point3<f32>, ground_height: f32) -> Self {
        let left_pos = Point3::new(
            initial_position.x - config.foot_spread,
            ground_height,
            initial_position.z + config.foot_offset_z,
        );
        let right_pos = Point3::new(
            initial_position.x + config.foot_spread,
            ground_height,
            initial_position.z + config.foot_offset_z,
        );

        Self {
            config,
            left_foot: FootState::new(left_pos),
            right_foot: FootState::new(right_pos),
            last_step_was_left: false,
        }
    }

    /// Update the stepping logic.
    ///
    /// `pelvis_position` - Current pelvis position.
    /// `ground_height` - Height of the ground at the pelvis position.
    /// `velocity` - Current movement velocity (for predicting where to step).
    /// `dt` - Delta time.
    pub fn update(
        &mut self,
        pelvis_position: Point3<f32>,
        ground_height: f32,
        velocity: Vector3<f32>,
        dt: f32,
    ) {
        // Calculate ideal foot positions (where feet "want" to be)
        let left_ideal = Point3::new(
            pelvis_position.x - self.config.foot_spread,
            ground_height,
            pelvis_position.z + self.config.foot_offset_z,
        );
        let right_ideal = Point3::new(
            pelvis_position.x + self.config.foot_spread,
            ground_height,
            pelvis_position.z + self.config.foot_offset_z,
        );

        // Update any ongoing steps
        if self.left_foot.is_stepping {
            self.left_foot.step_progress += self.config.step_speed * dt;
            if self.left_foot.step_progress >= 1.0 {
                // Step complete
                self.left_foot.is_stepping = false;
                self.left_foot.step_progress = 0.0;
                self.left_foot.planted_position = self.left_foot.step_target;
            }
        }

        if self.right_foot.is_stepping {
            self.right_foot.step_progress += self.config.step_speed * dt;
            if self.right_foot.step_progress >= 1.0 {
                // Step complete
                self.right_foot.is_stepping = false;
                self.right_foot.step_progress = 0.0;
                self.right_foot.planted_position = self.right_foot.step_target;
            }
        }

        // Check if either foot needs to step
        let left_drift = self.horizontal_distance(&self.left_foot.planted_position, &left_ideal);
        let right_drift = self.horizontal_distance(&self.right_foot.planted_position, &right_ideal);

        // Only one foot can step at a time
        let can_left_step = !self.left_foot.is_stepping && !self.right_foot.is_stepping;
        let can_right_step = !self.left_foot.is_stepping && !self.right_foot.is_stepping;

        // Trigger a step if needed
        if can_left_step && left_drift > self.config.step_threshold && !self.last_step_was_left {
            self.trigger_step_left(left_ideal, velocity, ground_height);
        } else if can_right_step
            && right_drift > self.config.step_threshold
            && self.last_step_was_left
        {
            self.trigger_step_right(right_ideal, velocity, ground_height);
        } else if can_left_step && left_drift > self.config.step_threshold {
            self.trigger_step_left(left_ideal, velocity, ground_height);
        } else if can_right_step && right_drift > self.config.step_threshold {
            self.trigger_step_right(right_ideal, velocity, ground_height);
        }
    }

    fn trigger_step_left(
        &mut self,
        ideal: Point3<f32>,
        velocity: Vector3<f32>,
        ground_height: f32,
    ) {
        self.left_foot.is_stepping = true;
        self.left_foot.step_progress = 0.0;
        self.left_foot.step_start = self.left_foot.planted_position;

        // Step target is ideal position + overshoot in velocity direction
        let overshoot = if velocity.magnitude() > 0.1 {
            velocity.normalize() * self.config.step_overshoot
        } else {
            Vector3::zeros()
        };
        self.left_foot.step_target =
            Point3::new(ideal.x + overshoot.x, ground_height, ideal.z + overshoot.z);

        self.last_step_was_left = true;
    }

    fn trigger_step_right(
        &mut self,
        ideal: Point3<f32>,
        velocity: Vector3<f32>,
        ground_height: f32,
    ) {
        self.right_foot.is_stepping = true;
        self.right_foot.step_progress = 0.0;
        self.right_foot.step_start = self.right_foot.planted_position;

        let overshoot = if velocity.magnitude() > 0.1 {
            velocity.normalize() * self.config.step_overshoot
        } else {
            Vector3::zeros()
        };
        self.right_foot.step_target =
            Point3::new(ideal.x + overshoot.x, ground_height, ideal.z + overshoot.z);

        self.last_step_was_left = false;
    }

    fn horizontal_distance(&self, a: &Point3<f32>, b: &Point3<f32>) -> f32 {
        let dx = a.x - b.x;
        let dz = a.z - b.z;
        (dx * dx + dz * dz).sqrt()
    }

    /// Get the current target for the left foot.
    pub fn left_target(&self) -> Point3<f32> {
        self.left_foot.current_target(self.config.step_height)
    }

    /// Get the current target for the right foot.
    pub fn right_target(&self) -> Point3<f32> {
        self.right_foot.current_target(self.config.step_height)
    }

    /// Immediately snap feet to ideal positions below the pelvis.
    ///
    /// Call this when landing or when feet need to reset to a known state.
    /// Cancels any in-progress steps.
    pub fn snap_to_position(&mut self, pelvis_position: Point3<f32>, ground_height: f32) {
        let left_ideal = Point3::new(
            pelvis_position.x - self.config.foot_spread,
            ground_height,
            pelvis_position.z + self.config.foot_offset_z,
        );
        let right_ideal = Point3::new(
            pelvis_position.x + self.config.foot_spread,
            ground_height,
            pelvis_position.z + self.config.foot_offset_z,
        );

        self.left_foot.planted_position = left_ideal;
        self.left_foot.is_stepping = false;
        self.left_foot.step_progress = 0.0;

        self.right_foot.planted_position = right_ideal;
        self.right_foot.is_stepping = false;
        self.right_foot.step_progress = 0.0;
    }
}

// ============================================================================
// Phase-Based Gait Controller
// ============================================================================

/// Configuration for the gait controller.
#[derive(Clone, Debug)]
pub struct GaitConfig {
    /// Lateral offset from pelvis center to each hip.
    pub hip_width: f32,
    /// Maximum stride length (forward/back from center) at full speed.
    pub stride_length: f32,
    /// Height of the step arc when foot is swinging.
    pub step_height: f32,
    /// How fast the gait cycle progresses per unit of speed.
    /// Higher = faster leg movement for the same walk speed.
    pub phase_rate: f32,
    /// Speed below which the character is considered standing still.
    pub idle_threshold: f32,
}

impl Default for GaitConfig {
    fn default() -> Self {
        Self {
            hip_width: 0.15,
            stride_length: 0.3,
            step_height: 0.2,
            phase_rate: 1.0,
            idle_threshold: 0.1,
        }
    }
}

/// Phase-based gait controller for natural walking/running animation.
///
/// Instead of reacting to drift, this controller drives foot placement
/// from a continuous walk phase. Feet alternate being ahead/behind the
/// pelvis based on velocity direction.
pub struct GaitController {
    pub config: GaitConfig,
    /// Current walk phase [0, 1). Drives the gait cycle.
    walk_phase: f32,
    /// Cached left foot position.
    left_foot: Point3<f32>,
    /// Cached right foot position.
    right_foot: Point3<f32>,
}

impl GaitController {
    pub fn new(config: GaitConfig) -> Self {
        Self {
            config,
            walk_phase: 0.0,
            left_foot: Point3::origin(),
            right_foot: Point3::origin(),
        }
    }

    /// Update the gait and calculate foot positions.
    ///
    /// `pelvis_position` - Center of the pelvis in world space.
    /// `ground_height` - Height of the ground below the pelvis.
    /// `velocity` - Current horizontal movement velocity.
    /// `facing` - Unit vector indicating which way the character is facing.
    /// `dt` - Delta time in seconds.
    pub fn update(
        &mut self,
        pelvis_position: Point3<f32>,
        ground_height: f32,
        velocity: Vector3<f32>,
        facing: Vector3<f32>,
        dt: f32,
    ) {
        // Calculate horizontal speed (ignore vertical velocity)
        let horizontal_vel = Vector3::new(velocity.x, 0.0, velocity.z);
        let speed = horizontal_vel.magnitude();

        // Advance phase based on speed
        if speed > self.config.idle_threshold {
            self.walk_phase += speed * self.config.phase_rate * dt;
            self.walk_phase %= 1.0; // Wrap to [0, 1)
        }
        // When idle, phase freezes (feet stay in place)

        // Calculate the "right" vector (perpendicular to facing, in XZ plane)
        let right = facing.cross(&Vector3::y()).normalize();

        // Hip positions (offset laterally from pelvis)
        let left_hip = pelvis_position - right * self.config.hip_width;
        let right_hip = pelvis_position + right * self.config.hip_width;

        // Calculate stride direction (direction of movement, or facing if idle)
        let stride_dir = if speed > self.config.idle_threshold {
            horizontal_vel.normalize()
        } else {
            facing
        };

        // Calculate stride offsets from phase
        // Left foot: sin(phase * 2π) oscillates -1 to 1
        // Right foot: opposite phase (180° offset)
        let phase_angle = self.walk_phase * std::f32::consts::TAU;
        let left_stride = phase_angle.sin();
        let right_stride = -left_stride; // Opposite phase

        // Scale stride by speed (longer strides when moving faster)
        let stride_scale = (speed / 2.0).min(1.0); // Clamp to max stride at speed=2
        let stride_amount = self.config.stride_length * stride_scale;

        // Calculate foot forward/back offset along stride direction
        let left_offset = stride_dir * (left_stride * stride_amount);
        let right_offset = stride_dir * (right_stride * stride_amount);

        // Calculate step height (foot lifts when swinging forward)
        // Foot is "swinging" when moving from back to front (derivative of sin is positive)
        // cos(phase) > 0 means left foot is swinging
        let left_swing = phase_angle.cos().max(0.0);
        let right_swing = (-phase_angle).cos().max(0.0); // Opposite phase

        let left_lift = left_swing * self.config.step_height * stride_scale;
        let right_lift = right_swing * self.config.step_height * stride_scale;

        // Final foot positions
        self.left_foot = Point3::new(
            left_hip.x + left_offset.x,
            ground_height + left_lift,
            left_hip.z + left_offset.z,
        );
        self.right_foot = Point3::new(
            right_hip.x + right_offset.x,
            ground_height + right_lift,
            right_hip.z + right_offset.z,
        );
    }

    /// Get the current left foot position.
    pub fn left_foot(&self) -> Point3<f32> {
        self.left_foot
    }

    /// Get the current right foot position.
    pub fn right_foot(&self) -> Point3<f32> {
        self.right_foot
    }

    /// Calculate foot positions for airborne state.
    ///
    /// Returns (left_foot, right_foot) hanging below the pelvis.
    pub fn airborne_feet(
        &self,
        pelvis_position: Point3<f32>,
        leg_length: f32,
        facing: Vector3<f32>,
    ) -> (Point3<f32>, Point3<f32>) {
        let right = facing.cross(&Vector3::y()).normalize();

        let left_foot = Point3::new(
            pelvis_position.x - right.x * self.config.hip_width,
            pelvis_position.y - leg_length,
            pelvis_position.z - right.z * self.config.hip_width,
        );
        let right_foot = Point3::new(
            pelvis_position.x + right.x * self.config.hip_width,
            pelvis_position.y - leg_length,
            pelvis_position.z + right.z * self.config.hip_width,
        );

        (left_foot, right_foot)
    }
}

impl Default for GaitController {
    fn default() -> Self {
        Self::new(GaitConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_biped_creation() {
        let biped = SimpleBipedSkeleton::default();

        // Pelvis should be at origin + 1.0 y
        let pelvis = biped.pelvis_position();
        assert!((pelvis.x - 0.0).abs() < 0.001);
        assert!((pelvis.y - 1.0).abs() < 0.001);
        assert!((pelvis.z - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_biped_set_positions() {
        let mut biped = SimpleBipedSkeleton::default();

        // Set new positions
        biped.set_pelvis_position(Point3::new(1.0, 2.0, 3.0));
        biped.set_left_foot_position(Point3::new(0.8, 1.0, 3.0));
        biped.set_right_foot_position(Point3::new(1.2, 1.0, 3.0));

        // Verify positions are set directly (no physics)
        let pelvis = biped.pelvis_position();
        assert!((pelvis.x - 1.0).abs() < 0.001);
        assert!((pelvis.y - 2.0).abs() < 0.001);
        assert!((pelvis.z - 3.0).abs() < 0.001);

        let left = biped.left_foot_position();
        assert!((left.x - 0.8).abs() < 0.001);
        assert!((left.y - 1.0).abs() < 0.001);

        let right = biped.right_foot_position();
        assert!((right.x - 1.2).abs() < 0.001);
        assert!((right.y - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_step_controller_triggers_step() {
        let config = StepConfig {
            step_threshold: 0.2,
            step_speed: 10.0, // Fast for testing
            ..Default::default()
        };
        let mut controller = SimpleStepController::new(config, Point3::new(0.0, 1.0, 0.0), 0.0);

        // Move pelvis far enough to trigger a step
        let new_pelvis = Point3::new(0.5, 1.0, 0.0);
        controller.update(new_pelvis, 0.0, Vector3::new(1.0, 0.0, 0.0), 0.016);

        // One foot should be stepping
        assert!(controller.left_foot.is_stepping || controller.right_foot.is_stepping);
    }
}
