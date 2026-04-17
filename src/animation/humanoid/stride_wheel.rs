//! Stride wheel gait system for humanoid locomotion.
//!
//! The stride wheel is an invisible wheel that rotates as the character moves.
//! The wheel angle parameterizes the gait cycle, driving keyframe interpolation.

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use nalgebra::{Point3, Vector2, Vector3};

use super::gait::GaitCycle;
use crate::animation::config::CharacterRigConfig;
use crate::animation::state::{AnimationState, FootState, HandState};

/// Phase offset for the left foot.
pub const LEFT_PHASE: f32 = 0.0;
/// Phase offset for the right foot (opposite side of cycle).
pub const RIGHT_PHASE: f32 = PI;

/// Advance wheel based on distance traveled.
pub fn advance_wheel(wheel_angle: &mut f32, speed: f32, dt: f32, radius: f32) {
    if speed > 0.0 && radius > 0.0 {
        *wheel_angle = (*wheel_angle + 2.0 * (speed * dt) / radius).rem_euclid(TAU);
    }
}

/// Check if foot is in swing phase (top half of wheel cycle).
pub fn is_swinging(wheel_angle: f32, phase_offset: f32) -> bool {
    let foot_angle = (wheel_angle + phase_offset).rem_euclid(TAU);
    foot_angle > FRAC_PI_2 && foot_angle < 3.0 * FRAC_PI_2
}

/// Update foot position based on gait cycle and wheel state.
pub fn update_foot(
    foot: &mut FootState,
    gait: &GaitCycle,
    wheel_angle: f32,
    phase_offset: f32,
    hip: Point3<f32>,
    facing: Vector3<f32>,
    lateral_sign: f32,
) {
    let foot_angle = (wheel_angle + phase_offset).rem_euclid(TAU);

    // Sample the gait cycle and pin foot directly to target
    let offset = gait.sample(foot_angle);
    foot.position = offset.to_world(hip, facing, lateral_sign);
}

/// Handle idle state - plant both feet at ground contacts.
pub fn handle_idle(state: &mut AnimationState, _config: &CharacterRigConfig) {
    let snap_threshold = 0.03;

    // Compute desired positions from ground contacts or current positions
    let left_desired = state.left.ground_contact.unwrap_or(state.left.position);
    let right_desired = state.right.ground_contact.unwrap_or(state.right.position);

    // Only update if significantly different (prevents jitter)
    let left_delta = (left_desired - state.left.planted_position).magnitude();
    let right_delta = (right_desired - state.right.planted_position).magnitude();

    if left_delta > snap_threshold {
        state.left.planted_position = left_desired;
    }
    if right_delta > snap_threshold {
        state.right.planted_position = right_desired;
    }

    // Snap feet to planted positions
    state.left.position = state.left.planted_position;
    state.right.position = state.right.planted_position;

    // Update normals from ground contact
    if let Some(normal) = state.left.ground_normal {
        state.left.normal = normal;
    }
    if let Some(normal) = state.right.ground_normal {
        state.right.normal = normal;
    }

    // Reset wheel angle
    state.wheel_angle = 0.0;
}

/// Update hand position from arm gait cycle.
///
/// Arms swing OPPOSITE to legs for natural counter-balance:
/// - Left arm uses RIGHT_PHASE (swings forward when right leg steps)
/// - Right arm uses LEFT_PHASE (swings forward when left leg steps)
pub fn update_hand(
    hand: &mut HandState,
    gait: &GaitCycle,
    wheel_angle: f32,
    phase_offset: f32,
    shoulder: Point3<f32>,
    facing: Vector3<f32>,
    lateral_sign: f32,
) {
    let hand_angle = (wheel_angle + phase_offset).rem_euclid(TAU);

    // Sample the arm gait cycle
    let offset = gait.sample(hand_angle);
    hand.position = offset.to_world(shoulder, facing, lateral_sign);
}

/// Compute shoulder twist angle from wheel angle.
///
/// Shoulders twist opposite to hip movement, creating natural torso rotation.
/// The twist is sinusoidal, 90° out of phase with the leg stride.
pub fn compute_shoulder_twist(wheel_angle: f32, max_twist: f32) -> f32 {
    // When left leg is forward (wheel_angle near 3π/2), left shoulder should be back
    // This is achieved by making twist proportional to sin(wheel_angle + π/2)
    (wheel_angle + FRAC_PI_2).sin() * max_twist
}

/// Compute head tilt from movement velocity.
///
/// Returns (forward_tilt, lateral_tilt) for velocity anticipation effect.
pub fn compute_head_tilt(
    velocity: Vector3<f32>,
    facing: Vector3<f32>,
    tilt_factor: f32,
) -> Vector2<f32> {
    let speed = velocity.magnitude();
    if speed < 0.01 {
        return Vector2::new(0.0, 0.0);
    }

    // Forward tilt based on forward velocity component
    let forward_speed = velocity.dot(&facing);
    let forward_tilt = (forward_speed * tilt_factor).clamp(-0.2, 0.2);

    // Lateral tilt based on sideways velocity component
    let right = facing.cross(&Vector3::y()).normalize();
    let lateral_speed = velocity.dot(&right);
    let lateral_tilt = (lateral_speed * tilt_factor * 0.5).clamp(-0.1, 0.1);

    Vector2::new(forward_tilt, lateral_tilt)
}

/// Compute head bob offset from wheel angle.
///
/// Creates a double-bounce per stride cycle (once per foot strike).
pub fn compute_head_bob(wheel_angle: f32, amplitude: f32) -> f32 {
    // Two bounces per cycle (2x frequency), always positive (downward)
    // Using abs of cosine gives peaks at 0, π/2, π, 3π/2
    (wheel_angle * 1.0).cos().abs() * amplitude
}
