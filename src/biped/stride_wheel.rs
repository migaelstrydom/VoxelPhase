//! Stride wheel gait system for biped locomotion.
//!
//! The stride wheel is an invisible wheel that rotates as the biped moves.
//! The wheel angle parameterizes the gait cycle, driving keyframe interpolation.

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use nalgebra::{Point3, Vector3};

use super::config::BipedConfig;
use super::gait::GaitCycle;
use super::state::{BipedState, FootState};

/// Phase offset for the left foot.
pub const LEFT_PHASE: f32 = 0.0;
/// Phase offset for the right foot (opposite side of cycle).
pub const RIGHT_PHASE: f32 = PI;

/// Advance wheel based on distance traveled.
pub fn advance_wheel(wheel_angle: &mut f32, speed: f32, dt: f32, radius: f32) {
    if speed > 0.0 && radius > 0.0 {
        *wheel_angle = (*wheel_angle + (speed * dt) / radius).rem_euclid(TAU);
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
pub fn handle_idle(state: &mut BipedState, _config: &BipedConfig) {
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
