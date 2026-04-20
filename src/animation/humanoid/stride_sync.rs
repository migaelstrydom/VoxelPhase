//! Stride phase synchronisation.
//!
//! The stride phase is a `[0, TAU)` scalar that parameterises upper-body
//! stylistic channels (arm swing, shoulder twist, head bob) against the
//! lower body's stepping cycle. It's derived directly from the foot
//! placer so arm swing tracks real step events, not a wheel driven by
//! speed.
//!
//! Convention matches the retired stride-wheel:
//! - phase ∈ `[0, PI]` while the **right** foot is swinging
//!   (left foot planted; right foot in flight)
//! - phase ∈ `[PI, TAU]` while the **left** foot is swinging
//! - when both feet are planted (idle, or both completed this tick) the
//!   phase holds its previous value so arms coast through the transition
//!   instead of snapping to rest.

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use nalgebra::{Point3, Vector2, Vector3};

use super::gait::GaitCycle;
use crate::animation::foot_placer::{FootPhase, FootPlacer};
use crate::animation::state::HandState;

/// Phase offset for the left foot/hand within the stride cycle.
pub const LEFT_PHASE: f32 = 0.0;
/// Phase offset for the right foot/hand — opposite side of the cycle.
pub const RIGHT_PHASE: f32 = PI;

/// Derive the current stride phase from the placer's per-foot state.
///
/// `prev_phase` is returned when no foot is currently stepping so the
/// phase doesn't snap to zero at idle, or in the single-frame window
/// where both feet completed their steps on the same tick.
pub fn phase_from_placer(placer: &FootPlacer, prev_phase: f32) -> f32 {
    match (placer.left.phase, placer.right.phase) {
        (FootPhase::Stepping { t, duration, .. }, _) => {
            let u = (t / duration.max(1e-4)).clamp(0.0, 1.0);
            (PI + PI * u).rem_euclid(TAU)
        }
        (_, FootPhase::Stepping { t, duration, .. }) => {
            let u = (t / duration.max(1e-4)).clamp(0.0, 1.0);
            (PI * u).rem_euclid(TAU)
        }
        _ => prev_phase,
    }
}

/// Update hand position from arm gait cycle.
///
/// Arms swing OPPOSITE to legs for natural counter-balance:
/// - Left arm uses RIGHT_PHASE (swings forward when right leg steps)
/// - Right arm uses LEFT_PHASE (swings forward when left leg steps)
pub fn update_hand(
    hand: &mut HandState,
    gait: &GaitCycle,
    stride_phase: f32,
    phase_offset: f32,
    shoulder: Point3<f32>,
    facing: Vector3<f32>,
    lateral_sign: f32,
) {
    let hand_angle = (stride_phase + phase_offset).rem_euclid(TAU);
    let offset = gait.sample(hand_angle);
    hand.position = offset.to_world(shoulder, facing, lateral_sign);
}

/// Compute shoulder twist angle from stride phase.
///
/// Shoulders twist opposite to the hips, creating natural torso rotation.
/// The twist is sinusoidal, 90° out of phase with the leg stride.
pub fn compute_shoulder_twist(stride_phase: f32, max_twist: f32) -> f32 {
    (stride_phase + FRAC_PI_2).sin() * max_twist
}

/// Compute head tilt from movement velocity.
///
/// Returns `(forward_tilt, lateral_tilt)` for a velocity-anticipation
/// effect. Independent of stride phase — pure velocity-driven.
pub fn compute_head_tilt(
    velocity: Vector3<f32>,
    facing: Vector3<f32>,
    tilt_factor: f32,
) -> Vector2<f32> {
    let speed = velocity.magnitude();
    if speed < 0.01 {
        return Vector2::new(0.0, 0.0);
    }

    let forward_speed = velocity.dot(&facing);
    let forward_tilt = (forward_speed * tilt_factor).clamp(-0.2, 0.2);

    let right = facing.cross(&Vector3::y()).normalize();
    let lateral_speed = velocity.dot(&right);
    let lateral_tilt = (lateral_speed * tilt_factor * 0.5).clamp(-0.1, 0.1);

    Vector2::new(forward_tilt, lateral_tilt)
}

/// Compute head bob offset from stride phase.
///
/// One bob per full stride cycle: trough at phase `0`/`TAU`, peak at
/// phase `PI`. Returns a value in `[0, amplitude]`.
pub fn compute_head_bob(stride_phase: f32, amplitude: f32) -> f32 {
    (1.0 - stride_phase.cos()) * 0.5 * amplitude
}
