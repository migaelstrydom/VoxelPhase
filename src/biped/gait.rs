//! Keyframe-based gait cycle for biped locomotion.
//!
//! Defines foot positions at key phases of the gait cycle and interpolates
//! between them based on the stride wheel angle.

use std::f32::consts::TAU;

use nalgebra::{Point3, Vector3};

/// Foot offset relative to hip in local coordinates.
#[derive(Debug, Clone, Copy)]
pub struct FootOffset {
    /// Lateral offset (positive = away from body center).
    pub lateral: f32,
    /// Vertical offset (positive = down from hip).
    pub vertical: f32,
    /// Forward offset (positive = in facing direction).
    pub forward: f32,
}

impl FootOffset {
    pub const fn new(lateral: f32, vertical: f32, forward: f32) -> Self {
        Self {
            lateral,
            vertical,
            forward,
        }
    }

    /// Linearly interpolate between two offsets.
    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        Self {
            lateral: self.lateral + (other.lateral - self.lateral) * t,
            vertical: self.vertical + (other.vertical - self.vertical) * t,
            forward: self.forward + (other.forward - self.forward) * t,
        }
    }

    /// Convert to world-space position given hip position and orientation.
    pub fn to_world(
        &self,
        hip: Point3<f32>,
        facing: Vector3<f32>,
        lateral_sign: f32,
    ) -> Point3<f32> {
        let right = facing.cross(&Vector3::y()).normalize();
        hip + right * self.lateral * lateral_sign - Vector3::y() * self.vertical
            + facing * self.forward
    }
}

/// A keyframe in the gait cycle.
#[derive(Debug, Clone, Copy)]
pub struct GaitKeyframe {
    /// Wheel angle for this keyframe [0, TAU).
    pub angle: f32,
    /// Foot position offset from hip.
    pub offset: FootOffset,
}

impl GaitKeyframe {
    pub const fn new(angle: f32, offset: FootOffset) -> Self {
        Self { angle, offset }
    }
}

/// A complete gait cycle definition.
///
/// Keyframes define foot positions at specific phases. The cycle interpolates
/// smoothly between keyframes as the stride wheel rotates.
#[derive(Debug, Clone)]
pub struct GaitCycle {
    /// Keyframes sorted by angle.
    keyframes: Vec<GaitKeyframe>,
}

impl GaitCycle {
    /// Create a gait cycle from keyframes.
    ///
    /// Keyframes will be sorted by angle automatically.
    pub fn new(mut keyframes: Vec<GaitKeyframe>) -> Self {
        keyframes.sort_by(|a, b| a.angle.partial_cmp(&b.angle).unwrap());
        Self { keyframes }
    }

    /// Sample the gait at a given angle, interpolating between keyframes.
    pub fn sample(&self, angle: f32) -> FootOffset {
        if self.keyframes.is_empty() {
            return FootOffset::new(0.0, 0.4, 0.0);
        }
        if self.keyframes.len() == 1 {
            return self.keyframes[0].offset;
        }

        let angle = angle.rem_euclid(TAU);

        // Find the two keyframes surrounding this angle
        let mut before_idx = 0;

        for (i, kf) in self.keyframes.iter().enumerate() {
            if kf.angle <= angle {
                before_idx = i;
            }
        }

        // After is the next keyframe (wrapping around)
        let after_idx = (before_idx + 1) % self.keyframes.len();

        let before = &self.keyframes[before_idx];
        let after = &self.keyframes[after_idx];

        // Calculate interpolation factor
        let t = if after.angle > before.angle {
            // Normal case: both keyframes in same cycle
            if after.angle == before.angle {
                0.0
            } else {
                (angle - before.angle) / (after.angle - before.angle)
            }
        } else {
            // Wraparound case: before is near end, after is near start
            let span = (TAU - before.angle) + after.angle;
            if span < 0.001 {
                0.0
            } else if angle >= before.angle {
                (angle - before.angle) / span
            } else {
                (TAU - before.angle + angle) / span
            }
        };

        // Smooth interpolation using smoothstep
        let t_smooth = t * t * (3.0 - 2.0 * t);

        before.offset.lerp(&after.offset, t_smooth)
    }

    /// Create a default walking gait cycle.
    pub fn walking(standing_height: f32, stride_length: f32, step_height: f32) -> Self {
        use std::f32::consts::{FRAC_PI_2, PI};

        // Keyframes around the wheel:
        // 0° = midstance (foot directly below hip, on ground)
        // 90° = toe-off (foot behind, leaving ground)
        // 180° = mid-swing (foot raised, passing under body)
        // 270° = heel-strike (foot forward, contacting ground)

        let half_stride = stride_length * 0.5;

        let keyframes = vec![
            // Midstance - foot directly below hip, fully planted
            GaitKeyframe::new(0.0, FootOffset::new(0.0, standing_height, 0.0)),
            // Late stance / push-off - foot behind, still on ground
            GaitKeyframe::new(
                FRAC_PI_2,
                FootOffset::new(0.0, standing_height, -half_stride),
            ),
            // Mid-swing - foot raised, moving forward
            GaitKeyframe::new(PI, FootOffset::new(0.0, standing_height - step_height, 0.0)),
            // Pre-contact / heel strike - foot forward, descending
            GaitKeyframe::new(
                PI + FRAC_PI_2,
                FootOffset::new(0.0, standing_height, half_stride),
            ),
        ];

        Self::new(keyframes)
    }

    /// Create an arm swing gait cycle.
    ///
    /// Arms swing opposite to legs - when the leg is back, the arm is forward.
    /// The swing creates a pendulum arc: hands come UP when swinging forward,
    /// and extend DOWN/BACK when swinging backward.
    pub fn arm_swing(arm_length: f32, swing_amplitude: f32) -> Self {
        use std::f32::consts::{FRAC_PI_2, PI};

        // Keyframes for arm swing (note: arms use OPPOSITE phase to legs)
        // 0° = arm at rest (neutral, hand hanging down)
        // 90° = arm fully back (hand behind and down)
        // 180° = arm at rest (passing through neutral)
        // 270° = arm fully forward (hand UP in front of torso)
        //
        // vertical = distance below shoulder (smaller = hand higher up)

        let keyframes = vec![
            // Neutral position - arm hanging down
            GaitKeyframe::new(0.0, FootOffset::new(0.0, arm_length, 0.0)),
            // Arm back - hand behind body and slightly down (extended back)
            GaitKeyframe::new(
                FRAC_PI_2,
                FootOffset::new(0.0, arm_length * 0.85, -swing_amplitude),
            ),
            // Neutral position - passing through
            GaitKeyframe::new(PI, FootOffset::new(0.0, arm_length, 0.0)),
            // Arm forward - hand UP in front of torso (theatrical forward swing)
            // Vertical is much smaller so hand comes up to chest/waist level
            GaitKeyframe::new(
                PI + FRAC_PI_2,
                FootOffset::new(0.0, arm_length * 0.01, swing_amplitude),
            ),
        ];

        Self::new(keyframes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_sample_at_keyframe() {
        let cycle = GaitCycle::walking(0.4, 0.3, 0.1);

        // At angle 0, should get the first keyframe's offset
        let offset = cycle.sample(0.0);
        assert!((offset.forward - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_sample_interpolation() {
        let cycle = GaitCycle::walking(0.4, 0.3, 0.1);

        // At PI (mid-swing), foot should be raised
        let offset = cycle.sample(PI);
        assert!(offset.vertical < 0.4); // Should be less than standing height
    }

    #[test]
    fn test_wraparound() {
        let cycle = GaitCycle::walking(0.4, 0.3, 0.1);

        // Sample just before TAU should interpolate toward angle 0
        let offset_before = cycle.sample(TAU - 0.1);
        let offset_after = cycle.sample(0.1);

        // Both should be close to the midstance position
        assert!((offset_before.forward).abs() < 0.1);
        assert!((offset_after.forward).abs() < 0.1);
    }
}
