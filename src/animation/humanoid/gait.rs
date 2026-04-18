//! Keyframe-based gait cycle for humanoid locomotion.
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
        keyframes.sort_by(|a, b| a.angle.total_cmp(&b.angle));
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
    /// Arms swing opposite to legs — forward peak curves INWARD toward a
    /// point in front of the pec; back peak extends straight behind near
    /// the hip. Peak hand height stays at pec level (never reaches the
    /// shoulder).
    ///
    /// `shoulder_width` caps the inward component so the hand can't cross
    /// the body midline. Inward amplitude scales with `swing_amplitude` so
    /// wide-armed gaits (sprint) keep a proportional curve and narrow ones
    /// (crouch) shrink naturally.
    pub fn arm_swing(arm_length: f32, swing_amplitude: f32, shoulder_width: f32) -> Self {
        use std::f32::consts::{FRAC_PI_2, PI};

        // Lateral component in the `FootOffset` frame is applied with
        // `lateral_sign` in `to_world`: for the right arm (sign +1) a
        // NEGATIVE lateral pulls the hand toward the body midline, and
        // likewise for the left arm (sign -1). So we store the inward
        // pull as a negative lateral value, once, and it mirrors per-side.
        let inward = -(swing_amplitude * 0.7).min(shoulder_width * 0.8);

        // Forward peak: hand must stay ≥ arm_length * 0.4 below shoulder
        // so it never rises above pec height.
        let fwd_down = arm_length * 0.3;
        // Back peak: hand sits near the hip — deeper below the shoulder.
        let back_down = arm_length * 0.75;
        let back_reach = swing_amplitude * 0.5;

        let keyframes = vec![
            // Neutral (rest) — arm hanging straight.
            GaitKeyframe::new(0.0, FootOffset::new(0.0, arm_length, 0.0)),
            // Back peak — hand behind and low, no inward curve.
            GaitKeyframe::new(FRAC_PI_2, FootOffset::new(0.0, back_down, -back_reach)),
            // Neutral (rest) — passing through hanging.
            GaitKeyframe::new(PI, FootOffset::new(0.0, arm_length, 0.0)),
            // Forward peak — hand in front of the pec (inward + pec-height).
            GaitKeyframe::new(
                PI + FRAC_PI_2,
                FootOffset::new(inward, fwd_down, swing_amplitude),
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
