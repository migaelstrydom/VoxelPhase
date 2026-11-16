//! The last word on where a foot is drawn: no faster against its hip than a
//! foot moves.
//!
//! Everything upstream decides where a foot *belongs*: the gait, a pose, a
//! blend between two of them, a landing. Each of those may change its mind
//! in a frame (a replant, a pose taking the feet over), and the foot it
//! draws then jumps half a stride. This is the one place that says a drawn
//! foot gets from where it was to where it belongs by moving there:
//!
//! ```text
//!   asked foot ──► against its hip, ──► at most FASTEST · dt from where
//!                  as the body faces     it was drawn last frame ──► drawn foot
//! ```
//!
//! A stride's own swing is slower than the limit and passes untouched; a
//! jump is drawn as two or three frames of a very fast step. The feet are
//! measured as the body faces, so a body that spins takes them round with
//! it at whatever rate it spins.

use nalgebra::{Point3, Vector3};

/// The fastest a foot is drawn moving against its hip, in m/s: a sprint's
/// swing passes the hip at about 12.
const FASTEST: f32 = 14.0;

/// Holds each drawn foot to the fastest a foot moves against its hip.
#[derive(Clone, Copy, Debug, Default)]
pub struct FootSlew {
    /// Where each foot was last drawn, from its hip, as the body faced
    /// then: to its right, up and ahead. `None` before the first frame.
    drawn: Option<[Vector3<f32>; 2]>,
}

impl FootSlew {
    /// Where to draw `feet` this frame, `dt` seconds after the last, as near
    /// to where they are asked for as feet moving from where they were
    /// drawn get. `hips` are the hips they hang from this frame, on a body
    /// that faces along `facing` (horizontal, unit length).
    pub fn limit(
        &mut self,
        hips: [Point3<f32>; 2],
        feet: [Point3<f32>; 2],
        facing: Vector3<f32>,
        dt: f32,
    ) -> [Point3<f32>; 2] {
        let right = facing.cross(&Vector3::y());
        let as_faced = |v: Vector3<f32>| Vector3::new(v.dot(&right), v.y, v.dot(&facing));
        let in_world = |v: Vector3<f32>| right * v.x + Vector3::y() * v.y + facing * v.z;
        let asked = [as_faced(feet[0] - hips[0]), as_faced(feet[1] - hips[1])];
        let furthest = FASTEST * dt.max(0.0);
        let mut drawn = feet;
        let mut from_hips = asked;
        if let Some(last) = self.drawn {
            for foot in 0..2 {
                let step = asked[foot] - last[foot];
                let length = step.norm();
                if length > furthest {
                    from_hips[foot] = last[foot] + step * (furthest / length);
                    drawn[foot] = hips[foot] + in_world(from_hips[foot]);
                }
            }
        }
        self.drawn = Some(from_hips);
        drawn
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    /// A body that spins on the spot with its feet turning with it, half a
    /// turn in an eighth of a second, moves them fast across the world and
    /// not at all against itself: they are drawn where they are asked for.
    #[test]
    fn feet_go_round_with_a_spinning_body_at_any_rate() {
        let mut slew = FootSlew::default();
        let pelvis = Point3::new(0.0, 0.45, 0.0);
        for frame in 0..9 {
            let yaw = std::f32::consts::PI * frame as f32 / 8.0;
            let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());
            let right = facing.cross(&Vector3::y());
            let hips = [pelvis - right * 0.12, pelvis + right * 0.12];
            // A stride's stance, turning with the body.
            let feet = [
                hips[0] + facing * 0.35 - Vector3::y() * 0.43,
                hips[1] - facing * 0.35 - Vector3::y() * 0.43,
            ];
            let drawn = slew.limit(hips, feet, facing, DT);
            for foot in 0..2 {
                assert!(
                    (drawn[foot] - feet[foot]).norm() < 1e-5,
                    "frame {frame}: held back {:.2} m",
                    (drawn[foot] - feet[foot]).norm()
                );
            }
        }
    }

    fn hips(at: f32) -> [Point3<f32>; 2] {
        [Point3::new(0.12, 0.45, at), Point3::new(-0.12, 0.45, at)]
    }

    /// A planted foot under a sprinting body and a swing passing the hip at
    /// a sprint's speed are drawn where they are asked for.
    #[test]
    fn a_strides_feet_are_drawn_where_they_are_asked() {
        let mut slew = FootSlew::default();
        let planted = Point3::new(0.12, 0.0, 0.3);
        for frame in 0..30 {
            let travelled = 8.0 * DT * frame as f32;
            // The swing foot moves at 20 m/s over the ground: 12 against a
            // hip moving at 8.
            let swinging = Point3::new(-0.12, 0.1, -0.3 + 20.0 * DT * frame as f32);
            let drawn = slew.limit(hips(travelled), [planted, swinging], Vector3::z(), DT);
            assert_eq!(drawn, [planted, swinging], "frame {frame}");
        }
    }

    /// A foot asked for half a stride away in one frame gets there over the
    /// next few, and is then drawn where it is asked again.
    #[test]
    fn a_foot_that_jumps_is_drawn_stepping_there() {
        let mut slew = FootSlew::default();
        let (under, ahead) = (Point3::new(0.12, 0.0, 0.0), Point3::new(0.12, 0.0, 0.5));
        let still = Point3::new(-0.12, 0.0, 0.0);
        slew.limit(hips(0.0), [under, still], Vector3::z(), DT);

        let mut last = under;
        let mut frames = 0;
        loop {
            let drawn = slew.limit(hips(0.0), [ahead, still], Vector3::z(), DT);
            assert_eq!(drawn[1], still);
            let moved = (drawn[0] - last).norm();
            assert!(moved <= FASTEST * DT + 1e-5, "moved {moved} m in a frame");
            last = drawn[0];
            frames += 1;
            if (drawn[0] - ahead).norm() < 1e-5 {
                break;
            }
            assert!(frames < 10, "never arrived");
        }
        assert_eq!(frames, 3);
    }
}
