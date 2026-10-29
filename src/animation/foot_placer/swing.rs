//! Swing-leg trajectory.
//!
//! Stage 1 uses a simple parabolic arc with a smoothed horizontal
//! progress (smoothstep). Minimum-jerk trajectories are a refinement
//! earmarked for Stage 3.

use nalgebra::Point3;

/// Interpolate a swinging foot along an arc from `from` to `to`. `t` is
/// normalised progress in `[0, 1]`. `peak_lift` is the maximum vertical
/// rise above the straight-line interpolation midpoint.
pub fn swing_position(from: Point3<f32>, to: Point3<f32>, t: f32, peak_lift: f32) -> Point3<f32> {
    let t = t.clamp(0.0, 1.0);
    // Smoothstep horizontal progress so lift-off and plant are eased.
    let s = t * t * (3.0 - 2.0 * t);
    let x = from.x + (to.x - from.x) * s;
    let z = from.z + (to.z - from.z) * s;
    let base_y = from.y + (to.y - from.y) * s;
    // Parabolic lift: peak at t = 0.5, zero at endpoints.
    let lift = peak_lift * 4.0 * t * (1.0 - t);
    Point3::new(x, base_y + lift, z)
}

/// The lift for a swing re-based mid-arc, starting again from the foot's
/// `position` onto `to`, that climbs no higher than the arc it replaces
/// (`from` onto `old_to` with `peak_lift`). An arc rises at most its lift
/// above its higher end, and that ceiling is kept: a full lift on top of
/// the height already gained, re-based frame after frame as a plan changes
/// its mind, ratchets the foot up past the hip.
pub fn rebased_lift(
    from: Point3<f32>,
    old_to: Point3<f32>,
    peak_lift: f32,
    position: Point3<f32>,
    to: Point3<f32>,
) -> f32 {
    let ceiling = from.y.max(old_to.y) + peak_lift;
    (ceiling - position.y.max(to.y)).clamp(0.0, peak_lift)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_swing_rebased_every_frame_climbs_no_higher_than_its_arc() {
        let (to, peak_lift) = (Point3::new(0.0, 0.0, 1.0), 0.4);
        let mut from = Point3::origin();
        let mut lift = peak_lift;
        let ceiling = peak_lift;
        let mut position = swing_position(from, to, 0.1, lift);
        let mut highest = position.y;
        for _ in 0..30 {
            lift = rebased_lift(from, to, lift, position, to);
            from = position;
            position = swing_position(from, to, 0.1, lift);
            highest = highest.max(position.y);
        }
        assert!(
            highest <= ceiling + 1.0e-5,
            "the foot climbed to {highest:.2} m"
        );
    }
}
