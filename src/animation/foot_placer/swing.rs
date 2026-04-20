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
