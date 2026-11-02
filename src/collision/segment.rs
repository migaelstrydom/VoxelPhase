//! Segment geometry utilities.
//!
//! Closest-point and distance queries on line segments, used by SAT edge-edge
//! contact generation and other collision routines.

use nalgebra::Point3;

/// Squared length below which a segment counts as a point, in m².
const EPS: f32 = 1e-6;

/// sin² of the angle below which two segments count as parallel.
const PARALLEL_SIN_SQ: f32 = 1e-6;

/// Closest point on the segment `a`–`b` to `p`; `a` for a segment of no
/// length.
pub fn closest_point_on_segment(p: Point3<f32>, a: Point3<f32>, b: Point3<f32>) -> Point3<f32> {
    let ab = b - a;
    let len_sq = ab.magnitude_squared();
    if len_sq < 1e-12 {
        return a;
    }
    let t = ((p - a).dot(&ab) / len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Closest points between two line segments.
///
/// Returns `(point_on_seg1, point_on_seg2)` — the pair of points, one on each
/// segment, that minimize the distance between them.
///
/// Handles degenerate cases where either or both segments collapse to a point.
pub fn segment_segment_closest_points(
    p1: Point3<f32>,
    q1: Point3<f32>,
    p2: Point3<f32>,
    q2: Point3<f32>,
) -> (Point3<f32>, Point3<f32>) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    // Both segments degenerate to points.
    if a <= EPS && e <= EPS {
        return (p1, p2);
    }

    // First segment degenerates to a point.
    if a <= EPS {
        let t = (f / e).clamp(0.0, 1.0);
        return (p1, p2 + d2 * t);
    }

    let c = d1.dot(&r);

    // Second segment degenerates to a point.
    if e <= EPS {
        let s = (-c / a).clamp(0.0, 1.0);
        return (p1 + d1 * s, p2);
    }

    // General case: both segments are non-degenerate. `denom` is
    // |d1|²|d2|² sin²θ, so the test for parallel segments is on sin²θ: an
    // absolute threshold on it would call two short segments parallel at
    // any angle (two 3 cm edges at right angles have denom 8e-7).
    let b = d1.dot(&d2);
    let denom = a * e - b * b;
    let mut s = if denom > PARALLEL_SIN_SQ * a * e {
        ((b * f - c * e) / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mut t = (b * s + f) / e;

    // Clamp t to [0,1] and recompute s if needed.
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }

    (p1 + d1 * s, p2 + d2 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    fn approx_eq_point(a: Point3<f32>, b: Point3<f32>, tol: f32) -> bool {
        (a - b).magnitude() < tol
    }

    /// Squared distance from a point to a line segment.
    pub fn point_segment_distance_sq(p: Point3<f32>, a: Point3<f32>, b: Point3<f32>) -> f32 {
        let ab = b - a;
        let ab_len_sq = ab.magnitude_squared();
        if ab_len_sq <= EPS {
            return (p - a).magnitude_squared();
        }
        let t = ((p - a).dot(&ab) / ab_len_sq).clamp(0.0, 1.0);
        let closest = a + ab * t;
        (p - closest).magnitude_squared()
    }

    /// Short segments crossing at right angles meet where they cross, not
    /// at an end, however short they are.
    #[test]
    fn short_crossing_segments_meet_where_they_cross() {
        let (on_first, on_second) = segment_segment_closest_points(
            Point3::new(-0.015, 0.0, 0.0),
            Point3::new(0.015, 0.0, 0.0),
            Point3::new(0.0, -0.015, 0.002),
            Point3::new(0.0, 0.015, 0.002),
        );
        assert!(
            approx_eq_point(on_first, Point3::origin(), 1e-6),
            "{on_first}"
        );
        assert!(
            approx_eq_point(on_second, Point3::new(0.0, 0.0, 0.002), 1e-6),
            "{on_second}"
        );
    }

    // --- point_segment_distance_sq ---

    #[test]
    fn point_on_segment() {
        let d = point_segment_distance_sq(
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        assert!(approx_eq(d, 0.0, 1e-6));
    }

    #[test]
    fn point_off_segment_midpoint() {
        let d = point_segment_distance_sq(
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        assert!(approx_eq(d, 1.0, 1e-6));
    }

    #[test]
    fn point_past_endpoint() {
        let d = point_segment_distance_sq(
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        assert!(approx_eq(d, 1.0, 1e-6));
    }

    #[test]
    fn degenerate_segment() {
        let d = point_segment_distance_sq(
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        assert!(approx_eq(d, 4.0, 1e-6));
    }

    // --- segment_segment_closest_points ---

    #[test]
    fn parallel_segments() {
        let (pa, pb) = segment_segment_closest_points(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        );
        // Parallel: should find some closest pair at distance 1.0
        let dist = (pa - pb).magnitude();
        assert!(approx_eq(dist, 1.0, 1e-5));
    }

    #[test]
    fn perpendicular_segments() {
        let (pa, pb) = segment_segment_closest_points(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, -1.0),
            Point3::new(0.5, 0.0, 1.0),
        );
        assert!(approx_eq_point(pa, Point3::new(0.5, 0.0, 0.0), 1e-5));
        assert!(approx_eq_point(pb, Point3::new(0.5, 0.0, 0.0), 1e-5));
    }

    #[test]
    fn skew_segments() {
        // Two segments offset in Y, crossing in projection.
        let (pa, pb) = segment_segment_closest_points(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, -1.0),
            Point3::new(1.0, 1.0, 1.0),
        );
        assert!(approx_eq_point(pa, Point3::new(1.0, 0.0, 0.0), 1e-5));
        assert!(approx_eq_point(pb, Point3::new(1.0, 1.0, 0.0), 1e-5));
    }

    #[test]
    fn both_degenerate() {
        let (pa, pb) = segment_segment_closest_points(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        );
        assert_eq!(pa, Point3::new(1.0, 0.0, 0.0));
        assert_eq!(pb, Point3::new(3.0, 0.0, 0.0));
    }

    #[test]
    fn one_degenerate() {
        let (pa, pb) = segment_segment_closest_points(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        );
        assert_eq!(pa, Point3::new(0.0, 0.0, 0.0));
        assert!(approx_eq_point(pb, Point3::new(0.0, 1.0, 0.0), 1e-5));
    }

    #[test]
    fn endpoint_clamping() {
        // Segments that would intersect if extended but don't overlap.
        let (pa, pb) = segment_segment_closest_points(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, -1.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
        );
        assert!(approx_eq_point(pa, Point3::new(1.0, 0.0, 0.0), 1e-5));
        assert!(approx_eq_point(pb, Point3::new(2.0, 0.0, 0.0), 1e-5));
    }
}
