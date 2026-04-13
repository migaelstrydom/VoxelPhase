//! Ray-triangle intersection using the Möller–Trumbore algorithm.

use nalgebra::{Point3, Vector3};

use super::triangle::Triangle;

/// Result of a ray-triangle intersection test.
#[derive(Debug, Clone, Copy)]
pub struct RayHit {
    /// Parametric distance along the ray (hit point = origin + t * direction).
    pub t: f32,
    /// World-space hit point on the triangle surface.
    pub point: Point3<f32>,
    /// Triangle face normal (unit length, same winding as the triangle).
    pub normal: Vector3<f32>,
}

/// Test whether a ray intersects a triangle using the Möller–Trumbore algorithm.
///
/// Returns the intersection if `t` falls within `[0, max_t]`.
/// Back-face hits are included (the caller can check the normal dot direction).
pub fn ray_triangle(
    origin: Point3<f32>,
    direction: Vector3<f32>,
    triangle: &Triangle,
    max_t: f32,
) -> Option<RayHit> {
    const EPSILON: f32 = 1e-7;

    let e1 = triangle.v1 - triangle.v0;
    let e2 = triangle.v2 - triangle.v0;
    let h = direction.cross(&e2);
    let det = e1.dot(&h);

    // Ray is parallel to the triangle.
    if det.abs() < EPSILON {
        return None;
    }

    let inv_det = 1.0 / det;
    let s = origin - triangle.v0;
    let u = inv_det * s.dot(&h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(&e1);
    let v = inv_det * direction.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = inv_det * e2.dot(&q);
    if t < 0.0 || t > max_t {
        return None;
    }

    let point = origin + direction * t;
    let normal = triangle.normal();

    Some(RayHit { t, point, normal })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn floor_triangle() -> Triangle {
        Triangle::new(
            Point3::new(-1.0, 0.0, -1.0),
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 1.0),
        )
    }

    #[test]
    fn hit_from_above() {
        let tri = floor_triangle();
        let hit = ray_triangle(
            Point3::new(0.0, 5.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            &tri,
            f32::MAX,
        );
        let hit = hit.expect("should hit");
        assert!((hit.t - 5.0).abs() < 1e-4);
        assert!((hit.point.y).abs() < 1e-4);
    }

    #[test]
    fn miss_outside_triangle() {
        let tri = floor_triangle();
        let hit = ray_triangle(
            Point3::new(5.0, 5.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            &tri,
            f32::MAX,
        );
        assert!(hit.is_none());
    }

    #[test]
    fn respects_max_t() {
        let tri = floor_triangle();
        let hit = ray_triangle(
            Point3::new(0.0, 5.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            &tri,
            3.0,
        );
        assert!(hit.is_none(), "hit at t=5 should be beyond max_t=3");
    }

    #[test]
    fn hit_from_below() {
        let tri = floor_triangle();
        let hit = ray_triangle(
            Point3::new(0.0, -5.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            &tri,
            f32::MAX,
        );
        assert!(hit.is_some(), "back-face hits should be returned");
    }

    #[test]
    fn parallel_ray_misses() {
        let tri = floor_triangle();
        let hit = ray_triangle(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            &tri,
            f32::MAX,
        );
        assert!(hit.is_none());
    }
}
