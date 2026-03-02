//! ConvexSupport trait and per-shape implementations.
//!
//! The support function is the core building block for GJK, EPA, and conservative
//! advancement. Every convex shape implements it to get immediate access to all
//! general-purpose collision algorithms.

use nalgebra::{Point3, Vector3};

use super::capsule::Capsule;
use super::obb::Obb;
use super::sphere_triangle::Triangle;

/// The furthest point on a convex shape in a given direction.
#[allow(unused)]
pub trait ConvexSupport {
    /// Returns the point on the shape's surface that is furthest along `direction`
    /// (world space). Used by GJK, EPA, and conservative advancement.
    fn support(&self, direction: Vector3<f32>) -> Point3<f32>;

    /// Upper bound on the shape's radius from its local origin.
    /// Used by conservative advancement to bound rotational surface speed.
    fn bounding_radius(&self) -> f32;
}

/// A sphere in world space for support function evaluation.
#[allow(unused)]
pub struct SupportSphere {
    pub center: Point3<f32>,
    pub radius: f32,
}

impl ConvexSupport for SupportSphere {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        let len = direction.magnitude();
        if len < 1e-10 {
            return self.center;
        }
        self.center + direction * (self.radius / len)
    }

    fn bounding_radius(&self) -> f32 {
        self.radius
    }
}

impl ConvexSupport for Obb {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        let rot = self.rotation.to_rotation_matrix();
        let local_dir = rot.inverse() * direction;

        // Sign-flip of half_extents: each component takes the sign of the local direction.
        // Uses >= 0.0 instead of copysign to avoid -0.0 issues from negated vectors.
        let local_support = Vector3::new(
            if local_dir.x >= 0.0 {
                self.half_extents.x
            } else {
                -self.half_extents.x
            },
            if local_dir.y >= 0.0 {
                self.half_extents.y
            } else {
                -self.half_extents.y
            },
            if local_dir.z >= 0.0 {
                self.half_extents.z
            } else {
                -self.half_extents.z
            },
        );

        self.center + rot * local_support
    }

    fn bounding_radius(&self) -> f32 {
        self.half_extents.norm()
    }
}

impl ConvexSupport for Capsule {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        let len = direction.magnitude();
        if len < 1e-10 {
            return self.center;
        }
        let (a, b) = self.segment_endpoints();
        // Pick the segment endpoint furthest along direction.
        let endpoint = if direction.dot(&(b - a)) >= 0.0 {
            b
        } else {
            a
        };
        // Inflate by radius in the given direction.
        endpoint + direction * (self.radius / len)
    }

    fn bounding_radius(&self) -> f32 {
        self.half_height
    }
}

impl ConvexSupport for Triangle {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        let d0 = (self.v0.coords).dot(&direction);
        let d1 = (self.v1.coords).dot(&direction);
        let d2 = (self.v2.coords).dot(&direction);

        if d0 >= d1 && d0 >= d2 {
            self.v0
        } else if d1 >= d2 {
            self.v1
        } else {
            self.v2
        }
    }

    fn bounding_radius(&self) -> f32 {
        let center = Point3::from((self.v0.coords + self.v1.coords + self.v2.coords) / 3.0);
        let r0 = (self.v0 - center).magnitude();
        let r1 = (self.v1 - center).magnitude();
        let r2 = (self.v2 - center).magnitude();
        r0.max(r1).max(r2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;
    use std::f32::consts::FRAC_1_SQRT_2;

    fn approx_eq_point(a: Point3<f32>, b: Point3<f32>, tol: f32) -> bool {
        (a - b).magnitude() < tol
    }

    // --- Sphere support ---

    #[test]
    fn sphere_support_positive_x() {
        let s = SupportSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let p = s.support(Vector3::x());
        assert!(approx_eq_point(p, Point3::new(1.0, 0.0, 0.0), 1e-6));
    }

    #[test]
    fn sphere_support_negative_y() {
        let s = SupportSphere {
            center: Point3::new(0.0, 5.0, 0.0),
            radius: 2.0,
        };
        let p = s.support(-Vector3::y());
        assert!(approx_eq_point(p, Point3::new(0.0, 3.0, 0.0), 1e-6));
    }

    #[test]
    fn sphere_support_diagonal() {
        let s = SupportSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let dir = Vector3::new(1.0, 1.0, 0.0);
        let p = s.support(dir);
        let expected = Point3::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0);
        assert!(approx_eq_point(p, expected, 1e-6));
    }

    #[test]
    fn sphere_support_zero_direction() {
        let s = SupportSphere {
            center: Point3::new(1.0, 2.0, 3.0),
            radius: 1.0,
        };
        let p = s.support(Vector3::zeros());
        assert_eq!(p, s.center);
    }

    #[test]
    fn sphere_bounding_radius() {
        let s = SupportSphere {
            center: Point3::origin(),
            radius: 3.5,
        };
        assert_eq!(s.bounding_radius(), 3.5);
    }

    // --- OBB support ---

    #[test]
    fn obb_support_axis_aligned() {
        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(2.0, 3.0, 4.0),
        );
        let p = obb.support(Vector3::x());
        assert!(approx_eq_point(p, Point3::new(2.0, 3.0, 4.0), 1e-6));
    }

    #[test]
    fn obb_support_negative_axis() {
        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(2.0, 3.0, 4.0),
        );
        let p = obb.support(-Vector3::y());
        assert!(approx_eq_point(p, Point3::new(2.0, -3.0, 4.0), 1e-6));
    }

    #[test]
    fn obb_support_rotated_90_y() {
        // 90° rotation around Y: local X → world Z, local Z → world -X.
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::y()),
            std::f32::consts::FRAC_PI_2,
        );
        let obb = Obb::new(Point3::origin(), rot, Vector3::new(1.0, 1.0, 1.0));

        // Support in world +X should be local -Z corner → world +X.
        let p = obb.support(Vector3::x());
        // After 90° Y rotation: max X comes from local -Z = -1 rotated → +X.
        assert!(p.x > 0.9, "Expected positive X, got {:?}", p);
    }

    #[test]
    fn obb_support_with_offset_center() {
        let obb = Obb::new(
            Point3::new(10.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let p = obb.support(Vector3::x());
        assert!(approx_eq_point(p, Point3::new(11.0, 1.0, 1.0), 1e-6));
    }

    #[test]
    fn obb_bounding_radius() {
        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(3.0, 4.0, 0.0),
        );
        assert!((obb.bounding_radius() - 5.0).abs() < 1e-6);
    }

    // --- Triangle support ---

    #[test]
    fn triangle_support_picks_furthest_vertex() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(2.0, 3.0, 0.0),
        );
        let p = tri.support(Vector3::x());
        assert_eq!(p, Point3::new(5.0, 0.0, 0.0));
    }

    #[test]
    fn triangle_support_negative_direction() {
        let tri = Triangle::new(
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(2.0, 3.0, 0.0),
        );
        let p = tri.support(-Vector3::x());
        assert_eq!(p, Point3::new(-1.0, 0.0, 0.0));
    }

    #[test]
    fn triangle_support_y_direction() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(2.0, 10.0, 0.0),
        );
        let p = tri.support(Vector3::y());
        assert_eq!(p, Point3::new(2.0, 10.0, 0.0));
    }

    #[test]
    fn triangle_bounding_radius() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        );
        let r = tri.bounding_radius();
        // Centroid at (1, 4/3, 0), farthest vertex is (3,0,0) at distance ~2.4
        assert!(r > 2.0);
        assert!(r < 3.0);
    }
}
