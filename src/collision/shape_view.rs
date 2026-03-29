//! ShapeView: world-space snapshot of a convex shape for collision dispatch.
//!
//! Also defines `SupportFace` and `SupportFaceExtractor` for manifold clipping
//! (used by GJK/EPA manifold generation).

use nalgebra::{Point3, UnitQuaternion, Vector3};
use smallvec::SmallVec;

use super::capsule::Capsule;
use super::discrete::clipping::obb_face;
use super::obb::Obb;
use super::support::{ConvexSupport, SupportSphere};
use super::AABB;
use crate::physics::ColliderShape;

/// World-space snapshot of a convex shape for collision dispatch.
///
/// Constructed from ColliderState (dynamic pairs) or from Collider + body
/// transform (static pairs). The dispatch layer reads the shape enum to
/// select the algorithm; GJK/EPA reads center/rotation/shape to evaluate
/// the support function.
pub struct ShapeView<'a> {
    pub center: Point3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub shape: &'a ColliderShape,
}

impl ShapeView<'_> {
    /// Compute the world-space AABB of this shape, expanded by margin.
    pub fn query_aabb(&self, margin: f32) -> AABB {
        match self.shape {
            ColliderShape::Sphere { radius } => {
                let r = radius + margin;
                AABB::new(
                    Point3::new(self.center.x - r, self.center.y - r, self.center.z - r),
                    Point3::new(self.center.x + r, self.center.y + r, self.center.z + r),
                )
            }
            ColliderShape::Box { half_extents } => {
                let obb = Obb::new(self.center, self.rotation, *half_extents);
                let (aabb_min, aabb_max) = obb.enclosing_aabb();
                let margin_vec = Vector3::new(margin, margin, margin);
                AABB::new(aabb_min - margin_vec, aabb_max + margin_vec)
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                let capsule = Capsule::new(self.center, self.rotation, *half_height, *radius);
                let (aabb_min, aabb_max) = capsule.enclosing_aabb();
                let margin_vec = Vector3::new(margin, margin, margin);
                AABB::new(aabb_min - margin_vec, aabb_max + margin_vec)
            }
        }
    }
}

impl ConvexSupport for ShapeView<'_> {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        match self.shape {
            ColliderShape::Sphere { radius } => {
                SupportSphere {
                    center: self.center,
                    radius: *radius,
                }
                .support(direction)
            }
            ColliderShape::Box { half_extents } => {
                Obb::new(self.center, self.rotation, *half_extents).support(direction)
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                Capsule::new(self.center, self.rotation, *half_height, *radius).support(direction)
            }
        }
    }

    fn bounding_radius(&self) -> f32 {
        self.shape.bounding_radius()
    }
}

/// Polygonal face of a convex shape, used for contact manifold clipping.
pub struct SupportFace {
    /// Vertices of the face polygon (CCW winding from outside).
    pub vertices: SmallVec<[Point3<f32>; 8]>,
    /// Outward face normal.
    pub normal: Vector3<f32>,
    /// Face index for FeatureId construction.
    pub face_index: u32,
}

/// Trait for shapes that can extract a support face for manifold clipping.
///
/// Not all ConvexSupport shapes have meaningful faces (spheres don't).
/// Shapes without faces return None, and the manifold generator falls
/// back to a single-point contact from the EPA witness point.
pub trait SupportFaceExtractor {
    fn support_face(&self, direction: Vector3<f32>) -> Option<SupportFace>;
}

impl SupportFaceExtractor for Obb {
    fn support_face(&self, direction: Vector3<f32>) -> Option<SupportFace> {
        let axes = self.axes();

        // Find the face most aligned with the direction.
        let mut best_idx = 0;
        let mut best_dot = 0.0f32;
        let mut best_sign = 1.0f32;
        for i in 0..3 {
            let dot = direction.dot(&axes[i]);
            if dot.abs() > best_dot.abs() {
                best_dot = dot;
                best_idx = i;
                best_sign = if dot >= 0.0 { 1.0 } else { -1.0 };
            }
        }

        let face = obb_face(self, best_idx, best_sign);
        let face_index = (best_idx as u32) * 2 + if best_sign > 0.0 { 0 } else { 1 };

        Some(SupportFace {
            vertices: SmallVec::from_slice(&face.vertices),
            normal: face.normal,
            face_index,
        })
    }
}

impl SupportFaceExtractor for ShapeView<'_> {
    fn support_face(&self, direction: Vector3<f32>) -> Option<SupportFace> {
        match self.shape {
            ColliderShape::Sphere { .. } => None,
            ColliderShape::Box { half_extents } => {
                Obb::new(self.center, self.rotation, *half_extents)
                    .support_face(direction)
            }
            ColliderShape::Capsule { .. } => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn query_aabb_sphere() {
        let view = ShapeView {
            center: Point3::new(1.0, 2.0, 3.0),
            rotation: UnitQuaternion::identity(),
            shape: &ColliderShape::Sphere { radius: 0.5 },
        };
        let aabb = view.query_aabb(0.02);
        let expected_r = 0.52;
        assert!(approx_eq(aabb.min.x, 1.0 - expected_r, 1e-6));
        assert!(approx_eq(aabb.max.x, 1.0 + expected_r, 1e-6));
        assert!(approx_eq(aabb.min.y, 2.0 - expected_r, 1e-6));
        assert!(approx_eq(aabb.max.y, 2.0 + expected_r, 1e-6));
    }

    #[test]
    fn query_aabb_box_identity() {
        let view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &ColliderShape::Box {
                half_extents: Vector3::new(1.0, 2.0, 3.0),
            },
        };
        let aabb = view.query_aabb(0.1);
        assert!(approx_eq(aabb.min.x, -1.1, 1e-5));
        assert!(approx_eq(aabb.max.x, 1.1, 1e-5));
        assert!(approx_eq(aabb.min.y, -2.1, 1e-5));
        assert!(approx_eq(aabb.max.y, 2.1, 1e-5));
        assert!(approx_eq(aabb.min.z, -3.1, 1e-5));
        assert!(approx_eq(aabb.max.z, 3.1, 1e-5));
    }

    #[test]
    fn query_aabb_capsule() {
        let view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &ColliderShape::Capsule {
                half_height: 2.0,
                radius: 0.5,
            },
        };
        let aabb = view.query_aabb(0.0);
        // Capsule along Y: AABB is [-0.5, -2.0, -0.5] to [0.5, 2.0, 0.5]
        assert!(approx_eq(aabb.min.y, -2.0, 1e-5));
        assert!(approx_eq(aabb.max.y, 2.0, 1e-5));
        assert!(approx_eq(aabb.min.x, -0.5, 1e-5));
        assert!(approx_eq(aabb.max.x, 0.5, 1e-5));
    }

    #[test]
    fn support_sphere_matches_direct() {
        let shape = ColliderShape::Sphere { radius: 1.0 };
        let view = ShapeView {
            center: Point3::new(1.0, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };
        let direct = SupportSphere {
            center: Point3::new(1.0, 0.0, 0.0),
            radius: 1.0,
        }
        .support(Vector3::x());
        let via_view = view.support(Vector3::x());
        assert!((direct - via_view).magnitude() < 1e-6);
    }

    #[test]
    fn support_box_matches_direct() {
        let he = Vector3::new(1.0, 2.0, 3.0);
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.5);
        let shape = ColliderShape::Box { half_extents: he };
        let view = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape,
        };
        let direct = Obb::new(Point3::origin(), rot, he).support(Vector3::new(1.0, 1.0, 0.0));
        let via_view = view.support(Vector3::new(1.0, 1.0, 0.0));
        assert!((direct - via_view).magnitude() < 1e-6);
    }
}
