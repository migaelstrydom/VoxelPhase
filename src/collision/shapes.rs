//! Shape view: a transform + shape reference for collision dispatch.
//!
//! `ShapeView` bundles everything the collision library needs to test a shape pair:
//! position, orientation, and shape data. It decouples collision geometry from the
//! ECS/physics representation so the collision library has no dependency on `specs`
//! or the solver.

use nalgebra::{Point3, UnitQuaternion};

use crate::physics::ColliderShape;

use super::obb::Obb;

/// A positioned shape ready for collision testing.
///
/// Constructed by the physics pipeline from ECS components before dispatch.
/// The collision library receives these as immutable references.
#[derive(Debug, Clone)]
pub struct ShapeView {
    /// World-space position (center of shape).
    pub position: Point3<f32>,
    /// World-space orientation.
    pub rotation: UnitQuaternion<f32>,
    /// The collision shape (type + dimensions).
    pub shape: ColliderShape,
}

impl ShapeView {
    pub fn new(
        position: Point3<f32>,
        rotation: UnitQuaternion<f32>,
        shape: ColliderShape,
    ) -> Self {
        Self {
            position,
            rotation,
            shape,
        }
    }

    /// Build an `Obb` from a Box shape variant. Panics if shape is not Box.
    pub fn as_obb(&self) -> Obb {
        match &self.shape {
            ColliderShape::Box { half_extents } => {
                Obb::new(self.position, self.rotation, *half_extents)
            }
            _ => panic!("ShapeView::as_obb called on non-Box shape"),
        }
    }

    /// Get the sphere radius. Panics if shape is not Sphere.
    pub fn sphere_radius(&self) -> f32 {
        match &self.shape {
            ColliderShape::Sphere { radius } => *radius,
            _ => panic!("ShapeView::sphere_radius called on non-Sphere shape"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn shape_view_sphere() {
        let sv = ShapeView::new(
            Point3::new(1.0, 2.0, 3.0),
            UnitQuaternion::identity(),
            ColliderShape::Sphere { radius: 0.5 },
        );
        assert_eq!(sv.sphere_radius(), 0.5);
    }

    #[test]
    fn shape_view_as_obb() {
        let he = Vector3::new(1.0, 2.0, 3.0);
        let sv = ShapeView::new(
            Point3::new(5.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            ColliderShape::Box { half_extents: he },
        );
        let obb = sv.as_obb();
        assert_eq!(obb.center, Point3::new(5.0, 0.0, 0.0));
        assert_eq!(obb.half_extents, he);
    }

    #[test]
    #[should_panic]
    fn shape_view_as_obb_panics_on_sphere() {
        let sv = ShapeView::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            ColliderShape::Sphere { radius: 1.0 },
        );
        sv.as_obb();
    }
}
