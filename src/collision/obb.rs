//! Oriented Bounding Box (OBB) representation and utilities.

use nalgebra::{Point3, UnitQuaternion, Vector3};

/// An oriented bounding box in world space.
#[derive(Debug, Clone, Copy)]
pub struct Obb {
    /// World-space center.
    pub center: Point3<f32>,
    /// Orientation in world space.
    pub rotation: UnitQuaternion<f32>,
    /// Half-widths along each local axis.
    pub half_extents: Vector3<f32>,
}

impl Obb {
    pub fn new(
        center: Point3<f32>,
        rotation: UnitQuaternion<f32>,
        half_extents: Vector3<f32>,
    ) -> Self {
        Self {
            center,
            rotation,
            half_extents,
        }
    }

    /// Local axes in world space: [right, up, forward].
    pub fn axes(&self) -> [Vector3<f32>; 3] {
        let m = self.rotation.to_rotation_matrix().into_inner();
        [m.column(0).into(), m.column(1).into(), m.column(2).into()]
    }

    /// Project the OBB's half-extent onto an arbitrary world-space axis.
    ///
    /// Returns the scalar half-width of the OBB's shadow on `axis`.
    pub fn project_half_extent(&self, axis: &Vector3<f32>) -> f32 {
        let axes = self.axes();
        (axes[0].dot(axis)).abs() * self.half_extents.x
            + (axes[1].dot(axis)).abs() * self.half_extents.y
            + (axes[2].dot(axis)).abs() * self.half_extents.z
    }

    /// Closest point on (or inside) the OBB to an external point.
    pub fn closest_point(&self, point: Point3<f32>) -> Point3<f32> {
        let d = point - self.center;
        let axes = self.axes();
        let mut result = self.center;

        for i in 0..3 {
            let dist = d.dot(&axes[i]);
            let clamped = dist.clamp(-self.half_extents[i], self.half_extents[i]);
            result += axes[i] * clamped;
        }

        result
    }

    /// Compute the AABB that fully encloses this OBB.
    pub fn enclosing_aabb(&self) -> (Point3<f32>, Point3<f32>) {
        let axes = self.axes();
        let extent = Vector3::new(
            axes[0].x.abs() * self.half_extents.x
                + axes[1].x.abs() * self.half_extents.y
                + axes[2].x.abs() * self.half_extents.z,
            axes[0].y.abs() * self.half_extents.x
                + axes[1].y.abs() * self.half_extents.y
                + axes[2].y.abs() * self.half_extents.z,
            axes[0].z.abs() * self.half_extents.x
                + axes[1].z.abs() * self.half_extents.y
                + axes[2].z.abs() * self.half_extents.z,
        );

        (self.center - extent, self.center + extent)
    }

    /// Get the 8 corner vertices in world space.
    pub fn corners(&self) -> [Point3<f32>; 8] {
        let axes = self.axes();
        let hx = axes[0] * self.half_extents.x;
        let hy = axes[1] * self.half_extents.y;
        let hz = axes[2] * self.half_extents.z;
        let c = self.center;

        [
            c - hx - hy - hz,
            c + hx - hy - hz,
            c + hx + hy - hz,
            c - hx + hy - hz,
            c - hx - hy + hz,
            c + hx - hy + hz,
            c + hx + hy + hz,
            c - hx + hy + hz,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_cube() -> Obb {
        Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        )
    }

    #[test]
    fn axes_identity() {
        let obb = unit_cube();
        let axes = obb.axes();
        assert!((axes[0] - Vector3::x()).magnitude() < 1e-6);
        assert!((axes[1] - Vector3::y()).magnitude() < 1e-6);
        assert!((axes[2] - Vector3::z()).magnitude() < 1e-6);
    }

    #[test]
    fn closest_point_inside() {
        let obb = unit_cube();
        let p = Point3::new(0.5, 0.5, 0.5);
        let closest = obb.closest_point(p);
        assert!((closest - p).magnitude() < 1e-6);
    }

    #[test]
    fn closest_point_outside() {
        let obb = unit_cube();
        let p = Point3::new(3.0, 0.0, 0.0);
        let closest = obb.closest_point(p);
        assert!((closest - Point3::new(1.0, 0.0, 0.0)).magnitude() < 1e-6);
    }

    #[test]
    fn enclosing_aabb_identity() {
        let obb = unit_cube();
        let (min, max) = obb.enclosing_aabb();
        assert!((min - Point3::new(-1.0, -1.0, -1.0)).magnitude() < 1e-6);
        assert!((max - Point3::new(1.0, 1.0, 1.0)).magnitude() < 1e-6);
    }

    #[test]
    fn project_half_extent_aligned() {
        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(2.0, 3.0, 4.0),
        );
        assert!((obb.project_half_extent(&Vector3::x()) - 2.0).abs() < 1e-6);
        assert!((obb.project_half_extent(&Vector3::y()) - 3.0).abs() < 1e-6);
        assert!((obb.project_half_extent(&Vector3::z()) - 4.0).abs() < 1e-6);
    }

    #[test]
    fn corners_count_and_symmetry() {
        let obb = unit_cube();
        let corners = obb.corners();
        assert_eq!(corners.len(), 8);
        let centroid: Vector3<f32> = corners.iter().map(|c| c.coords).sum::<Vector3<f32>>() / 8.0;
        assert!(centroid.magnitude() < 1e-6);
    }
}
