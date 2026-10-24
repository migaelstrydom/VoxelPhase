use nalgebra::{Point3, Vector3};
use traversal_planner::PathVec;

/// A straight path laid on the world: path coordinates `(s, d, z)` are
/// distance along the heading, offset to its left, and world height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathFrame {
    /// Where `s = 0, d = 0`; its height is ignored, since `z` is world height.
    origin: Point3<f32>,
    /// Horizontal unit vector along the path.
    forward: Vector3<f32>,
    /// Horizontal unit vector to the path's left.
    left: Vector3<f32>,
}

impl PathFrame {
    /// A path from `origin` along `heading_degrees`, turned about `+Y` from
    /// `+Z` as every yaw in the level format is.
    pub fn new(origin: Point3<f32>, heading_degrees: f32) -> Self {
        let (sin, cos) = heading_degrees.to_radians().sin_cos();
        let forward = Vector3::new(sin, 0.0, cos);
        Self {
            origin: Point3::new(origin.x, 0.0, origin.z),
            forward,
            left: Vector3::y().cross(&forward),
        }
    }

    /// A path from `origin` along the horizontal part of `heading`.
    pub fn along(origin: Point3<f32>, heading: Vector3<f32>) -> Self {
        let forward = Vector3::new(heading.x, 0.0, heading.z)
            .try_normalize(f32::EPSILON)
            .unwrap_or_else(Vector3::z);
        Self {
            origin: Point3::new(origin.x, 0.0, origin.z),
            forward,
            left: Vector3::y().cross(&forward),
        }
    }

    /// Horizontal unit vector along the path.
    pub fn forward(&self) -> Vector3<f32> {
        self.forward
    }

    pub fn world_point(&self, at: PathVec) -> Point3<f32> {
        self.origin + self.forward * at.s + self.left * at.d + Vector3::y() * at.z
    }

    pub fn path_vector(&self, world: Vector3<f32>) -> PathVec {
        PathVec::new(world.dot(&self.forward), world.dot(&self.left), world.y)
    }

    pub fn path_point(&self, world: Point3<f32>) -> PathVec {
        let offset = world - self.origin;
        PathVec::new(offset.dot(&self.forward), offset.dot(&self.left), world.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_heading_of_ninety_runs_along_x_with_its_left_toward_minus_z() {
        let frame = PathFrame::new(Point3::new(3.0, 7.0, 10.0), 90.0);
        let ahead = frame.world_point(PathVec::new(2.0, 0.5, 1.0));
        assert!(
            (ahead - Point3::new(5.0, 1.0, 9.5)).norm() < 1e-5,
            "{ahead}"
        );
        let back = frame.path_point(ahead);
        assert!((back - PathVec::new(2.0, 0.5, 1.0)).length() < 1e-5);
    }
}
