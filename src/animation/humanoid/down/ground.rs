//! The ground a lying body rests on, and limbs set down on it.

use nalgebra::{Point3, Vector3};

/// The ground a lying body rests on, taken as flat under it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundPlane {
    /// A point on the ground.
    pub point: Point3<f32>,
    /// Up from the ground, unit length.
    pub normal: Vector3<f32>,
}

impl GroundPlane {
    /// The ground a capsule `half_height` long each way from `centre`
    /// (caps included) and `radius` thick rests on, its axis along `axis`:
    /// level with the capsule's lowest point, facing `normal`.
    pub fn under_capsule(
        centre: Point3<f32>,
        axis: Vector3<f32>,
        normal: Vector3<f32>,
        half_height: f32,
        radius: f32,
    ) -> Self {
        let lowest = radius + (half_height - radius) * axis.dot(&normal).abs();
        Self {
            point: centre - normal * lowest,
            normal,
        }
    }

    /// How high `point` is over the ground, m.
    pub(super) fn height_of(&self, point: Point3<f32>) -> f32 {
        (point - self.point).dot(&self.normal)
    }

    /// The point on the ground under `point`.
    pub(super) fn under(&self, point: Point3<f32>) -> Point3<f32> {
        point - self.normal * self.height_of(point)
    }

    /// `direction` laid along the ground, unit length; `None` where it stands
    /// square to it.
    pub(super) fn along(&self, direction: Vector3<f32>) -> Option<Vector3<f32>> {
        (direction - self.normal * direction.dot(&self.normal)).try_normalize(1e-3)
    }

    /// Where a limb `reach` long from `root` sets its end on the ground: out
    /// along `toward` (laid along the ground) up to `wanted` from under the
    /// root, as far as the limb goes. Too high to reach the ground, it hangs
    /// toward it at full stretch.
    pub(super) fn reached(
        &self,
        root: Point3<f32>,
        toward: Vector3<f32>,
        wanted: f32,
        reach: f32,
    ) -> Point3<f32> {
        let height = self.height_of(root).max(0.0);
        if height >= reach {
            return root - self.normal * reach;
        }
        let across = (reach * reach - height * height).sqrt();
        let out = self.along(toward).unwrap_or_else(Vector3::zeros);
        self.under(root) + out * wanted.min(across)
    }
}

/// Level ground through the origin.
#[cfg(test)]
pub(super) fn level_ground() -> GroundPlane {
    GroundPlane {
        point: Point3::origin(),
        normal: Vector3::y(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The ground a capsule lies on is a radius under its axis lying flat,
    /// and its half height under its centre standing.
    #[test]
    fn a_capsule_rests_on_the_ground_at_its_lowest_point() {
        let centre = Point3::new(1.0, 2.0, 3.0);
        let flat = GroundPlane::under_capsule(centre, Vector3::z(), Vector3::y(), 0.5, 0.25);
        assert!((flat.point.y - 1.75).abs() < 1e-6);
        let standing = GroundPlane::under_capsule(centre, Vector3::y(), Vector3::y(), 0.5, 0.25);
        assert!((standing.point.y - 1.5).abs() < 1e-6);
    }

    /// A limb set on the ground ends on it, out as far as asked where it
    /// reaches that far, and as far as it reaches where not; one too high
    /// hangs toward the ground at full stretch.
    #[test]
    fn a_limb_set_on_the_ground_ends_on_it_within_its_reach() {
        let ground = level_ground();
        let root = Point3::new(0.0, 0.3, 0.0);
        let near = ground.reached(root, Vector3::x(), 0.2, 0.5);
        assert!((near - Point3::new(0.2, 0.0, 0.0)).norm() < 1e-6);
        let far = ground.reached(root, Vector3::x(), 2.0, 0.5);
        assert!((far - Point3::new(0.4, 0.0, 0.0)).norm() < 1e-5);
        let high = ground.reached(Point3::new(0.0, 0.8, 0.0), Vector3::x(), 0.2, 0.5);
        assert!((high - Point3::new(0.0, 0.3, 0.0)).norm() < 1e-6);
    }
}
