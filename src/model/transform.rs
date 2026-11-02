//! A part's translation, rotation and scale relative to its parent.

use nalgebra::{Matrix4, UnitQuaternion, Vector3};

/// Local transform relative to parent.
#[derive(Clone, Debug)]
pub struct Transform {
    pub translation: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub scale: Vector3<f32>,
}

impl Default for Transform {
    fn default() -> Self {
        Self {
            translation: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
            scale: Vector3::new(1.0, 1.0, 1.0),
        }
    }
}

impl Transform {
    /// Convert to a 4x4 transformation matrix.
    /// Order: Scale -> Rotate -> Translate
    pub fn to_matrix(&self) -> Matrix4<f32> {
        let scale = Matrix4::new_nonuniform_scaling(&self.scale);
        let rotation = self.rotation.to_homogeneous();
        let translation = Matrix4::new_translation(&self.translation);
        translation * rotation * scale
    }

    /// The transform that applies `other` first, then `self`, as
    /// `self.to_matrix() * other.to_matrix()` does: `other` is expressed in
    /// `self`'s frame. Exact when `self`'s scale is uniform; a non-uniform
    /// scale under a rotation shears, which a `Transform` cannot hold.
    pub fn compose(&self, other: &Transform) -> Transform {
        Transform {
            translation: self.translation
                + self.rotation * other.translation.component_mul(&self.scale),
            rotation: self.rotation * other.rotation,
            scale: self.scale.component_mul(&other.scale),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn compose_applies_the_argument_first() {
        let outer = Transform {
            translation: Vector3::new(1.0, 2.0, 3.0),
            rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7),
            scale: Vector3::repeat(2.0),
        };
        let inner = Transform {
            translation: Vector3::new(-0.5, 0.25, 4.0),
            rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -1.1),
            scale: Vector3::new(1.0, 3.0, 0.5),
        };
        let point = Point3::new(0.3, -0.8, 1.9);
        let composed = outer.compose(&inner).to_matrix().transform_point(&point);
        let in_turn = outer
            .to_matrix()
            .transform_point(&inner.to_matrix().transform_point(&point));
        assert!(
            (composed - in_turn).norm() < 1e-5,
            "{composed} vs {in_turn}"
        );
    }
}
