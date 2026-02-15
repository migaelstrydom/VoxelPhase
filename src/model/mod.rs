//! Model hierarchy and mesh primitive types.
//!
//! This module provides types for defining 3D models as hierarchies of parts,
//! each with transforms and mesh primitives. The design aligns with industry
//! standard formats like glTF.

use nalgebra::{Matrix4, UnitQuaternion, Vector3};

use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

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

    /// Compose two transforms: applies self first, then other.
    pub fn compose(&self, other: &Transform) -> Transform {
        Transform {
            translation: self.translation
                + self.rotation * other.translation.component_mul(&self.scale),
            rotation: self.rotation * other.rotation,
            scale: self.scale.component_mul(&other.scale),
        }
    }
}

/// A drawable unit: geometry + material reference.
#[derive(Clone)]
pub struct MeshPrimitive {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub material: MaterialId,
}

/// A part of a model with its own transform and mesh primitives.
#[derive(Clone)]
pub struct ModelPart {
    /// Local transform relative to model origin.
    pub local_transform: Transform,

    /// The mesh primitives making up this part.
    pub primitives: Vec<MeshPrimitive>,
}

impl ModelPart {
    /// Create a new part with default transform.
    pub fn new(primitives: Vec<MeshPrimitive>) -> Self {
        Self {
            local_transform: Transform::default(),
            primitives,
        }
    }
}

/// A complete model composed of multiple parts.
#[derive(Clone)]
pub struct Model {
    /// Flat array of parts.
    pub parts: Vec<ModelPart>,
}

impl Model {
    /// Create a model from a list of parts.
    pub fn flat(parts: Vec<ModelPart>) -> Self {
        Self { parts }
    }
}
