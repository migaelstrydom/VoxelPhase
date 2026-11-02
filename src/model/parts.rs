//! Models as flat lists of parts, each a set of mesh primitives under its
//! own transform.

use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

use super::transform::Transform;

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
