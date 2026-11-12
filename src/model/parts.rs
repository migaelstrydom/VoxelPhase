//! Models as flat lists of parts, each a set of mesh primitives under its
//! own transform.

use nalgebra::Vector3;

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

impl MeshPrimitive {
    /// Take out the triangles whose middle `take` picks, as a mesh of their
    /// own in the same material, leaving the rest. Each keeps only the
    /// vertices its triangles use.
    pub fn split_off(&mut self, take: impl Fn(Vector3<f32>) -> bool) -> MeshPrimitive {
        let (taken, kept): (Vec<&[u32]>, Vec<&[u32]>) =
            self.indices.chunks_exact(3).partition(|triangle| {
                let middle = triangle
                    .iter()
                    .map(|index| self.vertices[*index as usize].pos)
                    .sum::<Vector3<f32>>()
                    / 3.0;
                take(middle)
            });
        let taken = self.gathered(&taken);
        *self = self.gathered(&kept);
        taken
    }

    /// A mesh of just these triangles of this one.
    fn gathered(&self, triangles: &[&[u32]]) -> MeshPrimitive {
        let mut renumbered = vec![u32::MAX; self.vertices.len()];
        let mut vertices = Vec::new();
        let mut indices = Vec::with_capacity(triangles.len() * 3);
        for index in triangles.iter().flat_map(|triangle| triangle.iter()) {
            let slot = &mut renumbered[*index as usize];
            if *slot == u32::MAX {
                *slot = vertices.len() as u32;
                vertices.push(self.vertices[*index as usize]);
            }
            indices.push(*slot);
        }
        MeshPrimitive {
            vertices,
            indices,
            material: self.material,
        }
    }
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
