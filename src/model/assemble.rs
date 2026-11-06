//! Merging meshes placed in one frame into a model, one primitive per
//! material.

use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector3};

use super::parts::{MeshPrimitive, Model, ModelPart};
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// One already-meshed piece of an object, where it sits in the object's
/// frame, and what it is drawn with.
pub struct PlacedMesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub offset: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub material: MaterialId,
}

/// Merge placed meshes into one model, one primitive per material.
///
/// Grouped in first-seen order so the model is stable across rebuilds rather
/// than reshuffling with a hash map's iteration order.
pub fn assemble_by_material(pieces: Vec<PlacedMesh>) -> Arc<Model> {
    let mut groups: Vec<(MaterialId, Vec<Vertex>, Vec<u32>)> = Vec::new();
    for piece in pieces {
        let slot = match groups.iter().position(|(m, _, _)| *m == piece.material) {
            Some(slot) => slot,
            None => {
                groups.push((piece.material, Vec::new(), Vec::new()));
                groups.len() - 1
            }
        };
        let (_, vertices, indices) = &mut groups[slot];

        let base = vertices.len() as u32;
        let rotation = piece.rotation.to_rotation_matrix();
        vertices.extend(piece.vertices.into_iter().map(|mut vertex| {
            vertex.pos = rotation * vertex.pos + piece.offset;
            vertex.normal = rotation * vertex.normal;
            vertex
        }));
        indices.extend(piece.indices.iter().map(|index| index + base));
    }

    let primitives = groups
        .into_iter()
        .map(|(material, vertices, indices)| MeshPrimitive {
            vertices,
            indices,
            material,
        })
        .collect();

    Arc::new(Model::flat(vec![ModelPart::new(primitives)]))
}
