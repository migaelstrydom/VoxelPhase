//! Shared parametric model builders used by multiple spawnables.

use std::sync::Arc;

use nalgebra::Vector3;

use crate::geometry::{generate_cube_indices, generate_cube_vertices};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// Build a single-box model with the given half-extents.
pub fn cuboid_model(half_extents: Vector3<f32>, material: MaterialId) -> Arc<Model> {
    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: generate_cube_vertices(half_extents, Colour::WHITE),
        indices: generate_cube_indices(),
        material,
    }])];

    Arc::new(Model::flat(parts))
}

/// Build a compound model from multiple boxes, each at a local offset.
///
/// All boxes share the same material and are merged into a single draw call.
pub fn compound_cuboid_model(
    boxes: &[(Vector3<f32>, Vector3<f32>)],
    material: MaterialId,
) -> Arc<Model> {
    let mut all_vertices = Vec::new();
    let mut all_indices = Vec::new();

    for &(half_extents, offset) in boxes {
        let base_idx = all_vertices.len() as u32;
        all_vertices.extend(generate_offset_cube_vertices(
            half_extents,
            offset,
            Colour::WHITE,
        ));
        let indices = generate_cube_indices();
        all_indices.extend(indices.iter().map(|i| i + base_idx));
    }

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: all_vertices,
        indices: all_indices,
        material,
    }])];

    Arc::new(Model::flat(parts))
}

/// Build a compound model from multiple boxes with per-box materials.
///
/// Boxes sharing a material are merged into the same draw call. Each distinct
/// material becomes a separate `MeshPrimitive`.
pub fn multi_material_compound_cuboid_model(
    boxes: &[(Vector3<f32>, Vector3<f32>, MaterialId)],
) -> Arc<Model> {
    // Group by material, preserving order of first appearance.
    let mut groups: Vec<(MaterialId, Vec<Vertex>, Vec<u32>)> = Vec::new();

    for &(half_extents, offset, material) in boxes {
        let group = groups.iter_mut().find(|(m, _, _)| *m == material);
        let (_, verts, indices) = match group {
            Some(g) => g,
            None => {
                groups.push((material, Vec::new(), Vec::new()));
                groups.last_mut().unwrap()
            }
        };

        let base_idx = verts.len() as u32;
        verts.extend(generate_offset_cube_vertices(half_extents, offset, Colour::WHITE));
        let cube_indices = generate_cube_indices();
        indices.extend(cube_indices.iter().map(|i| i + base_idx));
    }

    let primitives = groups
        .into_iter()
        .map(|(material, vertices, indices)| MeshPrimitive {
            vertices,
            indices,
            material,
        })
        .collect();

    let parts = vec![ModelPart::new(primitives)];
    Arc::new(Model::flat(parts))
}

/// Generate cube vertices offset by a translation.
fn generate_offset_cube_vertices(
    half_extents: Vector3<f32>,
    offset: Vector3<f32>,
    colour: Colour,
) -> Vec<Vertex> {
    let mut verts = generate_cube_vertices(half_extents, colour);
    for v in &mut verts {
        v.pos.x += offset.x;
        v.pos.y += offset.y;
        v.pos.z += offset.z;
    }
    verts
}
