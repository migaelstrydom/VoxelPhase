//! Shared parametric model builders used by multiple spawnables.

use std::sync::Arc;

use nalgebra::{Vector2, Vector3, Vector4};
use smallvec::SmallVec;

use crate::collision::convex_hull::{ConvexHull, HullFace};
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

/// A face definition for convex solid model/hull building.
///
/// `vertex_indices` are indices into the vertex array. `opposite_vertex` is the
/// index of a vertex known to be on the interior side of this face — used to
/// ensure outward-facing normals.
pub struct SolidFace {
    pub vertex_indices: Vec<usize>,
    pub opposite_vertex: usize,
}

/// Build a rendering model for a convex solid defined by vertices and faces.
///
/// Each face is triangulated (fan from first vertex) with flat shading.
/// Normals are corrected to point away from `opposite_vertex`.
pub fn convex_solid_model(
    vertices: &[Vector3<f32>],
    faces: &[SolidFace],
    material: MaterialId,
) -> Arc<Model> {
    let color_v4 = Vector4::new(1.0, 1.0, 1.0, 1.0);
    let mut mesh_verts = Vec::new();
    let mut mesh_indices = Vec::new();

    for face in faces {
        let idx = &face.vertex_indices;
        if idx.len() < 3 {
            continue;
        }

        // Compute face normal from first triangle.
        let a = vertices[idx[0]];
        let b = vertices[idx[1]];
        let c = vertices[idx[2]];
        let opp = vertices[face.opposite_vertex];

        let mut normal = (b - a).cross(&(c - a));
        // Ensure outward: normal should point away from opposite vertex.
        if normal.dot(&(a - opp)) < 0.0 {
            normal = -normal;
        }
        let normal = normal.normalize();

        // Determine winding: if normal flipped, we reverse vertex order.
        let raw_normal = (b - a).cross(&(c - a));
        let flip = raw_normal.dot(&(a - opp)) < 0.0;

        let mut ordered: Vec<Vector3<f32>> = idx.iter().map(|&i| vertices[i]).collect();
        if flip {
            ordered[1..].reverse();
        }

        // Triangulate as fan from first vertex.
        let v0 = ordered[0];
        for i in 1..ordered.len() - 1 {
            let v1 = ordered[i];
            let v2 = ordered[i + 1];

            let base_idx = mesh_verts.len() as u32;
            let uvs = [
                Vector2::new(0.5, 0.0),
                Vector2::new(0.0, 1.0),
                Vector2::new(1.0, 1.0),
            ];

            for (j, v) in [v0, v1, v2].iter().enumerate() {
                mesh_verts.push(Vertex {
                    pos: Vector4::new(v.x, v.y, v.z, 1.0),
                    color: color_v4,
                    tex_coords: uvs[j],
                    normal,
                });
            }

            mesh_indices.push(base_idx);
            mesh_indices.push(base_idx + 1);
            mesh_indices.push(base_idx + 2);
        }
    }

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: mesh_verts,
        indices: mesh_indices,
        material,
    }])];
    Arc::new(Model::flat(parts))
}

/// Build a `ConvexHull` from vertices and face definitions.
///
/// Normals are corrected to point outward using each face's `opposite_vertex`.
pub fn build_convex_hull(vertices: &[Vector3<f32>], faces: &[SolidFace]) -> ConvexHull {
    let hull_faces: Vec<HullFace> = faces
        .iter()
        .map(|face| {
            let a = vertices[face.vertex_indices[0]];
            let b = vertices[face.vertex_indices[1]];
            let c = vertices[face.vertex_indices[2]];
            let opp = vertices[face.opposite_vertex];

            let raw_normal = (b - a).cross(&(c - a));
            let flip = raw_normal.dot(&(a - opp)) < 0.0;
            let normal = if flip { -raw_normal.normalize() } else { raw_normal.normalize() };

            // If normal was flipped, reverse winding so vertices remain CCW
            // from outside. This is critical for face clipping in GJK/EPA.
            let mut indices: SmallVec<[u16; 6]> = face
                .vertex_indices
                .iter()
                .map(|&i| i as u16)
                .collect();
            if flip {
                indices[1..].reverse();
            }

            HullFace {
                vertex_indices: indices,
                normal,
            }
        })
        .collect();

    ConvexHull::new(vertices.to_vec(), hull_faces)
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
