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
        verts.extend(generate_offset_cube_vertices(
            half_extents,
            offset,
            Colour::WHITE,
        ));
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

/// Build a compound model from multiple boxes with per-box materials and
/// optional per-box Y-axis rotation.
///
/// Same as [`multi_material_compound_cuboid_model`] but each box can be
/// individually rotated.
pub fn multi_material_rotated_compound_cuboid_model(
    boxes: &[(
        Vector3<f32>,
        Vector3<f32>,
        nalgebra::UnitQuaternion<f32>,
        MaterialId,
    )],
) -> Arc<Model> {
    let mut groups: Vec<(MaterialId, Vec<Vertex>, Vec<u32>)> = Vec::new();

    for &(half_extents, offset, rotation, material) in boxes {
        let group = groups.iter_mut().find(|(m, _, _)| *m == material);
        let (_, verts, indices) = match group {
            Some(g) => g,
            None => {
                groups.push((material, Vec::new(), Vec::new()));
                groups.last_mut().unwrap()
            }
        };

        let base_idx = verts.len() as u32;
        verts.extend(generate_rotated_offset_cube_vertices(
            half_extents,
            offset,
            rotation,
            Colour::WHITE,
        ));
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

        // Emit one vertex per face vertex (shared across fan triangles within
        // this face) to avoid visible seams from duplicated positions.
        let base_idx = mesh_verts.len() as u32;

        // Build a 2D coordinate frame on the face plane and project vertices
        // into it, then normalize to [0,1] so the texture fills the face.
        let face_uvs = planar_face_uvs(&ordered);

        for (j, v) in ordered.iter().enumerate() {
            mesh_verts.push(Vertex {
                pos: Vector3::new(v.x, v.y, v.z),
                color: color_v4,
                tex_coords: face_uvs[j],
                normal,
                ao: 1.0,
            });
        }

        // Triangulate as fan from first vertex, indexing into shared vertices.
        for i in 1..ordered.len() - 1 {
            mesh_indices.push(base_idx);
            mesh_indices.push(base_idx + i as u32);
            mesh_indices.push(base_idx + (i + 1) as u32);
        }
    }

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: mesh_verts,
        indices: mesh_indices,
        material,
    }])];
    Arc::new(Model::flat(parts))
}

/// Project face vertices onto a 2D coordinate frame on the face plane,
/// then normalize so UVs span [0,1] across the face.
fn planar_face_uvs(vertices: &[Vector3<f32>]) -> Vec<Vector2<f32>> {
    if vertices.len() < 2 {
        return vertices.iter().map(|_| Vector2::new(0.5, 0.5)).collect();
    }

    // Tangent: direction from first to second vertex.
    let center = vertices.iter().copied().sum::<Vector3<f32>>() / vertices.len() as f32;
    let edge = vertices[1] - vertices[0];
    let tangent = if edge.magnitude_squared() > 1e-12 {
        edge.normalize()
    } else {
        Vector3::x()
    };

    // Face normal from first triangle, then bitangent.
    let face_normal = if vertices.len() >= 3 {
        let n = (vertices[1] - vertices[0]).cross(&(vertices[2] - vertices[0]));
        if n.magnitude_squared() > 1e-12 {
            n.normalize()
        } else {
            Vector3::z()
        }
    } else {
        Vector3::z()
    };
    let bitangent = face_normal.cross(&tangent).normalize();

    // Project each vertex relative to center.
    let coords: Vec<(f32, f32)> = vertices
        .iter()
        .map(|v| {
            let local = v - center;
            (local.dot(&tangent), local.dot(&bitangent))
        })
        .collect();

    // Normalize to [0,1].
    let (mut min_u, mut max_u) = (f32::MAX, f32::MIN);
    let (mut min_v, mut max_v) = (f32::MAX, f32::MIN);
    for &(u, v) in &coords {
        min_u = min_u.min(u);
        max_u = max_u.max(u);
        min_v = min_v.min(v);
        max_v = max_v.max(v);
    }
    let range_u = (max_u - min_u).max(1e-6);
    let range_v = (max_v - min_v).max(1e-6);

    // Use the larger range for both axes to preserve aspect ratio, and
    // center the shorter axis.
    let range = range_u.max(range_v);
    let offset_u = (range - range_u) * 0.5;
    let offset_v = (range - range_v) * 0.5;

    coords
        .iter()
        .map(|&(u, v)| {
            Vector2::new(
                (u - min_u + offset_u) / range,
                (v - min_v + offset_v) / range,
            )
        })
        .collect()
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
            let normal = if flip {
                -raw_normal.normalize()
            } else {
                raw_normal.normalize()
            };

            // If normal was flipped, reverse winding so vertices remain CCW
            // from outside. This is critical for face clipping in GJK/EPA.
            let mut indices: SmallVec<[u16; 6]> =
                face.vertex_indices.iter().map(|&i| i as u16).collect();
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

/// Generate cube vertices rotated then translated.
fn generate_rotated_offset_cube_vertices(
    half_extents: Vector3<f32>,
    offset: Vector3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    colour: Colour,
) -> Vec<Vertex> {
    let rot = rotation.to_rotation_matrix();
    let mut verts = generate_cube_vertices(half_extents, colour);
    for v in &mut verts {
        let p = rot * Vector3::new(v.pos.x, v.pos.y, v.pos.z);
        v.pos.x = p.x + offset.x;
        v.pos.y = p.y + offset.y;
        v.pos.z = p.z + offset.z;
        v.normal = rot * v.normal;
    }
    verts
}
