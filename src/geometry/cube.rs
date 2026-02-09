//! Cube mesh generation.
//!
//! Generates axis-aligned box meshes with per-face normals.

use nalgebra::{Vector2, Vector3, Vector4};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// Generate cube vertices with per-face normals.
///
/// Creates 24 vertices (4 per face) so each face has its own normal direction.
///
/// # Arguments
/// * `half_extents` - Half-widths along each axis
/// * `colour` - Vertex colour
pub fn generate_cube_vertices(half_extents: Vector3<f32>, colour: Colour) -> Vec<Vertex> {
    let hx = half_extents.x;
    let hy = half_extents.y;
    let hz = half_extents.z;
    let c = colour.to_vec4();

    // Face definitions: (normal, 4 corner positions in CCW order from outside)
    let faces: [(Vector3<f32>, [Vector3<f32>; 4]); 6] = [
        // +X face
        (
            Vector3::new(1.0, 0.0, 0.0),
            [
                Vector3::new(hx, -hy, -hz),
                Vector3::new(hx, -hy, hz),
                Vector3::new(hx, hy, hz),
                Vector3::new(hx, hy, -hz),
            ],
        ),
        // -X face
        (
            Vector3::new(-1.0, 0.0, 0.0),
            [
                Vector3::new(-hx, -hy, hz),
                Vector3::new(-hx, -hy, -hz),
                Vector3::new(-hx, hy, -hz),
                Vector3::new(-hx, hy, hz),
            ],
        ),
        // +Y face
        (
            Vector3::new(0.0, 1.0, 0.0),
            [
                Vector3::new(-hx, hy, -hz),
                Vector3::new(hx, hy, -hz),
                Vector3::new(hx, hy, hz),
                Vector3::new(-hx, hy, hz),
            ],
        ),
        // -Y face
        (
            Vector3::new(0.0, -1.0, 0.0),
            [
                Vector3::new(-hx, -hy, hz),
                Vector3::new(hx, -hy, hz),
                Vector3::new(hx, -hy, -hz),
                Vector3::new(-hx, -hy, -hz),
            ],
        ),
        // +Z face
        (
            Vector3::new(0.0, 0.0, 1.0),
            [
                Vector3::new(hx, -hy, hz),
                Vector3::new(-hx, -hy, hz),
                Vector3::new(-hx, hy, hz),
                Vector3::new(hx, hy, hz),
            ],
        ),
        // -Z face
        (
            Vector3::new(0.0, 0.0, -1.0),
            [
                Vector3::new(-hx, -hy, -hz),
                Vector3::new(hx, -hy, -hz),
                Vector3::new(hx, hy, -hz),
                Vector3::new(-hx, hy, -hz),
            ],
        ),
    ];

    let mut vertices = Vec::with_capacity(24);
    let uvs = [
        Vector2::new(0.0, 1.0),
        Vector2::new(1.0, 1.0),
        Vector2::new(1.0, 0.0),
        Vector2::new(0.0, 0.0),
    ];

    for (normal, corners) in &faces {
        for (i, pos) in corners.iter().enumerate() {
            vertices.push(Vertex {
                pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                color: c,
                tex_coords: uvs[i],
                normal: *normal,
            });
        }
    }

    vertices
}

/// Generate cube indices for the 6 faces (24 vertices, 36 indices).
///
/// Two triangles per face, matching the vertex layout from `generate_cube_vertices`.
pub fn generate_cube_indices() -> Vec<u32> {
    let mut indices = Vec::with_capacity(36);
    for face in 0..6u32 {
        let base = face * 4;
        indices.push(base);
        indices.push(base + 2);
        indices.push(base + 1);
        indices.push(base);
        indices.push(base + 3);
        indices.push(base + 2);
    }
    indices
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cube_vertex_count() {
        let verts = generate_cube_vertices(Vector3::new(1.0, 1.0, 1.0), Colour::WHITE);
        assert_eq!(verts.len(), 24);
    }

    #[test]
    fn cube_index_count() {
        let indices = generate_cube_indices();
        assert_eq!(indices.len(), 36);
    }

    #[test]
    fn cube_indices_in_range() {
        let indices = generate_cube_indices();
        for &idx in &indices {
            assert!(idx < 24, "Index {} out of range", idx);
        }
    }
}
