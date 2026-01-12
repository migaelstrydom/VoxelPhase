//! Sphere mesh generation.
//!
//! Generates UV sphere meshes with configurable subdivision.

use std::f32::consts::{PI, TAU};

use nalgebra::{Vector2, Vector3, Vector4};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// Generate sphere vertices with the given colour.
///
/// # Arguments
/// * `radius` - Sphere radius
/// * `segments` - Number of horizontal segments (longitude divisions)
/// * `rings` - Number of vertical rings (latitude divisions)
/// * `colour` - Vertex colour
pub fn generate_sphere_vertices(
    radius: f32,
    segments: u32,
    rings: u32,
    colour: Colour,
) -> Vec<Vertex> {
    let mut vertices = Vec::with_capacity(((rings + 1) * (segments + 1)) as usize);
    let colour_vec = colour.to_vec4();

    for ring in 0..=rings {
        let theta = ring as f32 * PI / rings as f32;
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();

        for segment in 0..=segments {
            let phi = segment as f32 * TAU / segments as f32;
            let sin_phi = phi.sin();
            let cos_phi = phi.cos();

            let x = sin_theta * cos_phi;
            let y = cos_theta;
            let z = sin_theta * sin_phi;

            let position = Vector3::new(x * radius, y * radius, z * radius);
            let normal = Vector3::new(x, y, z);

            let u = segment as f32 / segments as f32;
            let v = ring as f32 / rings as f32;

            vertices.push(Vertex {
                pos: Vector4::new(position.x, position.y, position.z, 1.0),
                color: colour_vec,
                tex_coords: Vector2::new(u, v),
                normal,
            });
        }
    }

    vertices
}

/// Generate sphere indices for the given subdivision levels.
///
/// # Arguments
/// * `segments` - Number of horizontal segments (must match generate_sphere_vertices)
/// * `rings` - Number of vertical rings (must match generate_sphere_vertices)
pub fn generate_sphere_indices(segments: u32, rings: u32) -> Vec<u32> {
    let mut indices = Vec::with_capacity((rings * segments * 6) as usize);

    for ring in 0..rings {
        for segment in 0..segments {
            let current_ring_start = ring * (segments + 1);
            let next_ring_start = (ring + 1) * (segments + 1);

            let i0 = current_ring_start + segment;
            let i1 = next_ring_start + segment;
            let i2 = current_ring_start + segment + 1;
            let i3 = next_ring_start + segment + 1;

            // First triangle
            indices.push(i0);
            indices.push(i2);
            indices.push(i1);

            // Second triangle
            indices.push(i2);
            indices.push(i3);
            indices.push(i1);
        }
    }

    indices
}
