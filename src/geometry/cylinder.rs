//! Cylinder mesh generation.
//!
//! Generates cylinder meshes between two points, useful for debug line rendering.

use std::f32::consts::TAU;

use nalgebra::{Point3, Vector2, Vector3, Vector4};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// Generate a cylinder mesh between two points.
///
/// # Arguments
/// * `start` - Start point of the cylinder
/// * `end` - End point of the cylinder
/// * `radius` - Cylinder radius
/// * `segments` - Number of segments around the circumference
/// * `colour` - Vertex colour
///
/// Returns (vertices, indices) tuple.
pub fn generate_cylinder(
    start: Point3<f32>,
    end: Point3<f32>,
    radius: f32,
    segments: u32,
    colour: Colour,
) -> (Vec<Vertex>, Vec<u32>) {
    let colour_vec = colour.to_vec4();

    // Calculate cylinder axis and length
    let axis = end - start;
    let length = axis.magnitude();
    if length < 1e-6 {
        return (Vec::new(), Vec::new());
    }
    let axis_normalized = axis / length;

    // Build orthonormal basis for the cylinder cross-section
    let (tangent, bitangent) = build_orthonormal_basis(axis_normalized);

    let mut vertices = Vec::with_capacity((segments * 2 + 2) as usize);
    let mut indices = Vec::with_capacity((segments * 6) as usize);

    // Generate vertices for both caps
    for i in 0..segments {
        let angle = i as f32 * TAU / segments as f32;
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        let offset = tangent * cos_a * radius + bitangent * sin_a * radius;
        let normal = (tangent * cos_a + bitangent * sin_a).normalize();

        // Bottom cap vertex
        let bottom_pos = start + offset;
        vertices.push(Vertex {
            pos: Vector4::new(bottom_pos.x, bottom_pos.y, bottom_pos.z, 1.0),
            color: colour_vec,
            tex_coords: Vector2::new(i as f32 / segments as f32, 0.0),
            normal,
        });

        // Top cap vertex
        let top_pos = end + offset;
        vertices.push(Vertex {
            pos: Vector4::new(top_pos.x, top_pos.y, top_pos.z, 1.0),
            color: colour_vec,
            tex_coords: Vector2::new(i as f32 / segments as f32, 1.0),
            normal,
        });
    }

    // Generate indices for the cylinder sides
    for i in 0..segments {
        let i0 = i * 2;
        let i1 = i * 2 + 1;
        let i2 = ((i + 1) % segments) * 2;
        let i3 = ((i + 1) % segments) * 2 + 1;

        // Two triangles per quad
        indices.push(i0);
        indices.push(i2);
        indices.push(i1);

        indices.push(i2);
        indices.push(i3);
        indices.push(i1);
    }

    (vertices, indices)
}

/// Generate a vertical capped cylinder centered at origin.
///
/// The cylinder extends from `y = -half_height` to `y = +half_height` with
/// flat disc caps on top and bottom. UV mapping wraps the texture around the
/// barrel; caps use planar projection.
///
/// Returns (vertices, indices) tuple.
pub fn generate_capped_cylinder(
    half_height: f32,
    radius: f32,
    segments: u32,
    colour: Colour,
) -> (Vec<Vertex>, Vec<u32>) {
    let colour_vec = colour.to_vec4();

    // Barrel: 2 rings × segments, each with outward normal.
    // Top cap: center + segments verts with normal +Y.
    // Bottom cap: center + segments verts with normal -Y.
    let barrel_verts = (segments * 2) as usize;
    let cap_verts = (segments + 1) as usize; // center + ring
    let total_verts = barrel_verts + cap_verts * 2;
    let barrel_tris = (segments * 2) as usize;
    let cap_tris = segments as usize;
    let total_indices = (barrel_tris + cap_tris * 2) * 3;

    let mut vertices = Vec::with_capacity(total_verts);
    let mut indices = Vec::with_capacity(total_indices);

    // --- Barrel ---
    for i in 0..segments {
        let angle = i as f32 * TAU / segments as f32;
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let normal = Vector3::new(cos_a, 0.0, sin_a).normalize();
        let u = i as f32 / segments as f32;

        // Bottom ring
        vertices.push(Vertex {
            pos: Vector4::new(cos_a * radius, -half_height, sin_a * radius, 1.0),
            color: colour_vec,
            tex_coords: Vector2::new(u, 1.0),
            normal,
        });
        // Top ring
        vertices.push(Vertex {
            pos: Vector4::new(cos_a * radius, half_height, sin_a * radius, 1.0),
            color: colour_vec,
            tex_coords: Vector2::new(u, 0.0),
            normal,
        });
    }

    for i in 0..segments {
        let i0 = i * 2;
        let i1 = i * 2 + 1;
        let i2 = ((i + 1) % segments) * 2;
        let i3 = ((i + 1) % segments) * 2 + 1;
        indices.extend_from_slice(&[i0, i1, i2, i2, i1, i3]);
    }

    // --- Top cap ---
    let top_center = vertices.len() as u32;
    vertices.push(Vertex {
        pos: Vector4::new(0.0, half_height, 0.0, 1.0),
        color: colour_vec,
        tex_coords: Vector2::new(0.5, 0.5),
        normal: Vector3::y(),
    });
    for i in 0..segments {
        let angle = i as f32 * TAU / segments as f32;
        vertices.push(Vertex {
            pos: Vector4::new(angle.cos() * radius, half_height, angle.sin() * radius, 1.0),
            color: colour_vec,
            tex_coords: Vector2::new(0.5 + angle.cos() * 0.5, 0.5 + angle.sin() * 0.5),
            normal: Vector3::y(),
        });
    }
    for i in 0..segments {
        let curr = top_center + 1 + i;
        let next = top_center + 1 + (i + 1) % segments;
        indices.extend_from_slice(&[top_center, next, curr]);
    }

    // --- Bottom cap ---
    let bot_center = vertices.len() as u32;
    vertices.push(Vertex {
        pos: Vector4::new(0.0, -half_height, 0.0, 1.0),
        color: colour_vec,
        tex_coords: Vector2::new(0.5, 0.5),
        normal: -Vector3::y(),
    });
    for i in 0..segments {
        let angle = i as f32 * TAU / segments as f32;
        vertices.push(Vertex {
            pos: Vector4::new(
                angle.cos() * radius,
                -half_height,
                angle.sin() * radius,
                1.0,
            ),
            color: colour_vec,
            tex_coords: Vector2::new(0.5 + angle.cos() * 0.5, 0.5 - angle.sin() * 0.5),
            normal: -Vector3::y(),
        });
    }
    for i in 0..segments {
        let curr = bot_center + 1 + i;
        let next = bot_center + 1 + (i + 1) % segments;
        indices.extend_from_slice(&[bot_center, curr, next]);
    }

    (vertices, indices)
}

/// Build an orthonormal basis given a direction vector.
fn build_orthonormal_basis(direction: Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    // Pick a vector not parallel to direction
    let reference = if direction.y.abs() < 0.9 {
        Vector3::y()
    } else {
        Vector3::x()
    };

    let tangent = direction.cross(&reference).normalize();
    let bitangent = direction.cross(&tangent).normalize();

    (tangent, bitangent)
}
