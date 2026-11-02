//! Cylinder mesh generation.
//!
//! Generates cylinder meshes between two points, useful for debug line rendering.

use std::f32::consts::TAU;

use nalgebra::{Point3, Vector2, Vector3};

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

    let mut vertices = Vec::with_capacity(((segments + 1) * 2) as usize);
    let mut indices = Vec::with_capacity((segments * 6) as usize);

    // Generate vertices for both caps
    // One column more than there are faces: the seam's two columns share a
    // position and differ in u, 0 and 1, so the last face runs to the end of
    // the texture rather than back across all of it.
    for i in 0..=segments {
        let angle = i as f32 * TAU / segments as f32;
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        let offset = tangent * cos_a * radius + bitangent * sin_a * radius;
        let normal = (tangent * cos_a + bitangent * sin_a).normalize();

        // Bottom cap vertex
        let bottom_pos = start + offset;
        vertices.push(Vertex {
            pos: Vector3::new(bottom_pos.x, bottom_pos.y, bottom_pos.z),
            color: colour_vec,
            tex_coords: Vector2::new(i as f32 / segments as f32, 0.0),
            normal,
            ao: 1.0,
        });

        // Top cap vertex
        let top_pos = end + offset;
        vertices.push(Vertex {
            pos: Vector3::new(top_pos.x, top_pos.y, top_pos.z),
            color: colour_vec,
            tex_coords: Vector2::new(i as f32 / segments as f32, 1.0),
            normal,
            ao: 1.0,
        });
    }

    // Generate indices for the cylinder sides
    for i in 0..segments {
        let i0 = i * 2;
        let i1 = i * 2 + 1;
        let i2 = (i + 1) * 2;
        let i3 = (i + 1) * 2 + 1;

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

/// The widest span of u across any one triangle. A wrapped seam shows as a
/// triangle spanning almost the whole texture.
#[cfg(test)]
pub(crate) fn widest_u_span(vertices: &[Vertex], indices: &[u32]) -> f32 {
    indices
        .chunks_exact(3)
        .map(|triangle| {
            let us = triangle.iter().map(|&i| vertices[i as usize].tex_coords.x);
            let (lo, hi) = us.fold((f32::MAX, f32::MIN), |(lo, hi), u| (lo.min(u), hi.max(u)));
            hi - lo
        })
        .fold(0.0, f32::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The texture goes once round the cylinder: no face spans more than its
    /// own share of u, the seam's included.
    #[test]
    fn the_texture_wraps_once_round_without_a_backwards_face() {
        let segments = 16;
        let (vertices, indices) = generate_cylinder(
            Point3::origin(),
            Point3::new(0.0, 2.0, 0.0),
            0.5,
            segments,
            Colour::WHITE,
        );
        let share = 1.0 / segments as f32;
        assert!(widest_u_span(&vertices, &indices) <= share + 1e-5);
    }
}
