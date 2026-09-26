//! The barrel's render mesh: a smooth-shaded side and two flat heads.
//!
//! Finer than the contact hull and shaded smooth, so the belly reads as a
//! curve; the two differ by a few millimetres, which no one sees.

use std::f32::consts::TAU;

use nalgebra::{Vector2, Vector3};

use super::profile::BarrelProfile;
use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// A mesh as its vertices and triangle indices.
pub type MeshData = (Vec<Vertex>, Vec<u32>);

/// The side: `segments` around and `rings` from head to head.
///
/// Texture `u` runs once round the barrel and `v` from the top head (0) to
/// the bottom one (1), so a stave in the texture is a stave on the barrel.
pub fn side_mesh(profile: &BarrelProfile, segments: u32, rings: u32) -> MeshData {
    let colour = Colour::WHITE.to_vec4();
    let columns = segments + 1;
    let mut vertices = Vec::with_capacity((columns * rings) as usize);
    let mut indices = Vec::with_capacity((segments * (rings - 1) * 6) as usize);

    for ring in 0..rings {
        let y = profile.ring_height(ring as usize, rings as usize);
        let radius = profile.radius_at(y);
        let slope = profile.slope_at(y);
        let v = (profile.half_height - y) / (2.0 * profile.half_height);

        // The seam column repeats the first at u = 1 so the texture wraps.
        for column in 0..columns {
            let u = column as f32 / segments as f32;
            let (sin, cos) = (u * TAU).sin_cos();
            vertices.push(Vertex {
                pos: Vector3::new(radius * cos, y, radius * sin),
                color: colour,
                tex_coords: Vector2::new(u, v),
                normal: Vector3::new(cos, -slope, sin).normalize(),
                ao: 1.0,
            });
        }
    }

    for band in 0..rings - 1 {
        for column in 0..segments {
            let lower = band * columns + column;
            let upper = lower + columns;
            indices.extend_from_slice(&[lower, upper, lower + 1, lower + 1, upper, upper + 1]);
        }
    }

    (vertices, indices)
}

/// Both heads as flat discs, textured across their diameter.
pub fn head_meshes(profile: &BarrelProfile, segments: u32) -> MeshData {
    let mut vertices = Vec::with_capacity(((segments + 1) * 2) as usize);
    let mut indices = Vec::with_capacity((segments * 6) as usize);
    for up in [true, false] {
        append_head(profile, segments, up, &mut vertices, &mut indices);
    }
    (vertices, indices)
}

/// One head, at the top when `up`, wound to face outward.
fn append_head(
    profile: &BarrelProfile,
    segments: u32,
    up: bool,
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
) {
    let colour = Colour::WHITE.to_vec4();
    let (y, normal) = if up {
        (profile.half_height, Vector3::y())
    } else {
        (-profile.half_height, -Vector3::y())
    };
    let radius = profile.head_radius;

    let centre = vertices.len() as u32;
    vertices.push(Vertex {
        pos: Vector3::new(0.0, y, 0.0),
        color: colour,
        tex_coords: Vector2::new(0.5, 0.5),
        normal,
        ao: 1.0,
    });
    for segment in 0..segments {
        let (sin, cos) = (segment as f32 * TAU / segments as f32).sin_cos();
        vertices.push(Vertex {
            pos: Vector3::new(radius * cos, y, radius * sin),
            color: colour,
            tex_coords: Vector2::new(0.5 + 0.5 * cos, 0.5 + 0.5 * sin),
            normal,
            ao: 1.0,
        });
    }
    for segment in 0..segments {
        let current = centre + 1 + segment;
        let next = centre + 1 + (segment + 1) % segments;
        if up {
            indices.extend_from_slice(&[centre, next, current]);
        } else {
            indices.extend_from_slice(&[centre, current, next]);
        }
    }
}
