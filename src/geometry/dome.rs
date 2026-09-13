//! A thick spherical cap — a dome with a wall.
//!
//! The shape you want whenever something has to *cover* a sphere: an
//! eyelid over an eye, a shell over a dome, a cap over a post. An offset
//! second sphere covers the same region, but it reads as a ball balanced
//! on top rather than as a lid, because its silhouette is a ball's.
//!
//! Built with two surfaces and a rim rather than as an open shell, so it
//! is solid from every angle and has no unlit edge where it ends.

use std::f32::consts::TAU;

use nalgebra::{Vector2, Vector3};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// A dome covering the region within `half_angle` of local +Y.
#[derive(Clone, Copy, Debug)]
pub struct DomeSpec {
    /// Radius of the outer surface.
    pub outer_radius: f32,
    /// Wall thickness. The inner surface sits this far inside the outer
    /// one, so a dome laid over a sphere of `outer_radius - thickness`
    /// fits it exactly.
    pub thickness: f32,
    /// How far down from the pole the dome reaches, in radians. At
    /// `PI / 2` it is a hemisphere; beyond that it starts to close around
    /// whatever it covers.
    pub half_angle: f32,
    /// Subdivisions around the axis.
    pub segments: u32,
    /// Subdivisions from pole to rim.
    pub rings: u32,
    pub colour: Colour,
}

/// Vertices and indices for one dome, centred on the origin with its pole
/// along +Y.
pub fn generate_dome(spec: &DomeSpec) -> (Vec<Vertex>, Vec<u32>) {
    let segments = spec.segments.max(3);
    let rings = spec.rings.max(1);
    let half_angle = spec.half_angle.clamp(1e-3, std::f32::consts::PI);
    let outer = spec.outer_radius.max(1e-4);
    let inner = (outer - spec.thickness.max(0.0)).max(1e-4);
    let colour = spec.colour.to_vec4();

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Two grids of the same shape: the outer surface facing out, the inner
    // one facing in. The rim then stitches their last rings together.
    for (radius, facing_out) in [(outer, true), (inner, false)] {
        for ring in 0..=rings {
            let polar = half_angle * ring as f32 / rings as f32;
            let (sin_polar, cos_polar) = polar.sin_cos();

            for segment in 0..=segments {
                let azimuth = segment as f32 * TAU / segments as f32;
                let (sin_azimuth, cos_azimuth) = azimuth.sin_cos();
                let direction =
                    Vector3::new(sin_polar * cos_azimuth, cos_polar, sin_polar * sin_azimuth);

                vertices.push(Vertex {
                    pos: direction * radius,
                    color: colour,
                    tex_coords: Vector2::new(
                        segment as f32 / segments as f32,
                        ring as f32 / rings as f32,
                    ),
                    normal: if facing_out { direction } else { -direction },
                    ao: 1.0,
                });
            }
        }
    }

    let stride = segments + 1;
    let surface = (rings + 1) * stride;

    for (base, facing_out) in [(0, true), (surface, false)] {
        for ring in 0..rings {
            for segment in 0..segments {
                let i0 = base + ring * stride + segment;
                let i1 = base + (ring + 1) * stride + segment;
                let i2 = i0 + 1;
                let i3 = i1 + 1;

                // The inner surface is the outer one seen from behind, so
                // its triangles wind the other way.
                if facing_out {
                    indices.extend([i0, i2, i1, i2, i3, i1]);
                } else {
                    indices.extend([i0, i1, i2, i2, i1, i3]);
                }
            }
        }
    }

    // The rim: a band joining the two last rings, facing outward from the
    // axis and down.
    let outer_rim = rings * stride;
    let inner_rim = surface + rings * stride;
    for segment in 0..segments {
        let o0 = outer_rim + segment;
        let o1 = o0 + 1;
        let i0 = inner_rim + segment;
        let i1 = i0 + 1;
        indices.extend([o0, o1, i0, o1, i1, i0]);
    }

    let rim_normal = |polar: f32, azimuth: f32| {
        let (sin_polar, cos_polar) = polar.sin_cos();
        let (sin_azimuth, cos_azimuth) = azimuth.sin_cos();
        // Perpendicular to the surface at the rim, pointing away from the
        // pole: the direction the wall faces.
        Vector3::new(cos_polar * cos_azimuth, -sin_polar, cos_polar * sin_azimuth)
    };
    for segment in 0..=segments {
        let azimuth = segment as f32 * TAU / segments as f32;
        let normal = rim_normal(half_angle, azimuth);
        vertices[(outer_rim + segment) as usize].normal = normal;
        vertices[(inner_rim + segment) as usize].normal = normal;
    }

    (vertices, indices)
}
