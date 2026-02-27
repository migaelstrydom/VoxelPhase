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

/// Configuration for magic sphere visual effects.
#[derive(Clone)]
pub struct MagicSphereConfig {
    /// Base hue (0.0 to 1.0, wraps around)
    pub base_hue: f32,
    /// How many spiral arms/bands
    pub spiral_frequency: f32,
    /// How tightly wound the spiral is
    pub spiral_tightness: f32,
    /// Secondary color hue offset (0.0 to 1.0)
    pub accent_hue_offset: f32,
    /// How much the colors shift across the surface
    pub color_variation: f32,
    /// Brightness/saturation style
    pub glow_intensity: f32,
}

impl Default for MagicSphereConfig {
    fn default() -> Self {
        Self {
            base_hue: 0.0,
            spiral_frequency: 3.0,
            spiral_tightness: 2.0,
            accent_hue_offset: 0.33,
            color_variation: 0.15,
            glow_intensity: 1.0,
        }
    }
}

/// Convert HSV to RGB.
fn hsv_to_rgb(h: f32, s: f32, v: f32) -> (f32, f32, f32) {
    let h = h.fract();
    let h = if h < 0.0 { h + 1.0 } else { h };
    let i = (h * 6.0).floor() as i32;
    let f = h * 6.0 - i as f32;
    let p = v * (1.0 - s);
    let q = v * (1.0 - f * s);
    let t = v * (1.0 - (1.0 - f) * s);

    match i % 6 {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        _ => (v, p, q),
    }
}

/// Generate a magic sphere with swirling cosmic patterns.
///
/// Creates a sphere with procedurally generated per-vertex colors that form
/// swirling, iridescent patterns visible when the sphere rotates.
///
/// # Arguments
/// * `radius` - Sphere radius
/// * `segments` - Number of horizontal segments (longitude divisions)
/// * `rings` - Number of vertical rings (latitude divisions)
/// * `config` - Magic sphere visual configuration
pub fn generate_magic_sphere_vertices(
    radius: f32,
    segments: u32,
    rings: u32,
    config: &MagicSphereConfig,
) -> Vec<Vertex> {
    let mut vertices = Vec::with_capacity(((rings + 1) * (segments + 1)) as usize);

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

            // Create swirling pattern using spherical coordinates
            // Frequencies must be integers for seamless wrapping around the sphere
            let freq1 = config.spiral_frequency.round();
            let freq2 = (config.spiral_frequency * 1.5).round();

            let spiral = phi + theta * config.spiral_tightness;
            let spiral_value = (spiral * freq1).sin() * 0.5 + 0.5;

            // Secondary spiral going the other direction for complexity
            let counter_spiral = phi - theta * config.spiral_tightness * 0.7;
            let counter_value = (counter_spiral * freq2).sin() * 0.5 + 0.5;

            // Latitude-based variation (poles vs equator)
            let latitude_factor = (theta * 2.0).sin();

            // Mix spirals for final pattern
            let pattern = spiral_value * 0.6 + counter_value * 0.3 + latitude_factor * 0.1;

            // Calculate hue with variation
            let hue_shift =
                pattern * config.color_variation + (y * 0.5 + 0.5) * config.color_variation;
            let primary_hue = config.base_hue + hue_shift;
            let secondary_hue = config.base_hue + config.accent_hue_offset + hue_shift * 0.5;

            // Blend between primary and secondary colors based on spiral
            let blend = spiral_value * spiral_value; // More contrast
            let final_hue = primary_hue * (1.0 - blend) + secondary_hue * blend;

            // Saturation varies slightly for depth
            let saturation = 0.7 + spiral_value * 0.3;

            // Value/brightness with glow effect at certain bands
            let glow_freq = (config.spiral_frequency * 2.0).round();
            let glow_band = ((spiral * glow_freq).sin() * 0.5 + 0.5).powf(3.0);
            let value = (0.7 + glow_band * 0.3) * config.glow_intensity;

            let (r, g, b) = hsv_to_rgb(final_hue, saturation, value.min(1.0));

            vertices.push(Vertex {
                pos: Vector4::new(position.x, position.y, position.z, 1.0),
                color: Vector4::new(r, g, b, 1.0),
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

            if ring != 0 {
                // First triangle
                indices.push(i0);
                indices.push(i2);
                indices.push(i1);
            }

            if ring != rings - 1 {
                // Second triangle
                indices.push(i2);
                indices.push(i3);
                indices.push(i1);
            }
        }
    }

    indices
}
