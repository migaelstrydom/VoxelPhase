//! Capsule mesh generation.
//!
//! Generates a capsule mesh: a cylinder body with hemisphere caps, oriented
//! along the Y axis. Uses the same UV sphere approach for the caps as
//! `sphere.rs`.

use std::f32::consts::{PI, TAU};

use nalgebra::{Vector2, Vector3};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// Generate a capsule mesh centered at the origin, oriented along Y.
///
/// `half_height` includes the caps (total height = 2 * half_height).
/// The cylinder body spans from `-(half_height - radius)` to `+(half_height - radius)`.
///
/// # Arguments
/// * `half_height` — half of the total capsule height (must be >= radius)
/// * `radius` — radius of the cylinder and hemisphere caps
/// * `segments` — circumference divisions
/// * `cap_rings` — latitude divisions per hemisphere cap
/// * `colour` — vertex colour
pub fn generate_capsule(
    half_height: f32,
    radius: f32,
    segments: u32,
    cap_rings: u32,
    colour: Colour,
) -> (Vec<Vertex>, Vec<u32>) {
    let colour_vec = colour.to_vec4();
    let seg_half = half_height - radius;

    // Total rings: cap_rings (top cap) + 1 (cylinder body) + cap_rings (bottom cap)
    // Vertex rows: top pole + cap_rings rings + 1 cylinder seam + cap_rings rings + bottom pole
    // Simplified: we generate a UV sphere and stretch the equatorial band.

    // We'll generate it as:
    //   - Top hemisphere: cap_rings + 1 rows (including pole and equator)
    //   - Cylinder body: 2 rows (top seam and bottom seam)
    //   - Bottom hemisphere: cap_rings + 1 rows (including equator and pole)
    //
    // Total vertex rows: (cap_rings + 1) + 2 + (cap_rings + 1) = 2 * cap_rings + 4
    // But the equator rows are shared, so: cap_rings + 1 (top) + 1 (cylinder bottom) + cap_rings + 1 (bottom pole) = 2*cap_rings + 3

    let total_rows = 2 * cap_rings + 2; // number of "strips"
    let verts_per_row = segments + 1;
    let num_rows_verts = total_rows + 1;

    let mut vertices = Vec::with_capacity((num_rows_verts * verts_per_row) as usize);
    let mut indices = Vec::new();

    // Helper to push a ring of vertices at a given (y_center, y_normal_component, xz_scale)
    let total_height = 2.0 * half_height;

    // Row 0: top pole
    // Rows 1..cap_rings: top hemisphere
    // Row cap_rings: top equator (cylinder top)
    // Row cap_rings+1: cylinder bottom equator
    // Rows cap_rings+2..2*cap_rings+1: bottom hemisphere
    // Row 2*cap_rings+2: bottom pole

    for row in 0..=total_rows {
        let (y, nx_scale, ny) = if row <= cap_rings {
            // Top hemisphere
            let theta = row as f32 * (PI / 2.0) / cap_rings as f32;
            let y = seg_half + radius * theta.cos();
            let xz = theta.sin();
            (y, xz, theta.cos())
        } else if row == cap_rings + 1 {
            // Cylinder bottom
            let y = -seg_half;
            (y, 1.0_f32, 0.0_f32)
        } else {
            // Bottom hemisphere: rows cap_rings+2 to 2*cap_rings+2
            let local_row = row - cap_rings - 1;
            let theta = local_row as f32 * (PI / 2.0) / cap_rings as f32;
            let y = -seg_half - radius * theta.sin();
            let xz = theta.cos();
            (y, xz, -theta.sin())
        };

        let v_coord = (half_height - y) / total_height;

        for seg in 0..=segments {
            let phi = seg as f32 * TAU / segments as f32;
            let cos_phi = phi.cos();
            let sin_phi = phi.sin();

            let px = nx_scale * cos_phi * radius;
            let pz = nx_scale * sin_phi * radius;

            let normal = Vector3::new(nx_scale * cos_phi, ny, nx_scale * sin_phi);
            let normal = if normal.magnitude_squared() > 1e-10 {
                normal.normalize()
            } else {
                if y > 0.0 {
                    Vector3::y()
                } else {
                    -Vector3::y()
                }
            };

            let u = seg as f32 / segments as f32;

            vertices.push(Vertex {
                pos: Vector3::new(px, y, pz),
                color: colour_vec,
                tex_coords: Vector2::new(u, v_coord),
                normal,
                ao: 1.0,
            });
        }
    }

    // Generate indices
    for row in 0..(total_rows - 1) {
        let row_start = row * verts_per_row;
        let next_row_start = (row + 1) * verts_per_row;

        for seg in 0..segments {
            let i0 = row_start + seg;
            let i1 = next_row_start + seg;
            let i2 = row_start + seg + 1;
            let i3 = next_row_start + seg + 1;

            if row != 0 {
                indices.push(i0);
                indices.push(i2);
                indices.push(i1);
            }

            if row != total_rows - 2 {
                indices.push(i2);
                indices.push(i3);
                indices.push(i1);
            }
        }
    }

    (vertices, indices)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generates_nonempty_mesh() {
        let (verts, indices) = generate_capsule(1.0, 0.5, 16, 8, Colour::WHITE);
        assert!(!verts.is_empty());
        assert!(!indices.is_empty());
        assert!(indices.len() % 3 == 0);
    }

    #[test]
    fn all_vertices_within_bounds() {
        let hh = 2.0;
        let r = 0.5;
        let (verts, _) = generate_capsule(hh, r, 16, 8, Colour::WHITE);
        for v in &verts {
            assert!(
                v.pos.y <= hh + 0.01,
                "y={} exceeds half_height={}",
                v.pos.y,
                hh
            );
            assert!(
                v.pos.y >= -hh - 0.01,
                "y={} below -half_height={}",
                v.pos.y,
                -hh
            );
            let xz = (v.pos.x * v.pos.x + v.pos.z * v.pos.z).sqrt();
            assert!(
                xz <= r + 0.01,
                "xz radius {} exceeds capsule radius {}",
                xz,
                r
            );
        }
    }

    #[test]
    fn degenerate_sphere_case() {
        let (verts, indices) = generate_capsule(0.5, 0.5, 12, 6, Colour::WHITE);
        assert!(!verts.is_empty());
        assert!(!indices.is_empty());
    }
}
