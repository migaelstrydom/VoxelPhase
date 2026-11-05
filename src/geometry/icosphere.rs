//! Geodesic spheres: an icosahedron subdivided, its vertices pushed onto the
//! unit sphere. [`generate_icosphere`] splits every triangle in four, round
//! after round; [`generate_geodesic`] cuts each of the icosahedron's faces
//! into any number of rows, for a sphere of any resolution.
//!
//! Unlike a latitude–longitude sphere, the vertices are spread nearly evenly
//! with no poles, so a surface displaced from it (bumps, nubs) has the same
//! resolution everywhere.

use std::collections::HashMap;

use nalgebra::Vector3;

/// A unit sphere: positions (each also its own normal) and counter-clockwise
/// triangles seen from outside.
pub struct Icosphere {
    /// Vertex positions on the unit sphere.
    pub positions: Vec<Vector3<f32>>,
    /// Triangle list, three indices per face.
    pub indices: Vec<u32>,
}

/// The unit icosahedron: 12 vertices and 20 counter-clockwise faces.
fn icosahedron() -> Icosphere {
    let t = (1.0 + 5.0f32.sqrt()) / 2.0;
    let positions: Vec<Vector3<f32>> = [
        (-1.0, t, 0.0),
        (1.0, t, 0.0),
        (-1.0, -t, 0.0),
        (1.0, -t, 0.0),
        (0.0, -1.0, t),
        (0.0, 1.0, t),
        (0.0, -1.0, -t),
        (0.0, 1.0, -t),
        (t, 0.0, -1.0),
        (t, 0.0, 1.0),
        (-t, 0.0, -1.0),
        (-t, 0.0, 1.0),
    ]
    .iter()
    .map(|&(x, y, z)| Vector3::new(x, y, z).normalize())
    .collect();
    let indices: Vec<u32> = vec![
        0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7,
        1, 8, 3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9, 4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9,
        8, 1,
    ];

    Icosphere { positions, indices }
}

/// A geodesic unit sphere after `subdivisions` rounds of splitting every
/// triangle in four: 12 vertices at 0, 642 at 3, 2562 at 4.
pub fn generate_icosphere(subdivisions: u32) -> Icosphere {
    let Icosphere {
        mut positions,
        mut indices,
    } = icosahedron();
    for _ in 0..subdivisions {
        let mut midpoints: HashMap<(u32, u32), u32> = HashMap::new();
        let mut midpoint = |a: u32, b: u32, positions: &mut Vec<Vector3<f32>>| {
            let key = (a.min(b), a.max(b));
            *midpoints.entry(key).or_insert_with(|| {
                let p = ((positions[a as usize] + positions[b as usize]) * 0.5).normalize();
                positions.push(p);
                (positions.len() - 1) as u32
            })
        };
        let mut next = Vec::with_capacity(indices.len() * 4);
        for face in indices.chunks_exact(3) {
            let (a, b, c) = (face[0], face[1], face[2]);
            let ab = midpoint(a, b, &mut positions);
            let bc = midpoint(b, c, &mut positions);
            let ca = midpoint(c, a, &mut positions);
            next.extend_from_slice(&[a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca]);
        }
        indices = next;
    }

    Icosphere { positions, indices }
}

/// A geodesic unit sphere whose every icosahedron face is cut into
/// `frequency` rows: `10·f² + 2` vertices, `20·f²` triangles. A frequency
/// of `2^n` has the topology of [`generate_icosphere`]`(n)`.
pub fn generate_geodesic(frequency: u32) -> Icosphere {
    let f = frequency.max(1);
    let base = icosahedron();
    let mut positions = Vec::new();
    let mut indices = Vec::with_capacity(base.indices.len() / 3 * (f * f * 3) as usize);
    // Vertices on a face's edges are shared with its neighbour: one each,
    // found by position.
    let mut seen: HashMap<[i32; 3], u32> = HashMap::new();
    let mut vertex = |p: Vector3<f32>, positions: &mut Vec<Vector3<f32>>| {
        let p = p.normalize();
        let key = [p.x, p.y, p.z].map(|c| (c * 1.0e5).round() as i32);
        *seen.entry(key).or_insert_with(|| {
            positions.push(p);
            (positions.len() - 1) as u32
        })
    };
    for face in base.indices.chunks_exact(3) {
        let [a, b, c] = [0, 1, 2].map(|i| base.positions[face[i] as usize]);
        let (along_b, along_c) = ((b - a) / f as f32, (c - a) / f as f32);
        // Row-major over the face's triangular grid: (i, j) for i + j <= f.
        let mut grid = Vec::new();
        for i in 0..=f {
            for j in 0..=(f - i) {
                grid.push(vertex(
                    a + along_b * i as f32 + along_c * j as f32,
                    &mut positions,
                ));
            }
        }
        let at = |i: u32, j: u32| {
            let row: u32 = (0..i).map(|r| f - r + 1).sum();
            grid[(row + j) as usize]
        };
        for i in 0..f {
            for j in 0..(f - i) {
                indices.extend_from_slice(&[at(i, j), at(i + 1, j), at(i, j + 1)]);
                if i + j + 1 < f {
                    indices.extend_from_slice(&[at(i + 1, j), at(i + 1, j + 1), at(i, j + 1)]);
                }
            }
        }
    }
    Icosphere { positions, indices }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every vertex is on the unit sphere and every face winds outward.
    #[test]
    fn faces_wind_outward_on_the_unit_sphere() {
        let sphere = generate_icosphere(2);
        assert_eq!(sphere.positions.len(), 162);
        for p in &sphere.positions {
            assert!((p.magnitude() - 1.0).abs() < 1e-5);
        }
        for face in sphere.indices.chunks_exact(3) {
            let [a, b, c] = [0, 1, 2].map(|i| sphere.positions[face[i] as usize]);
            let normal = (b - a).cross(&(c - a));
            assert!(normal.dot(&(a + b + c)) > 0.0, "a face winds inward");
        }
    }

    /// A geodesic sphere of any frequency has the expected counts, its
    /// vertices on the sphere and its faces wound outward.
    #[test]
    fn a_geodesic_of_any_frequency_is_closed_and_outward() {
        for f in [1, 3, 7] {
            let sphere = generate_geodesic(f);
            assert_eq!(sphere.positions.len() as u32, 10 * f * f + 2);
            assert_eq!(sphere.indices.len() as u32, 3 * 20 * f * f);
            for face in sphere.indices.chunks_exact(3) {
                let [a, b, c] = [0, 1, 2].map(|i| sphere.positions[face[i] as usize]);
                assert!((b - a).cross(&(c - a)).dot(&(a + b + c)) > 0.0);
            }
        }
    }
}
