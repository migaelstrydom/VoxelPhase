//! Geodesic spheres: an icosahedron subdivided, its vertices pushed onto the
//! unit sphere.
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

/// A geodesic unit sphere after `subdivisions` rounds of splitting every
/// triangle in four: 12 vertices at 0, 642 at 3, 2562 at 4.
pub fn generate_icosphere(subdivisions: u32) -> Icosphere {
    let t = (1.0 + 5.0f32.sqrt()) / 2.0;
    let mut positions: Vec<Vector3<f32>> = [
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
    let mut indices: Vec<u32> = vec![
        0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7,
        1, 8, 3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9, 4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9,
        8, 1,
    ];

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
}
