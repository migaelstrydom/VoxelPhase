//! The five Platonic solids: the only convex polyhedra whose faces are all
//! the same regular polygon, with the same number meeting at every corner.
//!
//! ```text
//!   PlatonicSolid ──vertices()──▶ corners on the unit sphere
//!                 ──polyhedron()─▶ Polyhedron { vertices, faces }
//!                                    faces found as the planes every other
//!                                    corner lies behind, wound outward
//!                                    (`Polyhedron::convex_hull`, which
//!                                    any corners in convex position take)
//!                                  ──edges()──▶ each edge once
//! ```
//!
//! Only the corners are written down. The faces follow from them, so there
//! is no table of indices to get wrong.

use std::f32::consts::TAU;

use nalgebra::Vector3;
use serde::Deserialize;

/// The golden ratio, which the icosahedron and dodecahedron are built on.
const PHI: f32 = 1.618_034;

/// How far off a face's plane a corner may lie and still be on it, m (or
/// on the unit sphere).
const ON_PLANE: f32 = 1e-4;

/// One of the five Platonic solids.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Deserialize)]
pub enum PlatonicSolid {
    /// Four triangles.
    Tetrahedron,
    /// Six squares.
    Cube,
    /// Eight triangles.
    Octahedron,
    /// Twenty triangles.
    Icosahedron,
    /// Twelve pentagons.
    Dodecahedron,
}

/// A convex polyhedron as corners and the faces between them.
#[derive(Clone, Debug)]
pub struct Polyhedron {
    /// Its corners.
    pub vertices: Vec<Vector3<f32>>,
    /// Each face as the indices of its corners in order round it,
    /// anticlockwise seen from outside.
    pub faces: Vec<Vec<usize>>,
}

impl PlatonicSolid {
    /// All five, fewest faces first.
    pub const ALL: [PlatonicSolid; 5] = [
        PlatonicSolid::Tetrahedron,
        PlatonicSolid::Cube,
        PlatonicSolid::Octahedron,
        PlatonicSolid::Icosahedron,
        PlatonicSolid::Dodecahedron,
    ];

    /// Its corners, on the sphere of unit radius about its centre.
    pub fn vertices(self) -> Vec<Vector3<f32>> {
        let corners: Vec<Vector3<f32>> = match self {
            PlatonicSolid::Tetrahedron => vec![
                Vector3::new(1.0, 1.0, 1.0),
                Vector3::new(1.0, -1.0, -1.0),
                Vector3::new(-1.0, 1.0, -1.0),
                Vector3::new(-1.0, -1.0, 1.0),
            ],
            PlatonicSolid::Cube => signs3().map(|(x, y, z)| Vector3::new(x, y, z)).collect(),
            PlatonicSolid::Octahedron => (0..3)
                .flat_map(|axis| {
                    [1.0, -1.0].map(|sign| {
                        let mut v = Vector3::zeros();
                        v[axis] = sign;
                        v
                    })
                })
                .collect(),
            PlatonicSolid::Icosahedron => cyclic(signs2().map(|(a, b)| (0.0, a, b * PHI))),
            PlatonicSolid::Dodecahedron => signs3()
                .map(|(x, y, z)| Vector3::new(x, y, z))
                .chain(cyclic(signs2().map(|(a, b)| (0.0, a / PHI, b * PHI))))
                .collect(),
        };
        corners.into_iter().map(|v| v.normalize()).collect()
    }

    /// Its corners and faces.
    pub fn polyhedron(self) -> Polyhedron {
        Polyhedron::convex_hull(self.vertices())
    }

    /// How many corners meet at a face: three, four or five.
    pub fn face_sides(self) -> usize {
        match self {
            PlatonicSolid::Cube => 4,
            PlatonicSolid::Dodecahedron => 5,
            _ => 3,
        }
    }

    /// The radius of the sphere that touches the middle of every face, as a
    /// share of the radius through its corners.
    pub fn inradius(self) -> f32 {
        match self {
            PlatonicSolid::Tetrahedron => 1.0 / 3.0,
            PlatonicSolid::Cube | PlatonicSolid::Octahedron => 1.0 / 3.0_f32.sqrt(),
            // Both are cut from the same sphere pair.
            PlatonicSolid::Icosahedron | PlatonicSolid::Dodecahedron => 0.794_654_5,
        }
    }
}

impl Polyhedron {
    /// The convex hull of `vertices`, every one of which must be a corner
    /// of it: a plane is a face's where every corner lies on it or behind
    /// it, and the corners on it are put in order round its middle. Its
    /// cost grows as the fourth power of the corners, so it is for a few
    /// dozen of them.
    pub fn convex_hull(vertices: Vec<Vector3<f32>>) -> Self {
        let n = vertices.len();
        let mut normals: Vec<Vector3<f32>> = Vec::new();
        let mut faces = Vec::new();
        for a in 0..n {
            for b in a + 1..n {
                for c in b + 1..n {
                    let (pa, pb, pc) = (vertices[a], vertices[b], vertices[c]);
                    let Some(mut normal) = (pb - pa).cross(&(pc - pa)).try_normalize(1e-9) else {
                        continue;
                    };
                    if normal.dot(&pa) < 0.0 {
                        normal = -normal;
                    }
                    let offset = normal.dot(&pa);
                    if vertices.iter().any(|v| normal.dot(v) > offset + ON_PLANE) {
                        continue;
                    }
                    if normals
                        .iter()
                        .any(|seen| seen.dot(&normal) > 1.0 - ON_PLANE)
                    {
                        continue;
                    }
                    let on: Vec<usize> = (0..n)
                        .filter(|&i| (normal.dot(&vertices[i]) - offset).abs() <= ON_PLANE)
                        .collect();
                    faces.push(wound_round(&vertices, on, normal));
                    normals.push(normal);
                }
            }
        }
        Self { vertices, faces }
    }

    /// Each edge once, as the indices of its two ends, lower first.
    pub fn edges(&self) -> Vec<(usize, usize)> {
        let mut edges: Vec<(usize, usize)> = self
            .faces
            .iter()
            .flat_map(|face| {
                (0..face.len()).map(move |i| {
                    let (a, b) = (face[i], face[(i + 1) % face.len()]);
                    (a.min(b), a.max(b))
                })
            })
            .collect();
        edges.sort_unstable();
        edges.dedup();
        edges
    }

    /// The outward normal of face `face`.
    pub fn face_normal(&self, face: usize) -> Vector3<f32> {
        let corners = &self.faces[face];
        let (a, b, c) = (
            self.vertices[corners[0]],
            self.vertices[corners[1]],
            self.vertices[corners[2]],
        );
        (b - a).cross(&(c - a)).normalize()
    }

    /// The middle of face `face`.
    pub fn face_centre(&self, face: usize) -> Vector3<f32> {
        let corners = &self.faces[face];
        corners
            .iter()
            .map(|&i| self.vertices[i])
            .sum::<Vector3<f32>>()
            / corners.len() as f32
    }
}

/// The corners `on` a face with outward `normal`, put in order round its
/// middle, anticlockwise seen from outside.
fn wound_round(vertices: &[Vector3<f32>], mut on: Vec<usize>, normal: Vector3<f32>) -> Vec<usize> {
    let middle = on.iter().map(|&i| vertices[i]).sum::<Vector3<f32>>() / on.len() as f32;
    let across = (vertices[on[0]] - middle).normalize();
    let up = normal.cross(&across);
    let angle = |i: usize| {
        let d = vertices[i] - middle;
        d.dot(&up).atan2(d.dot(&across)).rem_euclid(TAU)
    };
    on.sort_by(|&a, &b| angle(a).total_cmp(&angle(b)));
    on
}

/// Every pair of signs.
fn signs2() -> impl Iterator<Item = (f32, f32)> {
    [1.0, -1.0]
        .into_iter()
        .flat_map(|a| [1.0, -1.0].map(move |b| (a, b)))
}

/// Every triple of signs: the corners of a cube.
fn signs3() -> impl Iterator<Item = (f32, f32, f32)> {
    signs2().flat_map(|(a, b)| [1.0, -1.0].map(move |c| (a, b, c)))
}

/// Each point, and the two its coordinates make when turned round
/// x → y → z → x.
fn cyclic(points: impl Iterator<Item = (f32, f32, f32)>) -> Vec<Vector3<f32>> {
    points
        .flat_map(|(x, y, z)| {
            [
                Vector3::new(x, y, z),
                Vector3::new(z, x, y),
                Vector3::new(y, z, x),
            ]
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Corners, edges and faces, as the solids have them.
    fn counts(solid: PlatonicSolid) -> (usize, usize, usize) {
        match solid {
            PlatonicSolid::Tetrahedron => (4, 6, 4),
            PlatonicSolid::Cube => (8, 12, 6),
            PlatonicSolid::Octahedron => (6, 12, 8),
            PlatonicSolid::Icosahedron => (12, 30, 20),
            PlatonicSolid::Dodecahedron => (20, 30, 12),
        }
    }

    #[test]
    fn each_solid_has_its_corners_edges_and_faces() {
        for solid in PlatonicSolid::ALL {
            let shape = solid.polyhedron();
            let found = (shape.vertices.len(), shape.edges().len(), shape.faces.len());
            assert_eq!(found, counts(solid), "{solid:?}");
        }
    }

    #[test]
    fn every_face_is_the_same_regular_polygon() {
        for solid in PlatonicSolid::ALL {
            let shape = solid.polyhedron();
            let edge = {
                let (a, b) = shape.edges()[0];
                (shape.vertices[a] - shape.vertices[b]).norm()
            };
            for face in &shape.faces {
                assert_eq!(face.len(), solid.face_sides(), "{solid:?}");
                for i in 0..face.len() {
                    let side = (shape.vertices[face[i]]
                        - shape.vertices[face[(i + 1) % face.len()]])
                    .norm();
                    assert!(
                        (side - edge).abs() < 1e-4,
                        "{solid:?}: {side} against {edge}"
                    );
                }
            }
        }
    }

    /// Wound anticlockwise from outside, the normal the winding gives points
    /// away from the centre, and every corner of the face is on its plane.
    #[test]
    fn faces_are_flat_and_wound_outward() {
        for solid in PlatonicSolid::ALL {
            let shape = solid.polyhedron();
            for (index, face) in shape.faces.iter().enumerate() {
                let normal = shape.face_normal(index);
                let centre = shape.face_centre(index);
                assert!(normal.dot(&centre) > 0.0, "{solid:?} face {index} faces in");
                for &corner in face {
                    let off = normal.dot(&(shape.vertices[corner] - centre));
                    assert!(off.abs() < 1e-5, "{solid:?} face {index} is not flat");
                }
                assert!(
                    (centre.norm() - solid.inradius()).abs() < 1e-4,
                    "{solid:?}: face middle at {} against {}",
                    centre.norm(),
                    solid.inradius()
                );
            }
        }
    }
}
