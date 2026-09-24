//! A rectangular prism with every edge cut back: the shape of anything
//! polished enough that its shine has to come from geometry.
//!
//! A polished surface shows almost nothing on a flat face — a mirror-finish
//! plane reflects the sun in one direction and is dull from every other — so
//! a glossy block with square edges reads as a grey slab. The twelve narrow
//! bevels catch the key light at whatever angle the block has tumbled to, and
//! the eight corner facets do the same for the parts of the sky the bevels
//! miss.

use nalgebra::{Vector2, Vector3};

use super::models::{dominant_axis, tangent_axes};
use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// A rectangular prism of the given half-extents with every edge cut back by
/// `bevel`.
///
/// The solid is the box intersected with its twelve edge planes, which gives
/// 24 vertices — every permutation of one coordinate at its half-extent and
/// the other two at `half - bevel` — arranged as six rectangular faces, twelve
/// rectangular bevels and eight corner triangles.
///
/// The cut-back is one distance rather than one fraction per axis, which is
/// what keeps every bevel a true 45° facet between the two faces it joins.
///
/// Flat-shaded: each face carries its own copies of its vertices with the
/// face's own normal, because a shared normal would round the bevels off into
/// exactly the soft edge they exist to avoid.
pub fn bevelled_box(half: Vector3<f32>, bevel: f32, uv_scale: f32) -> (Vec<Vertex>, Vec<u32>) {
    let inner = half.map(|h| h - bevel);
    let mut builder = FaceBuilder::new(uv_scale);

    // Six faces, each a rectangle inset to the bevel.
    for axis in 0..3 {
        for sign in [1.0_f32, -1.0] {
            let normal = axis_vector(axis, sign);
            let (u_axis, v_axis) = tangent_axes(axis);
            // Wound so the face is counter-clockwise seen from outside, which
            // is what flips with the sign of the normal.
            let (first, second) = if sign > 0.0 {
                (u_axis, v_axis)
            } else {
                (v_axis, u_axis)
            };
            let u = axis_vector(first, 1.0) * inner[first];
            let v = axis_vector(second, 1.0) * inner[second];

            let centre = normal * half[axis];
            builder.quad(
                normal,
                [
                    centre - u - v,
                    centre + u - v,
                    centre + u + v,
                    centre - u + v,
                ],
            );
        }
    }

    // Twelve bevels, one per box edge: a rectangle spanning the shared axis.
    for axis in 0..3 {
        let (a, b) = tangent_axes(axis);
        for sign_a in [1.0_f32, -1.0] {
            for sign_b in [1.0_f32, -1.0] {
                let dir_a = axis_vector(a, sign_a);
                let dir_b = axis_vector(b, sign_b);
                let along = axis_vector(axis, 1.0);

                // The two vertices on each end of the cut edge, one lying on
                // the `a` face and one on the `b` face.
                let on_a = dir_a * half[a] + dir_b * inner[b];
                let on_b = dir_a * inner[a] + dir_b * half[b];

                let normal = (dir_a + dir_b).normalize();
                let corners = [
                    on_a - along * inner[axis],
                    on_b - along * inner[axis],
                    on_b + along * inner[axis],
                    on_a + along * inner[axis],
                ];
                builder.quad_facing(normal, corners);
            }
        }
    }

    // Eight corner triangles, one per box corner. The three cut points are one
    // bevel apart along each pair of axes, so the facet they span is the same
    // 45°-from-every-face plane a cube's corner facet is, whatever the box's
    // proportions.
    for sx in [1.0_f32, -1.0] {
        for sy in [1.0_f32, -1.0] {
            for sz in [1.0_f32, -1.0] {
                let sign = Vector3::new(sx, sy, sz);
                let normal = sign.normalize();
                let corners = [
                    Vector3::new(sx * half.x, sy * inner.y, sz * inner.z),
                    Vector3::new(sx * inner.x, sy * half.y, sz * inner.z),
                    Vector3::new(sx * inner.x, sy * inner.y, sz * half.z),
                ];
                builder.triangle_facing(normal, corners);
            }
        }
    }

    (builder.vertices, builder.indices)
}

/// Accumulates flat-shaded faces into one mesh.
struct FaceBuilder {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    /// Texture coordinates per metre.
    uv_scale: f32,
}

impl FaceBuilder {
    fn new(uv_scale: f32) -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            uv_scale,
        }
    }

    /// Texture coordinates for a point, projected down the face's dominant
    /// axis at a fixed world scale.
    ///
    /// A directional pattern or grain follows `v`, so this projection is
    /// what decides which way the markings run — per face, rather than per
    /// block. That is the right answer for a block: a real one shows a
    /// different section of the same structure on each face, and a single
    /// direction carried across all six would read as a printed wrapper.
    fn uv(&self, pos: Vector3<f32>, normal: Vector3<f32>) -> Vector2<f32> {
        let axis = dominant_axis(normal);
        let (u_axis, v_axis) = tangent_axes(axis);
        Vector2::new(
            pos[u_axis] * self.uv_scale + 0.5,
            pos[v_axis] * self.uv_scale + 0.5,
        )
    }

    fn push_vertex(&mut self, pos: Vector3<f32>, normal: Vector3<f32>) -> u32 {
        let index = self.vertices.len() as u32;
        self.vertices.push(Vertex {
            pos,
            color: Colour::WHITE.to_vec4(),
            tex_coords: self.uv(pos, normal),
            normal,
            ao: 1.0,
        });
        index
    }

    /// A quad with the given winding, as two triangles.
    fn quad(&mut self, normal: Vector3<f32>, corners: [Vector3<f32>; 4]) {
        let base = self.push_vertex(corners[0], normal);
        for corner in &corners[1..] {
            self.push_vertex(*corner, normal);
        }
        self.indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// A quad, flipped if needed so that it faces outwards.
    ///
    /// The bevels and corner facets are generated by sign permutation, which
    /// gets the geometry right and the winding right only half the time; the
    /// faces are generated by hand, where the winding is the more direct
    /// statement. Both end up with an outward face, which is the only
    /// information that is unambiguous.
    fn quad_facing(&mut self, normal: Vector3<f32>, corners: [Vector3<f32>; 4]) {
        let corners = if faces_outward(normal, corners[0], corners[1], corners[2]) {
            corners
        } else {
            [corners[3], corners[2], corners[1], corners[0]]
        };
        self.quad(normal, corners);
    }

    /// A triangle, flipped if needed so that it faces outwards.
    fn triangle_facing(&mut self, normal: Vector3<f32>, corners: [Vector3<f32>; 3]) {
        let corners = if faces_outward(normal, corners[0], corners[1], corners[2]) {
            corners
        } else {
            [corners[2], corners[1], corners[0]]
        };

        let base = self.push_vertex(corners[0], normal);
        self.push_vertex(corners[1], normal);
        self.push_vertex(corners[2], normal);
        self.indices.extend_from_slice(&[base, base + 1, base + 2]);
    }
}

/// Whether a triangle wound `a, b, c` has its geometric normal on the same
/// side as `normal`.
fn faces_outward(normal: Vector3<f32>, a: Vector3<f32>, b: Vector3<f32>, c: Vector3<f32>) -> bool {
    (b - a).cross(&(c - a)).dot(&normal) > 0.0
}

/// The unit vector along `axis`, times `sign`.
fn axis_vector(axis: usize, sign: f32) -> Vector3<f32> {
    let mut v = Vector3::zeros();
    v[axis] = sign;
    v
}

#[cfg(test)]
/// The directions the faces of a mesh point in, one entry each.
pub fn distinct_normals(vertices: &[Vertex]) -> Vec<Vector3<f32>> {
    let mut seen: Vec<Vector3<f32>> = Vec::new();
    for vertex in vertices {
        if !seen.iter().any(|n| (n - vertex.normal).norm() < 1e-3) {
            seen.push(vertex.normal);
        }
    }
    seen
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A cube and a slab, because every property here is one a generalisation
    /// from a cube can lose.
    fn shapes() -> [Vector3<f32>; 2] {
        [cube(), slab()]
    }

    fn cube() -> Vector3<f32> {
        Vector3::new(0.5, 0.5, 0.5)
    }

    fn slab() -> Vector3<f32> {
        Vector3::new(2.5, 0.5, 1.2)
    }

    fn mesh(half: Vector3<f32>) -> (Vec<Vertex>, Vec<u32>) {
        bevelled_box(half, half.min() * 0.14, 1.0)
    }

    /// Six rectangles, twelve bevel rectangles and eight corner triangles:
    /// 2 + 2 + 2 triangles' worth per quad and one per corner.
    #[test]
    fn the_block_has_every_face_a_bevelled_box_should() {
        for half in shapes() {
            let (_, indices) = mesh(half);
            let triangles = indices.len() / 3;
            assert_eq!(triangles, 6 * 2 + 12 * 2 + 8);
        }
    }

    /// The property a bevel exists for: no vertex sits on a box corner any
    /// more. A vertex with all three coordinates at a half-extent would mean
    /// the chamfer never happened.
    #[test]
    fn no_vertex_is_left_on_a_sharp_corner() {
        for half in shapes() {
            let (vertices, _) = mesh(half);
            for vertex in &vertices {
                let at_extent = (0..3)
                    .filter(|&axis| (vertex.pos[axis].abs() - half[axis]).abs() < 1e-5)
                    .count();
                assert_eq!(
                    at_extent, 1,
                    "vertex {:?} lies on {} faces of the box",
                    vertex.pos, at_extent
                );
            }
        }
    }

    /// Every face must point away from the centre. A single inverted winding
    /// is invisible on an opaque object under back-face culling and glaring on
    /// a transparent one, where both sides are drawn — which is exactly why it
    /// is worth a test here and was never worth one on the crate.
    #[test]
    fn every_triangle_faces_outwards() {
        for half in shapes() {
            let (vertices, indices) = mesh(half);
            for triangle in indices.chunks_exact(3) {
                let a = vertices[triangle[0] as usize].pos;
                let b = vertices[triangle[1] as usize].pos;
                let c = vertices[triangle[2] as usize].pos;
                let centroid = (a + b + c) / 3.0;
                let geometric = (b - a).cross(&(c - a));
                assert!(
                    geometric.dot(&centroid) > 0.0,
                    "triangle at {centroid:?} is wound inside out"
                );
            }
        }
    }

    /// And every vertex normal must agree with the face it belongs to, or the
    /// flat shading the bevels depend on has been smoothed away.
    #[test]
    fn each_face_carries_its_own_normal() {
        for half in shapes() {
            let (vertices, indices) = mesh(half);
            for triangle in indices.chunks_exact(3) {
                let normals: Vec<Vector3<f32>> = triangle
                    .iter()
                    .map(|&i| vertices[i as usize].normal)
                    .collect();
                assert!((normals[0] - normals[1]).norm() < 1e-5);
                assert!((normals[0] - normals[2]).norm() < 1e-5);
                assert!((normals[0].norm() - 1.0).abs() < 1e-5);
            }
        }
    }

    /// The block must not outgrow the box collider spawned alongside it, or it
    /// would visibly sink into whatever it lands on.
    #[test]
    fn nothing_pokes_out_past_the_collider() {
        for half in shapes() {
            let (vertices, _) = mesh(half);
            for vertex in &vertices {
                for axis in 0..3 {
                    assert!(
                        vertex.pos[axis].abs() <= half[axis] + 1e-5,
                        "{:?}",
                        vertex.pos
                    );
                }
            }
        }
    }

    /// A slab must actually reach its authored extents on every axis — the
    /// failure mode of a generalisation from a cube is a block that is drawn
    /// as a cube of the smallest half-extent inside its own collider.
    #[test]
    fn a_slab_fills_its_half_extents() {
        let (vertices, _) = mesh(slab());
        for axis in 0..3 {
            let reach = vertices
                .iter()
                .fold(0.0_f32, |acc, v| acc.max(v.pos[axis].abs()));
            assert!(
                (reach - slab()[axis]).abs() < 1e-5,
                "axis {axis} reaches {reach}, wanted {}",
                slab()[axis]
            );
        }
    }

    /// Every bevel facet is a 45° cut between the two faces it joins, on a
    /// slab as much as on a cube. This is the property that fails the moment
    /// the cut-back is authored per axis rather than as one distance.
    #[test]
    fn bevels_are_true_forty_five_degree_facets() {
        let (vertices, indices) = mesh(slab());
        for triangle in indices.chunks_exact(3) {
            let normal = vertices[triangle[0] as usize].normal;
            let components: Vec<f32> = (0..3).map(|axis| normal[axis].abs()).collect();
            let off_axis = components.iter().filter(|c| **c > 1e-4).count();
            // A face normal has one component, a bevel two, a corner three;
            // and whenever there is more than one they must be equal.
            if off_axis > 1 {
                let first = components.iter().find(|c| **c > 1e-4).unwrap();
                for component in components.iter().filter(|c| **c > 1e-4) {
                    assert!((component - first).abs() < 1e-4, "normal {normal:?}");
                }
            }
        }
    }
}
