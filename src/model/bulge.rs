//! A mesh blown out of a surface, as a cheek is out of a face: sculpted
//! blown full, and sunk back under the surface it rises from when empty.
//!
//! ```text
//!   blown ●───────●        each vertex moves on the line between where
//!          ╲  full ╱       it was sculpted and where it lies empty, so
//!   ───────●─────●───      the edge, where the bulge meets the surface,
//!   empty   ╲___╱          hardly moves and the crown moves most
//! ```
//!
//! Its owner shapes it each frame into a mesh drawn as it is (a
//! `ProceduralMesh`): unlike a part scaled about a point, its edge stays
//! on the surface it grows from at every size.

use nalgebra::{Matrix4, Point3, Vector3};

use super::parts::MeshPrimitive;
use crate::rendering::material::MaterialId;

/// A mesh between two shapes of it: blown full and empty.
#[derive(Clone)]
pub struct Bulge {
    /// The mesh blown full, in its owner's frame.
    blown: MeshPrimitive,
    /// Each vertex's position and normal when it is empty.
    empty: Vec<(Vector3<f32>, Vector3<f32>)>,
}

impl Bulge {
    /// `blown`, whose vertices lie at `empty`'s positions, facing its
    /// normals, when the bulge is empty: one to a vertex, in order.
    pub fn new(blown: MeshPrimitive, empty: Vec<(Vector3<f32>, Vector3<f32>)>) -> Self {
        assert_eq!(
            blown.vertices.len(),
            empty.len(),
            "a bulge needs its empty shape for every vertex"
        );
        Self { blown, empty }
    }

    /// The mesh blown full.
    pub fn blown(&self) -> &MeshPrimitive {
        &self.blown
    }

    /// The same bulge in another material.
    pub fn recoloured(&self, to: impl Fn(MaterialId) -> MaterialId) -> Self {
        let mut bulge = self.clone();
        bulge.blown.material = to(bulge.blown.material);
        bulge
    }

    /// Shape `out` as the bulge `full` of the way from empty to blown (past
    /// 1 is blown further, below 0 sunk further), placed by `place`.
    pub fn shape(&self, full: f32, place: &Matrix4<f32>, out: &mut MeshPrimitive) {
        if out.vertices.len() != self.blown.vertices.len() {
            *out = self.blown.clone();
        }
        // Normals go by the inverse transpose, which a non-uniform scale
        // needs.
        let turn = place.fixed_view::<3, 3>(0, 0).into_owned();
        let normals = turn
            .try_inverse()
            .map_or(turn, |inverse| inverse.transpose());
        for ((vertex, blown), (empty, faced)) in out
            .vertices
            .iter_mut()
            .zip(&self.blown.vertices)
            .zip(&self.empty)
        {
            let at = empty + (blown.pos - empty) * full;
            let facing = faced.lerp(&blown.normal, full.clamp(0.0, 1.0));
            vertex.pos = place.transform_point(&Point3::from(at)).coords;
            vertex.normal = (normals * facing)
                .try_normalize(1e-9)
                .unwrap_or(blown.normal);
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector2, Vector4};

    use super::*;
    use crate::rendering::vertex::Vertex;

    /// A square of two triangles lifted 1 m off the plane y = 0, its two
    /// near corners on the plane.
    fn bulge() -> Bulge {
        let vertex = |x: f32, y: f32, z: f32| Vertex {
            pos: Vector3::new(x, y, z),
            color: Vector4::repeat(1.0),
            tex_coords: Vector2::zeros(),
            normal: Vector3::new(0.0, 1.0, 1.0).normalize(),
            ao: 1.0,
        };
        let blown = MeshPrimitive {
            vertices: vec![
                vertex(0.0, 0.0, 0.0),
                vertex(1.0, 0.0, 0.0),
                vertex(1.0, 1.0, 1.0),
                vertex(0.0, 1.0, 1.0),
            ],
            indices: vec![0, 1, 2, 0, 2, 3],
            material: MaterialId(0),
        };
        let empty = blown
            .vertices
            .iter()
            .map(|vertex| (Vector3::new(vertex.pos.x, 0.0, vertex.pos.z), Vector3::y()))
            .collect();
        Bulge::new(blown, empty)
    }

    #[test]
    fn it_moves_from_empty_to_blown_and_its_edge_stays_put() {
        let bulge = bulge();
        let mut mesh = MeshPrimitive {
            vertices: Vec::new(),
            indices: Vec::new(),
            material: MaterialId(0),
        };
        let height = |mesh: &MeshPrimitive| mesh.vertices[2].pos.y;
        for (full, expected) in [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0), (1.2, 1.2)] {
            bulge.shape(full, &Matrix4::identity(), &mut mesh);
            assert!((height(&mesh) - expected).abs() < 1e-6);
            assert!(mesh.vertices[0].pos.norm() < 1e-6, "its edge moved");
        }
        bulge.shape(0.0, &Matrix4::identity(), &mut mesh);
        assert!((mesh.vertices[2].normal - Vector3::y()).norm() < 1e-6);
        assert_eq!(mesh.indices, bulge.blown().indices);

        // Placed, it is carried whole.
        let place = Matrix4::new_translation(&Vector3::new(0.0, 2.0, 0.0));
        bulge.shape(1.0, &place, &mut mesh);
        assert!((height(&mesh) - 3.0).abs() < 1e-6);
    }
}
