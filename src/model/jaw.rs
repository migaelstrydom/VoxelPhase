//! Flesh that opens as a jaw does: sculpted as one with the face it belongs
//! to, and each vertex turned about the jaw's hinge by its own share of how
//! far the jaw opens.
//!
//! ```text
//!   share 0 ─────────────╮      the face above the mouth, and the edge
//!             upper lip ═╡      where the flesh meets the rest of the
//!   hinge ●              │      skin, stay where they were sculpted; the
//!             lower lip ═╡      lip and chin turn whole; the skin between
//!   share 1 ─── chin ────╯      them stretches as the jaw drops
//! ```
//!
//! Its owner shapes it each frame into meshes drawn as they are (a
//! `ProceduralMesh`), as a [`super::Bulge`] is: a jaw that is a part of its
//! own shows where it meets the face; this one is the face.

use nalgebra::{UnitQuaternion, Vector3};

use super::parts::MeshPrimitive;
use crate::rendering::material::MaterialId;

/// Meshes whose vertices each turn by a share of one hinge's turn.
#[derive(Clone)]
pub struct Jaw {
    /// The meshes as sculpted, one to a material, in their owner's frame.
    rest: Vec<MeshPrimitive>,
    /// Each vertex's share of the jaw's turn, 0 to 1: one list to a mesh,
    /// in order.
    shares: Vec<Vec<f32>>,
    /// A point on the hinge, in the owner's frame.
    pivot: Vector3<f32>,
    /// The hinge's axis, signed so that a positive turn opens the jaw.
    axis: Vector3<f32>,
}

impl Jaw {
    /// `rest` hinged at `pivot` about `axis` (unit), each vertex turning by
    /// its share in `shares`: one list to a mesh, one share to a vertex.
    pub fn new(
        rest: Vec<MeshPrimitive>,
        shares: Vec<Vec<f32>>,
        pivot: Vector3<f32>,
        axis: Vector3<f32>,
    ) -> Self {
        assert_eq!(rest.len(), shares.len(), "a jaw needs shares for each mesh");
        for (mesh, shares) in rest.iter().zip(&shares) {
            assert_eq!(
                mesh.vertices.len(),
                shares.len(),
                "a jaw needs a share for every vertex"
            );
        }
        Self {
            rest,
            shares,
            pivot,
            axis: axis.normalize(),
        }
    }

    /// The meshes as sculpted.
    pub fn rest(&self) -> &[MeshPrimitive] {
        &self.rest
    }

    /// The same jaw in other materials.
    pub fn recoloured(&self, to: impl Fn(MaterialId) -> MaterialId) -> Self {
        let mut jaw = self.clone();
        for mesh in &mut jaw.rest {
            mesh.material = to(mesh.material);
        }
        jaw
    }

    /// Shape `out`, one mesh to each of the jaw's, with the jaw turned
    /// `turn` radians open from as it was sculpted.
    pub fn shape(&self, turn: f32, out: &mut [MeshPrimitive]) {
        for ((mesh, rest), shares) in out.iter_mut().zip(&self.rest).zip(&self.shares) {
            if mesh.vertices.len() != rest.vertices.len() {
                *mesh = rest.clone();
            }
            for ((vertex, rest), share) in mesh.vertices.iter_mut().zip(&rest.vertices).zip(shares)
            {
                let turned = UnitQuaternion::from_scaled_axis(self.axis * (turn * share));
                vertex.pos = self.pivot + turned * (rest.pos - self.pivot);
                vertex.normal = turned * rest.normal;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector2, Vector4};

    use super::*;
    use crate::rendering::vertex::Vertex;

    fn vertex(y: f32, z: f32) -> Vertex {
        Vertex {
            pos: Vector3::new(0.0, y, z),
            color: Vector4::repeat(1.0),
            tex_coords: Vector2::zeros(),
            normal: Vector3::z(),
            ao: 1.0,
        }
    }

    /// A lip above the hinge, a lip below, and the skin between them and
    /// the face, hinged across X at the origin.
    fn jaw() -> Jaw {
        let mesh = MeshPrimitive {
            vertices: vec![vertex(0.1, 0.3), vertex(-0.1, 0.3), vertex(-0.1, -0.2)],
            indices: vec![0, 1, 2],
            material: MaterialId(0),
        };
        Jaw::new(
            vec![mesh],
            vec![vec![0.0, 1.0, 0.5]],
            Vector3::zeros(),
            Vector3::x(),
        )
    }

    #[test]
    fn the_lower_lip_drops_by_the_whole_turn_and_the_upper_stays() {
        let jaw = jaw();
        let mut out = jaw.rest().to_vec();
        jaw.shape(0.4, &mut out);
        let [upper, lower, between] = [0, 1, 2].map(|i| out[0].vertices[i]);
        assert_eq!(upper.pos, jaw.rest()[0].vertices[0].pos);
        // Turned the whole 0.4 rad about the hinge: as far from it, lower.
        let rest = jaw.rest()[0].vertices[1].pos;
        assert!((lower.pos.norm() - rest.norm()).abs() < 1e-6);
        let turned = rest.normalize().dot(&lower.pos.normalize()).acos();
        assert!((turned - 0.4).abs() < 1e-5);
        assert!(lower.pos.y < rest.y);
        assert!(lower.normal.y < -0.3, "its normal did not turn with it");
        // The skin between turns half as far.
        let rest = jaw.rest()[0].vertices[2].pos;
        let turned = rest.normalize().dot(&between.pos.normalize()).acos();
        assert!((turned - 0.2).abs() < 1e-5);
        // Shaped back to nothing, it is as sculpted.
        jaw.shape(0.0, &mut out);
        for (shaped, rest) in out[0].vertices.iter().zip(&jaw.rest()[0].vertices) {
            assert!((shaped.pos - rest.pos).norm() < 1e-6);
        }
    }
}
