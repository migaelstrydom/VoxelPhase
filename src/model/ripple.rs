//! A fin that ripples along its root, as a flatfish's fringe does: sculpted
//! lying still, and each vertex swung about the line it grows from by as
//! much as the wave has it there.
//!
//! ```text
//!   root ●────●────●────●────●      each vertex is hinged on the nearest
//!         ╲    ╲    ╲    ╲    ╲     point of the root: it turns about the
//!   rest   ╲____╲____╲____╲____╲    line there, by an angle its owner
//!                                   gives for where along the wave that is
//!   swung  ╱‾‾‾‾╲____╱‾‾‾‾╲____╱    and how far out the vertex lies
//! ```
//!
//! Its owner shapes it each frame into a mesh drawn as it is (a
//! `ProceduralMesh`), as a [`super::Bulge`] is. The root stays where it was
//! sculpted, so the fin never pulls away from the body it grows from.

use nalgebra::{UnitQuaternion, Vector3};

use super::parts::MeshPrimitive;
use crate::rendering::material::MaterialId;

/// Shortest an offset from the root may be to say which way the vertex
/// lies, m: nearer than this it is on the root and does not move.
const ON_ROOT: f32 = 1e-5;

/// Where one vertex of a ripple swings from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hinge {
    /// The point of the root it turns about.
    pub at: Vector3<f32>,
    /// The axis it turns about there, signed so that a positive turn lifts
    /// it (toward +Y).
    pub axis: Vector3<f32>,
    /// Where along the wave it is: whatever measure the root was given
    /// (metres from a fish's snout, say).
    pub along: f32,
    /// How far out from the root it lies, as a share of the farthest any
    /// vertex does: 0 on the root, 1 at the fin's edge.
    pub out: f32,
}

/// A mesh whose vertices swing on hinges along a root.
#[derive(Clone)]
pub struct Ripple {
    /// The fin as sculpted, lying still.
    rest: MeshPrimitive,
    /// Each vertex's hinge, in order.
    hinges: Vec<Hinge>,
}

impl Ripple {
    /// `rest`, each vertex hinged on the nearest point of `root`: a line
    /// through points, each with its place along the wave, which is
    /// measured between them as a straight line is.
    pub fn rooted(rest: MeshPrimitive, root: &[(Vector3<f32>, f32)]) -> Self {
        assert!(root.len() >= 2, "a ripple's root needs two points at least");
        let mut hinges: Vec<Hinge> = rest
            .vertices
            .iter()
            .map(|vertex| hinge_on(root, vertex.pos))
            .collect();
        let farthest = rest
            .vertices
            .iter()
            .zip(&hinges)
            .map(|(vertex, hinge)| (vertex.pos - hinge.at).norm())
            .fold(0.0, f32::max);
        for (vertex, hinge) in rest.vertices.iter().zip(&mut hinges) {
            hinge.out = (vertex.pos - hinge.at).norm() / farthest.max(ON_ROOT);
        }
        Self { rest, hinges }
    }

    /// The fin as sculpted.
    pub fn rest(&self) -> &MeshPrimitive {
        &self.rest
    }

    /// Each vertex's hinge, in order.
    pub fn hinges(&self) -> &[Hinge] {
        &self.hinges
    }

    /// The same ripple in another material.
    pub fn recoloured(&self, to: impl Fn(MaterialId) -> MaterialId) -> Self {
        let mut ripple = self.clone();
        ripple.rest.material = to(ripple.rest.material);
        ripple
    }

    /// Shape `out` with every vertex turned `swing(hinge)` radians about
    /// its hinge.
    pub fn shape(&self, swing: impl Fn(&Hinge) -> f32, out: &mut MeshPrimitive) {
        if out.vertices.len() != self.rest.vertices.len() {
            *out = self.rest.clone();
        }
        for ((vertex, rest), hinge) in out
            .vertices
            .iter_mut()
            .zip(&self.rest.vertices)
            .zip(&self.hinges)
        {
            let turn = UnitQuaternion::from_scaled_axis(hinge.axis * swing(hinge));
            vertex.pos = hinge.at + turn * (rest.pos - hinge.at);
            vertex.normal = turn * rest.normal;
        }
    }
}

/// The hinge of a vertex at `pos` on `root`, its `out` not yet measured.
fn hinge_on(root: &[(Vector3<f32>, f32)], pos: Vector3<f32>) -> Hinge {
    let mut best: Option<(f32, Hinge)> = None;
    for pair in root.windows(2) {
        let ((from, from_along), (to, to_along)) = (pair[0], pair[1]);
        let run = to - from;
        let length = run.norm();
        if length < ON_ROOT {
            continue;
        }
        let direction = run / length;
        let t = ((pos - from).dot(&direction) / length).clamp(0.0, 1.0);
        let at = from + run * t;
        let distance = (pos - at).norm();
        if best
            .as_ref()
            .is_some_and(|(nearest, _)| *nearest <= distance)
        {
            continue;
        }
        // Whichever way along the root turns the vertex upward.
        let offset = pos - at;
        let lift = direction.cross(&offset).y;
        let axis = if lift < 0.0 { -direction } else { direction };
        best = Some((
            distance,
            Hinge {
                at,
                axis,
                along: from_along + (to_along - from_along) * t,
                out: 0.0,
            },
        ));
    }
    best.map(|(_, hinge)| hinge)
        .expect("a ripple's root has a segment of some length")
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector2, Vector4};

    use super::*;
    use crate::rendering::vertex::Vertex;

    fn vertex(x: f32, y: f32, z: f32) -> Vertex {
        Vertex {
            pos: Vector3::new(x, y, z),
            color: Vector4::repeat(1.0),
            tex_coords: Vector2::zeros(),
            normal: Vector3::y(),
            ao: 1.0,
        }
    }

    /// A strip of fin lying flat out along +X from a root along Z, with a
    /// second strip on the other side reaching out along −X.
    fn fins() -> MeshPrimitive {
        MeshPrimitive {
            vertices: vec![
                vertex(0.0, 0.0, 0.0),
                vertex(1.0, 0.0, 0.0),
                vertex(0.0, 0.0, 2.0),
                vertex(1.0, 0.0, 2.0),
                vertex(-1.0, 0.0, 1.0),
            ],
            indices: vec![0, 1, 2, 1, 3, 2],
            material: MaterialId(0),
        }
    }

    fn root() -> Vec<(Vector3<f32>, f32)> {
        vec![(Vector3::zeros(), 0.0), (Vector3::new(0.0, 0.0, 2.0), 4.0)]
    }

    #[test]
    fn each_vertex_is_hinged_where_it_grows_and_measured_along_the_root() {
        let ripple = Ripple::rooted(fins(), &root());
        let hinges = ripple.hinges();
        assert_eq!(hinges.len(), 5);
        assert!((hinges[3].at - Vector3::new(0.0, 0.0, 2.0)).norm() < 1e-6);
        assert!((hinges[3].along - 4.0).abs() < 1e-6);
        assert!((hinges[4].along - 2.0).abs() < 1e-6);
        assert_eq!(hinges[0].out, 0.0);
        assert!((hinges[1].out - 1.0).abs() < 1e-6);
    }

    #[test]
    fn a_positive_swing_lifts_a_fin_on_either_side_and_leaves_its_root() {
        let ripple = Ripple::rooted(fins(), &root());
        let mut mesh = ripple.rest().clone();
        ripple.shape(|_| 0.5, &mut mesh);
        for tip in [1, 3, 4] {
            let lifted = mesh.vertices[tip].pos.y;
            assert!((lifted - 0.5f32.sin()).abs() < 1e-5, "{tip} rose {lifted}");
        }
        assert!(mesh.vertices[0].pos.norm() < 1e-6, "the root moved");
        // The normal turns with the fin: still facing away from the side
        // it lifts toward, and off true up.
        assert!(mesh.vertices[1].normal.y < 0.99);
        assert!(mesh.vertices[1].normal.y > 0.8);

        // Shaped back to nothing, it is as sculpted.
        ripple.shape(|_| 0.0, &mut mesh);
        for (shaped, rest) in mesh.vertices.iter().zip(&ripple.rest().vertices) {
            assert!((shaped.pos - rest.pos).norm() < 1e-6);
        }
    }

    #[test]
    fn the_swing_is_asked_of_each_vertex_by_its_place_on_the_wave() {
        let ripple = Ripple::rooted(fins(), &root());
        let mut mesh = ripple.rest().clone();
        // Only what lies past the wave's middle swings.
        ripple.shape(|hinge| if hinge.along > 3.0 { 0.4 } else { 0.0 }, &mut mesh);
        assert!(mesh.vertices[1].pos.y.abs() < 1e-6);
        assert!(mesh.vertices[3].pos.y > 0.3);
    }
}
