//! The convex crease edges of a patch, and the region each one owns.
//!
//! A convex crease is a ridge in the mesh: a thin post's upright edges, a
//! wall's corner, a hilltop fold. A shape past both of a crease's faces is
//! closest to the crease itself, and no face contact reaches it:
//!
//! ```text
//!        ●  ← in the crease's region:
//!         ╲    past face a's side and face b's side of it
//!   ▓▓▓▓▓▓▓┐
//!   ▓▓▓▓▓▓▓│ b
//!   ▓▓▓▓▓▓▓│
//!      a
//! ```
//!
//! Concave creases own no region: the faces either side of a valley meet
//! whatever rests in it.

use nalgebra::{Point3, Vector3};

use crate::collision::mesh::seam_filter::{ContactEdge, ContactFace, FilteredPatch};

/// A convex crease edge with its neighbouring face's normal.
pub struct ConvexCrease<'a> {
    /// The edge, walked in its owning face's winding.
    pub edge: &'a ContactEdge,
    /// Outward normal of the face across the edge.
    pub normal_b: Vector3<f32>,
}

impl ConvexCrease<'_> {
    /// Whether `direction`, from a point on the crease out to a shape, lies
    /// in the crease's region: past the side of the crease each face is on.
    pub fn owns(&self, direction: &Vector3<f32>) -> bool {
        let run = self.edge.b - self.edge.a;
        let into_face_a = self.edge.normal_a.cross(&run);
        let into_face_b = -self.normal_b.cross(&run);
        direction.dot(&into_face_a) <= 0.0 && direction.dot(&into_face_b) <= 0.0
    }
}

/// Every convex crease in `patch`, once each.
pub fn convex_creases(patch: &FilteredPatch) -> impl Iterator<Item = ConvexCrease<'_>> {
    patch.boundary_edges.iter().filter_map(|edge| {
        let normal_b = edge.normal_b?;
        // Each crease is listed once from each side, walked in opposite
        // directions.
        let canonical = (edge.a.x, edge.a.y, edge.a.z) < (edge.b.x, edge.b.y, edge.b.z);
        (canonical && is_convex(edge, &normal_b)).then_some(ConvexCrease { edge, normal_b })
    })
}

/// A convex crease of one face: a ridge a shape lying across it may be pushed
/// off over.
pub struct Ridge {
    /// Outward normal of the face across the crease.
    pub normal_b: Vector3<f32>,
    /// The ends of the straight crease this one is a piece of.
    pub run: [Point3<f32>; 2],
}

/// `face`'s convex creases.
pub fn ridges_of<'a>(
    patch: &'a FilteredPatch,
    face: &'a ContactFace,
) -> impl Iterator<Item = Ridge> + 'a {
    patch.boundary_edges.iter().filter_map(move |edge| {
        let normal_b = edge.normal_b?;
        let owned = face.vertices.contains(&edge.a)
            && face.vertices.contains(&edge.b)
            && edge.normal_a.dot(&face.normal) > OWN_SIDE_DOT;
        (owned && is_convex(edge, &normal_b)).then(|| Ridge {
            normal_b,
            run: straight_run(patch, edge),
        })
    })
}

/// The ends of the straight crease `edge` is one piece of: on through every
/// convex crease edge that carries on in its direction.
///
/// A crease is only as long as it runs. A thin pillar's upright edge comes in
/// short pieces stacked into one line up the pillar, and a box skewered on it
/// clears no piece by moving along it: the next piece is there. A crater's
/// radial crease ends at the rim, and a slab lying over it clears it by
/// rising a few millimetres.
pub fn straight_run(patch: &FilteredPatch, edge: &ContactEdge) -> [Point3<f32>; 2] {
    let Some(along) = (edge.b - edge.a).try_normalize(f32::EPSILON) else {
        return [edge.a, edge.b];
    };
    [run_on(patch, edge.a, -along), run_on(patch, edge.b, along)]
}

/// The far end of the crease from `end`, walking on in `direction`.
fn run_on(patch: &FilteredPatch, mut end: Point3<f32>, direction: Vector3<f32>) -> Point3<f32> {
    for _ in 0..patch.boundary_edges.len() {
        let next = patch.boundary_edges.iter().find_map(|edge| {
            let far = match end {
                at if at == edge.a => edge.b,
                at if at == edge.b => edge.a,
                _ => return None,
            };
            let convex = edge
                .normal_b
                .is_some_and(|normal_b| is_convex(edge, &normal_b));
            let straight = (far - end).try_normalize(f32::EPSILON)?.dot(&direction) > STRAIGHT_DOT;
            (convex && straight).then_some(far)
        });
        match next {
            Some(far) => end = far,
            None => break,
        }
    }
    end
}

/// How nearly, as a cosine, the next crease edge must carry on in a crease's
/// direction to be more of the same crease.
const STRAIGHT_DOT: f32 = 0.95;

/// How closely an edge's own face normal must match a face for the edge to
/// be that face's side of a crease, rather than its neighbour's. The two
/// sides of one crease share their ends.
const OWN_SIDE_DOT: f32 = 0.99;

/// Whether the crease folds away from the shape's side, as a ridge does,
/// rather than towards it, as a valley does.
///
/// Edges run in their triangle's winding, which is counter-clockwise about
/// its normal, so the face lies to the left: along `normal_a × (b - a)`. A
/// convex neighbour's normal points away from that side.
fn is_convex(edge: &ContactEdge, normal_b: &Vector3<f32>) -> bool {
    let into_face_a = edge.normal_a.cross(&(edge.b - edge.a));
    normal_b.dot(&into_face_a) < 0.0
}
