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

use nalgebra::Vector3;

use crate::collision::mesh::seam_filter::{ContactEdge, FilteredPatch};

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
