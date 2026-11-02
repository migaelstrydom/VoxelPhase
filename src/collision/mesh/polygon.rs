//! Point-in-polygon for the merged faces of a mesh patch.

use nalgebra::{Point3, Vector3};

/// Whether `point`, taken as lying in the polygon's plane, is inside the
/// convex polygon `verts` with face normal `normal`.
///
/// Winding-agnostic: a point is inside iff it is on the same side of every
/// edge (all edge × to-point products agree in sign along the normal). A
/// point on an edge, within 1e-6, counts as inside.
pub fn point_in_convex_polygon(
    point: &Point3<f32>,
    verts: &[Point3<f32>],
    normal: &Vector3<f32>,
) -> bool {
    let n = verts.len();
    let mut positive = 0u32;
    let mut negative = 0u32;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        let cross_dot = (b - a).cross(&(*point - a)).dot(normal);
        if cross_dot > 1e-6 {
            positive += 1;
        } else if cross_dot < -1e-6 {
            negative += 1;
        }
    }
    positive == 0 || negative == 0
}
