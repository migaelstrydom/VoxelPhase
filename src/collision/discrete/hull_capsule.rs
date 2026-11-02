//! Hull-capsule discrete collision detection.
//!
//! GJK/EPA finds the closest features and gives one contact. When that
//! contact is on a hull face the capsule's shaft lies along, the shaft is
//! clipped to the face and a contact is placed at each end of the clipped
//! span instead, as `obb_capsule` does for a box face: every point of the
//! span is equally close, so a single contact would land at whichever end
//! floating point noise picked, and the capsule would see-saw on it.

use nalgebra::Point3;
use smallvec::SmallVec;

use crate::collision::capsule::Capsule;
use crate::collision::contact::{ContactManifold, ContactPoint};
use crate::collision::discrete::gjk::GjkCache;
use crate::collision::discrete::gjk_epa_manifold::gjk_epa_manifold_cached;
use crate::collision::shape_view::{ShapeView, SupportFace, SupportFaceExtractor};

/// A contact normal within this of a hull face's normal is a face contact.
const FACE_NORMAL_DOT: f32 = 0.999;
/// A capsule axis within this of perpendicular to a face normal lies along the
/// face, as for a box face.
const SHAFT_ALONG_FACE_DOT: f32 = 0.03;
/// Clipped spans shorter than this, as a fraction of the shaft, are a point.
const MIN_SPAN_T: f32 = 1e-3;

/// Contacts between a convex hull (`hull`) and a capsule (`capsule_view`,
/// whose shape `capsule` is). Normals point from the hull toward the capsule.
pub fn hull_capsule_manifold(
    hull: &ShapeView,
    capsule_view: &ShapeView,
    capsule: &Capsule,
    margin: f32,
    gjk_cache: Option<&mut GjkCache>,
) -> ContactManifold {
    let closest = gjk_epa_manifold_cached(hull, capsule_view, margin, gjk_cache);
    let [primary] = closest.points.as_slice() else {
        return closest;
    };
    let Some(face) = hull.support_face(primary.normal) else {
        return closest;
    };
    shaft_on_face(&face, capsule, primary, margin).unwrap_or(closest)
}

/// A contact at each end of the span of the capsule's shaft lying over
/// `face`, or `None` unless the contact is on that face and the shaft lies
/// along it, over it, and within reach at both ends.
fn shaft_on_face(
    face: &SupportFace,
    capsule: &Capsule,
    primary: &ContactPoint,
    margin: f32,
) -> Option<ContactManifold> {
    if face.normal.dot(&primary.normal) < FACE_NORMAL_DOT || face.vertices.len() < 3 {
        return None;
    }
    let (seg_a, seg_b) = capsule.segment_endpoints();
    let shaft = seg_b - seg_a;
    let length = shaft.magnitude();
    if length < 1e-6 || (shaft / length).dot(&face.normal).abs() > SHAFT_ALONG_FACE_DOT {
        return None;
    }

    let (t0, t1) = span_over_face(face, seg_a, seg_b)?;
    let on_plane = face.vertices[0];
    let mut points: SmallVec<[ContactPoint; 4]> = SmallVec::new();
    for (end, t) in [t0, t1].into_iter().enumerate() {
        let centre = seg_a + shaft * t;
        let height = (centre - on_plane).dot(&face.normal);
        let depth = capsule.radius - height;
        if depth < -2.0 * margin {
            return None;
        }
        points.push(ContactPoint::new(
            centre - face.normal * capsule.radius,
            face.normal,
            depth,
            primary.feature_id.with_vertex(end as u32),
        ));
    }
    Some(ContactManifold::from_vec(points))
}

/// The part of segment `a`–`b`, as parameters `(t0, t1)`, that lies over the
/// face polygon: inside every plane through one of its edges and along its
/// normal.
fn span_over_face(face: &SupportFace, a: Point3<f32>, b: Point3<f32>) -> Option<(f32, f32)> {
    let corners = &face.vertices;
    let centroid = corners
        .iter()
        .map(|p| p.coords)
        .sum::<nalgebra::Vector3<f32>>()
        / corners.len() as f32;
    let (mut t0, mut t1) = (0.0_f32, 1.0_f32);
    for (i, &start) in corners.iter().enumerate() {
        let end = corners[(i + 1) % corners.len()];
        let side = face.normal.cross(&(end - start));
        let inward = if side.dot(&(centroid - start.coords)) >= 0.0 {
            side
        } else {
            -side
        };
        let from_a = (a - start).dot(&inward);
        let from_b = (b - start).dot(&inward);
        if from_a < 0.0 && from_b < 0.0 {
            return None;
        }
        if from_a < 0.0 {
            t0 = t0.max(from_a / (from_a - from_b));
        } else if from_b < 0.0 {
            t1 = t1.min(from_a / (from_a - from_b));
        }
    }
    (t1 - t0 > MIN_SPAN_T).then_some((t0, t1))
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use nalgebra::{UnitQuaternion, Vector3};

    use super::*;
    use crate::collision::convex_hull::cube_hull;
    use crate::physics::ColliderShape;

    /// A capsule lying flat on top of a hull block, sunk `depth` into it.
    fn lying_on_a_block(depth: f32) -> ContactManifold {
        let hull = ColliderShape::ConvexHull {
            hull: Arc::new(cube_hull(Vector3::new(1.0, 0.5, 1.0))),
        };
        let hull_view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &hull,
        };
        // Lying along x: the capsule's own axis is y, turned a quarter about z.
        let (half_height, radius) = (0.6, 0.2);
        let rotation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2);
        let center = Point3::new(0.1, 0.5 + radius - depth, 0.0);
        let shape = ColliderShape::Capsule {
            half_height,
            radius,
        };
        let capsule_view = ShapeView {
            center,
            rotation,
            shape: &shape,
        };
        let capsule = Capsule::new(center, rotation, half_height, radius);
        hull_capsule_manifold(&hull_view, &capsule_view, &capsule, 0.02, None)
    }

    /// Lying along the top face, the capsule rests on both ends of its shaft,
    /// at the depth it is sunk, pushed straight up.
    #[test]
    fn a_capsule_lying_on_a_hull_face_rests_on_both_ends() {
        let manifold = lying_on_a_block(0.01);
        assert_eq!(manifold.len(), 2, "{:?}", manifold.points);
        let xs: Vec<f32> = manifold.points.iter().map(|p| p.point.x).collect();
        assert!(
            xs.iter().any(|&x| x < -0.2) && xs.iter().any(|&x| x > 0.4),
            "{xs:?}"
        );
        for point in &manifold.points {
            assert!(point.normal.y > 0.999, "{:?}", point.normal);
            assert!((point.raw_depth - 0.01).abs() < 1e-3, "{}", point.raw_depth);
        }
    }
}
