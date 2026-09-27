//! OBB-capsule discrete collision detection (tier 2).
//!
//! Finds the closest point on the capsule segment to the OBB and runs a
//! sphere-OBB query there. When that contact is on a face the capsule lies
//! along, the shaft is clipped to the face and a contact is placed at each end
//! of the clipped span instead, as the terrain path does for capsule on ground.

use crate::collision::capsule::Capsule;
use crate::collision::contact::{ContactManifold, ContactPoint};
use crate::collision::discrete::sphere_obb::sphere_obb_manifold;
use crate::collision::obb::Obb;
use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

/// A contact normal within this of an OBB face axis is a face contact.
const FACE_NORMAL_DOT: f32 = 0.999;
/// A capsule axis within this of perpendicular to a face normal lies along the
/// face. As loose as the terrain path's, for the same reason: a capsule that
/// leans a little against a wall still rests its whole shaft on it.
const SHAFT_ALONG_FACE_DOT: f32 = 0.03;
/// Clipped spans shorter than this, as a fraction of the shaft, are a point.
const MIN_SPAN_T: f32 = 1e-3;

/// Test an OBB against a capsule with margin support.
///
/// One contact where the capsule meets the box at a point — a cap, an edge, a
/// corner, a shaft crossing a face — and two where its shaft lies along a
/// face. A single point there would be wherever along the shaft the closest-point
/// search happened to settle, since every point of the span is equally close,
/// so the push would land at the top or the bottom of the shaft on floating
/// point noise alone.
///
/// Normal points from the OBB (A) toward the capsule (B).
///
/// # Arguments
/// * `obb` — the oriented bounding box in world space
/// * `capsule` — the capsule in world space
/// * `contact_margin` — inflation distance for speculative contacts
pub fn obb_capsule_manifold(obb: &Obb, capsule: &Capsule, contact_margin: f32) -> ContactManifold {
    let (seg_a, seg_b) = capsule.segment_endpoints();
    let sample = closest_point_on_segment_to_obb(obb, seg_a, seg_b);
    let closest = sphere_obb_manifold(obb, sample, capsule.radius, contact_margin);
    let Some(primary) = closest.points.first().copied() else {
        return closest;
    };
    match face_span(obb, seg_a, seg_b, &primary.normal) {
        Some(span) => span_contacts(obb, capsule, seg_a, seg_b, span, &primary, contact_margin)
            .unwrap_or(closest),
        None => closest,
    }
}

/// The span of the capsule's shaft, as segment parameters `(t0, t1)`, lying
/// over the face whose normal is `normal` — `None` unless `normal` is a face
/// normal and the shaft runs along that face and over it.
fn face_span(
    obb: &Obb,
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    normal: &Vector3<f32>,
) -> Option<(f32, f32)> {
    let axes = obb.axes();
    let face_axis = (0..3).find(|&k| axes[k].dot(normal).abs() > FACE_NORMAL_DOT)?;
    let shaft = seg_b - seg_a;
    let length = shaft.magnitude();
    if length < 1e-6 || (shaft / length).dot(&axes[face_axis]).abs() > SHAFT_ALONG_FACE_DOT {
        return None;
    }

    let a_local = world_to_obb_local(obb, seg_a);
    let d_local = obb.rotation.inverse_transform_vector(&shaft);
    let (mut t0, mut t1) = (0.0_f32, 1.0_f32);
    for i in (0..3).filter(|&i| i != face_axis) {
        let e = obb.half_extents[i];
        if d_local[i].abs() < 1e-9 {
            if a_local[i].abs() > e {
                return None;
            }
            continue;
        }
        let enter = (-e - a_local[i]) / d_local[i];
        let exit = (e - a_local[i]) / d_local[i];
        t0 = t0.max(enter.min(exit));
        t1 = t1.min(enter.max(exit));
    }
    (t1 - t0 > MIN_SPAN_T).then_some((t0, t1))
}

/// A contact at each end of `span`, each on the same face as `primary` — or
/// `None` if either end is not, in which case the single closest contact
/// stands.
fn span_contacts(
    obb: &Obb,
    capsule: &Capsule,
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    (t0, t1): (f32, f32),
    primary: &ContactPoint,
    contact_margin: f32,
) -> Option<ContactManifold> {
    let mut points: SmallVec<[ContactPoint; 4]> = SmallVec::new();
    for (end, t) in [t0, t1].into_iter().enumerate() {
        let center = seg_a + (seg_b - seg_a) * t;
        let manifold = sphere_obb_manifold(obb, center, capsule.radius, contact_margin);
        let mut contact = *manifold.points.first()?;
        if contact.normal.dot(&primary.normal) < FACE_NORMAL_DOT {
            return None;
        }
        contact.feature_id = primary.feature_id.with_vertex(end as u32);
        points.push(contact);
    }
    Some(ContactManifold::from_vec(points))
}

/// Closest point on segment AB to the OBB.
///
/// Solves segment-vs-AABB distance analytically in OBB-local space by minimizing
/// a piecewise-quadratic distance function over t in [0, 1].
fn closest_point_on_segment_to_obb(obb: &Obb, a: Point3<f32>, b: Point3<f32>) -> Point3<f32> {
    let a_local = world_to_obb_local(obb, a);
    let b_local = world_to_obb_local(obb, b);
    let t = closest_t_on_segment_to_aabb_local(a_local, b_local, obb.half_extents);
    a + (b - a) * t
}

fn world_to_obb_local(obb: &Obb, p: Point3<f32>) -> Point3<f32> {
    let v = p - obb.center;
    Point3::from(obb.rotation.inverse_transform_vector(&v))
}

fn distance_sq_point_to_aabb_local(p: Point3<f32>, e: Vector3<f32>) -> f32 {
    let mut sq = 0.0_f32;
    for i in 0..3 {
        let v = p[i];
        let d = if v > e[i] {
            v - e[i]
        } else if v < -e[i] {
            v + e[i]
        } else {
            0.0
        };
        sq += d * d;
    }
    sq
}

fn closest_t_on_segment_to_aabb_local(a: Point3<f32>, b: Point3<f32>, e: Vector3<f32>) -> f32 {
    let d = b - a;
    if d.magnitude_squared() < 1e-12 {
        return 0.0;
    }

    let mut cuts = vec![0.0_f32, 1.0_f32];
    for i in 0..3 {
        let di = d[i];
        if di.abs() < 1e-12 {
            continue;
        }
        for side in [-e[i], e[i]] {
            let t = (side - a[i]) / di;
            if t > 0.0 && t < 1.0 {
                cuts.push(t);
            }
        }
    }
    cuts.sort_by(|x, y| x.total_cmp(y));
    cuts.dedup_by(|x, y| (*x - *y).abs() < 1e-7);

    let mut best_t = 0.0_f32;
    let mut best_dist_sq = distance_sq_point_to_aabb_local(a, e);
    let dist_at_1 = distance_sq_point_to_aabb_local(b, e);
    if dist_at_1 < best_dist_sq {
        best_dist_sq = dist_at_1;
        best_t = 1.0;
    }

    for w in cuts.windows(2) {
        let lo = w[0];
        let hi = w[1];
        if hi - lo < 1e-7 {
            continue;
        }

        let mid = 0.5 * (lo + hi);
        let mut quad_a = 0.0_f32;
        let mut quad_b = 0.0_f32;

        for i in 0..3 {
            let x_mid = a[i] + d[i] * mid;
            let c = if x_mid > e[i] {
                a[i] - e[i]
            } else if x_mid < -e[i] {
                a[i] + e[i]
            } else {
                continue;
            };

            quad_a += d[i] * d[i];
            quad_b += 2.0 * c * d[i];
        }

        let mut t = mid;
        if quad_a > 1e-12 {
            t = (-quad_b / (2.0 * quad_a)).clamp(lo, hi);
        }

        let p = a + d * t;
        let dist_sq = distance_sq_point_to_aabb_local(p, e);
        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_t = t;
        }
    }

    best_t.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, UnitQuaternion, Vector3};
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

    fn unit_box() -> Obb {
        Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        )
    }

    fn upright_capsule(x: f32, y: f32) -> Capsule {
        Capsule::new(Point3::new(x, y, 0.0), UnitQuaternion::identity(), 1.0, 0.3)
    }

    fn long_capsule_along_x() -> Capsule {
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::z()),
            -FRAC_PI_2,
        );
        Capsule::new(Point3::origin(), rot, 10.0, 0.35)
    }

    /// A shaft lying along a face touches it along its length: one contact at
    /// each end of the span, not one wherever the closest-point search settled.
    #[test]
    fn capsule_touching_box_face() {
        let obb = unit_box();
        let capsule = upright_capsule(1.3, 0.0);
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert_eq!(m.len(), 2);
        for c in &m.points {
            assert!(c.normal.x > 0.9);
            assert!(c.raw_depth.abs() < 0.05);
        }
        let heights: Vec<f32> = m.points.iter().map(|c| c.point.y).collect();
        // The shaft runs ±0.7: half-height 1.0, less the 0.3 caps.
        assert!(
            heights.iter().any(|y| (y + 0.7).abs() < 1e-4),
            "{heights:?}"
        );
        assert!(
            heights.iter().any(|y| (y - 0.7).abs() < 1e-4),
            "{heights:?}"
        );
        assert_ne!(m.points[0].feature_id, m.points[1].feature_id);
    }

    /// A shaft that overhangs the face is clipped to it: the contacts sit at
    /// the face's edges, not out in the air beyond them.
    #[test]
    fn a_shaft_taller_than_the_face_is_clipped_to_it() {
        let obb = unit_box();
        let capsule = Capsule::new(
            Point3::new(1.3, 1.5, 0.0),
            UnitQuaternion::identity(),
            1.0,
            0.3,
        );
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert_eq!(m.len(), 2);
        let mut heights: Vec<f32> = m.points.iter().map(|c| c.point.y).collect();
        heights.sort_by(f32::total_cmp);
        // The shaft runs 0.8..2.2; the face stops at 1.0.
        assert!((heights[0] - 0.8).abs() < 1e-4, "{heights:?}");
        assert!((heights[1] - 1.0).abs() < 1e-4, "{heights:?}");
    }

    /// A capsule leaning into a face touches it at one point, as before.
    #[test]
    fn a_leaning_shaft_touches_at_one_point() {
        let obb = unit_box();
        let lean = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let capsule = Capsule::new(Point3::new(1.45, 0.0, 0.0), lean, 1.0, 0.3);
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert_eq!(m.len(), 1);
    }

    #[test]
    fn capsule_overlapping_box() {
        let obb = unit_box();
        let capsule = upright_capsule(1.0, 0.0);
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert_eq!(m.len(), 2);
        assert!(m.points.iter().all(|c| c.raw_depth > 0.0));
    }

    #[test]
    fn capsule_separated_from_box() {
        let obb = unit_box();
        let capsule = upright_capsule(5.0, 0.0);
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert!(m.is_empty());
    }

    #[test]
    fn capsule_cap_touching_box() {
        let obb = unit_box();
        // Capsule above the box: center at y=2.0, half_height=1.0, radius=0.3
        // Bottom cap at y = 2.0 - 1.0 = 1.0 which is exactly the box top face.
        // Move slightly lower to ensure penetration.
        let capsule = upright_capsule(0.0, 1.9);
        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert_eq!(m.len(), 1);
        assert!(m.points[0].normal.y > 0.9);
    }

    #[test]
    fn margin_only() {
        let obb = unit_box();
        let capsule = upright_capsule(1.35, 0.0);
        let m = obb_capsule_manifold(&obb, &capsule, 0.1);
        assert_eq!(m.len(), 2);
        for c in &m.points {
            assert!(c.raw_depth < 0.0);
            assert_eq!(c.depth, 0.0);
        }
    }

    #[test]
    fn long_oblique_plank_hits_capsule_side() {
        let capsule = long_capsule_along_x();
        let plank_rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::y()),
            FRAC_PI_4,
        );
        let obb = Obb::new(
            Point3::new(0.0, 0.0, 4.7),
            plank_rot,
            Vector3::new(6.0, 0.2, 0.3),
        );

        let m = obb_capsule_manifold(&obb, &capsule, 0.0);
        assert_eq!(m.len(), 1, "Expected side contact for oblique long shapes");

        let (seg_a, seg_b) = capsule.segment_endpoints();
        let seg_dir = seg_b - seg_a;
        let seg_len_sq = seg_dir.magnitude_squared();
        let t = ((m.points[0].point - seg_a).dot(&seg_dir) / seg_len_sq).clamp(0.0, 1.0);
        assert!(
            (0.15..0.85).contains(&t),
            "Expected side contact away from capsule caps, got t={t}"
        );
    }
}
