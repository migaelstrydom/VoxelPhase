//! OBB vs FilteredPatch manifold generation.
//!
//! A mesh has no inside, so a face whose plane the OBB's centre has crossed
//! is ambiguous: the OBB may be a little too deep in the ground, or most of
//! the way through a thin wall and looking at its far side. Such a face is
//! used while the centre is within the OBB's own reach behind it, unless a
//! face it stands back to back with would push the OBB out a shorter way —
//! so an OBB always leaves solid on the side its centre is on:
//!
//! ```text
//!     back to back: one pushes          face to face: both push
//!       ← │▓▓▓▓│ →                        ▓▓▓│ →    ← │▓▓▓
//!    ┌────┼──┐ │                          ▓▓▓│ ┌────┐ │▓▓▓
//!    │  ● │  │ │   centre left of the     ▓▓▓│ │ ●  │ │▓▓▓
//!    └────┼──┘ │   midline: push ←        ▓▓▓│ └────┘ │▓▓▓
//! ```
//!
//! For each merged face in the filtered patch, tests OBB overlap via
//! half-extent projection. For the face with deepest penetration, clips
//! the OBB's support face against the merged polygon and projects the
//! clipped points onto the contact plane. Produces a multi-point manifold
//! directly, replacing the old per-triangle + coplanar_stabilizer approach.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::{ContactManifold, ContactPoint};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::discrete::clipping::{clip_polygon, obb_face, ClipPolygon};
use crate::collision::mesh::seam_filter::{ContactFace, FilteredPatch};
use crate::collision::obb::Obb;
use crate::collision::segment::segment_segment_closest_points;

/// Maximum contacts in the final manifold.
const MAX_MANIFOLD_POINTS: usize = 4;
/// Small tolerance for rejecting backfacing mesh contacts.
const BACKFACE_EPSILON: f32 = 1e-4;

/// Tolerance for calling one face's centroid behind another's plane.
const BACK_TO_BACK_EPSILON: f32 = 1e-4;

/// Generate a contact manifold for an OBB against a filtered mesh patch.
///
/// Returns up to 4 contact points. The normal points from the terrain
/// surface toward the OBB.
///
/// # Arguments
/// * `obb` — the oriented bounding box in world space
/// * `patch` — seam-filtered mesh patch
/// * `contact_margin` — inflation distance for speculative contacts
pub fn obb_patch_manifold(
    obb: &Obb,
    patch: &FilteredPatch,
    contact_margin: f32,
) -> ContactManifold {
    let mut all_points: SmallVec<[ContactPoint; 4]> = SmallVec::new();

    let overlaps: SmallVec<[(&ContactFace, FaceOverlap); 16]> = patch
        .faces
        .iter()
        .filter(|face| face.vertices.len() >= 3)
        .filter_map(|face| Some((face, test_obb_face_overlap(obb, face, contact_margin)?)))
        .collect();

    // Collect contacts from ALL overlapping faces (individual triangles).
    // Each triangle is always convex so clipping is always correct.
    for (index, (face, overlap)) in overlaps.iter().enumerate() {
        if overlap.centre_behind && is_outpushed(index, &overlaps) {
            continue;
        }

        let normal = overlap.normal;
        let support = find_support_face(obb, &normal);
        let clipped = clip_against_polygon(&support.vertices, &face.vertices, &normal);

        let face_point = face.vertices[0];

        if clipped.is_empty() {
            // Clipping produced no points — try projecting OBB corners.
            let corners = obb.corners();
            for corner in &corners {
                let signed_dist = (corner - face_point).dot(&normal);
                let raw_depth = -signed_dist;
                if raw_depth < -contact_margin {
                    continue;
                }
                let projected = corner - normal * signed_dist;
                if point_in_convex_polygon(&projected, &face.vertices, &normal) {
                    let vi = nearest_support_vertex(&projected, &support.vertices);
                    all_points.push(
                        ContactPoint::new(
                            projected,
                            normal,
                            raw_depth,
                            face.feature_id.with_vertex(vi as u32),
                        )
                        .on(face.surface),
                    );
                }
            }
            continue;
        }

        for p in &clipped {
            let signed_dist = (p - face_point).dot(&normal);
            let raw_depth = -signed_dist;
            if raw_depth < -contact_margin {
                continue;
            }
            let projected = p - normal * signed_dist;
            let vi = nearest_support_vertex(&projected, &support.vertices);
            all_points.push(
                ContactPoint::new(
                    projected,
                    normal,
                    raw_depth,
                    face.feature_id.with_vertex(vi as u32),
                )
                .on(face.surface),
            );
        }
    }

    // No face contacts — try boundary edges as a fallback.
    if all_points.is_empty() {
        return obb_vs_boundary_edges(obb, patch, contact_margin);
    }

    // Reduce to MAX_MANIFOLD_POINTS.
    if all_points.len() > MAX_MANIFOLD_POINTS {
        let reducer = ContactReducer::new(MAX_MANIFOLD_POINTS);
        let reduced = reducer.reduce(&all_points);
        return ContactManifold::from_vec(reduced.into_iter().collect());
    }

    ContactManifold::from_vec(all_points)
}

/// Result of testing OBB overlap against a single face.
struct FaceOverlap {
    /// Face normal from the mesh surface toward the OBB.
    normal: Vector3<f32>,
    /// How far the OBB must move along `normal` to clear the face's plane.
    push: f32,
    /// Whether the OBB's centre is behind the face's plane.
    centre_behind: bool,
}

/// Whether another overlapping face, back to back with this one, would push
/// the OBB out of the solid between them a shorter way.
///
/// Back to back means each face lies behind the other's plane: the two sides
/// of something solid, which the OBB can leave by only one of. Faces that
/// face each other bound a gap instead, and both push. Pushes less than a
/// right angle apart never oppose, so they are summed as ever: the two slopes
/// of a roof both lift what rests across its ridge.
///
/// Equal pushes — the centre exactly on the midline — go to the lower index,
/// so one of the two always survives.
fn is_outpushed(index: usize, overlaps: &[(&ContactFace, FaceOverlap)]) -> bool {
    let (face, overlap) = &overlaps[index];
    overlaps
        .iter()
        .enumerate()
        .any(|(other_index, (other, other_overlap))| {
            other_index != index
                && overlap.normal.dot(&other_overlap.normal) < 0.0
                && is_behind(face, other)
                && is_behind(other, face)
                && (other_overlap.push < overlap.push
                    || (other_overlap.push == overlap.push && other_index < index))
        })
}

/// Whether `face`'s centroid lies on or behind `plane_of`'s plane.
fn is_behind(face: &ContactFace, plane_of: &ContactFace) -> bool {
    (centroid(face) - plane_of.vertices[0]).dot(&plane_of.normal) <= BACK_TO_BACK_EPSILON
}

fn centroid(face: &ContactFace) -> Point3<f32> {
    let sum = face
        .vertices
        .iter()
        .fold(Vector3::zeros(), |acc, v| acc + v.coords);
    Point3::from(sum / face.vertices.len() as f32)
}

/// Test if an OBB overlaps a face via half-extent projection.
///
/// Mesh faces are treated as one-sided: they only ever push the OBB along
/// their normal, and only while its centre is within its own reach behind
/// the plane. Past that, the face belongs to something the OBB has already
/// gone through.
fn test_obb_face_overlap(
    obb: &Obb,
    face: &ContactFace,
    contact_margin: f32,
) -> Option<FaceOverlap> {
    let normal = face.normal;
    let face_point = face.vertices[0];

    // Signed distance from OBB center to face plane.
    let signed_dist = (obb.center - face_point).dot(&normal);

    // OBB half-extent projected onto face normal.
    let half_proj = obb.project_half_extent(&normal);

    if signed_dist < -half_proj - BACKFACE_EPSILON {
        return None;
    }

    // Geometric depth: how far the OBB extends below the face plane.
    let depth = half_proj - signed_dist;

    // Accept contacts within margin range.
    if depth < -contact_margin {
        return None;
    }

    Some(FaceOverlap {
        normal,
        push: depth,
        centre_behind: signed_dist < -BACKFACE_EPSILON,
    })
}

/// Information about the OBB support face for clipping.
struct SupportFace {
    vertices: [Point3<f32>; 4],
}

/// Find the OBB face most aligned with -face_normal (the "bottom" face).
fn find_support_face(obb: &Obb, face_normal: &Vector3<f32>) -> SupportFace {
    let axes = obb.axes();

    let mut best_axis = 0;
    let mut best_alignment = 0.0f32;
    for i in 0..3 {
        let d = axes[i].dot(face_normal).abs();
        if d > best_alignment {
            best_alignment = d;
            best_axis = i;
        }
    }

    // Sign: we want the face pointing toward the terrain (in the -face_normal direction).
    let sign = if axes[best_axis].dot(face_normal) > 0.0 {
        -1.0
    } else {
        1.0
    };

    let obb_face_data = obb_face(obb, best_axis, sign);

    SupportFace {
        vertices: obb_face_data.vertices,
    }
}

/// Clip a convex polygon against another convex polygon's edge planes.
///
/// Uses Sutherland-Hodgman: for each edge of the clip polygon, clip the
/// subject polygon against the edge's half-plane. Winding-agnostic: uses
/// the polygon centroid to determine the inward direction for each edge.
fn clip_against_polygon(
    subject: &[Point3<f32>],
    clip_verts: &[Point3<f32>],
    face_normal: &Vector3<f32>,
) -> ClipPolygon {
    if clip_verts.len() < 3 {
        return ClipPolygon::from_slice(subject);
    }

    let centroid = Point3::from(
        clip_verts.iter().map(|v| v.coords).sum::<Vector3<f32>>() / clip_verts.len() as f32,
    );

    let mut clipped = ClipPolygon::from_slice(subject);

    for i in 0..clip_verts.len() {
        if clipped.is_empty() {
            break;
        }
        let a = clip_verts[i];
        let b = clip_verts[(i + 1) % clip_verts.len()];
        let edge_dir = b - a;
        let candidate_inward = edge_dir.cross(face_normal);

        let to_centroid = centroid - a;
        let inward = if to_centroid.dot(&candidate_inward) >= 0.0 {
            candidate_inward
        } else {
            -candidate_inward
        };

        clipped = clip_polygon(&clipped, a, inward);
    }

    clipped
}

/// Fallback: test OBB edges against boundary edges of the patch.
fn obb_vs_boundary_edges(obb: &Obb, patch: &FilteredPatch, contact_margin: f32) -> ContactManifold {
    if patch.boundary_edges.is_empty() {
        return ContactManifold::empty();
    }

    let obb_edges = obb_edge_segments(obb);
    let mut best: Option<(ContactPoint, f32)> = None;

    for boundary_edge in &patch.boundary_edges {
        for &(obb_a, obb_b) in &obb_edges {
            let (pa, pb) =
                segment_segment_closest_points(obb_a, obb_b, boundary_edge.a, boundary_edge.b);
            let delta = pa - pb;
            let dist_sq = delta.magnitude_squared();
            let expanded = contact_margin;

            if dist_sq > expanded * expanded && dist_sq > 1e-10 {
                continue;
            }

            let dist = dist_sq.sqrt();
            let normal = if dist > 1e-6 {
                delta / dist
            } else {
                Vector3::y()
            };

            let raw_depth = -dist - contact_margin;

            if best.as_ref().map_or(true, |b| raw_depth > b.1) {
                let point = Point3::from((pa.coords + pb.coords) * 0.5);
                best = Some((
                    ContactPoint::new(point, normal, raw_depth, boundary_edge.feature_id)
                        .on(boundary_edge.surface),
                    raw_depth,
                ));
            }
        }
    }

    match best {
        Some((contact, _)) => ContactManifold::single(contact),
        None => ContactManifold::empty(),
    }
}

/// Get the 12 edge segments of an OBB.
fn obb_edge_segments(obb: &Obb) -> [(Point3<f32>, Point3<f32>); 12] {
    let c = obb.corners();
    [
        (c[0], c[1]),
        (c[1], c[2]),
        (c[2], c[3]),
        (c[3], c[0]),
        (c[4], c[5]),
        (c[5], c[6]),
        (c[6], c[7]),
        (c[7], c[4]),
        (c[0], c[4]),
        (c[1], c[5]),
        (c[2], c[6]),
        (c[3], c[7]),
    ]
}

/// Find the index of the nearest OBB support face vertex to a point.
///
/// Used to assign temporally stable per-contact FeatureIds: each clipped
/// contact point is associated with the OBB corner it's closest to, so
/// small frame-to-frame rotations don't change the assignment.
fn nearest_support_vertex(point: &Point3<f32>, support_verts: &[Point3<f32>; 4]) -> usize {
    let mut best = 0;
    let mut best_dist = f32::INFINITY;
    for (i, v) in support_verts.iter().enumerate() {
        let d = (point - v).magnitude_squared();
        if d < best_dist {
            best_dist = d;
            best = i;
        }
    }
    best
}

/// Test if a point lies inside a convex polygon (winding-agnostic).
fn point_in_convex_polygon(
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
        let edge = b - a;
        let to_point = *point - a;
        let cross_dot = edge.cross(&to_point).dot(normal);
        if cross_dot > 1e-6 {
            positive += 1;
        } else if cross_dot < -1e-6 {
            negative += 1;
        }
    }
    positive == 0 || negative == 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::contact::FeatureId;
    use crate::collision::mesh::seam_filter::FilteredPatch;
    use crate::collision::SurfaceId;
    use nalgebra::UnitQuaternion;

    fn large_flat_patch() -> FilteredPatch {
        FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-10.0, 0.0, -10.0),
                        Point3::new(10.0, 0.0, -10.0),
                        Point3::new(10.0, 0.0, 10.0),
                        Point3::new(-10.0, 0.0, 10.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                    surface: SurfaceId::UNSPECIFIED,
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        }
    }

    fn unit_obb_at(y: f32) -> Obb {
        Obb::new(
            Point3::new(0.0, y, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        )
    }

    #[test]
    fn box_resting_on_flat_face() {
        let patch = large_flat_patch();
        let obb = unit_obb_at(0.5);
        let m = obb_patch_manifold(&obb, &patch, 0.0);

        assert_eq!(
            m.len(),
            4,
            "Box resting flat should produce 4 contacts, got {}",
            m.len()
        );
        for c in &m.points {
            assert!(c.normal.y > 0.99, "Normal should point up");
            assert!(
                c.raw_depth.abs() < 0.01,
                "Should be touching, got {}",
                c.raw_depth
            );
        }
    }

    #[test]
    fn box_penetrating_flat_face() {
        let patch = large_flat_patch();
        let obb = unit_obb_at(0.3);
        let m = obb_patch_manifold(&obb, &patch, 0.0);

        assert!(!m.is_empty(), "Should have contacts");
        for c in &m.points {
            assert!(
                c.raw_depth > 0.1,
                "Expected penetration, got {}",
                c.raw_depth
            );
        }
    }

    #[test]
    fn box_above_face_no_contact() {
        let patch = large_flat_patch();
        let obb = unit_obb_at(2.0);
        let m = obb_patch_manifold(&obb, &patch, 0.0);
        assert!(m.is_empty());
    }

    #[test]
    fn box_margin_contact() {
        let patch = large_flat_patch();
        let obb = unit_obb_at(0.55);
        // Margin 0.1: OBB bottom at y=0.05, which is 0.05 above the face.
        // With margin, should produce contacts. Per-point raw_depth ≈ -0.05
        // (the OBB face points are 0.05 above the terrain face).
        let m = obb_patch_manifold(&obb, &patch, 0.1);

        assert!(!m.is_empty(), "Margin should catch nearby contacts");
        for c in &m.points {
            assert!(
                c.raw_depth < 0.0,
                "Should be margin-only, got {}",
                c.raw_depth
            );
            assert_eq!(c.depth, 0.0, "Solver depth should be 0 for margin contacts");
        }
    }

    #[test]
    fn rotated_box_on_flat_face() {
        let patch = large_flat_patch();
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_4);
        let obb = Obb::new(
            Point3::new(0.0, 0.707, 0.0),
            rot,
            Vector3::new(0.5, 0.5, 0.5),
        );
        let m = obb_patch_manifold(&obb, &patch, 0.02);

        assert!(!m.is_empty(), "Rotated box should contact the face");
    }

    #[test]
    fn box_contacts_have_correct_normals() {
        let patch = large_flat_patch();
        let obb = unit_obb_at(0.5);
        let m = obb_patch_manifold(&obb, &patch, 0.0);

        for c in &m.points {
            let normal_len = c.normal.magnitude();
            assert!(
                (normal_len - 1.0).abs() < 1e-5,
                "Normal should be unit, got {}",
                normal_len
            );
        }
    }

    #[test]
    fn obb_patch_colliding_throughput() {
        let patch = large_flat_patch();
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.2)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.15);
        let obb = Obb::new(Point3::new(0.0, 0.6, 0.0), rot, Vector3::new(0.5, 0.5, 0.5));

        // Verify it produces contacts.
        assert!(!obb_patch_manifold(&obb, &patch, 0.02).is_empty());

        let iterations = 100_000;

        let t0 = std::time::Instant::now();
        for _ in 0..iterations {
            let m = obb_patch_manifold(&obb, &patch, 0.02);
            std::hint::black_box(&m);
        }
        let elapsed = t0.elapsed();

        let ns_per_call = elapsed.as_nanos() / iterations as u128;
        eprintln!(
            "obb_patch_manifold throughput: {ns_per_call}ns/call \
             ({iterations} iterations in {:.1}ms)",
            elapsed.as_secs_f64() * 1000.0
        );
    }

    #[test]
    fn box_on_small_face_clips_correctly() {
        // Small terrain face that doesn't fully cover the OBB bottom.
        let patch = FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-0.3, 0.0, -0.3),
                        Point3::new(0.3, 0.0, -0.3),
                        Point3::new(0.3, 0.0, 0.3),
                        Point3::new(-0.3, 0.0, 0.3),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                    surface: SurfaceId::UNSPECIFIED,
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        };
        let obb = unit_obb_at(0.5);
        let m = obb_patch_manifold(&obb, &patch, 0.0);

        // The OBB face is larger than the terrain face, so clipping should
        // produce the terrain face's corners (or a subset).
        assert!(!m.is_empty(), "Should have contacts on the small face");
        assert!(m.len() <= 4);
    }

    /// Captured patch geometry from an in-game pop dump: two flat floor faces
    /// at y=-3 and two sloped faces forming a step.
    fn pop_replay_patch_minimal() -> FilteredPatch {
        FilteredPatch {
            faces: SmallVec::from_vec(vec![
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(12.0, -3.0, -4.0),
                        Point3::new(10.0, -3.0, -4.0),
                        Point3::new(10.0, -3.0, -2.0),
                        Point3::new(12.0, -3.0, -2.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(58),
                    surface: SurfaceId::UNSPECIFIED,
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(12.0, -3.0, -2.0),
                        Point3::new(10.0, -3.0, -2.0),
                        Point3::new(10.0, -3.0, 0.0),
                        Point3::new(12.0, -3.0, 0.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(59),
                    surface: SurfaceId::UNSPECIFIED,
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(9.0, -2.0, -4.0),
                        Point3::new(10.0, -2.0, -5.0),
                        Point3::new(10.0, -1.0, -6.0),
                        Point3::new(8.0, -1.0, -4.0),
                    ]),
                    normal: Vector3::new(0.577350, 0.577350, 0.577350),
                    feature_id: FeatureId::from_face(35),
                    surface: SurfaceId::UNSPECIFIED,
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(10.0, -3.0, -4.0),
                        Point3::new(10.0, -2.0, -5.0),
                        Point3::new(9.0, -2.0, -4.0),
                    ]),
                    normal: Vector3::new(0.577350, 0.577350, 0.577350),
                    feature_id: FeatureId::from_face(53),
                    surface: SurfaceId::UNSPECIFIED,
                },
            ]),
            boundary_edges: SmallVec::new(),
        }
    }

    /// A box straddling a terrain step legitimately touches faces with
    /// different normals, so a mixed-normal manifold is correct here. What
    /// must hold is that every depth is real overlap: each contact sits inside
    /// a face it was generated from, and stepping back along the normal by the
    /// reported depth lands on the box. A depth measured against a face's
    /// infinite plane, beyond its polygon, fails one or the other.
    #[test]
    fn step_replay_depths_are_real_overlap() {
        const TOLERANCE: f32 = 1e-3;

        let patch = pop_replay_patch_minimal();
        let center = Point3::new(10.586787, -1.690402, -4.410592);
        let rot = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            0.335174, 0.163065, -0.581374, 0.723237,
        ));
        let obb = Obb::new(center, rot, Vector3::new(1.8, 4.0, 1.8));

        let manifold = obb_patch_manifold(&obb, &patch, 0.02);
        assert!(
            !manifold.is_empty(),
            "box overlaps the step and must produce contacts"
        );

        for (i, cp) in manifold.points.iter().enumerate() {
            let normal = cp.raw_normal.normalize();
            let on_a_face = patch.faces.iter().any(|face| {
                face.normal.dot(&normal) > 0.999
                    && (cp.point - face.vertices[0]).dot(&face.normal).abs() < TOLERANCE
                    && point_in_convex_polygon(&cp.point, &face.vertices, &face.normal)
            });
            assert!(
                on_a_face,
                "contact {i} at {:?} lies outside every face with its normal",
                cp.point
            );

            let witness = cp.point - normal * cp.raw_depth;
            let gap = (obb.closest_point(witness) - witness).magnitude();
            assert!(
                gap < TOLERANCE,
                "contact {i} claims depth {:.3} but its witness {witness:?} is {gap:.3} outside the box",
                cp.raw_depth
            );
        }
    }

    #[test]
    fn thin_shell_ignores_bottom_face_contacts() {
        let patch = FilteredPatch {
            faces: SmallVec::from_vec(vec![
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-10.0, 0.0, -10.0),
                        Point3::new(10.0, 0.0, -10.0),
                        Point3::new(10.0, 0.0, 10.0),
                        Point3::new(-10.0, 0.0, 10.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                    surface: SurfaceId::UNSPECIFIED,
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-10.0, -0.6, -10.0),
                        Point3::new(-10.0, -0.6, 10.0),
                        Point3::new(10.0, -0.6, 10.0),
                        Point3::new(10.0, -0.6, -10.0),
                    ]),
                    normal: -Vector3::y(),
                    feature_id: FeatureId::from_face(1),
                    surface: SurfaceId::UNSPECIFIED,
                },
            ]),
            boundary_edges: SmallVec::new(),
        };
        let obb = unit_obb_at(0.5);
        let m = obb_patch_manifold(&obb, &patch, 0.0);

        assert!(!m.is_empty(), "Top face should still produce contacts");
        for c in &m.points {
            assert!(
                c.normal.y > 0.99,
                "Backfacing shell face should be rejected"
            );
            assert!(
                c.point.y.abs() < 1e-4,
                "Contacts should lie on the top shell face"
            );
        }
    }
}
