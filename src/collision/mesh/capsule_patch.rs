//! Capsule vs FilteredPatch manifold generation.
//!
//! For each face: test both capsule segment endpoints as sphere centers
//! against the face, and add one interior shaft sample when the capsule
//! axis is not parallel to the face. This preserves the two-endpoint
//! behavior on flat terrain while allowing true shaft support on uneven
//! terrain.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::mesh::seam_filter::{ContactEdge, ContactFace, FilteredPatch};
use crate::collision::mesh::sphere_patch::{closest_point_on_segment, point_in_convex_polygon};

/// Maximum contacts emitted before reduction.
const MAX_CAPSULE_PATCH_CONTACTS: usize = 4;
/// Treat near-parallel support faces as a single stitched span.
///
/// This is intentionally much looser than exact geometric parallelism so
/// rolling/resting capsules on flat triangulated terrain still collapse seam
/// contacts before they reach the solver.
const CAPSULE_FACE_SEAM_COLLAPSE_DOT_EPS: f32 = 0.03;
/// Treat an interior sample extremely close to an endpoint as that endpoint.
const CAPSULE_FACE_ENDPOINT_T_EPS: f32 = 1e-3;
/// Group nearly identical support planes so stitched coplanar faces collapse to one span.
const CAPSULE_FACE_CLUSTER_NORMAL_DOT: f32 = 0.999;
/// Plane offset tolerance for near-parallel support clustering.
const CAPSULE_FACE_CLUSTER_PLANE_EPS: f32 = 1e-3;
/// Join adjacent intervals with tiny parametric gaps caused by clipping tolerance.
const CAPSULE_FACE_INTERVAL_JOIN_EPS: f32 = 1e-3;

/// Generate a contact manifold for a capsule against a filtered mesh patch.
///
/// # Arguments
/// * `seg_a`, `seg_b` — world-space capsule segment endpoints
/// * `radius` — capsule radius (without margin)
/// * `patch` — seam-filtered mesh patch
/// * `contact_margin` — inflation distance for speculative contacts
pub fn capsule_patch_manifold(
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    radius: f32,
    patch: &FilteredPatch,
    contact_margin: f32,
) -> ContactManifold {
    let expanded_radius = radius + contact_margin;

    let mut face_hits: SmallVec<[CapsuleContact; 4]> = SmallVec::new();
    let mut parallel_spans: SmallVec<[ParallelFaceSpan; 8]> = SmallVec::new();
    let mut best_boundary: Option<CapsuleContact> = None;

    for (face_index, face) in patch.faces.iter().enumerate() {
        if face.vertices.len() < 3 {
            continue;
        }
        let face_result = capsule_vs_face(seg_a, seg_b, expanded_radius, face_index, face);
        if let Some(span) = face_result.parallel_span {
            parallel_spans.push(span);
        }
        for contact in face_result.contacts {
            if contact.is_face {
                face_hits.push(contact);
            } else if best_boundary
                .as_ref()
                .map_or(true, |b| contact.dist_sq < b.dist_sq)
            {
                best_boundary = Some(contact);
            }
        }
    }

    extend_with_parallel_face_spans(
        seg_a,
        seg_b,
        expanded_radius,
        &patch.faces,
        &parallel_spans,
        &mut face_hits,
    );

    for edge in &patch.boundary_edges {
        if let Some(contact) = capsule_vs_edge(seg_a, seg_b, expanded_radius, edge) {
            if best_boundary
                .as_ref()
                .map_or(true, |b| contact.dist_sq < b.dist_sq)
            {
                best_boundary = Some(contact);
            }
        }
    }

    if face_hits.is_empty() {
        return match best_boundary {
            Some(hit) => {
                let raw_depth = hit.raw_depth(radius);
                ContactManifold::single(ContactPoint::new(
                    hit.point,
                    hit.normal,
                    raw_depth,
                    hit.feature_id,
                ))
            }
            None => ContactManifold::empty(),
        };
    }

    let points: SmallVec<[ContactPoint; 4]> = face_hits
        .iter()
        .map(|hit| {
            let raw_depth = hit.raw_depth(radius);
            ContactPoint::new(hit.point, hit.normal, raw_depth, hit.feature_id)
        })
        .collect();

    if points.len() > MAX_CAPSULE_PATCH_CONTACTS {
        let reducer = ContactReducer::new(MAX_CAPSULE_PATCH_CONTACTS);
        let reduced = reducer.reduce(&points);
        ContactManifold::from_vec(reduced.into_iter().collect())
    } else {
        ContactManifold::from_vec(points)
    }
}

/// Intermediate result from a capsule-vs-feature test.
struct CapsuleContact {
    point: Point3<f32>,
    normal: Vector3<f32>,
    dist_sq: f32,
    is_face: bool,
    face_signed_dist: f32,
    feature_id: FeatureId,
}

impl CapsuleContact {
    fn raw_depth(&self, radius: f32) -> f32 {
        if self.is_face {
            radius - self.face_signed_dist
        } else {
            radius - self.dist_sq.sqrt()
        }
    }
}

struct CapsuleFaceResult {
    contacts: SmallVec<[CapsuleContact; 3]>,
    parallel_span: Option<ParallelFaceSpan>,
}

#[derive(Clone, Copy)]
struct ParallelFaceSpan {
    t_min: f32,
    t_max: f32,
    normal: Vector3<f32>,
    plane_offset: f32,
    face_index: usize,
}

#[derive(Clone, Copy)]
struct MergedParallelFaceSpan {
    start_t: f32,
    end_t: f32,
    start_face_index: usize,
    end_face_index: usize,
}

/// Test a capsule segment against a face.
///
/// Returns the two endpoint samples plus one interior shaft sample when the
/// capsule axis is not parallel to the face. The interior sample is skipped
/// for near-parallel faces so flat terrain still reduces to the two end
/// contacts.
fn capsule_vs_face(
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    expanded_radius: f32,
    face_index: usize,
    face: &ContactFace,
) -> CapsuleFaceResult {
    let verts = &face.vertices;
    let normal = face.normal;

    let signed_a = (seg_a - verts[0]).dot(&normal);
    let signed_b = (seg_b - verts[0]).dot(&normal);
    let axis = seg_b - seg_a;
    let axis_len_sq = axis.magnitude_squared();

    let mut results: SmallVec<[CapsuleContact; 3]> = SmallVec::new();
    let mut parallel_span = None;

    if axis_len_sq > 1e-12 {
        let axis_len = axis_len_sq.sqrt();
        let axis_dot_n = axis.dot(&normal) / axis_len;
        if let Some((t_min, t_max)) =
            projected_segment_face_interval(seg_a, signed_a, seg_b, signed_b, face)
        {
            if axis_dot_n.abs() <= CAPSULE_FACE_SEAM_COLLAPSE_DOT_EPS {
                parallel_span = Some(ParallelFaceSpan {
                    t_min,
                    t_max,
                    normal,
                    plane_offset: verts[0].coords.dot(&normal),
                    face_index,
                });
            } else {
                let t = if signed_b >= signed_a { t_min } else { t_max };
                if t > CAPSULE_FACE_ENDPOINT_T_EPS && t < 1.0 - CAPSULE_FACE_ENDPOINT_T_EPS {
                    let center = seg_a + axis * t;
                    let signed_dist = signed_a + (signed_b - signed_a) * t;
                    if let Some(contact) =
                        sample_center_vs_face(center, signed_dist, expanded_radius, face)
                    {
                        results.push(contact);
                    }
                }
            }
        }
    }

    if parallel_span.is_none() {
        for &(center, signed_dist) in &[(seg_a, signed_a), (seg_b, signed_b)] {
            if let Some(contact) = sample_center_vs_face(center, signed_dist, expanded_radius, face)
            {
                results.push(contact);
            }
        }
    }

    CapsuleFaceResult {
        contacts: results,
        parallel_span,
    }
}

fn sample_center_vs_face(
    center: Point3<f32>,
    signed_dist: f32,
    expanded_radius: f32,
    face: &ContactFace,
) -> Option<CapsuleContact> {
    let verts = &face.vertices;
    let normal = face.normal;
    if signed_dist > expanded_radius || signed_dist < -expanded_radius {
        return None;
    }

    let projected = center - normal * signed_dist;

    if point_in_convex_polygon(&projected, verts, &normal) {
        return Some(CapsuleContact {
            point: projected,
            normal,
            dist_sq: signed_dist * signed_dist,
            is_face: true,
            face_signed_dist: signed_dist,
            feature_id: face.feature_id,
        });
    }

    // Projection outside — find closest point on face boundary.
    let mut best_dist_sq = f32::MAX;
    let mut best_point = projected;

    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        let cp = closest_point_on_segment(center, a, b);
        let d_sq = (center - cp).magnitude_squared();
        if d_sq < best_dist_sq {
            best_dist_sq = d_sq;
            best_point = cp;
        }
    }

    if best_dist_sq > expanded_radius * expanded_radius {
        return None;
    }

    let to_center = center - best_point;
    let dist = best_dist_sq.sqrt();
    let contact_normal = if dist > 1e-6 {
        to_center / dist
    } else {
        normal
    };

    Some(CapsuleContact {
        point: best_point,
        normal: contact_normal,
        dist_sq: best_dist_sq,
        is_face: false,
        face_signed_dist: signed_dist,
        feature_id: face.feature_id,
    })
}

fn projected_segment_face_interval(
    seg_a: Point3<f32>,
    signed_a: f32,
    seg_b: Point3<f32>,
    signed_b: f32,
    face: &ContactFace,
) -> Option<(f32, f32)> {
    let verts = &face.vertices;
    let normal = face.normal;
    let projected_a = seg_a - normal * signed_a;
    let projected_b = seg_b - normal * signed_b;
    let centroid = {
        let sum = verts.iter().fold(Vector3::zeros(), |acc, v| acc + v.coords);
        Point3::from(sum / verts.len() as f32)
    };

    let mut t_min = 0.0f32;
    let mut t_max = 1.0f32;
    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        let edge = b - a;
        let inside_sign = edge.cross(&(centroid - a)).dot(&normal).signum();
        let d0 = inside_sign * edge.cross(&(projected_a - a)).dot(&normal);
        let d1 = inside_sign * edge.cross(&(projected_b - a)).dot(&normal);

        if d0 < -1e-6 && d1 < -1e-6 {
            return None;
        }

        let delta = d1 - d0;
        if delta.abs() <= 1e-6 {
            continue;
        }

        let t = (-d0 / delta).clamp(0.0, 1.0);
        if d0 < -1e-6 {
            t_min = t_min.max(t);
        } else if d1 < -1e-6 {
            t_max = t_max.min(t);
        }

        if t_min > t_max {
            return None;
        }
    }

    Some((t_min, t_max))
}

fn extend_with_parallel_face_spans(
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    expanded_radius: f32,
    faces: &[ContactFace],
    spans: &[ParallelFaceSpan],
    out: &mut SmallVec<[CapsuleContact; 4]>,
) {
    if spans.is_empty() {
        return;
    }

    let mut used = vec![false; spans.len()];
    for i in 0..spans.len() {
        if used[i] {
            continue;
        }

        let mut group: SmallVec<[ParallelFaceSpan; 8]> = SmallVec::new();
        let seed = spans[i];
        used[i] = true;
        group.push(seed);

        for j in (i + 1)..spans.len() {
            if used[j] || !same_parallel_support_plane(&seed, &spans[j]) {
                continue;
            }
            used[j] = true;
            group.push(spans[j]);
        }

        group.sort_by(|a, b| a.t_min.total_cmp(&b.t_min));
        let mut merged = MergedParallelFaceSpan {
            start_t: group[0].t_min,
            end_t: group[0].t_max,
            start_face_index: group[0].face_index,
            end_face_index: group[0].face_index,
        };

        for span in group.iter().skip(1) {
            if span.t_min <= merged.end_t + CAPSULE_FACE_INTERVAL_JOIN_EPS {
                if span.t_max > merged.end_t {
                    merged.end_t = span.t_max;
                    merged.end_face_index = span.face_index;
                }
            } else {
                push_parallel_span_contacts(seg_a, seg_b, expanded_radius, faces, merged, out);
                merged = MergedParallelFaceSpan {
                    start_t: span.t_min,
                    end_t: span.t_max,
                    start_face_index: span.face_index,
                    end_face_index: span.face_index,
                };
            }
        }

        push_parallel_span_contacts(seg_a, seg_b, expanded_radius, faces, merged, out);
    }
}

fn same_parallel_support_plane(a: &ParallelFaceSpan, b: &ParallelFaceSpan) -> bool {
    a.normal.dot(&b.normal) >= CAPSULE_FACE_CLUSTER_NORMAL_DOT
        && (a.plane_offset - b.plane_offset).abs() <= CAPSULE_FACE_CLUSTER_PLANE_EPS
}

fn push_parallel_span_contacts(
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    expanded_radius: f32,
    faces: &[ContactFace],
    span: MergedParallelFaceSpan,
    out: &mut SmallVec<[CapsuleContact; 4]>,
) {
    if let Some(start) = sample_axis_t_vs_face(
        seg_a,
        seg_b,
        expanded_radius,
        &faces[span.start_face_index],
        span.start_t,
    ) {
        out.push(start);
    }

    if span.end_t - span.start_t <= CAPSULE_FACE_ENDPOINT_T_EPS {
        return;
    }

    if let Some(end) = sample_axis_t_vs_face(
        seg_a,
        seg_b,
        expanded_radius,
        &faces[span.end_face_index],
        span.end_t,
    ) {
        let duplicate = out.last().map_or(false, |prev| {
            (prev.point - end.point).magnitude_squared() < 1e-8
        });
        if !duplicate {
            out.push(end);
        }
    }
}

fn sample_axis_t_vs_face(
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    expanded_radius: f32,
    face: &ContactFace,
    t: f32,
) -> Option<CapsuleContact> {
    let center = seg_a + (seg_b - seg_a) * t;
    let signed_dist = (center - face.vertices[0]).dot(&face.normal);
    sample_center_vs_face(center, signed_dist, expanded_radius, face)
}

/// Test capsule against a boundary/crease edge.
fn capsule_vs_edge(
    seg_a: Point3<f32>,
    seg_b: Point3<f32>,
    expanded_radius: f32,
    edge: &ContactEdge,
) -> Option<CapsuleContact> {
    // Closest points between capsule segment and edge segment.
    let (closest_on_capsule, closest_on_edge) =
        crate::collision::segment::segment_segment_closest_points(seg_a, seg_b, edge.a, edge.b);

    let to_capsule = closest_on_capsule - closest_on_edge;
    let dist_sq = to_capsule.magnitude_squared();

    if dist_sq > expanded_radius * expanded_radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-6 {
        to_capsule / dist
    } else {
        Vector3::y()
    };

    Some(CapsuleContact {
        point: closest_on_edge,
        normal,
        dist_sq,
        is_face: false,
        face_signed_dist: 0.0,
        feature_id: edge.feature_id,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::mesh::seam_filter::{ContactFace, FilteredPatch};
    use smallvec::SmallVec;

    fn flat_face(y: f32) -> FilteredPatch {
        FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-5.0, y, -5.0),
                        Point3::new(5.0, y, -5.0),
                        Point3::new(5.0, y, 5.0),
                        Point3::new(-5.0, y, 5.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        }
    }

    fn flat_strip_face(x_min: f32, x_max: f32, feature: u32) -> ContactFace {
        ContactFace {
            vertices: SmallVec::from_vec(vec![
                Point3::new(x_min, 0.0, -1.0),
                Point3::new(x_max, 0.0, -1.0),
                Point3::new(x_max, 0.0, 1.0),
                Point3::new(x_min, 0.0, 1.0),
            ]),
            normal: Vector3::y(),
            feature_id: FeatureId::from_face(feature),
        }
    }

    #[test]
    fn capsule_resting_on_flat_face() {
        // Upright capsule: half_height=1.0, radius=0.5, segment half=0.5
        // Bottom of capsule at y = center_y - half_height = 1.0 - 1.0 = 0.0
        let seg_a = Point3::new(0.0, 0.5, 0.0); // bottom of segment
        let seg_b = Point3::new(0.0, 1.5, 0.0); // top of segment
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.normal.y > 0.99);
        assert!(c.raw_depth.abs() < 1e-4);
    }

    #[test]
    fn capsule_penetrating_flat_face() {
        let seg_a = Point3::new(0.0, 0.3, 0.0);
        let seg_b = Point3::new(0.0, 1.3, 0.0);
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(m.len(), 1);
        assert!((m.points[0].raw_depth - 0.2).abs() < 1e-4);
    }

    #[test]
    fn capsule_above_no_contact() {
        let seg_a = Point3::new(0.0, 3.0, 0.0);
        let seg_b = Point3::new(0.0, 5.0, 0.0);
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);
        assert!(m.is_empty());
    }

    #[test]
    fn capsule_margin_only() {
        let seg_a = Point3::new(0.0, 0.55, 0.0);
        let seg_b = Point3::new(0.0, 1.55, 0.0);
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.1);

        assert_eq!(m.len(), 1);
        let c = &m.points[0];
        assert!(c.raw_depth < 0.0);
        assert_eq!(c.depth, 0.0);
    }

    #[test]
    fn horizontal_capsule_on_flat_face() {
        // Capsule lying on its side: segment along X axis
        let seg_a = Point3::new(-0.5, 0.5, 0.0);
        let seg_b = Point3::new(0.5, 0.5, 0.0);
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(
            m.len(),
            2,
            "horizontal capsule should produce 2 face contacts"
        );
        for c in &m.points[..2] {
            assert!(c.normal.y > 0.99);
            assert!(c.raw_depth.abs() < 1e-4);
        }
        // Contacts should be at different X positions (one per endpoint).
        let x0 = m.points[0].point.x;
        let x1 = m.points[1].point.x;
        assert!((x0 - x1).abs() > 0.5);
    }

    #[test]
    fn parallel_capsule_above_face_with_margin() {
        // Segment parallel to the face, slightly above contact but within margin.
        let seg_a = Point3::new(-1.0, 0.55, 0.0);
        let seg_b = Point3::new(1.0, 0.55, 0.0);
        let patch = flat_face(0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.1);

        assert_eq!(
            m.len(),
            2,
            "parallel margin capsule should produce 2 contacts"
        );
        for c in &m.points[..2] {
            assert!(c.normal.y > 0.99);
            assert!(c.raw_depth < 0.0, "should be speculative (negative depth)");
            assert_eq!(c.depth, 0.0, "clamped depth should be zero");
        }
    }

    #[test]
    fn interior_face_under_shaft_emits_face_contacts() {
        let patch = FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-1.0, 0.0, -1.0),
                        Point3::new(1.0, 0.0, -1.0),
                        Point3::new(1.0, 0.0, 1.0),
                        Point3::new(-1.0, 0.0, 1.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(1),
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        };
        let seg_a = Point3::new(-3.0, 0.5, 0.0);
        let seg_b = Point3::new(3.0, 0.5, 0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(
            m.len(),
            2,
            "face overlap under the shaft should emit interior contacts"
        );
        let has_contact_near = |x: f32| m.points.iter().any(|c| (c.point.x - x).abs() < 0.15);
        assert!(has_contact_near(-1.0));
        assert!(has_contact_near(1.0));
    }

    #[test]
    fn coplanar_adjacent_faces_collapse_to_widest_span() {
        let patch = FilteredPatch {
            faces: SmallVec::from_vec(vec![
                flat_strip_face(-3.0, -1.0, 0),
                flat_strip_face(-1.0, 1.0, 1),
                flat_strip_face(1.0, 3.0, 2),
            ]),
            boundary_edges: SmallVec::new(),
        };
        let seg_a = Point3::new(-2.5, 0.5, 0.0);
        let seg_b = Point3::new(2.5, 0.5, 0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(
            m.len(),
            2,
            "adjacent coplanar spans should collapse to 2 contacts"
        );
        let has_contact_near = |x: f32| m.points.iter().any(|c| (c.point.x - x).abs() < 0.15);
        assert!(has_contact_near(-2.5));
        assert!(has_contact_near(2.5));
        assert!(!has_contact_near(-1.0));
        assert!(!has_contact_near(1.0));
    }

    #[test]
    fn slightly_tilted_capsule_still_collapses_coplanar_spans() {
        let patch = FilteredPatch {
            faces: SmallVec::from_vec(vec![
                flat_strip_face(-3.0, -1.0, 0),
                flat_strip_face(-1.0, 1.0, 1),
                flat_strip_face(1.0, 3.0, 2),
            ]),
            boundary_edges: SmallVec::new(),
        };
        let seg_a = Point3::new(-2.5, 0.47, 0.0);
        let seg_b = Point3::new(2.5, 0.5, 0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(
            m.len(),
            2,
            "near-parallel resting cases should still collapse to 2 contacts"
        );
        let has_contact_near = |x: f32| m.points.iter().any(|c| (c.point.x - x).abs() < 0.15);
        assert!(has_contact_near(-2.5));
        assert!(has_contact_near(2.5));
        assert!(!has_contact_near(-1.0));
        assert!(!has_contact_near(1.0));
    }

    #[test]
    fn disjoint_coplanar_faces_keep_separate_spans() {
        let patch = FilteredPatch {
            faces: SmallVec::from_vec(vec![
                flat_strip_face(-3.0, -1.0, 0),
                flat_strip_face(1.0, 3.0, 1),
            ]),
            boundary_edges: SmallVec::new(),
        };
        let seg_a = Point3::new(-2.5, 0.5, 0.0);
        let seg_b = Point3::new(2.5, 0.5, 0.0);
        let m = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, 0.0);

        assert_eq!(
            m.len(),
            4,
            "disjoint coplanar support spans should stay separate"
        );
        let has_contact_near = |x: f32| m.points.iter().any(|c| (c.point.x - x).abs() < 0.15);
        assert!(has_contact_near(-2.5));
        assert!(has_contact_near(-1.0));
        assert!(has_contact_near(1.0));
        assert!(has_contact_near(2.5));
    }
}
