//! Hull vs OBB collision using SAT with native OBB support functions.
//!
//! Dedicated fast-path for ConvexHull-Box pairs. Uses the OBB's
//! `project_half_extent()` for SAT overlap and `obb_face()` for clipping,
//! avoiding the per-frame `Arc<ConvexHull>` allocation of the generic path.
//!
//! Normal convention: hull is `a`, OBB is `b`, normal points A→B (hull→OBB).

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use super::clipping::{clip_against_face_sides, face_centroid, obb_face};
use super::gjk::{gjk_query_seeded, GjkCache, GjkResult};
use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::convex_hull::{ConvexHull, TransformedHull};
use crate::collision::obb::Obb;
use crate::collision::sat::{
    edge_pair_overlap, is_minkowski_face, SatCache, AXIS_EPS, OVERLAP_EPS,
};
use crate::collision::segment::segment_segment_closest_points;
use crate::collision::shape_view::ShapeView;
use crate::collision::support::ConvexSupport;

/// Maximum contacts to retain after reduction.
const MAX_CONTACTS: usize = 4;

/// Tolerance for GJK separation distance check.
const GJK_TOLERANCE: f32 = 1e-4;

/// Fraction of `margin` by which an edge-edge overlap must beat the best face
/// overlap to win.
const EDGE_WIN_MARGIN_FRACTION: f32 = 0.1;

/// Maximum dot² between an edge-edge axis and the best face normal for the
/// edge axis to win classification.
const EDGE_FACE_ALIGN_THRESHOLD: f32 = 0.99;

/// An edge of a convex hull: pair of vertex indices.
type HullEdge = (u16, u16);

/// Generate a contact manifold between a convex hull and an OBB using SAT.
///
/// Uses GJK as a pre-filter for separated pairs, then runs SAT over hull face
/// normals, OBB face normals, and (for small hulls) edge-edge cross products.
/// Contact geometry is generated via Sutherland-Hodgman clipping using native
/// OBB face geometry.
pub fn hull_obb_manifold(
    hull_view: &ShapeView,
    obb: &Obb,
    margin: f32,
    sat_cache: &mut SatCache,
    gjk_cache: Option<&mut GjkCache>,
) -> ContactManifold {
    let hull = match hull_view.shape {
        crate::physics::ColliderShape::ConvexHull { hull } => hull,
        _ => return ContactManifold::empty(),
    };

    let th = TransformedHull {
        hull,
        center: hull_view.center,
        rotation: hull_view.rotation,
    };

    // GJK pre-filter: fast separation test.
    let seed = gjk_cache.as_ref().and_then(|c| c.last_direction);
    let gjk_result = gjk_query_seeded(&th, obb, seed);

    match &gjk_result {
        GjkResult::Separated {
            distance,
            closest_a,
            closest_b,
        } => {
            if let Some(cache) = gjk_cache {
                let sep_dir = closest_b - closest_a;
                if sep_dir.magnitude_squared() > 1e-10 {
                    cache.last_direction = Some(sep_dir);
                }
            }
            if *distance > 2.0 * margin + GJK_TOLERANCE {
                return ContactManifold::empty();
            }
        }
        GjkResult::Intersecting { .. } => {
            if let Some(cache) = gjk_cache {
                let dir = obb.center - hull_view.center;
                if dir.magnitude_squared() > 1e-10 {
                    cache.last_direction = Some(dir);
                }
            }
        }
    }

    // SAT over face normals.
    let center_dir = obb.center - hull_view.center;
    let obb_axes = obb.axes();

    // Try cached separating axis first.
    if let Some(cached_axis) = sat_cache.separating_axis {
        let overlap = support_overlap_mixed(&th, obb, cached_axis, margin);
        if overlap < -OVERLAP_EPS {
            return ContactManifold::empty();
        }
    }

    let mut best_face_overlap = f32::MAX;
    let mut best_face_axis = Vector3::y();
    let mut best_face_from_hull = true;
    let mut best_separating: Option<(Vector3<f32>, f32)> = None;

    let mut update_separating = |axis: Vector3<f32>, overlap: f32| {
        if overlap < 0.0 {
            if best_separating.map_or(true, |(_, best)| overlap < best) {
                best_separating = Some((axis, overlap));
            }
        }
    };

    // Test hull face normals.
    for face in &hull.faces {
        let mut axis = hull_view.rotation * face.normal;
        if axis.dot(&center_dir) < 0.0 {
            axis = -axis;
        }

        let overlap = support_overlap_mixed(&th, obb, axis, margin);

        if overlap < -OVERLAP_EPS {
            sat_cache.separating_axis = Some(axis);
            return ContactManifold::empty();
        }

        update_separating(axis, overlap);

        if overlap < best_face_overlap {
            best_face_overlap = overlap;
            best_face_axis = axis;
            best_face_from_hull = true;
        }
    }

    // Test OBB face normals (3 axes).
    for i in 0..3 {
        let mut axis = obb_axes[i];
        if axis.dot(&center_dir) < 0.0 {
            axis = -axis;
        }

        let overlap = support_overlap_mixed(&th, obb, axis, margin);

        if overlap < -OVERLAP_EPS {
            sat_cache.separating_axis = Some(axis);
            return ContactManifold::empty();
        }

        update_separating(axis, overlap);

        if overlap < best_face_overlap {
            best_face_overlap = overlap;
            best_face_axis = axis;
            best_face_from_hull = false;
        }
    }

    // Edge-edge SAT with Gauss map filtering (Gregorius, GDC 2013).
    //
    // For each hull edge, test against each of the 3 OBB edge directions.
    // An OBB edge along axis i has adjacent face normals on the two
    // perpendicular axes. All 4 parallel variants of that OBB edge share
    // the same adjacent normals, so the Minkowski face test is per-axis,
    // not per-variant.
    let mut best_edge: Option<EdgeEdgeResult> = None;

    // Precompute OBB adjacent face normal pairs (one pair per OBB axis).
    let obb_adj_normals: [(Vector3<f32>, Vector3<f32>); 3] = [
        (obb_axes[1], obb_axes[2]), // axis 0: adjacent faces on axes 1, 2
        (obb_axes[0], obb_axes[2]), // axis 1: adjacent faces on axes 0, 2
        (obb_axes[0], obb_axes[1]), // axis 2: adjacent faces on axes 0, 1
    ];

    for hull_edge in &hull.edges {
        let na1 = hull_view.rotation * hull_edge.normal_a;
        let na2 = hull_view.rotation * hull_edge.normal_b;
        let dir_a = hull_view.rotation
            * (hull.vertices[hull_edge.v1 as usize] - hull.vertices[hull_edge.v0 as usize]);

        for obb_axis_idx in 0..3 {
            let (ob_n1, ob_n2) = obb_adj_normals[obb_axis_idx];

            if !is_minkowski_face(na1, na2, -ob_n1, -ob_n2) {
                continue;
            }

            let edge_b = obb_axes[obb_axis_idx];
            let cross = dir_a.cross(&edge_b);
            let len_sq = cross.magnitude_squared();
            if len_sq < AXIS_EPS {
                continue;
            }

            let ha0 = hull_view.center + hull_view.rotation * hull.vertices[hull_edge.v0 as usize];
            let ha1 = hull_view.center + hull_view.rotation * hull.vertices[hull_edge.v1 as usize];

            // Pick the OBB edge variant closest to the hull edge (support of
            // OBB in -axis direction). The OBB edge along `obb_axis_idx` has
            // 4 parallel variants at different offsets on the two perp axes.
            let mut axis = cross / len_sq.sqrt();
            if axis.dot(&(na1 + na2)) < 0.0 {
                axis = -axis;
            }
            let variant = obb_support_edge_variant(&obb_axes, obb_axis_idx, -axis);
            let (b0, b1) = obb_edge_variant(obb, &obb_axes, obb_axis_idx, variant);

            let overlap = edge_pair_overlap(axis, ha0, ha1, b0, b1, margin);

            if overlap < -OVERLAP_EPS {
                sat_cache.separating_axis = Some(axis);
                return ContactManifold::empty();
            }

            update_separating(axis, overlap);

            let d = axis.dot(&best_face_axis);
            let face_len_sq = best_face_axis.magnitude_squared();
            if face_len_sq > 1e-12 && d * d > EDGE_FACE_ALIGN_THRESHOLD * face_len_sq {
                continue;
            }

            if best_edge.as_ref().map_or(true, |e| overlap < e.overlap) {
                best_edge = Some(EdgeEdgeResult {
                    axis,
                    overlap,
                    hull_edge: (hull_edge.v0, hull_edge.v1),
                    obb_axis_idx,
                    obb_variant: variant,
                });
            }
        }
    }

    // Decide classification: edge-edge only wins if it beats faces and is penetrating.
    let edge_win_slop = EDGE_WIN_MARGIN_FRACTION * margin.max(OVERLAP_EPS);
    let (best_overlap, best_axis, classification) = if let Some(ref ee) = best_edge {
        let penetrating = ee.overlap > 2.0 * margin;
        if penetrating && ee.overlap + edge_win_slop < best_face_overlap {
            (
                ee.overlap,
                ee.axis,
                AxisClassification::EdgeEdge {
                    hull_edge: ee.hull_edge,
                    obb_axis_idx: ee.obb_axis_idx,
                    obb_variant: ee.obb_variant,
                },
            )
        } else {
            (
                best_face_overlap,
                best_face_axis,
                AxisClassification::Face {
                    from_hull: best_face_from_hull,
                },
            )
        }
    } else {
        (
            best_face_overlap,
            best_face_axis,
            AxisClassification::Face {
                from_hull: best_face_from_hull,
            },
        )
    };

    if best_overlap > f32::MAX * 0.5 {
        sat_cache.separating_axis = best_separating.map(|(axis, _)| axis);
        return ContactManifold::empty();
    }

    sat_cache.separating_axis = None;

    let normal = best_axis.normalize();
    let geometric_depth = best_overlap - 2.0 * margin;

    match classification {
        AxisClassification::Face { from_hull } => {
            face_contact(hull, hull_view, obb, &obb_axes, normal, margin, from_hull)
        }
        AxisClassification::EdgeEdge {
            hull_edge,
            obb_axis_idx,
            obb_variant,
        } => {
            match edge_edge_contact(
                hull,
                hull_view,
                obb,
                &obb_axes,
                normal,
                geometric_depth,
                hull_edge,
                obb_axis_idx,
                obb_variant,
            ) {
                Some(manifold) => manifold,
                None => face_contact(
                    hull,
                    hull_view,
                    obb,
                    &obb_axes,
                    best_face_axis.normalize(),
                    margin,
                    best_face_from_hull,
                ),
            }
        }
    }
}

enum AxisClassification {
    Face {
        from_hull: bool,
    },
    EdgeEdge {
        hull_edge: HullEdge,
        obb_axis_idx: usize,
        obb_variant: usize,
    },
}

struct EdgeEdgeResult {
    axis: Vector3<f32>,
    overlap: f32,
    hull_edge: HullEdge,
    obb_axis_idx: usize,
    obb_variant: usize,
}

/// Compute SAT overlap between a hull and an OBB along a candidate axis.
fn support_overlap_mixed(
    hull: &TransformedHull,
    obb: &Obb,
    axis: Vector3<f32>,
    margin: f32,
) -> f32 {
    let hull_max = hull.support(axis).coords.dot(&axis);
    let obb_min = obb.center.coords.dot(&axis) - obb.project_half_extent(&axis);
    hull_max - obb_min + 2.0 * margin
}

/// Generate face contacts via reference/incident clipping.
fn face_contact(
    hull: &ConvexHull,
    hull_view: &ShapeView,
    obb: &Obb,
    obb_axes: &[Vector3<f32>; 3],
    normal: Vector3<f32>,
    margin: f32,
    from_hull: bool,
) -> ContactManifold {
    if from_hull {
        // Hull face is reference, OBB face is incident.
        face_contact_hull_ref(hull, hull_view, obb, obb_axes, normal, margin)
    } else {
        // OBB face is reference, hull face is incident.
        face_contact_obb_ref(hull, hull_view, obb, obb_axes, normal, margin)
    }
}

/// Hull face as reference, OBB face as incident.
fn face_contact_hull_ref(
    hull: &ConvexHull,
    hull_view: &ShapeView,
    obb: &Obb,
    obb_axes: &[Vector3<f32>; 3],
    normal: Vector3<f32>,
    margin: f32,
) -> ContactManifold {
    let ref_face_idx = find_most_aligned_hull_face(hull, hull_view, normal);
    let ref_face = &hull.faces[ref_face_idx];
    let ref_world_verts: SmallVec<[Point3<f32>; 8]> = ref_face
        .vertex_indices
        .iter()
        .map(|&i| hull_view.center + hull_view.rotation * hull.vertices[i as usize])
        .collect();
    let ref_world_normal = hull_view.rotation * ref_face.normal;

    // Find the OBB incident face (most opposed to ref normal).
    let (inc_axis_idx, inc_axis_sign) = find_most_opposed_obb_face(obb_axes, ref_world_normal);
    let inc_face = obb_face(obb, inc_axis_idx, inc_axis_sign);
    let inc_face_idx = (inc_axis_idx as u32) * 2 + if inc_axis_sign > 0.0 { 0 } else { 1 };

    let clipped = clip_against_face_sides(&ref_world_verts, ref_world_normal, &inc_face.vertices);

    project_clipped_contacts(
        &clipped,
        &ref_world_verts,
        ref_world_normal,
        normal,
        margin,
        FeatureId::from_face_pair(ref_face_idx as u32, inc_face_idx),
    )
}

/// OBB face as reference, hull face as incident.
fn face_contact_obb_ref(
    hull: &ConvexHull,
    hull_view: &ShapeView,
    obb: &Obb,
    obb_axes: &[Vector3<f32>; 3],
    normal: Vector3<f32>,
    margin: f32,
) -> ContactManifold {
    // OBB reference face: most aligned with -normal (faces toward hull).
    let ref_normal_dir = -normal;
    let (ref_axis_idx, ref_axis_sign) = find_most_aligned_obb_face(obb_axes, ref_normal_dir);
    let ref_face = obb_face(obb, ref_axis_idx, ref_axis_sign);
    let ref_face_idx = (ref_axis_idx as u32) * 2 + if ref_axis_sign > 0.0 { 0 } else { 1 };

    // Hull incident face: most opposed to ref normal.
    let inc_face_idx = find_incident_hull_face(hull, hull_view, ref_face.normal);
    let inc_face_data = &hull.faces[inc_face_idx];
    let inc_world_verts: SmallVec<[Point3<f32>; 8]> = inc_face_data
        .vertex_indices
        .iter()
        .map(|&i| hull_view.center + hull_view.rotation * hull.vertices[i as usize])
        .collect();

    let clipped = ref_face.clip_against_sides(&inc_world_verts);

    let ref_world_verts: SmallVec<[Point3<f32>; 8]> = SmallVec::from_slice(&ref_face.vertices);

    project_clipped_contacts(
        &clipped,
        &ref_world_verts,
        ref_face.normal,
        normal,
        margin,
        FeatureId::from_face_pair(inc_face_idx as u32, ref_face_idx),
    )
}

/// Project clipped vertices onto the reference plane and filter by depth.
fn project_clipped_contacts(
    clipped: &[Point3<f32>],
    ref_world_verts: &[Point3<f32>],
    ref_world_normal: Vector3<f32>,
    contact_normal: Vector3<f32>,
    margin: f32,
    base_feature: FeatureId,
) -> ContactManifold {
    if clipped.is_empty() {
        return ContactManifold::empty();
    }

    let ref_center = face_centroid(ref_world_verts);
    let margin_tolerance = 2.0 * margin + OVERLAP_EPS;

    let mut contacts: SmallVec<[ContactPoint; 4]> = SmallVec::new();
    for vertex in clipped {
        let signed_dist = (vertex - ref_center).dot(&ref_world_normal);
        if signed_dist <= margin_tolerance {
            let raw_depth = -signed_dist;
            contacts.push(ContactPoint::new(
                *vertex,
                contact_normal,
                raw_depth,
                base_feature,
            ));
        }
    }

    if contacts.is_empty() {
        return ContactManifold::empty();
    }

    if contacts.len() > MAX_CONTACTS {
        let reducer = ContactReducer::new(MAX_CONTACTS);
        let reduced = reducer.reduce(&contacts);
        return ContactManifold::from_vec(SmallVec::from_vec(reduced));
    }

    ContactManifold::from_vec(contacts)
}

/// Generate a single edge-edge contact between a hull edge and an OBB edge.
///
/// Returns `None` if the closest point clamps to a segment endpoint.
fn edge_edge_contact(
    hull: &ConvexHull,
    hull_view: &ShapeView,
    obb: &Obb,
    obb_axes: &[Vector3<f32>; 3],
    normal: Vector3<f32>,
    geometric_depth: f32,
    hull_edge: HullEdge,
    obb_axis_idx: usize,
    obb_variant: usize,
) -> Option<ContactManifold> {
    let a0 = hull_view.center + hull_view.rotation * hull.vertices[hull_edge.0 as usize];
    let a1 = hull_view.center + hull_view.rotation * hull.vertices[hull_edge.1 as usize];
    let (b0, b1) = obb_edge_variant(obb, obb_axes, obb_axis_idx, obb_variant);
    let (best_pa, best_pb) = segment_segment_closest_points(a0, a1, b0, b1);

    // Check if closest point is clamped to a segment endpoint.
    let edge_eps = 1e-4;
    let da = a1 - a0;
    let da_len_sq = da.magnitude_squared();
    if da_len_sq > 1e-10 {
        let t_a = (best_pa - a0).dot(&da) / da_len_sq;
        if t_a < edge_eps || t_a > 1.0 - edge_eps {
            return None;
        }
    }

    let db = b1 - b0;
    let db_len_sq = db.magnitude_squared();
    if db_len_sq > 1e-10 {
        let t_b = (best_pb - b0).dot(&db) / db_len_sq;
        if t_b < edge_eps || t_b > 1.0 - edge_eps {
            return None;
        }
    }

    let point = Point3::from((best_pa.coords + best_pb.coords) * 0.5);

    let feature_id = FeatureId::from_edge_pair(
        hull_edge.0 as u32 * 64 + hull_edge.1 as u32,
        obb_axis_idx as u32 * 4 + obb_variant as u32,
    );

    Some(ContactManifold::single(ContactPoint::new(
        point,
        normal,
        geometric_depth,
        feature_id,
    )))
}

/// Get the endpoints of one of the 4 edge variants for an OBB axis.
fn obb_edge_variant(
    obb: &Obb,
    axes: &[Vector3<f32>; 3],
    axis_idx: usize,
    variant: usize,
) -> (Point3<f32>, Point3<f32>) {
    let (perp0, perp1) = match axis_idx {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };

    let edge_dir = axes[axis_idx] * obb.half_extents[axis_idx];
    let he0 = obb.half_extents[perp0];
    let he1 = obb.half_extents[perp1];

    let signs: [(f32, f32); 4] = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)];
    let (s0, s1) = signs[variant];
    let offset = axes[perp0] * (he0 * s0) + axes[perp1] * (he1 * s1);

    (
        obb.center + offset - edge_dir,
        obb.center + offset + edge_dir,
    )
}

/// Find the hull face most aligned with a direction.
fn find_most_aligned_hull_face(
    hull: &ConvexHull,
    view: &ShapeView,
    direction: Vector3<f32>,
) -> usize {
    let mut best_idx = 0;
    let mut best_dot = f32::NEG_INFINITY;
    for (idx, face) in hull.faces.iter().enumerate() {
        let world_normal = view.rotation * face.normal;
        let dot = world_normal.dot(&direction);
        if dot > best_dot {
            best_dot = dot;
            best_idx = idx;
        }
    }
    best_idx
}

/// Find the hull face most opposed to a direction (incident face).
fn find_incident_hull_face(hull: &ConvexHull, view: &ShapeView, ref_normal: Vector3<f32>) -> usize {
    let mut best_idx = 0;
    let mut best_dot = f32::MAX;
    for (idx, face) in hull.faces.iter().enumerate() {
        let world_normal = view.rotation * face.normal;
        let dot = world_normal.dot(&ref_normal);
        if dot < best_dot {
            best_dot = dot;
            best_idx = idx;
        }
    }
    best_idx
}

/// Find the OBB face most aligned with a direction.
fn find_most_aligned_obb_face(axes: &[Vector3<f32>; 3], direction: Vector3<f32>) -> (usize, f32) {
    let mut best_idx = 0;
    let mut best_dot = 0.0f32;
    let mut best_sign = 1.0f32;
    for i in 0..3 {
        let dot = direction.dot(&axes[i]);
        if dot.abs() > best_dot.abs() {
            best_dot = dot;
            best_idx = i;
            best_sign = if dot >= 0.0 { 1.0 } else { -1.0 };
        }
    }
    (best_idx, best_sign)
}

/// Find the OBB face most opposed to a direction (incident face).
fn find_most_opposed_obb_face(axes: &[Vector3<f32>; 3], direction: Vector3<f32>) -> (usize, f32) {
    let mut best_idx = 0;
    let mut best_dot = 0.0f32;
    let mut best_sign = 1.0f32;
    for i in 0..3 {
        let dot = direction.dot(&axes[i]);
        if dot.abs() > best_dot.abs() {
            best_dot = dot;
            best_idx = i;
            best_sign = if dot >= 0.0 { -1.0 } else { 1.0 };
        }
    }
    (best_idx, best_sign)
}

/// Find the OBB edge variant along `axis_idx` that is the support feature
/// in the given `direction`. Returns the variant index (0..3) for `obb_edge_variant`.
fn obb_support_edge_variant(
    axes: &[Vector3<f32>; 3],
    axis_idx: usize,
    direction: Vector3<f32>,
) -> usize {
    let (perp0, perp1) = match axis_idx {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };

    // The support edge variant has signs that maximize projection onto `direction`.
    let s0: f32 = if direction.dot(&axes[perp0]) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let s1: f32 = if direction.dot(&axes[perp1]) >= 0.0 {
        1.0
    } else {
        -1.0
    };

    match (s0 > 0.0, s1 > 0.0) {
        (true, true) => 0,
        (false, true) => 1,
        (false, false) => 2,
        (true, false) => 3,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::convex_hull::cube_hull;
    use crate::collision::discrete::obb_obb::obb_obb_manifold_cached;
    use crate::collision::sat::SatCache;
    use crate::physics::ColliderShape;
    use nalgebra::UnitQuaternion;
    use std::sync::Arc;

    #[test]
    fn axis_aligned_matches_obb_obb() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;
        let rot = UnitQuaternion::identity();

        let obb_a = Obb::new(Point3::origin(), rot, he);
        let obb_b = Obb::new(Point3::new(1.5, 0.0, 0.0), rot, he);
        let obb_manifold =
            obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut SatCache::default());

        let hull = Arc::new(cube_hull(he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        let view = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape,
        };
        let hull_manifold =
            hull_obb_manifold(&view, &obb_b, margin, &mut SatCache::default(), None);

        assert!(!obb_manifold.is_empty(), "OBB-OBB should produce contacts");
        assert!(
            !hull_manifold.is_empty(),
            "Hull-OBB should produce contacts"
        );
        assert_eq!(
            obb_manifold.len(),
            hull_manifold.len(),
            "Contact count mismatch: obb={}, hull_obb={}",
            obb_manifold.len(),
            hull_manifold.len(),
        );

        let obb_depth = obb_manifold
            .points
            .iter()
            .map(|p| p.raw_depth)
            .fold(f32::NEG_INFINITY, f32::max);
        let hull_depth = hull_manifold
            .points
            .iter()
            .map(|p| p.raw_depth)
            .fold(f32::NEG_INFINITY, f32::max);
        assert!(
            (obb_depth - hull_depth).abs() < 0.05,
            "Depth mismatch: obb={obb_depth:.4}, hull_obb={hull_depth:.4}"
        );
    }

    #[test]
    fn rotated_hull_vs_obb() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;

        let hull = Arc::new(cube_hull(he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.3);
        let view = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape,
        };

        let obb = Obb::new(Point3::new(1.5, 0.0, 0.0), UnitQuaternion::identity(), he);
        let manifold = hull_obb_manifold(&view, &obb, margin, &mut SatCache::default(), None);

        assert!(
            !manifold.is_empty(),
            "Rotated hull-obb should produce contacts"
        );
        assert!(manifold.points[0].raw_depth > 0.0, "Should be penetrating");
    }

    #[test]
    fn separated_pair_empty() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;

        let hull = Arc::new(cube_hull(he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        let view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };
        let obb = Obb::new(Point3::new(5.0, 0.0, 0.0), UnitQuaternion::identity(), he);

        let manifold = hull_obb_manifold(&view, &obb, margin, &mut SatCache::default(), None);
        assert!(manifold.is_empty(), "Well-separated pair should be empty");
    }

    #[test]
    fn sat_cache_works() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;

        let hull = Arc::new(cube_hull(he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        let view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };
        let obb = Obb::new(Point3::new(5.0, 0.0, 0.0), UnitQuaternion::identity(), he);

        let mut cache = SatCache::default();
        let m1 = hull_obb_manifold(&view, &obb, margin, &mut cache, None);
        assert!(m1.is_empty());

        // Second call should use cached axis.
        let m2 = hull_obb_manifold(&view, &obb, margin, &mut cache, None);
        assert!(m2.is_empty());
    }

    #[test]
    fn cache_cleared_on_collision() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;

        let hull = Arc::new(cube_hull(he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        let view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };
        let obb = Obb::new(Point3::new(1.5, 0.0, 0.0), UnitQuaternion::identity(), he);

        let mut cache = SatCache::default();
        cache.separating_axis = Some(Vector3::x());

        let manifold = hull_obb_manifold(&view, &obb, margin, &mut cache, None);
        assert!(!manifold.is_empty());
        assert!(cache.separating_axis.is_none(), "Cache should be cleared");
    }

    #[test]
    fn margin_only_contact() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;

        let hull = Arc::new(cube_hull(he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        let view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };
        // Separated by 0.01 — within 2*margin (0.04).
        let obb = Obb::new(Point3::new(2.01, 0.0, 0.0), UnitQuaternion::identity(), he);

        let manifold = hull_obb_manifold(&view, &obb, margin, &mut SatCache::default(), None);
        assert!(!manifold.is_empty(), "Margin contact expected");
        assert!(
            manifold.points[0].raw_depth < 0.0,
            "Margin contact should have negative depth"
        );
    }

    #[test]
    fn non_unit_half_extents() {
        let hull_he = Vector3::new(1.0, 1.0, 1.0);
        let obb_he = Vector3::new(2.0, 0.5, 3.0);
        let margin = 0.02;

        let hull = Arc::new(cube_hull(hull_he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        let view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };
        let obb = Obb::new(
            Point3::new(2.5, 0.0, 0.0),
            UnitQuaternion::identity(),
            obb_he,
        );

        let manifold = hull_obb_manifold(&view, &obb, margin, &mut SatCache::default(), None);
        assert!(!manifold.is_empty(), "Should produce contacts");

        // Expected depth: hull extends to x=1, OBB extends to x=2.5-2=0.5. Overlap = 0.5.
        let depth = manifold.points[0].raw_depth;
        assert!(
            (depth - 0.5).abs() < 0.1,
            "Expected ~0.5 depth, got {depth:.4}"
        );
    }

    #[test]
    fn rotated_obb() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;

        let hull = Arc::new(cube_hull(he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        let view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4);
        // sqrt(2) ≈ 1.414, so rotated OBB extends to ~1.414 on each side.
        // Place close enough to overlap: hull +X at 1.0, OBB -X at 2.0-1.414=0.586.
        let obb = Obb::new(Point3::new(2.0, 0.0, 0.0), rot, he);

        let manifold = hull_obb_manifold(&view, &obb, margin, &mut SatCache::default(), None);
        assert!(!manifold.is_empty(), "Rotated OBB should produce contacts");
    }

    #[test]
    fn normal_points_hull_to_obb() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;

        let hull = Arc::new(cube_hull(he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        let view = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };
        let obb = Obb::new(Point3::new(1.5, 0.0, 0.0), UnitQuaternion::identity(), he);

        let manifold = hull_obb_manifold(&view, &obb, margin, &mut SatCache::default(), None);
        assert!(!manifold.is_empty());

        let n = manifold.points[0].normal;
        let hull_to_obb = (obb.center - Point3::origin()).normalize();
        assert!(
            n.dot(&hull_to_obb) > 0.9,
            "Normal should point hull→OBB, got {n:?}"
        );
    }

    #[test]
    fn matches_obb_obb_across_rotations() {
        let he = Vector3::new(1.0, 1.5, 0.8);
        let margin = 0.02;

        let hull = Arc::new(cube_hull(he));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };

        for angle_idx in 0..12 {
            let angle = angle_idx as f32 * std::f32::consts::FRAC_PI_6;
            let rot_a = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle);
            let rot_b = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle * 0.7);

            let view = ShapeView {
                center: Point3::origin(),
                rotation: rot_a,
                shape: &shape,
            };
            let obb_a = Obb::new(Point3::origin(), rot_a, he);
            let obb_b = Obb::new(Point3::new(1.8, 0.3, 0.0), rot_b, he);

            let obb_m = obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut SatCache::default());
            let hull_m = hull_obb_manifold(&view, &obb_b, margin, &mut SatCache::default(), None);

            // Both should agree on empty/non-empty.
            assert_eq!(
                obb_m.is_empty(),
                hull_m.is_empty(),
                "angle={angle:.2}: OBB empty={}, hull_obb empty={}",
                obb_m.is_empty(),
                hull_m.is_empty(),
            );

            if !obb_m.is_empty() && !hull_m.is_empty() {
                let d_obb = obb_m
                    .points
                    .iter()
                    .map(|p| p.raw_depth)
                    .fold(f32::NEG_INFINITY, f32::max);
                let d_hull = hull_m
                    .points
                    .iter()
                    .map(|p| p.raw_depth)
                    .fold(f32::NEG_INFINITY, f32::max);
                assert!(
                    (d_obb - d_hull).abs() < 0.15,
                    "angle={angle:.2}: max depth mismatch obb={d_obb:.4} hull_obb={d_hull:.4}"
                );
            }
        }
    }

    #[test]
    fn obb_support_edge_variant_selects_correct_variant() {
        // For an edge along axis 0, the support in direction (0, -1, 1)
        // should select the variant with perp signs (-1, +1) = variant 1.
        let obb_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
        let direction = Vector3::new(0.0, -1.0, 1.0);

        let variant = obb_support_edge_variant(&obb_axes, 0, direction);
        assert_eq!(
            variant, 1,
            "Expected variant 1 (perp signs: -y, +z) for direction (0, -1, 1)"
        );
    }
}
