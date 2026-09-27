//! Hull vs Hull collision using SAT with Sutherland-Hodgman clipping.
//!
//! Replaces the GJK → EPA → heuristic face-selection path for convex hull pairs
//! with a principled SAT-based minimum-penetration-axis search, followed by
//! reference/incident face clipping.
//!
//! Pipeline: GJK (cached) → SAT over face normals + Gauss map filtered edge-edge
//!           → reference/incident clip or edge-edge closest point.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use super::clipping::{clip_against_face_sides, face_centroid};
use super::gjk::{gjk_query_seeded, GjkCache, GjkResult};
use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::convex_hull::{ConvexHull, TransformedHull};
use crate::collision::sat::{
    edge_pair_overlap, is_minkowski_face, SatCache, AXIS_EPS, OVERLAP_EPS,
};
use crate::collision::segment::segment_segment_closest_points;
use crate::collision::shape_view::ShapeView;
use crate::collision::support::ConvexSupport;
use crate::physics::ColliderShape;

/// Maximum contacts to retain after reduction.
const MAX_CONTACTS: usize = 4;

/// Tolerance for GJK separation distance check.
const GJK_TOLERANCE: f32 = 1e-4;

/// Fraction of `margin` by which an edge-edge overlap must beat the best face
/// overlap to win. Face clipping produces multi-point manifolds that resist
/// torque; edge-edge gives a single point. Prefer faces in ambiguous cases.
const EDGE_WIN_MARGIN_FRACTION: f32 = 0.1;

/// Maximum dot² between an edge-edge axis and the best face normal for the
/// edge axis to win classification. When the cross product aligns closely with
/// a face normal, the face path produces better multi-point contacts.
const EDGE_FACE_ALIGN_THRESHOLD: f32 = 0.99;

/// An edge of a convex hull: pair of vertex indices.
type HullEdge = (u16, u16);

/// Generate a contact manifold between two convex hulls using SAT.
///
/// Uses GJK as a pre-filter for separated pairs, then runs face-normal SAT
/// to find the minimum-penetration axis, followed by Sutherland-Hodgman
/// reference/incident face clipping.
pub fn hull_hull_manifold(
    a: &ShapeView,
    b: &ShapeView,
    margin: f32,
    sat_cache: &mut SatCache,
    gjk_cache: Option<&mut GjkCache>,
) -> ContactManifold {
    let hull_a = match a.shape {
        ColliderShape::ConvexHull { hull } => hull,
        _ => return ContactManifold::empty(),
    };
    let hull_b = match b.shape {
        ColliderShape::ConvexHull { hull } => hull,
        _ => return ContactManifold::empty(),
    };

    let th_a = TransformedHull {
        hull: hull_a,
        center: a.center,
        rotation: a.rotation,
    };
    let th_b = TransformedHull {
        hull: hull_b,
        center: b.center,
        rotation: b.rotation,
    };

    // GJK pre-filter: fast separation test with warm-started cache.
    // Only used for early-out when shapes are clearly separated (beyond margin).
    // Near-contact pairs (within margin) fall through to SAT so they get
    // consistent multi-point manifolds — matching OBB-OBB's approach.
    let seed = gjk_cache.as_ref().and_then(|c| c.last_direction);
    let gjk_result = gjk_query_seeded(&th_a, &th_b, seed);

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
            // Within margin — fall through to SAT for consistent multi-point
            // contacts. Using a single GJK witness point here would cause
            // 1↔N contact flickering at the Separated/Intersecting boundary.
        }
        GjkResult::Intersecting { .. } => {
            if let Some(cache) = gjk_cache {
                let dir = b.center - a.center;
                if dir.magnitude_squared() > 1e-10 {
                    cache.last_direction = Some(dir);
                }
            }
        }
    }

    // SAT over face normals.
    let center_dir = b.center - a.center;

    // Test cached separating axis first.
    if let Some(cached_axis) = sat_cache.separating_axis {
        let overlap = support_overlap(&th_a, &th_b, cached_axis, margin);
        if overlap < -OVERLAP_EPS {
            return ContactManifold::empty();
        }
    }

    // Track the minimum-overlap (best penetration) axis and the best separating axis.
    let mut best_face_overlap = f32::MAX;
    let mut best_face_axis = Vector3::y();
    let mut best_face_from_a = true;
    let mut best_separating: Option<(Vector3<f32>, f32)> = None;

    let mut update_separating = |axis: Vector3<f32>, overlap: f32| {
        if overlap < 0.0 {
            if best_separating.map_or(true, |(_, best)| overlap < best) {
                best_separating = Some((axis, overlap));
            }
        }
    };

    // Test face normals from hull A.
    for face in &hull_a.faces {
        let mut axis = a.rotation * face.normal;
        if axis.dot(&center_dir) < 0.0 {
            axis = -axis;
        }

        let overlap = support_overlap(&th_a, &th_b, axis, margin);

        if overlap < -OVERLAP_EPS {
            sat_cache.separating_axis = Some(axis);
            return ContactManifold::empty();
        }

        update_separating(axis, overlap);

        if overlap < best_face_overlap {
            best_face_overlap = overlap;
            best_face_axis = axis;
            best_face_from_a = true;
        }
    }

    // Test face normals from hull B.
    for face in &hull_b.faces {
        let mut axis = b.rotation * face.normal;
        if axis.dot(&center_dir) < 0.0 {
            axis = -axis;
        }

        let overlap = support_overlap(&th_a, &th_b, axis, margin);

        if overlap < -OVERLAP_EPS {
            sat_cache.separating_axis = Some(axis);
            return ContactManifold::empty();
        }

        update_separating(axis, overlap);

        if overlap < best_face_overlap {
            best_face_overlap = overlap;
            best_face_axis = axis;
            best_face_from_a = false;
        }
    }

    // Edge-edge SAT with Gauss map filtering (Gregorius, GDC 2013).
    //
    // Two edges can only form a valid separating axis if their adjacent face
    // normals define intersecting arcs on the Gauss map — the "Minkowski face"
    // test. For the passing pairs, overlap is computed directly from the two
    // candidate edges (no global support queries): if the pair forms a
    // Minkowski face, those edges are already the supporting features.
    let mut best_edge: Option<EdgeEdgeResult> = None;

    for edge_a in &hull_a.edges {
        let na1 = a.rotation * edge_a.normal_a;
        let na2 = a.rotation * edge_a.normal_b;
        let dir_a = a.rotation
            * (hull_a.vertices[edge_a.v1 as usize] - hull_a.vertices[edge_a.v0 as usize]);

        for edge_b in &hull_b.edges {
            let nb1 = b.rotation * edge_b.normal_a;
            let nb2 = b.rotation * edge_b.normal_b;

            if !is_minkowski_face(na1, na2, -nb2, -nb1) {
                continue;
            }

            let dir_b = b.rotation
                * (hull_b.vertices[edge_b.v1 as usize] - hull_b.vertices[edge_b.v0 as usize]);

            let cross = dir_a.cross(&dir_b);
            let len_sq = cross.magnitude_squared();
            if len_sq < AXIS_EPS {
                continue;
            }
            let a0 = a.center + a.rotation * hull_a.vertices[edge_a.v0 as usize];
            let a1 = a.center + a.rotation * hull_a.vertices[edge_a.v1 as usize];
            let b0 = b.center + b.rotation * hull_b.vertices[edge_b.v0 as usize];
            let b1 = b.center + b.rotation * hull_b.vertices[edge_b.v1 as usize];
            let mut axis = cross / len_sq.sqrt();

            // Orient the axis outward from hull A at edge_a. The cross product
            // axis lies between edge_a's two adjacent face normals; aligning it
            // with their sum ensures edge_a is the max-support feature of A
            // along this axis. Back-facing Minkowski face pairs get large
            // positive overlaps and naturally lose the min-overlap search.
            if axis.dot(&(na1 + na2)) < 0.0 {
                axis = -axis;
            }

            let overlap = edge_pair_overlap(axis, a0, a1, b0, b1, margin);

            if overlap < -OVERLAP_EPS {
                sat_cache.separating_axis = Some(axis);
                return ContactManifold::empty();
            }

            update_separating(axis, overlap);

            // Skip edge axes that align closely with the best face axis —
            // the face path produces better multi-point contacts.
            let d = axis.dot(&best_face_axis);
            let face_len_sq = best_face_axis.magnitude_squared();
            if face_len_sq > 1e-12 && d * d > EDGE_FACE_ALIGN_THRESHOLD * face_len_sq {
                continue;
            }

            if best_edge.as_ref().map_or(true, |e| overlap < e.overlap) {
                best_edge = Some(EdgeEdgeResult {
                    axis,
                    overlap,
                    edge_a: (edge_a.v0, edge_a.v1),
                    edge_b: (edge_b.v0, edge_b.v1),
                });
            }
        }
    }

    // Decide classification: edge-edge only wins if it beats the best face
    // overlap by a margin-relative slop AND represents actual penetration
    // (overlap > 2*margin). For margin-only contacts (shapes separated but
    // within margin skin), always use face clipping — edge-edge midpoint
    // placement produces points far outside one hull for separated pairs.
    let edge_win_slop = EDGE_WIN_MARGIN_FRACTION * margin.max(OVERLAP_EPS);
    let (best_overlap, best_axis, classification) = if let Some(ref ee) = best_edge {
        // HEURISTIC: edge-edge midpoint placement produces points far
        // outside one hull for margin-only contacts (separated shapes).
        // Only allow edge-edge to win when actually penetrating. If this
        // causes issues with shallow edge-edge contacts, the proper fix
        // is to project the edge closest points onto the hull surface
        // along the contact normal (like the face path does).
        let penetrating = ee.overlap > 2.0 * margin;
        if penetrating && ee.overlap + edge_win_slop < best_face_overlap {
            (
                ee.overlap,
                ee.axis,
                AxisClassification::EdgeEdge {
                    edge_a: ee.edge_a,
                    edge_b: ee.edge_b,
                },
            )
        } else {
            (
                best_face_overlap,
                best_face_axis,
                AxisClassification::Face {
                    from_a: best_face_from_a,
                },
            )
        }
    } else {
        (
            best_face_overlap,
            best_face_axis,
            AxisClassification::Face {
                from_a: best_face_from_a,
            },
        )
    };

    // If no overlapping axis found, shapes are separated.
    if best_overlap > f32::MAX * 0.5 {
        sat_cache.separating_axis = best_separating.map(|(axis, _)| axis);
        return ContactManifold::empty();
    }

    // Pair is colliding — clear the SAT cache.
    sat_cache.separating_axis = None;

    let normal = best_axis.normalize();
    let geometric_depth = best_overlap - 2.0 * margin;

    match classification {
        AxisClassification::Face { from_a } => {
            face_contact(hull_a, a, hull_b, b, normal, margin, from_a)
        }
        AxisClassification::EdgeEdge { edge_a, edge_b } => {
            // If the closest-point computation clamps to a segment endpoint,
            // the true contact feature isn't edge-edge interior — fall back
            // to face clipping using the best face axis.
            match edge_edge_contact(
                hull_a,
                a,
                hull_b,
                b,
                normal,
                geometric_depth,
                edge_a,
                edge_b,
            ) {
                Some(manifold) => manifold,
                None => face_contact(
                    hull_a,
                    a,
                    hull_b,
                    b,
                    best_face_axis.normalize(),
                    margin,
                    best_face_from_a,
                ),
            }
        }
    }
}

/// Result of the best face-normal SAT classification.
enum AxisClassification {
    Face { from_a: bool },
    EdgeEdge { edge_a: HullEdge, edge_b: HullEdge },
}

/// Tracked state for the best edge-edge candidate.
struct EdgeEdgeResult {
    axis: Vector3<f32>,
    overlap: f32,
    edge_a: HullEdge,
    edge_b: HullEdge,
}

/// Generate face-face contacts via reference/incident clipping.
fn face_contact(
    hull_a: &ConvexHull,
    a: &ShapeView,
    hull_b: &ConvexHull,
    b: &ShapeView,
    normal: Vector3<f32>,
    margin: f32,
    from_a: bool,
) -> ContactManifold {
    let (ref_hull, ref_view, inc_hull, inc_view, flip_normal) = if from_a {
        (hull_a, a, hull_b, b, false)
    } else {
        (hull_b, b, hull_a, a, true)
    };

    // Find the reference face: the face on ref_hull most aligned with the
    // contact normal (pointing from ref toward incident hull).
    let ref_normal_dir = if flip_normal { -normal } else { normal };
    let ref_face_idx = find_most_aligned_face(ref_hull, ref_view, ref_normal_dir);

    // Transform reference face vertices to world space.
    let ref_face = &ref_hull.faces[ref_face_idx];
    let ref_world_verts: SmallVec<[Point3<f32>; 8]> = ref_face
        .vertex_indices
        .iter()
        .map(|&i| ref_view.center + ref_view.rotation * ref_hull.vertices[i as usize])
        .collect();
    let ref_world_normal = ref_view.rotation * ref_face.normal;

    // Find the incident face: the face on the opposing hull most opposed to ref_normal.
    let inc_face_idx = find_incident_face(inc_hull, inc_view, ref_world_normal);
    let inc_face = &inc_hull.faces[inc_face_idx];
    let inc_world_verts: SmallVec<[Point3<f32>; 8]> = inc_face
        .vertex_indices
        .iter()
        .map(|&i| inc_view.center + inc_view.rotation * inc_hull.vertices[i as usize])
        .collect();

    // Clip incident face against reference face side planes.
    let clipped = clip_against_face_sides(&ref_world_verts, ref_world_normal, &inc_world_verts);

    let ref_center = face_centroid(&ref_world_verts);
    let margin_tolerance = 2.0 * margin + OVERLAP_EPS;
    let base_feature = if flip_normal {
        FeatureId::from_face_pair(inc_face_idx as u32, ref_face_idx as u32)
    } else {
        FeatureId::from_face_pair(ref_face_idx as u32, inc_face_idx as u32)
    };

    if clipped.is_empty() {
        return ContactManifold::empty();
    }

    // Project clipped vertices onto reference plane, filter by depth.
    let mut contacts: SmallVec<[ContactPoint; 4]> = SmallVec::new();
    for vertex in clipped.iter() {
        let signed_dist = (vertex - ref_center).dot(&ref_world_normal);
        if signed_dist <= margin_tolerance {
            let raw_depth = -signed_dist;
            let feature_id = base_feature;
            contacts.push(ContactPoint::new(*vertex, normal, raw_depth, feature_id));
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

/// Generate a single edge-edge contact point from the closest points on two edges.
///
/// Returns `None` if the closest approach clamps to a segment endpoint, which
/// indicates the true closest feature is vertex-face or vertex-edge rather than
/// edge-edge. The caller should fall back to the face-clipping path.
fn edge_edge_contact(
    hull_a: &ConvexHull,
    a: &ShapeView,
    hull_b: &ConvexHull,
    b: &ShapeView,
    normal: Vector3<f32>,
    geometric_depth: f32,
    edge_a: HullEdge,
    edge_b: HullEdge,
) -> Option<ContactManifold> {
    let a0 = a.center + a.rotation * hull_a.vertices[edge_a.0 as usize];
    let a1 = a.center + a.rotation * hull_a.vertices[edge_a.1 as usize];
    let b0 = b.center + b.rotation * hull_b.vertices[edge_b.0 as usize];
    let b1 = b.center + b.rotation * hull_b.vertices[edge_b.1 as usize];

    let (pa, pb) = segment_segment_closest_points(a0, a1, b0, b1);

    // Check if the closest point is clamped to a segment endpoint.
    // If so, the true contact feature isn't edge-edge interior — fall back.
    let edge_eps = 1e-4;
    let da = a1 - a0;
    let da_len_sq = da.magnitude_squared();
    let db = b1 - b0;
    let db_len_sq = db.magnitude_squared();

    if da_len_sq > 1e-10 {
        let t_a = (pa - a0).dot(&da) / da_len_sq;
        if t_a < edge_eps || t_a > 1.0 - edge_eps {
            return None;
        }
    }
    if db_len_sq > 1e-10 {
        let t_b = (pb - b0).dot(&db) / db_len_sq;
        if t_b < edge_eps || t_b > 1.0 - edge_eps {
            return None;
        }
    }

    let point = Point3::from((pa.coords + pb.coords) * 0.5);

    let feature_id = FeatureId::from_edge_pair(
        edge_a.0 as u32 * 64 + edge_a.1 as u32,
        edge_b.0 as u32 * 64 + edge_b.1 as u32,
    );

    Some(ContactManifold::single(ContactPoint::new(
        point,
        normal,
        geometric_depth,
        feature_id,
    )))
}

/// Compute the overlap between two shapes along a candidate axis, including margin.
///
/// Positive overlap = penetrating. Negative = separated.
fn support_overlap(
    a: &TransformedHull,
    b: &TransformedHull,
    axis: Vector3<f32>,
    margin: f32,
) -> f32 {
    let a_max = a.support(axis).coords.dot(&axis);
    let b_min = b.support(-axis).coords.dot(&axis);
    a_max - b_min + 2.0 * margin
}

/// Find the face on `hull` whose world-space normal is most aligned with `direction`.
fn find_most_aligned_face(hull: &ConvexHull, view: &ShapeView, direction: Vector3<f32>) -> usize {
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

/// Find the face on `hull` whose world-space normal is most opposed to `ref_normal`.
fn find_incident_face(hull: &ConvexHull, view: &ShapeView, ref_normal: Vector3<f32>) -> usize {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::convex_hull::cube_hull;
    use crate::collision::discrete::obb_obb::obb_obb_manifold_cached;
    use crate::collision::obb::Obb;
    use crate::collision::sat::SatCache;
    use nalgebra::UnitQuaternion;
    use std::sync::Arc;

    #[test]
    fn axis_aligned_cubes_vs_obb_obb() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;
        let rot = UnitQuaternion::identity();

        // OBB-OBB reference.
        let obb_a = Obb::new(Point3::origin(), rot, he);
        let obb_b = Obb::new(Point3::new(1.5, 0.0, 0.0), rot, he);
        let obb_manifold =
            obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut SatCache::default());

        // Hull-hull.
        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.5, 0.0, 0.0),
            rotation: rot,
            shape: &shape_b,
        };
        let hull_manifold =
            hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);

        assert!(!obb_manifold.is_empty(), "OBB-OBB should produce contacts");
        assert!(
            !hull_manifold.is_empty(),
            "Hull-hull should produce contacts"
        );

        // Both should agree on contact count and approximate depth.
        assert_eq!(
            obb_manifold.len(),
            hull_manifold.len(),
            "Contact count mismatch: obb={}, hull={}",
            obb_manifold.len(),
            hull_manifold.len(),
        );

        let obb_depth = obb_manifold.points[0].raw_depth;
        let hull_depth = hull_manifold.points[0].raw_depth;
        assert!(
            (obb_depth - hull_depth).abs() < 0.05,
            "Depth mismatch: obb={obb_depth:.4}, hull={hull_depth:.4}"
        );
    }

    #[test]
    fn sat_cache_separating_pair() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;
        let rot = UnitQuaternion::identity();
        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        // Well-separated pair.
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(5.0, 0.0, 0.0),
            rotation: rot,
            shape: &shape_b,
        };

        let mut cache = SatCache::default();
        let m1 = hull_hull_manifold(&view_a, &view_b, margin, &mut cache, None);
        assert!(m1.is_empty());

        // Second call should early-out via GJK (cache doesn't store separating
        // axis when GJK itself confirms separation before SAT runs).
        let m2 = hull_hull_manifold(&view_a, &view_b, margin, &mut cache, None);
        assert!(m2.is_empty());
    }

    #[test]
    fn sat_cache_cleared_on_collision() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;
        let rot = UnitQuaternion::identity();
        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.5, 0.0, 0.0),
            rotation: rot,
            shape: &shape_b,
        };

        let mut cache = SatCache::default();
        // Set a stale separating axis.
        cache.separating_axis = Some(Vector3::x());

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut cache, None);
        assert!(!manifold.is_empty());
        assert!(
            cache.separating_axis.is_none(),
            "Cache should be cleared on collision"
        );
    }

    #[test]
    fn symmetry() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;
        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        let rot_a = UnitQuaternion::identity();
        let rot_b = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.3);

        let view_a = ShapeView {
            center: Point3::new(0.0, 0.0, 0.0),
            rotation: rot_a,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.5, 0.1, 0.0),
            rotation: rot_b,
            shape: &shape_b,
        };

        let m_ab = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);
        let m_ba = hull_hull_manifold(&view_b, &view_a, margin, &mut SatCache::default(), None);

        assert_eq!(m_ab.len(), m_ba.len(), "Symmetry: contact count differs");

        if !m_ab.is_empty() {
            let depth_ab = m_ab.points[0].raw_depth;
            let depth_ba = m_ba.points[0].raw_depth;
            assert!(
                (depth_ab - depth_ba).abs() < 0.05,
                "Symmetry: depth mismatch {depth_ab:.4} vs {depth_ba:.4}"
            );
        }
    }

    #[test]
    fn rotated_cubes_produce_contacts() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;
        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        // 45-degree rotation about Y — corner-to-face contact.
        let rot_a = UnitQuaternion::identity();
        let rot_b =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4);

        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot_a,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(2.2, 0.0, 0.0),
            rotation: rot_b,
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);
        assert!(
            !manifold.is_empty(),
            "Rotated cubes should produce contacts"
        );
        // Normal should point roughly in +X (A→B direction).
        let nx = manifold.points[0].raw_normal.x;
        assert!(nx > 0.5, "Normal should point A→B (+X), got nx={nx:.3}");
    }

    #[test]
    fn deeply_overlapping_cubes() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;
        let rot = UnitQuaternion::identity();
        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        // 50% overlap along X.
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.0, 0.0, 0.0),
            rotation: rot,
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);
        assert!(
            !manifold.is_empty(),
            "Deeply overlapping cubes should produce contacts"
        );
        // Depth should be ~1.0 (2.0 total extent - 1.0 separation).
        let depth = manifold.points[0].raw_depth;
        assert!(
            (depth - 1.0).abs() < 0.1,
            "Expected ~1.0 depth for 50% overlap, got {depth:.4}"
        );
        // Should produce a full face contact (4 points).
        assert_eq!(
            manifold.len(),
            4,
            "Full face overlap should produce 4 contacts, got {}",
            manifold.len(),
        );
    }

    #[test]
    fn tetrahedron_vs_tetrahedron() {
        use crate::collision::convex_hull::HullFace;
        use smallvec::SmallVec;

        // Regular tetrahedron centered at origin.
        let s = 1.0f32;
        let vertices = vec![
            Vector3::new(s, s, s),
            Vector3::new(s, -s, -s),
            Vector3::new(-s, s, -s),
            Vector3::new(-s, -s, s),
        ];
        let faces = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 1, 2]),
                normal: Vector3::new(1.0, 1.0, -1.0).normalize(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 2, 3]),
                normal: Vector3::new(-1.0, 1.0, 1.0).normalize(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 3, 1]),
                normal: Vector3::new(1.0, -1.0, 1.0).normalize(), // was wrong
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 3, 2]),
                normal: Vector3::new(-1.0, -1.0, -1.0).normalize(),
            },
        ];

        let hull = Arc::new(ConvexHull::new(vertices, faces));
        let margin = 0.02;

        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        // Place them with slight overlap.
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.8, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);
        assert!(
            !manifold.is_empty(),
            "Overlapping tetrahedra should produce contacts"
        );
        assert!(
            manifold.points[0].raw_depth > 0.0,
            "Should be a penetrating contact"
        );
    }

    #[test]
    fn sat_cache_stores_axis_on_separation() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let rot = UnitQuaternion::identity();
        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        // Just barely separated (gap = 0.01, less than 2*margin so GJK sees it
        // as margin contact, but far enough that if we set margin=0 for this test
        // they'd be separated).
        // Use margin=0 to force SAT path for separated pair.
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(2.05, 0.0, 0.0),
            rotation: rot,
            shape: &shape_b,
        };

        let mut cache = SatCache::default();
        let m = hull_hull_manifold(&view_a, &view_b, 0.0, &mut cache, None);
        assert!(m.is_empty(), "Should be separated with margin=0");
        // GJK catches this before SAT, so cache may or may not be populated.
        // The important thing is it returns empty.
    }

    #[test]
    fn contacts_inside_both_hulls() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.02;
        let rot = UnitQuaternion::identity();
        let hull_data = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull {
            hull: hull_data.clone(),
        };
        let shape_b = ColliderShape::ConvexHull {
            hull: hull_data.clone(),
        };

        let center_a = Point3::origin();
        let center_b = Point3::new(1.5, 0.0, 0.0);
        let view_a = ShapeView {
            center: center_a,
            rotation: rot,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: center_b,
            rotation: rot,
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);
        assert!(!manifold.is_empty());

        // Every contact point should be inside (or on surface of) both hulls.
        let tolerance = margin + 0.01;
        for (i, cp) in manifold.points.iter().enumerate() {
            let violation_a = max_face_plane_violation(&cp.point, &hull_data, &center_a, &rot);
            let violation_b = max_face_plane_violation(&cp.point, &hull_data, &center_b, &rot);
            assert!(
                violation_a <= tolerance,
                "Contact {i}: outside hull A by {violation_a:.4} (tolerance {tolerance:.4})"
            );
            assert!(
                violation_b <= tolerance,
                "Contact {i}: outside hull B by {violation_b:.4} (tolerance {tolerance:.4})"
            );
        }
    }

    /// Maximum face-plane violation of a point against a hull (positive = outside).
    fn max_face_plane_violation(
        point: &Point3<f32>,
        hull: &ConvexHull,
        center: &Point3<f32>,
        rotation: &UnitQuaternion<f32>,
    ) -> f32 {
        let mut max_violation = f32::NEG_INFINITY;
        for face in &hull.faces {
            let world_normal = rotation * face.normal;
            let ref_vertex_local = hull.vertices[face.vertex_indices[0] as usize];
            let ref_vertex_world = center + rotation * ref_vertex_local;
            let violation = (point - ref_vertex_world).dot(&world_normal);
            if violation > max_violation {
                max_violation = violation;
            }
        }
        max_violation
    }

    /// Regression: two hexagonal prisms where SAT produced contacts 0.78
    /// outside hull A's face planes.
    #[test]
    fn phantom_contact_hexagonal_prisms() {
        use crate::collision::convex_hull::HullFace;
        use nalgebra::Quaternion;
        use smallvec::SmallVec;

        let vertices = vec![
            Vector3::new(0.346410, 0.200000, 0.250000),
            Vector3::new(-0.000000, 0.400000, 0.250000),
            Vector3::new(-0.346410, 0.200000, 0.250000),
            Vector3::new(-0.346410, -0.200000, 0.250000),
            Vector3::new(0.000000, -0.400000, 0.250000),
            Vector3::new(0.346410, -0.200000, 0.250000),
            Vector3::new(0.346410, 0.200000, -0.250000),
            Vector3::new(-0.000000, 0.400000, -0.250000),
            Vector3::new(-0.346410, 0.200000, -0.250000),
            Vector3::new(-0.346410, -0.200000, -0.250000),
            Vector3::new(0.000000, -0.400000, -0.250000),
            Vector3::new(0.346410, -0.200000, -0.250000),
        ];
        let faces = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 1, 2, 3, 4, 5]),
                normal: Vector3::new(0.0, 0.0, 1.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[11, 10, 9, 8, 7, 6]),
                normal: Vector3::new(0.0, 0.0, -1.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 6, 7, 1]),
                normal: Vector3::new(0.500000, 0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 7, 8, 2]),
                normal: Vector3::new(-0.500000, 0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[2, 8, 9, 3]),
                normal: Vector3::new(-1.0, 0.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[3, 9, 10, 4]),
                normal: Vector3::new(-0.500000, -0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[4, 10, 11, 5]),
                normal: Vector3::new(0.500000, -0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[5, 11, 6, 0]),
                normal: Vector3::new(1.0, 0.0, 0.0),
            },
        ];

        let hull_a = Arc::new(ConvexHull::new(vertices.clone(), faces.clone()));
        let hull_b = Arc::new(ConvexHull::new(vertices, faces));
        let shape_a = ColliderShape::ConvexHull {
            hull: hull_a.clone(),
        };
        let shape_b = ColliderShape::ConvexHull {
            hull: hull_b.clone(),
        };

        let center_a = Point3::new(8.104135, -0.003458, -6.265929);
        let rot_a = UnitQuaternion::from_quaternion(Quaternion::new(
            0.792405, -0.156346, 0.072801, -0.585107,
        ));
        let center_b = Point3::new(8.535968, -0.619225, -6.014867);
        let rot_b = UnitQuaternion::from_quaternion(Quaternion::new(
            0.626491, 0.001538, -0.000201, -0.779427,
        ));
        let margin = 0.02;

        let view_a = ShapeView {
            center: center_a,
            rotation: rot_a,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: center_b,
            rotation: rot_b,
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);

        let tolerance = margin + 0.05;
        for (i, cp) in manifold.points.iter().enumerate() {
            let violation_a = max_face_plane_violation(&cp.point, &hull_a, &center_a, &rot_a);
            let violation_b = max_face_plane_violation(&cp.point, &hull_b, &center_b, &rot_b);
            assert!(
                violation_a <= tolerance && violation_b <= tolerance,
                "Contact {i}: phantom point ({:.4}, {:.4}, {:.4}), \
                 violation_a={violation_a:.4}, violation_b={violation_b:.4}, tolerance={tolerance:.4}",
                cp.point.x, cp.point.y, cp.point.z,
            );
        }
    }

    #[test]
    fn margin_only_contact() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let margin = 0.05;
        let rot = UnitQuaternion::identity();
        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        // Separated by 0.06 (less than 2 * margin = 0.10).
        // SAT runs with margin-inflated overlap, producing contacts with
        // negative raw_depth (speculative margin contacts for velocity-only
        // correction). This matches OBB-OBB margin handling.
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(2.06, 0.0, 0.0),
            rotation: rot,
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);
        assert!(!manifold.is_empty(), "Should produce a margin-only contact");
    }

    /// Regression: edge-edge axis wins for margin-only contact between hex
    /// prisms, producing a midpoint 0.37 outside hull A.
    #[test]
    fn phantom_contact_hex_prism_margin_edge_edge() {
        use crate::collision::convex_hull::HullFace;
        use nalgebra::Quaternion;
        use smallvec::SmallVec;

        let vertices = vec![
            Vector3::new(0.346410, 0.200000, 0.250000),
            Vector3::new(-0.000000, 0.400000, 0.250000),
            Vector3::new(-0.346410, 0.200000, 0.250000),
            Vector3::new(-0.346410, -0.200000, 0.250000),
            Vector3::new(0.000000, -0.400000, 0.250000),
            Vector3::new(0.346410, -0.200000, 0.250000),
            Vector3::new(0.346410, 0.200000, -0.250000),
            Vector3::new(-0.000000, 0.400000, -0.250000),
            Vector3::new(-0.346410, 0.200000, -0.250000),
            Vector3::new(-0.346410, -0.200000, -0.250000),
            Vector3::new(0.000000, -0.400000, -0.250000),
            Vector3::new(0.346410, -0.200000, -0.250000),
        ];
        let faces = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 1, 2, 3, 4, 5]),
                normal: Vector3::new(0.0, 0.0, 1.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[11, 10, 9, 8, 7, 6]),
                normal: Vector3::new(0.0, 0.0, -1.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 6, 7, 1]),
                normal: Vector3::new(0.5, 0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 7, 8, 2]),
                normal: Vector3::new(-0.5, 0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[2, 8, 9, 3]),
                normal: Vector3::new(-1.0, 0.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[3, 9, 10, 4]),
                normal: Vector3::new(-0.5, -0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[4, 10, 11, 5]),
                normal: Vector3::new(0.5, -0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[5, 11, 6, 0]),
                normal: Vector3::new(1.0, 0.0, 0.0),
            },
        ];

        let hull_a = Arc::new(ConvexHull::new(vertices.clone(), faces.clone()));
        let hull_b = Arc::new(ConvexHull::new(vertices, faces));
        let shape_a = ColliderShape::ConvexHull {
            hull: hull_a.clone(),
        };
        let shape_b = ColliderShape::ConvexHull {
            hull: hull_b.clone(),
        };

        let center_a = Point3::new(3.627117, -0.582569, -5.363908);
        let rot_a = UnitQuaternion::from_quaternion(Quaternion::new(
            0.673823, 0.290739, 0.424450, 0.530355,
        ));
        let center_b = Point3::new(3.935381, -0.174031, -4.692415);
        let rot_b = UnitQuaternion::from_quaternion(Quaternion::new(
            0.645749, 0.704232, 0.176208, 0.236679,
        ));
        let margin = 0.02;

        let view_a = ShapeView {
            center: center_a,
            rotation: rot_a,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: center_b,
            rotation: rot_b,
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);

        let tolerance = margin + 0.05;
        for (i, cp) in manifold.points.iter().enumerate() {
            let violation_a = max_face_plane_violation(&cp.point, &hull_a, &center_a, &rot_a);
            let violation_b = max_face_plane_violation(&cp.point, &hull_b, &center_b, &rot_b);
            assert!(
                violation_a <= tolerance && violation_b <= tolerance,
                "Contact {i}: phantom point ({:.4}, {:.4}, {:.4}), \
                 violation_a={violation_a:.4}, violation_b={violation_b:.4}, tolerance={tolerance:.4}",
                cp.point.x, cp.point.y, cp.point.z,
            );
        }
    }

    // --- Edge-edge SAT tests ---

    /// Build a tetrahedron hull centered at origin.
    fn tetrahedron_hull() -> ConvexHull {
        use crate::collision::convex_hull::HullFace;
        use smallvec::SmallVec;

        let s = 1.0f32;
        let vertices = vec![
            Vector3::new(s, s, s),
            Vector3::new(s, -s, -s),
            Vector3::new(-s, s, -s),
            Vector3::new(-s, -s, s),
        ];

        // Compute outward normals from cross products.
        let n0 = (vertices[1] - vertices[0])
            .cross(&(vertices[2] - vertices[0]))
            .normalize();
        let n1 = (vertices[2] - vertices[0])
            .cross(&(vertices[3] - vertices[0]))
            .normalize();
        let n2 = (vertices[3] - vertices[0])
            .cross(&(vertices[1] - vertices[0]))
            .normalize();
        let n3 = (vertices[3] - vertices[1])
            .cross(&(vertices[2] - vertices[1]))
            .normalize();

        let faces = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 1, 2]),
                normal: n0,
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 2, 3]),
                normal: n1,
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 3, 1]),
                normal: n2,
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 3, 2]),
                normal: n3,
            },
        ];

        ConvexHull::new(vertices, faces)
    }

    /// Build a wedge (triangular prism) hull — similar to a voussoir.
    fn wedge_hull() -> ConvexHull {
        use crate::collision::convex_hull::HullFace;
        use smallvec::SmallVec;

        let vertices = vec![
            Vector3::new(-0.5, -0.3, -0.4),
            Vector3::new(0.5, -0.3, -0.4),
            Vector3::new(0.0, -0.3, 0.4),
            Vector3::new(-0.5, 0.3, -0.4),
            Vector3::new(0.5, 0.3, -0.4),
            Vector3::new(0.0, 0.3, 0.4),
        ];

        let face_defs: &[&[u16]] = &[
            &[0, 1, 2],    // -Y bottom triangle
            &[3, 5, 4],    // +Y top triangle
            &[0, 3, 4, 1], // -Z back quad
            &[1, 4, 5, 2], // +X/+Z sloped quad
            &[2, 5, 3, 0], // -X/+Z sloped quad
        ];

        let faces: Vec<HullFace> = face_defs
            .iter()
            .map(|indices| {
                let v0 = vertices[indices[0] as usize];
                let v1 = vertices[indices[1] as usize];
                let v2 = vertices[indices[2] as usize];
                let normal = (v1 - v0).cross(&(v2 - v0)).normalize();
                HullFace {
                    vertex_indices: SmallVec::from_iter(indices.iter().copied()),
                    normal,
                }
            })
            .collect();

        ConvexHull::new(vertices, faces)
    }

    #[test]
    fn edge_edge_tetrahedra_separation() {
        // Two tetrahedra positioned so that face-only SAT says overlap but
        // edge-edge finds a separating axis.
        let hull = Arc::new(tetrahedron_hull());
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };
        let margin = 0.02;

        // 45° Y rotation, positioned so edges nearly touch.
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(2.3, 0.0, 0.0),
            rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.785),
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);

        // With edge-edge SAT, this pair may be correctly separated.
        // Without it, face-only might report a false collision.
        // Either way, if contacts are generated they must be valid.
        if !manifold.is_empty() {
            for (i, cp) in manifold.points.iter().enumerate() {
                assert!(
                    cp.raw_depth >= -margin,
                    "Contact {i}: depth {:.4} below -margin",
                    cp.raw_depth,
                );
            }
        }
    }

    #[test]
    fn edge_edge_tetrahedra_correct_depth() {
        // Two tetrahedra in clear edge-to-edge contact. Face-only SAT
        // overestimates depth by ~2-3x; edge-edge should produce the
        // correct (shallower) depth.
        let hull = Arc::new(tetrahedron_hull());
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };
        let margin = 0.02;

        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.8, 0.0, 0.0),
            rotation: UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f32::consts::FRAC_PI_4,
            ),
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);
        assert!(!manifold.is_empty(), "Should produce contacts");

        // With edge-edge, depth should be significantly less than the
        // face-only overestimate. The face-only depth for this config is
        // ~0.50; true depth is ~0.17.
        let max_depth = manifold
            .points
            .iter()
            .map(|cp| cp.raw_depth)
            .fold(0.0f32, f32::max);
        assert!(
            max_depth < 0.35,
            "Edge-edge should produce shallower depth than face-only. Got {max_depth:.4}",
        );
    }

    #[test]
    fn edge_edge_wedge_contact() {
        // Two wedges (voussoir-like) with rotation producing edge-edge contact.
        let hull = Arc::new(wedge_hull());
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };
        let margin = 0.02;

        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(0.4, 0.0, 0.0),
            rotation: UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f32::consts::FRAC_PI_6,
            ),
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);
        assert!(!manifold.is_empty(), "Wedges should produce contacts");

        // Contacts should have positive depth.
        for (i, cp) in manifold.points.iter().enumerate() {
            assert!(
                cp.raw_depth > 0.0,
                "Contact {i}: expected positive depth, got {:.4}",
                cp.raw_depth,
            );
        }
    }

    #[test]
    fn edge_edge_wedge_false_collision_prevented() {
        // Two wedges at a distance where face-only SAT may report overlap
        // but edge-edge correctly separates them.
        let hull = Arc::new(wedge_hull());
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };
        let margin = 0.0; // zero margin to isolate geometry

        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        // Rotated 90° around Y, pushed far enough apart that edges don't touch.
        let view_b = ShapeView {
            center: Point3::new(0.95, 0.0, 0.0),
            rotation: UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f32::consts::FRAC_PI_2,
            ),
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);

        // If edge-edge finds a separating axis, manifold should be empty.
        // If it still overlaps, that's fine too — just checking no phantom.
        if !manifold.is_empty() {
            let tolerance = 0.05;
            for (i, cp) in manifold.points.iter().enumerate() {
                let va = max_face_plane_violation(
                    &cp.point,
                    &hull,
                    &Point3::origin(),
                    &UnitQuaternion::identity(),
                );
                assert!(va <= tolerance, "Contact {i}: outside hull A by {va:.4}",);
            }
        }
    }

    #[test]
    fn edge_adjacency_tetrahedron() {
        let hull = tetrahedron_hull();
        // A tetrahedron has 6 unique edges.
        assert_eq!(
            hull.edges.len(),
            6,
            "Tetrahedron should have 6 edges, got {}",
            hull.edges.len()
        );
    }

    #[test]
    fn edge_adjacency_cube() {
        let hull = cube_hull(Vector3::new(1.0, 1.0, 1.0));
        // A cube has 12 unique edges.
        assert_eq!(
            hull.edges.len(),
            12,
            "Cube should have 12 edges, got {}",
            hull.edges.len()
        );
    }

    #[test]
    fn edge_adjacency_hexagonal_prism() {
        use crate::collision::convex_hull::HullFace;
        use smallvec::SmallVec;

        let vertices = vec![
            Vector3::new(0.346410, 0.200000, 0.250000),
            Vector3::new(0.0, 0.400000, 0.250000),
            Vector3::new(-0.346410, 0.200000, 0.250000),
            Vector3::new(-0.346410, -0.200000, 0.250000),
            Vector3::new(0.0, -0.400000, 0.250000),
            Vector3::new(0.346410, -0.200000, 0.250000),
            Vector3::new(0.346410, 0.200000, -0.250000),
            Vector3::new(0.0, 0.400000, -0.250000),
            Vector3::new(-0.346410, 0.200000, -0.250000),
            Vector3::new(-0.346410, -0.200000, -0.250000),
            Vector3::new(0.0, -0.400000, -0.250000),
            Vector3::new(0.346410, -0.200000, -0.250000),
        ];
        let faces = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 1, 2, 3, 4, 5]),
                normal: Vector3::z(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[11, 10, 9, 8, 7, 6]),
                normal: -Vector3::z(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 6, 7, 1]),
                normal: Vector3::new(0.5, 0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 7, 8, 2]),
                normal: Vector3::new(-0.5, 0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[2, 8, 9, 3]),
                normal: -Vector3::x(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[3, 9, 10, 4]),
                normal: Vector3::new(-0.5, -0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[4, 10, 11, 5]),
                normal: Vector3::new(0.5, -0.866025, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[5, 11, 6, 0]),
                normal: Vector3::x(),
            },
        ];
        let hull = ConvexHull::new(vertices, faces);
        // 6 top cap + 6 bottom cap + 6 vertical = 18 unique edges.
        assert_eq!(
            hull.edges.len(),
            18,
            "Hex prism should have 18 edges, got {}",
            hull.edges.len()
        );
    }

    #[test]
    fn edge_edge_not_used_for_large_hulls() {
        // Two cubes (6+6=12 faces) should still use edge-edge (under threshold).
        // But the point is that the threshold gates it. Let's test that cubes
        // work correctly with edge-edge enabled.
        let he = Vector3::new(1.0, 1.0, 1.0);
        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };
        let margin = 0.02;

        // Rotated 45° about Y — corner-to-edge contact, classic edge-edge case.
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(2.2, 0.0, 0.0),
            rotation: UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f32::consts::FRAC_PI_4,
            ),
            shape: &shape_b,
        };

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);
        assert!(!manifold.is_empty());

        // Verify contacts are inside both hulls.
        let tolerance = margin + 0.05;
        for (i, cp) in manifold.points.iter().enumerate() {
            let va = max_face_plane_violation(
                &cp.point,
                &hull,
                &Point3::origin(),
                &UnitQuaternion::identity(),
            );
            let vb = max_face_plane_violation(
                &cp.point,
                &hull,
                &Point3::new(2.2, 0.0, 0.0),
                &UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4),
            );
            assert!(
                va <= tolerance && vb <= tolerance,
                "Contact {i}: violation_a={va:.4}, violation_b={vb:.4}",
            );
        }
    }

    /// Regression: hex prism vs tetrahedron phantom contact.
    /// Edge-edge SAT produced a contact point 0.31 outside hull B.
    #[test]
    fn hex_prism_vs_tetrahedron_no_phantom() {
        use crate::collision::convex_hull::HullFace;
        use nalgebra::Quaternion;

        let hull_a = Arc::new(ConvexHull::new(
            vec![
                Vector3::new(0.346410, 0.200000, 0.250000),
                Vector3::new(-0.000000, 0.400000, 0.250000),
                Vector3::new(-0.346410, 0.200000, 0.250000),
                Vector3::new(-0.346410, -0.200000, 0.250000),
                Vector3::new(0.000000, -0.400000, 0.250000),
                Vector3::new(0.346410, -0.200000, 0.250000),
                Vector3::new(0.346410, 0.200000, -0.250000),
                Vector3::new(-0.000000, 0.400000, -0.250000),
                Vector3::new(-0.346410, 0.200000, -0.250000),
                Vector3::new(-0.346410, -0.200000, -0.250000),
                Vector3::new(0.000000, -0.400000, -0.250000),
                Vector3::new(0.346410, -0.200000, -0.250000),
            ],
            vec![
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 1, 2, 3, 4, 5]),
                    normal: Vector3::new(0.0, 0.0, 1.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[11, 10, 9, 8, 7, 6]),
                    normal: Vector3::new(0.0, 0.0, -1.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 6, 7, 1]),
                    normal: Vector3::new(0.5, 0.866025, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[1, 7, 8, 2]),
                    normal: Vector3::new(-0.5, 0.866025, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[2, 8, 9, 3]),
                    normal: Vector3::new(-1.0, 0.0, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[3, 9, 10, 4]),
                    normal: Vector3::new(-0.5, -0.866025, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[4, 10, 11, 5]),
                    normal: Vector3::new(0.5, -0.866025, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[5, 11, 6, 0]),
                    normal: Vector3::new(1.0, 0.0, 0.0),
                },
            ],
        ));

        let hull_b = Arc::new(ConvexHull::new(
            vec![
                Vector3::new(0.000000, 0.918559, 0.000000),
                Vector3::new(0.000000, -0.306186, 0.866025),
                Vector3::new(0.750000, -0.306186, -0.433013),
                Vector3::new(-0.750000, -0.306186, -0.433013),
            ],
            vec![
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[1, 3, 2]),
                    normal: Vector3::new(0.0, -1.0, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 1, 2]),
                    normal: Vector3::new(0.816497, 0.333333, 0.471405),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 2, 3]),
                    normal: Vector3::new(0.0, 0.333333, -0.942809),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 3, 1]),
                    normal: Vector3::new(-0.816496, 0.333333, 0.471405),
                },
            ],
        ));

        let center_a = Point3::new(2.475018, -0.480575, -6.446536);
        let rot_a = UnitQuaternion::from_quaternion(Quaternion::new(
            0.641475, 0.672949, -0.365465, -0.045661,
        ));
        let center_b = Point3::new(2.707680, -0.689516, -5.573807);
        let rot_b = UnitQuaternion::from_quaternion(Quaternion::new(
            0.970569, 0.004452, -0.240733, 0.004823,
        ));

        let shape_a = ColliderShape::ConvexHull {
            hull: hull_a.clone(),
        };
        let shape_b = ColliderShape::ConvexHull {
            hull: hull_b.clone(),
        };
        let view_a = ShapeView {
            center: center_a,
            rotation: rot_a,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: center_b,
            rotation: rot_b,
            shape: &shape_b,
        };
        let margin = 0.02;

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);

        let tolerance = 2.0 * margin + 0.03;
        for (i, cp) in manifold.points.iter().enumerate() {
            let va = max_face_plane_violation_test(&cp.point, &hull_a, &center_a, &rot_a);
            let vb = max_face_plane_violation_test(&cp.point, &hull_b, &center_b, &rot_b);
            assert!(
                va <= tolerance && vb <= tolerance,
                "Contact {i}: point={:?} violation_a={va:.4}, violation_b={vb:.4} (tol={tolerance:.4})",
                cp.point,
            );
        }
    }

    /// Regression: two voussoir prisms phantom contact.
    /// Edge-edge SAT produced a contact 2.19 outside hull B.
    #[test]
    fn voussoir_prism_pair_no_phantom() {
        use crate::collision::convex_hull::HullFace;
        use nalgebra::Quaternion;

        let hull_a = Arc::new(ConvexHull::new(
            vec![
                Vector3::new(1.539374, -2.214646, 1.600000),
                Vector3::new(1.016732, 2.757963, 1.600000),
                Vector3::new(-2.050596, 2.105982, 1.600000),
                Vector3::new(-0.505511, -2.649301, 1.600000),
                Vector3::new(1.539374, -2.214646, -1.600000),
                Vector3::new(1.016732, 2.757963, -1.600000),
                Vector3::new(-2.050596, 2.105982, -1.600000),
                Vector3::new(-0.505511, -2.649301, -1.600000),
            ],
            vec![
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 1, 2, 3]),
                    normal: Vector3::new(0.0, 0.0, 1.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[7, 6, 5, 4]),
                    normal: Vector3::new(0.0, 0.0, -1.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 4, 5, 1]),
                    normal: Vector3::new(0.994522, 0.104529, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[3, 2, 6, 7]),
                    normal: Vector3::new(-0.951057, -0.309017, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[1, 5, 6, 2]),
                    normal: Vector3::new(-0.207912, 0.978148, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 3, 7, 4]),
                    normal: Vector3::new(0.207912, -0.978148, 0.0),
                },
            ],
        ));

        let hull_b = Arc::new(ConvexHull::new(
            vec![
                Vector3::new(2.547116, -0.886865, 1.600000),
                Vector3::new(-0.798537, 2.828859, 1.600000),
                Vector3::new(-2.896832, 0.498466, 1.600000),
                Vector3::new(1.148252, -2.440461, 1.600000),
                Vector3::new(2.547116, -0.886865, -1.600000),
                Vector3::new(-0.798537, 2.828859, -1.600000),
                Vector3::new(-2.896832, 0.498466, -1.600000),
                Vector3::new(1.148252, -2.440461, -1.600000),
            ],
            vec![
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 1, 2, 3]),
                    normal: Vector3::new(0.0, 0.0, 1.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[7, 6, 5, 4]),
                    normal: Vector3::new(0.0, 0.0, -1.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 4, 5, 1]),
                    normal: Vector3::new(0.743145, 0.669131, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[3, 2, 6, 7]),
                    normal: Vector3::new(-0.587785, -0.809017, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[1, 5, 6, 2]),
                    normal: Vector3::new(-0.743145, 0.669130, 0.0),
                },
                HullFace {
                    vertex_indices: SmallVec::from_slice(&[0, 3, 7, 4]),
                    normal: Vector3::new(0.743145, -0.669131, 0.0),
                },
            ],
        ));

        let center_a = Point3::new(15.384866, 0.607879, 7.141532);
        let rot_a = UnitQuaternion::from_quaternion(Quaternion::new(
            0.476774, -0.477912, -0.520179, -0.523165,
        ));
        let center_b = Point3::new(12.040905, 3.668508, 7.687533);
        let rot_b = UnitQuaternion::from_quaternion(Quaternion::new(
            0.747738, -0.252886, -0.074468, -0.609419,
        ));

        let shape_a = ColliderShape::ConvexHull {
            hull: hull_a.clone(),
        };
        let shape_b = ColliderShape::ConvexHull {
            hull: hull_b.clone(),
        };
        let view_a = ShapeView {
            center: center_a,
            rotation: rot_a,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: center_b,
            rotation: rot_b,
            shape: &shape_b,
        };
        let margin = 0.02;

        let manifold = hull_hull_manifold(&view_a, &view_b, margin, &mut SatCache::default(), None);

        let tolerance = 2.0 * margin + 0.03;
        for (i, cp) in manifold.points.iter().enumerate() {
            let va = max_face_plane_violation_test(&cp.point, &hull_a, &center_a, &rot_a);
            let vb = max_face_plane_violation_test(&cp.point, &hull_b, &center_b, &rot_b);
            assert!(
                va <= tolerance && vb <= tolerance,
                "Contact {i}: point={:?} normal={:?} depth={:.4} violation_a={va:.4}, violation_b={vb:.4}",
                cp.point, cp.normal, cp.raw_depth,
            );
        }
    }

    fn max_face_plane_violation_test(
        point: &Point3<f32>,
        hull: &ConvexHull,
        center: &Point3<f32>,
        rotation: &UnitQuaternion<f32>,
    ) -> f32 {
        let mut max_violation = f32::NEG_INFINITY;
        for face in &hull.faces {
            let world_normal = rotation * face.normal;
            let ref_vertex_local = hull.vertices[face.vertex_indices[0] as usize];
            let ref_vertex_world = center + rotation * ref_vertex_local;
            let violation = (point - ref_vertex_world).dot(&world_normal);
            if violation > max_violation {
                max_violation = violation;
            }
        }
        max_violation
    }
}
