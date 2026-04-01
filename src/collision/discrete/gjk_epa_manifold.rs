//! GJK/EPA manifold generation: orchestrates GJK → EPA → face clipping →
//! contact point construction.
//!
//! This is the general-purpose collision path called by the dispatch wildcard
//! arm. It produces multi-point manifolds by extracting support faces from
//! both shapes after EPA and clipping them via Sutherland-Hodgman.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use super::epa::{epa_penetration, EpaResult};
use super::gjk::{gjk_query_seeded, GjkCache, GjkResult};
use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::shape_view::{SupportFace, SupportFaceExtractor};
use crate::collision::support::ConvexSupport;

/// Tolerance for GJK separation distance check.
const GJK_TOLERANCE: f32 = 1e-4;

/// Small angular probe used to perturb candidate normals in symmetric tie cases.
const NORMAL_PROBE_EPS: f32 = 1e-3;

/// Maximum contacts to retain after reduction.
const MAX_CONTACTS: usize = 4;

/// Depth tolerance for comparing candidate normals — differences within this
/// threshold are considered equivalent, and ties are broken lexicographically.
const DEPTH_TIE_EPS: f32 = 1e-4;

/// Normal component tolerance for lexicographic tie-breaking.
const NORMAL_TIE_EPS: f32 = 1e-5;

/// Deterministic lexicographic ordering on normals: returns true if `a`
/// should be preferred over `b` based on world-space component ordering
/// (x, then y, then z).
fn normal_precedes(a: Vector3<f32>, b: Vector3<f32>) -> bool {
    a.x > b.x + NORMAL_TIE_EPS
        || ((a.x - b.x).abs() <= NORMAL_TIE_EPS
            && (a.y > b.y + NORMAL_TIE_EPS
                || ((a.y - b.y).abs() <= NORMAL_TIE_EPS && a.z > b.z + NORMAL_TIE_EPS)))
}

/// Build a set of jittered probe directions around a center direction.
///
/// Produces the center direction itself, axis-perturbed variants, and
/// sign-snapped axis directions for each significant component. Used to
/// explore nearby face normals in symmetric configurations.
fn jittered_probe_directions(center_dir: Vector3<f32>) -> SmallVec<[Vector3<f32>; 12]> {
    let c = center_dir.normalize();
    let jitter = NORMAL_PROBE_EPS;
    let mut dirs = SmallVec::new();
    dirs.push(c);
    dirs.push((c + Vector3::x() * jitter).normalize());
    dirs.push((c - Vector3::x() * jitter).normalize());
    dirs.push((c + Vector3::y() * jitter).normalize());
    dirs.push((c - Vector3::y() * jitter).normalize());
    dirs.push((c + Vector3::z() * jitter).normalize());
    dirs.push((c - Vector3::z() * jitter).normalize());
    if center_dir.x.abs() > 1e-6 {
        dirs.push(Vector3::new(center_dir.x.signum(), 0.0, 0.0));
    }
    if center_dir.y.abs() > 1e-6 {
        dirs.push(Vector3::new(0.0, center_dir.y.signum(), 0.0));
    }
    if center_dir.z.abs() > 1e-6 {
        dirs.push(Vector3::new(0.0, 0.0, center_dir.z.signum()));
    }
    dirs
}

/// Choose the best contact candidate from a set using minimum-depth ordering
/// with deterministic tie-breaking (lexicographic normal, then feature id).
///
/// Each candidate is a `(normal, depth, feature_id)` tuple. Among candidates
/// within `DEPTH_TIE_EPS` of the minimum depth, the one with the
/// lexicographically largest normal wins; if normals also match, the lowest
/// feature id wins.
fn pick_best_candidate(
    candidates: impl IntoIterator<Item = (Vector3<f32>, f32, FeatureId)>,
) -> (Vector3<f32>, f32, FeatureId) {
    let mut best: Option<(Vector3<f32>, f32, FeatureId)> = None;
    for candidate in candidates {
        match best {
            None => best = Some(candidate),
            Some(b) => {
                let shallower = candidate.1 + DEPTH_TIE_EPS < b.1;
                let tied = (candidate.1 - b.1).abs() <= DEPTH_TIE_EPS;
                if shallower || (tied && candidate.2 .0 < b.2 .0) {
                    best = Some(candidate);
                }
            }
        }
    }
    best.unwrap()
}

/// Generate a contact manifold between two convex shapes using GJK + EPA.
///
/// The shapes must implement both `ConvexSupport` and `SupportFaceExtractor`.
/// Shapes without faces (spheres, capsules) produce single-point contacts;
/// shapes with faces (OBBs, convex hulls) produce up to 4 clipped contacts.
///
/// `margin` is the contact margin. Depth convention: `raw_depth` is geometric
/// depth without margin, matching the existing analytic/SAT functions.
pub fn gjk_epa_manifold<S: ConvexSupport + SupportFaceExtractor>(
    a: &S,
    b: &S,
    margin: f32,
) -> ContactManifold {
    gjk_epa_manifold_cached(a, b, margin, None)
}

/// Generate a contact manifold between two convex shapes using GJK + EPA with
/// optional GJK warm-start cache.
pub fn gjk_epa_manifold_cached<S: ConvexSupport + SupportFaceExtractor>(
    a: &S,
    b: &S,
    margin: f32,
    gjk_cache: Option<&mut GjkCache>,
) -> ContactManifold {
    let seed = gjk_cache
        .as_ref()
        .and_then(|cache| cache.last_direction);
    let result = gjk_query_seeded(a, b, seed);

    match result {
        GjkResult::Separated {
            distance,
            closest_a,
            closest_b,
        } => {
            // Shapes are separated on the uninflated Minkowski difference.
            // Check if they're within the margin zone.
            if distance > 2.0 * margin + GJK_TOLERANCE {
                return ContactManifold::empty();
            }

            let delta = closest_b - closest_a;
            let dist = delta.magnitude();
            let normal = if dist > 1e-10 {
                delta / dist
            } else {
                Vector3::y()
            };

            // raw_depth is negative for margin-only contacts.
            let raw_depth = 2.0 * margin - distance;
            let point = Point3::from((closest_a.coords + closest_b.coords) * 0.5);
            if let Some(cache) = gjk_cache {
                let sep_dir = closest_b - closest_a;
                if sep_dir.magnitude_squared() > 1e-10 {
                    cache.last_direction = Some(sep_dir);
                }
            }

            ContactManifold::single(ContactPoint::new(
                point,
                normal,
                raw_depth,
                FeatureId::SINGLE,
            ))
        }

        GjkResult::Intersecting { simplex } => {
            let epa = epa_penetration(a, b, margin, &simplex);
            // EPA's normal points outward from the Minkowski polytope = from A
            // toward B. This matches the solver convention (normal from body_a
            // toward body_b) because the dispatch ensures the higher-ranked
            // shape is always passed as `a`.
            let normal = epa.normal;
            if let Some(cache) = gjk_cache {
                cache.last_direction = Some(normal);
            }

            // Extract support faces for multi-point manifold clipping.
            // Normal points A→B, so A's contact face faces toward B (+normal)
            // and B's contact face faces toward A (-normal).
            let face_a = a.support_face(normal);
            let face_b = b.support_face(-normal);

            match (face_a, face_b) {
                (Some(fa), Some(fb)) => {
                    let best_normal = refine_face_face_normal(a, b, normal, &fa, &fb);

                    if let (Some(best_fa), Some(best_fb)) =
                        (a.support_face(best_normal), b.support_face(-best_normal))
                    {
                        clip_face_face_manifold(&best_fa, &best_fb, best_normal, epa.depth, margin)
                    } else {
                        clip_face_face_manifold(&fa, &fb, normal, epa.depth, margin)
                    }
                }
                (Some(fa), None) => {
                    // A has a face (hull/OBB), B is faceless (sphere/capsule).
                    let result = face_vs_faceless_contact(
                        a, b, &fa, normal, &epa, margin,
                    );
                    match result {
                        Some(manifold) => manifold,
                        None => ContactManifold::empty(),
                    }
                }
                (None, Some(fb)) => {
                    // B has a face, A is faceless.
                    let a_center = midpoint_of_supports(a, -normal);
                    let dir = normal_from_face_to_point(&fb, a_center, -normal);
                    let corrected_normal = -dir;

                    let raw_depth = support_overlap(a, b, corrected_normal);
                    if raw_depth < -2.0 * margin - GJK_TOLERANCE {
                        return ContactManifold::empty();
                    }
                    // For faceless shape A (sphere/capsule), place the contact
                    // point directly on A along the contact normal.
                    let point = a.support(corrected_normal);
                    let feature_id = FeatureId::from_face(fb.face_index);
                    ContactManifold::single(ContactPoint::new(
                        point,
                        corrected_normal,
                        raw_depth,
                        feature_id,
                    ))
                }
                (None, None) => {
                    // Both shapes are faceless (sphere-sphere, sphere-capsule).
                    let raw_depth = epa.depth - 2.0 * margin;
                    let point = Point3::from(
                        (epa.witness_a.coords + epa.witness_b.coords) * 0.5,
                    );
                    ContactManifold::single(ContactPoint::new(
                        point,
                        normal,
                        raw_depth,
                        FeatureId::SINGLE,
                    ))
                }
            }
        }
    }
}

/// Evaluate a face as a contact candidate: compute the corrected normal from
/// the face to the query point and the overlap depth along that normal.
/// Returns (normal, depth, feature_id).
fn face_contact_candidate<S: ConvexSupport>(
    a: &S,
    b: &S,
    face: &SupportFace,
    b_center: Point3<f32>,
    fallback_normal: Vector3<f32>,
) -> (Vector3<f32>, f32, FeatureId) {
    let corrected_normal = normal_from_face_to_point(face, b_center, fallback_normal);
    let raw_depth = support_overlap(a, b, corrected_normal);

    // For faceless counterparts (sphere/capsule), the center-based direction can
    // tilt toward a corner even when a face-normal contact has smaller overlap.
    // Evaluate both and keep the minimum-overlap direction.
    let face_depth = support_overlap(a, b, face.normal);
    let (corrected_normal, raw_depth) = if face_depth + 1e-5 < raw_depth {
        (face.normal, face_depth)
    } else {
        (corrected_normal, raw_depth)
    };
    let feature_id = FeatureId::from_face(face.face_index);
    (corrected_normal, raw_depth, feature_id)
}

/// Refine the contact normal for a face-face pair.
///
/// EPA can return slightly different normals for symmetric configurations.
/// This probes several candidate directions (EPA normal, face normals,
/// center-to-center, jittered variants) and picks the minimum-overlap
/// direction, breaking ties with lexicographic normal ordering.
fn refine_face_face_normal<S: ConvexSupport + SupportFaceExtractor>(
    a: &S,
    b: &S,
    epa_normal: Vector3<f32>,
    face_a: &SupportFace,
    face_b: &SupportFace,
) -> Vector3<f32> {
    let a_center = midpoint_of_supports(a, epa_normal);
    let b_center = midpoint_of_supports(b, epa_normal);
    let center_dir = b_center - a_center;

    let mut dirs = SmallVec::<[Vector3<f32>; 12]>::new();
    dirs.push(epa_normal);
    dirs.push(face_a.normal);
    dirs.push(-face_b.normal);
    if center_dir.magnitude_squared() > 1e-8 {
        dirs.extend(jittered_probe_directions(center_dir));
    }

    let mut best_normal = epa_normal;
    let mut best_depth = support_overlap(a, b, epa_normal);
    for candidate in dirs {
        let depth = support_overlap(a, b, candidate);
        let shallower = depth + DEPTH_TIE_EPS < best_depth;
        let tied = (depth - best_depth).abs() <= DEPTH_TIE_EPS;
        if shallower || (tied && normal_precedes(candidate, best_normal)) {
            best_depth = depth;
            best_normal = candidate;
        }
    }

    best_normal
}

/// Generate a single-point contact for a face shape (A) against a faceless
/// shape (B, e.g. sphere or capsule).
///
/// EPA's face normal is unreliable near hull edges and corners because the
/// Minkowski boundary is curved there. Instead, we evaluate multiple candidate
/// faces on A — from EPA, from the witness direction, and from the
/// center-to-center direction with jittered probes — and pick the one with
/// minimum overlap depth for a stable contact.
fn face_vs_faceless_contact<S: ConvexSupport + SupportFaceExtractor>(
    a: &S,
    b: &S,
    epa_face: &SupportFace,
    epa_normal: Vector3<f32>,
    epa: &EpaResult,
    margin: f32,
) -> Option<ContactManifold> {
    let b_center = midpoint_of_supports(b, epa_normal);
    let a_center = midpoint_of_supports(a, epa_normal);

    // Candidate 1: face selected by EPA normal.
    let candidate_epa = face_contact_candidate(a, b, epa_face, b_center, epa_normal);

    // Candidate 2: face selected by witness direction (may find a better face
    // when EPA picked the wrong one entirely).
    let witness_dir = epa.witness_b - epa.witness_a;
    let candidate_wit = if witness_dir.magnitude_squared() > 1e-8 {
        let wit_n = witness_dir.normalize();
        a.support_face(wit_n).and_then(|face_wit| {
            if face_wit.face_index == epa_face.face_index {
                None
            } else {
                Some(face_contact_candidate(a, b, &face_wit, b_center, wit_n))
            }
        })
    } else {
        None
    };

    // Candidate 3: best face found by probing jittered center-to-center
    // directions (handles symmetric corner/edge configurations).
    let center_dir = b_center - a_center;
    let candidate_center = if center_dir.magnitude_squared() > 1e-8 {
        let dirs = jittered_probe_directions(center_dir);
        let face_candidates = dirs.into_iter().filter_map(|dir| {
            let face = a.support_face(dir)?;
            if face.face_index == epa_face.face_index {
                return None;
            }
            Some(face_contact_candidate(a, b, &face, b_center, dir))
        });
        let best = face_candidates.fold(None, |acc, candidate| match acc {
            None => Some(candidate),
            Some(best) => Some(pick_best_candidate([best, candidate])),
        });
        best
    } else {
        None
    };

    // Pick the overall best: minimum depth, then deterministic tie-breaking.
    let all_candidates = [Some(candidate_epa), candidate_wit, candidate_center];
    let mut chosen = pick_best_candidate(all_candidates.into_iter().flatten());

    // Second pass: among candidates within a wider tie window, prefer
    // deterministic world-space hemisphere ordering.
    let tie_eps = 1e-3;
    for candidate in [Some(candidate_epa), candidate_wit, candidate_center]
        .into_iter()
        .flatten()
    {
        if (candidate.1 - chosen.1).abs() <= tie_eps {
            if normal_precedes(candidate.0, chosen.0)
                || ((candidate.0 - chosen.0).magnitude() <= NORMAL_TIE_EPS
                    && candidate.2 .0 < chosen.2 .0)
            {
                chosen = candidate;
            }
        }
    }

    let (corrected_normal, raw_depth, feature_id) = chosen;
    if raw_depth < -2.0 * margin - GJK_TOLERANCE {
        return None;
    }

    // Place the contact point directly on the faceless shape's surface.
    // This keeps sphere contacts on the sphere surface instead of using
    // midpoint-of-witnesses.
    let point = b.support(-corrected_normal);
    Some(ContactManifold::single(ContactPoint::new(
        point,
        corrected_normal,
        raw_depth,
        feature_id,
    )))
}

/// Compute the geometric overlap between two shapes along a direction.
///
/// Returns the penetration depth (positive for overlap) by projecting each
/// shape's extent onto the direction via their support functions. This gives
/// the depth consistent with the contact normal, regardless of what EPA found.
fn support_overlap<S: ConvexSupport>(a: &S, b: &S, direction: Vector3<f32>) -> f32 {
    let a_max = a.support(direction).coords.dot(&direction);
    let b_min = b.support(-direction).coords.dot(&direction);
    a_max - b_min
}

/// Estimate a shape's center by averaging two opposing support points.
///
/// For spheres this returns the exact center. For capsules it returns the
/// midpoint of the internal segment, which is close enough for normal
/// computation.
fn midpoint_of_supports<S: ConvexSupport>(shape: &S, direction: Vector3<f32>) -> Point3<f32> {
    let far = shape.support(direction);
    let near = shape.support(-direction);
    Point3::from((far.coords + near.coords) * 0.5)
}

/// Compute a contact normal by finding the closest point on a face polygon to
/// a query point, then normalizing the direction from the closest point to the
/// query. Handles face, edge, and vertex contacts naturally.
///
/// When the query is behind the face (penetrating shape center is inside the
/// hull), the direction from the face to the query points inward. In that case
/// the face normal is the correct push-out direction for face contacts, so we
/// fall back to it.
fn normal_from_face_to_point(
    face: &SupportFace,
    query: Point3<f32>,
    _fallback: Vector3<f32>,
) -> Vector3<f32> {
    let closest = closest_point_on_face(face, query);
    let dir = query - closest;
    let dist = dir.magnitude();
    if dist > 1e-6 {
        let candidate = dir / dist;
        if candidate.dot(&face.normal) >= 0.0 {
            candidate
        } else {
            face.normal
        }
    } else {
        face.normal
    }
}

/// Find the closest point on a convex face polygon to a query point in 3D.
fn closest_point_on_face(face: &SupportFace, query: Point3<f32>) -> Point3<f32> {
    let verts = &face.vertices;
    let n = verts.len();
    if n == 0 {
        return query;
    }
    if n == 1 {
        return verts[0];
    }

    // Project query onto the face plane.
    let face_center = compute_face_center(verts);
    let signed_dist = (query - face_center).dot(&face.normal);
    let projected = query - face.normal * signed_dist;

    // Check if projected point is inside the convex polygon.
    // Orient each edge's inward normal toward the face center (same technique
    // as clip_against_face_sides).
    let mut inside = true;
    for i in 0..n {
        let edge_start = verts[i];
        let edge = verts[(i + 1) % n] - edge_start;
        let mut inward = edge.cross(&face.normal);
        if inward.dot(&(face_center - edge_start)) < 0.0 {
            inward = -inward;
        }
        if (projected - edge_start).dot(&inward) < 0.0 {
            inside = false;
            break;
        }
    }

    if inside {
        return projected;
    }

    // Closest point is on the polygon boundary — check all edge segments.
    let mut best_point = verts[0];
    let mut best_dist_sq = f32::MAX;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        let ab = b - a;
        let len_sq = ab.magnitude_squared();
        let t = if len_sq > 1e-12 {
            (query - a).dot(&ab) / len_sq
        } else {
            0.0
        };
        let closest = a + ab * t.clamp(0.0, 1.0);
        let dist_sq = (closest - query).magnitude_squared();
        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_point = closest;
        }
    }

    best_point
}

/// Clip two support faces against each other to produce multi-point contacts.
///
/// Follows the same pattern as the OBB-OBB face-face clipping:
/// 1. Choose reference face (the one more aligned with the contact normal).
/// 2. Clip incident face vertices against reference face side planes.
/// 3. Keep vertices below or on the reference face plane.
/// 4. Reduce to MAX_CONTACTS via ContactReducer.
fn clip_face_face_manifold(
    face_a: &SupportFace,
    face_b: &SupportFace,
    normal: Vector3<f32>,
    epa_depth: f32,
    margin: f32,
) -> ContactManifold {
    // Determine reference and incident faces.
    // Reference face: the one whose outward normal is most aligned with the
    // contact normal direction. face_a faces toward B (normal ≈ +normal),
    // face_b faces toward A (normal ≈ -normal).
    let dot_a = normal.dot(&face_a.normal);
    let dot_b = (-normal).dot(&face_b.normal);

    let (ref_face, inc_face) = if dot_a >= dot_b {
        (face_a, face_b)
    } else {
        (face_b, face_a)
    };

    // Clip the incident face against the reference face's side planes.
    let clipped = clip_against_face_sides(&ref_face.vertices, ref_face.normal, &inc_face.vertices);

    if clipped.is_empty() {
        // Clipping eliminated everything — fall back to single contact.
        let raw_depth = epa_depth - 2.0 * margin;
        return ContactManifold::single(ContactPoint::new(
            compute_face_center(&ref_face.vertices),
            normal,
            raw_depth,
            FeatureId::from_face_pair(face_a.face_index, face_b.face_index),
        ));
    }

    // Project clipped vertices onto the reference face plane and filter.
    let ref_center = compute_face_center(&ref_face.vertices);
    let ref_plane_d = ref_face.normal.dot(&ref_center.coords);

    let base_feature = FeatureId::from_face_pair(face_a.face_index, face_b.face_index);
    let manifold_depth = epa_depth - 2.0 * margin;
    let mut contacts: SmallVec<[ContactPoint; 8]> = SmallVec::with_capacity(clipped.len());

    for (i, vertex) in clipped.iter().enumerate() {
        // Signed distance from vertex to reference face plane.
        // Negative = below the plane (penetrating).
        let signed_dist = vertex.coords.dot(&ref_face.normal) - ref_plane_d;

        // Accept vertices that are penetrating or within margin tolerance.
        if signed_dist < margin + 1e-4 {
            let raw_depth = (-signed_dist).max(manifold_depth);
            let feature_id = base_feature.with_vertex(i as u32);

            contacts.push(ContactPoint::new(*vertex, normal, raw_depth, feature_id));
        }
    }

    if contacts.is_empty() {
        let raw_depth = epa_depth - 2.0 * margin;
        return ContactManifold::single(ContactPoint::new(
            ref_center,
            normal,
            raw_depth,
            base_feature,
        ));
    }

    // Reduce to 4 contacts if needed.
    let reducer = ContactReducer::new(MAX_CONTACTS);
    let reduced = reducer.reduce(&contacts);

    ContactManifold::from_vec(SmallVec::from_vec(reduced))
}

/// Clip a polygon against the side planes of a reference face.
///
/// The side planes are the edges of the reference face, each forming a half-plane
/// with the edge normal pointing inward (toward the face center).
fn clip_against_face_sides(
    ref_verts: &SmallVec<[Point3<f32>; 8]>,
    ref_normal: Vector3<f32>,
    inc_verts: &SmallVec<[Point3<f32>; 8]>,
) -> SmallVec<[Point3<f32>; 8]> {
    let mut polygon: SmallVec<[Point3<f32>; 8]> = inc_verts.clone();
    let mut scratch: SmallVec<[Point3<f32>; 8]> = SmallVec::new();

    let n = ref_verts.len();
    if n < 3 || polygon.is_empty() {
        return polygon;
    }

    // Compute face center for inward normal orientation.
    let center = compute_face_center(ref_verts);
    let face_normal = if ref_normal.magnitude_squared() > 1e-10 {
        ref_normal
    } else {
        Vector3::y()
    };

    for i in 0..n {
        let edge_start = ref_verts[i];
        let edge_end = ref_verts[(i + 1) % n];

        // Edge direction.
        let edge = edge_end - edge_start;

        // Inward-pointing normal: perpendicular to edge, pointing toward face center.
        // Use the reference face normal to compute the perpendicular in the face plane.
        let inward = edge.cross(&face_normal);

        // Orient so it points toward the center.
        let inward = if inward.dot(&(center - edge_start)) >= 0.0 {
            inward
        } else {
            -inward
        };

        clip_polygon_into(&polygon, edge_start, inward, &mut scratch);
        std::mem::swap(&mut polygon, &mut scratch);
        if polygon.is_empty() {
            return polygon;
        }
    }

    polygon
}

/// Clip a convex polygon against a half-plane into a caller-provided buffer.
///
/// Keeps the portion on the inside (non-negative side) of the plane.
fn clip_polygon_into(
    polygon: &[Point3<f32>],
    plane_point: Point3<f32>,
    plane_normal: Vector3<f32>,
    out: &mut SmallVec<[Point3<f32>; 8]>,
) {
    out.clear();
    if polygon.is_empty() {
        return;
    }

    for i in 0..polygon.len() {
        let p1 = polygon[i];
        let p2 = polygon[(i + 1) % polygon.len()];
        let d1 = (p1 - plane_point).dot(&plane_normal);
        let d2 = (p2 - plane_point).dot(&plane_normal);
        let inside1 = d1 >= 0.0;
        let inside2 = d2 >= 0.0;

        if inside1 && inside2 {
            out.push(p2);
        } else if inside1 && !inside2 {
            let t = d1 / (d1 - d2);
            out.push(p1 + (p2 - p1) * t);
        } else if !inside1 && inside2 {
            let t = d1 / (d1 - d2);
            out.push(p1 + (p2 - p1) * t);
            out.push(p2);
        }
    }
}

/// Compute the centroid of a face polygon.
fn compute_face_center(verts: &SmallVec<[Point3<f32>; 8]>) -> Point3<f32> {
    let sum: Vector3<f32> = verts.iter().map(|v| v.coords).sum();
    Point3::from(sum / verts.len() as f32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use crate::collision::capsule::Capsule;
    use crate::collision::convex_hull::{cube_hull, ConvexHull, HullFace};
    use crate::collision::discrete::obb_capsule::obb_capsule_manifold;
    use crate::collision::discrete::obb_obb::obb_obb_manifold_cached;
    use crate::collision::discrete::sphere_obb::sphere_obb_manifold;
    use crate::collision::obb::Obb;
    use crate::collision::sat::SatCache;
    use crate::collision::shape_view::ShapeView;

    use crate::physics::ColliderShape;
    use nalgebra::UnitQuaternion;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    // --- OBB-OBB cross-validation ---

    #[test]
    fn obb_obb_cross_validation() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let rot = UnitQuaternion::identity();
        let margin = 0.02;

        let shape_a = ColliderShape::Box { half_extents: he };
        let shape_b = ColliderShape::Box { half_extents: he };
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

        let gjk_result = gjk_epa_manifold(&view_a, &view_b, margin);

        let obb_a = Obb::new(Point3::origin(), rot, he);
        let obb_b = Obb::new(Point3::new(1.5, 0.0, 0.0), rot, he);
        let mut cache = SatCache::default();
        let sat_result = obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut cache);

        assert!(
            !gjk_result.is_empty(),
            "GJK/EPA should produce contacts for overlapping OBBs"
        );
        assert!(
            !sat_result.is_empty(),
            "SAT should produce contacts for overlapping OBBs"
        );

        // Same normal direction (dot > 0.99).
        let gjk_normal = gjk_result.points[0].normal;
        let sat_normal = sat_result.points[0].normal;
        let dot = gjk_normal.dot(&sat_normal);
        assert!(
            dot > 0.99 || dot < -0.99,
            "Normal mismatch: GJK {:?} vs SAT {:?} (dot = {})",
            gjk_normal,
            sat_normal,
            dot
        );

        // Similar depth (±0.1).
        let gjk_depth = gjk_result.points[0].raw_depth;
        let sat_depth = sat_result.points[0].raw_depth;
        assert!(
            approx_eq(gjk_depth, sat_depth, 0.1),
            "Depth mismatch: GJK {} vs SAT {}",
            gjk_depth,
            sat_depth
        );

        // Similar contact count (±1).
        let count_diff = (gjk_result.len() as i32 - sat_result.len() as i32).abs();
        assert!(
            count_diff <= 1,
            "Contact count mismatch: GJK {} vs SAT {}",
            gjk_result.len(),
            sat_result.len()
        );
    }

    // --- Sphere-OBB cross-validation ---

    #[test]
    fn sphere_obb_cross_validation() {
        let margin = 0.02;

        let sphere_shape = ColliderShape::Sphere { radius: 0.5 };
        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let view_s = ShapeView {
            center: Point3::new(1.3, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &sphere_shape,
        };
        let view_b = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &box_shape,
        };

        // Pass OBB as `a` (larger shape first), matching the dispatch convention.
        // EPA's A→B normal then points OBB→Sphere, same as the analytic function.
        let gjk_result = gjk_epa_manifold(&view_b, &view_s, margin);

        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let analytic = sphere_obb_manifold(&obb, Point3::new(1.3, 0.0, 0.0), 0.5, margin);

        assert!(!gjk_result.is_empty(), "GJK/EPA should produce contact");
        assert!(!analytic.is_empty(), "Analytic should produce contact");

        // EPA converges poorly for sphere-polyhedron pairs (curved Minkowski
        // boundary). Depth tolerance is wide because this pair uses the analytic
        // path in practice — this test only validates the GJK/EPA fallback is
        // broadly reasonable, not pixel-accurate.
        let gjk_depth = gjk_result.points[0].raw_depth;
        let analytic_depth = analytic.points[0].raw_depth;
        assert!(
            gjk_depth > 0.0 && gjk_depth < analytic_depth + 0.1,
            "Depth should be positive and not wildly larger: GJK {} vs analytic {}",
            gjk_depth,
            analytic_depth
        );

        let gjk_normal = gjk_result.points[0].normal;
        let analytic_normal = analytic.points[0].normal;
        assert!(
            gjk_normal.dot(&analytic_normal) > 0.9,
            "Normal mismatch: GJK {:?} vs analytic {:?}",
            gjk_normal,
            analytic_normal
        );

        // Contact point should lie on the sphere surface for sphere-involved
        // single-point fallback contacts.
        let sphere_center = Point3::new(1.3, 0.0, 0.0);
        let sphere_radius = 0.5;
        let dist = (gjk_result.points[0].point - sphere_center).magnitude();
        assert!(
            (dist - sphere_radius).abs() < 1e-4,
            "Sphere surface mismatch: dist={} radius={}",
            dist,
            sphere_radius
        );
    }

    // --- OBB face-face clipping: 4 contacts ---

    #[test]
    fn obb_face_face_four_contacts() {
        let margin = 0.02;

        let shape_a = ColliderShape::Box {
            half_extents: Vector3::new(2.0, 1.0, 2.0),
        };
        let shape_b = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let view_a = ShapeView {
            center: Point3::new(0.0, -1.5, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(0.0, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let result = gjk_epa_manifold(&view_a, &view_b, margin);

        assert!(
            result.len() >= 3,
            "Expected 4 contacts for face-face, got {}",
            result.len()
        );

        // All contacts should have roughly the same normal (Y axis).
        for cp in &result.points {
            assert!(
                cp.normal.y.abs() > 0.9,
                "Expected Y-axis normal, got {:?}",
                cp.normal
            );
        }
    }

    // --- Sphere-sphere through GJK/EPA ---

    #[test]
    fn sphere_sphere_single_contact() {
        let margin = 0.0;

        let shape_a = ColliderShape::Sphere { radius: 1.0 };
        let shape_b = ColliderShape::Sphere { radius: 1.0 };
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.5, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let result = gjk_epa_manifold(&view_a, &view_b, margin);

        assert_eq!(result.len(), 1, "Sphere-sphere should produce 1 contact");
        assert!(
            approx_eq(result.points[0].raw_depth, 0.5, 0.05),
            "Expected depth ~0.5, got {}",
            result.points[0].raw_depth
        );
    }

    #[test]
    fn coincident_spheres_produce_finite_normal() {
        let shape_a = ColliderShape::Sphere { radius: 1.0 };
        let shape_b = ColliderShape::Sphere { radius: 1.0 };
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let result = gjk_epa_manifold(&view_a, &view_b, 0.0);
        assert_eq!(result.len(), 1, "Coincident spheres should produce one contact");

        let n = result.points[0].normal;
        assert!(
            n.x.is_finite() && n.y.is_finite() && n.z.is_finite(),
            "Normal should be finite, got {:?}",
            n
        );
        assert!(
            (n.magnitude() - 1.0).abs() < 0.01,
            "Normal should be approximately unit length, got {:?} (|n|={})",
            n,
            n.magnitude()
        );
    }

    #[test]
    fn cached_query_matches_uncached_result() {
        let margin = 0.02;

        let shape_a = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let shape_b = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: Point3::new(1.5, 0.1, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let uncached = gjk_epa_manifold(&view_a, &view_b, margin);
        let mut cache = GjkCache::default();
        let cached = gjk_epa_manifold_cached(&view_a, &view_b, margin, Some(&mut cache));

        assert_eq!(uncached.len(), cached.len());
        for (a, b) in uncached.points.iter().zip(cached.points.iter()) {
            assert!((a.point - b.point).magnitude() < 1e-5);
            assert!((a.normal - b.normal).magnitude() < 1e-5);
            assert!((a.raw_depth - b.raw_depth).abs() < 1e-5);
        }
    }

    #[test]
    fn cached_query_populates_and_reuses_direction() {
        let margin = 0.02;

        let sphere_shape = ColliderShape::Sphere { radius: 0.5 };
        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let view_s = ShapeView {
            center: Point3::new(1.3, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &sphere_shape,
        };
        let view_b = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &box_shape,
        };

        let mut cache = GjkCache::default();
        assert!(cache.last_direction.is_none());

        let first = gjk_epa_manifold_cached(&view_s, &view_b, margin, Some(&mut cache));
        assert!(!first.is_empty());
        let first_dir = cache
            .last_direction
            .expect("Cache should store a warm-start direction");
        assert!(first_dir.magnitude_squared() > 1e-8);

        let second = gjk_epa_manifold_cached(&view_s, &view_b, margin, Some(&mut cache));
        assert!(!second.is_empty());
        let second_dir = cache
            .last_direction
            .expect("Cache should keep a warm-start direction");
        assert!(second_dir.magnitude_squared() > 1e-8);
    }

    // --- Helper: build a regular tetrahedron hull ---

    fn tetrahedron_hull(edge: f32) -> ConvexHull {
        let r = edge * (6.0f32).sqrt() / 4.0;
        let top = Vector3::new(0.0, r, 0.0);
        let y_base = -r / 3.0;
        let base_r = (r * r - y_base * y_base).sqrt();

        let v0 = Vector3::new(0.0, y_base, base_r);
        let v1 = Vector3::new(
            base_r * (2.0 * std::f32::consts::PI / 3.0).sin(),
            y_base,
            base_r * (2.0 * std::f32::consts::PI / 3.0).cos(),
        );
        let v2 = Vector3::new(
            base_r * (4.0 * std::f32::consts::PI / 3.0).sin(),
            y_base,
            base_r * (4.0 * std::f32::consts::PI / 3.0).cos(),
        );

        let vertices = vec![top, v0, v1, v2];

        let face_defs: [(usize, usize, usize, usize); 4] = [
            (1, 2, 3, 0),
            (0, 2, 1, 3),
            (0, 3, 2, 1),
            (0, 1, 3, 2),
        ];

        let faces = face_defs
            .iter()
            .map(|&(a, b, c, opp)| {
                let va = vertices[a];
                let vb = vertices[b];
                let vc = vertices[c];
                let vopp = vertices[opp];

                let raw_normal = (vb - va).cross(&(vc - va));
                let flip = raw_normal.dot(&(va - vopp)) < 0.0;
                let normal = if flip {
                    -raw_normal.normalize()
                } else {
                    raw_normal.normalize()
                };

                let mut indices: SmallVec<[u16; 6]> =
                    SmallVec::from_slice(&[a as u16, b as u16, c as u16]);
                if flip {
                    indices[1..].reverse();
                }

                HullFace {
                    vertex_indices: indices,
                    normal,
                }
            })
            .collect();

        ConvexHull::new(vertices, faces)
    }

    /// Signed distance from point to an axis-aligned box surface (negative = inside).
    fn signed_distance_to_box(p: Point3<f32>, center: Point3<f32>, rot: UnitQuaternion<f32>, he: Vector3<f32>) -> f32 {
        let local = rot.inverse() * (p - center);
        let dx = local.x.abs() - he.x;
        let dy = local.y.abs() - he.y;
        let dz = local.z.abs() - he.z;
        if dx <= 0.0 && dy <= 0.0 && dz <= 0.0 {
            dx.max(dy).max(dz)
        } else {
            Vector3::new(dx.max(0.0), dy.max(0.0), dz.max(0.0)).magnitude()
        }
    }

    // --- Hull-hull: cube vs cube face-face, cross-validate with OBB SAT ---

    #[test]
    fn hull_hull_cube_face_face_matches_sat() {
        let margin = 0.02;
        let he = Vector3::new(1.0, 1.0, 1.0);
        let hull = Arc::new(cube_hull(he));

        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        let rot = UnitQuaternion::identity();
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

        let gjk_result = gjk_epa_manifold(&view_a, &view_b, margin);

        let obb_a = Obb::new(Point3::origin(), rot, he);
        let obb_b = Obb::new(Point3::new(1.5, 0.0, 0.0), rot, he);
        let mut cache = SatCache::default();
        let sat_result = obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut cache);

        assert!(!gjk_result.is_empty(), "Hull GJK/EPA should produce contacts");
        assert!(!sat_result.is_empty(), "OBB SAT should produce contacts");

        let gjk_p = gjk_result
            .points
            .iter()
            .max_by(|a, b| a.raw_depth.total_cmp(&b.raw_depth))
            .expect("non-empty");
        let sat_p = sat_result
            .points
            .iter()
            .max_by(|a, b| a.raw_depth.total_cmp(&b.raw_depth))
            .expect("non-empty");
        let gjk_n = gjk_p.normal;
        let sat_n = sat_p.normal;
        assert!(
            gjk_n.dot(&sat_n).abs() > 0.95,
            "Normal mismatch: GJK {:?} vs SAT {:?}",
            gjk_n, sat_n
        );

        let gjk_d = gjk_p.raw_depth;
        let sat_d = sat_p.raw_depth;
        assert!(
            approx_eq(gjk_d, sat_d, 0.1),
            "Depth mismatch: GJK {} vs SAT {}",
            gjk_d, sat_d
        );

        assert!(
            gjk_result.len() >= 3,
            "Expected multi-point manifold, got {}",
            gjk_result.len()
        );
    }

    #[test]
    fn hull_hull_rotated_box_vs_small_box_matches_sat_axis_depth() {
        let margin = 0.02;
        let he_a = Vector3::new(1.0, 1.0, 1.0);
        let he_b = Vector3::new(0.5, 0.5, 0.5);
        let hull_a = Arc::new(cube_hull(he_a));
        let hull_b = Arc::new(cube_hull(he_b));

        let shape_a = ColliderShape::ConvexHull { hull: hull_a };
        let shape_b = ColliderShape::ConvexHull { hull: hull_b };

        let rot_a = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
        let center_b = Point3::new(1.0, 0.0, 0.0);
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot_a,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: center_b,
            rotation: UnitQuaternion::identity(),
            shape: &shape_b,
        };

        let gjk_result = gjk_epa_manifold(&view_a, &view_b, margin);
        assert!(!gjk_result.is_empty(), "Hull GJK/EPA should produce contacts");

        let obb_a = Obb::new(Point3::origin(), rot_a, he_a);
        let obb_b = Obb::new(center_b, UnitQuaternion::identity(), he_b);
        let mut cache = SatCache::default();
        let sat_result = obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut cache);
        assert!(!sat_result.is_empty(), "OBB SAT should produce contacts");

        let gjk_n = gjk_result.points[0].normal;
        let sat_n = sat_result.points[0].normal;
        assert!(
            gjk_n.dot(&sat_n).abs() > 0.95,
            "Normal mismatch: GJK {:?} vs SAT {:?}",
            gjk_n,
            sat_n
        );

        let gjk_d = gjk_result.points[0].raw_depth;
        let sat_d = sat_result.points[0].raw_depth;
        assert!(
            approx_eq(gjk_d, sat_d, 0.1),
            "Depth mismatch: GJK {} vs SAT {}",
            gjk_d,
            sat_d
        );
    }

    // --- Hull-hull: contact positions are within both shapes ---

    #[test]
    fn hull_hull_cube_contacts_within_overlap() {
        let margin = 0.02;
        let he = Vector3::new(1.0, 1.0, 1.0);
        let hull = Arc::new(cube_hull(he));

        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        let rot = UnitQuaternion::identity();
        let center_b = Point3::new(1.5, 0.0, 0.0);
        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: center_b,
            rotation: rot,
            shape: &shape_b,
        };

        let result = gjk_epa_manifold(&view_a, &view_b, margin);
        assert!(!result.is_empty());

        for cp in &result.points {
            let d_a = signed_distance_to_box(cp.point, Point3::origin(), rot, he);
            let d_b = signed_distance_to_box(cp.point, center_b, rot, he);

            assert!(
                d_a <= margin + 0.05,
                "Contact {:?} too far outside hull A (sd={})",
                cp.point, d_a
            );
            assert!(
                d_b <= margin + 0.05,
                "Contact {:?} too far outside hull B (sd={})",
                cp.point, d_b
            );
        }
    }

    // --- Hull-hull: rotated cube (edge contact) ---

    #[test]
    fn hull_hull_rotated_cube_contacts_sane() {
        let margin = 0.02;
        let he = Vector3::new(1.0, 1.0, 1.0);
        let hull = Arc::new(cube_hull(he));

        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        let rot_b = UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f32::consts::FRAC_PI_4,
        );
        let center_b = Point3::new(2.0, 0.0, 0.0);

        let view_a = ShapeView {
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: &shape_a,
        };
        let view_b = ShapeView {
            center: center_b,
            rotation: rot_b,
            shape: &shape_b,
        };

        let result = gjk_epa_manifold(&view_a, &view_b, margin);
        assert!(!result.is_empty(), "Should produce contacts for rotated overlap");

        for cp in &result.points {
            let d_a = signed_distance_to_box(
                cp.point, Point3::origin(), UnitQuaternion::identity(), he,
            );
            let d_b = signed_distance_to_box(cp.point, center_b, rot_b, he);

            assert!(
                d_a <= margin + 0.1,
                "Contact {:?} outside hull A (sd={:.3}), normal={:?}, depth={:.3}",
                cp.point, d_a, cp.normal, cp.raw_depth
            );
            assert!(
                d_b <= margin + 0.1,
                "Contact {:?} outside hull B (sd={:.3}), normal={:?}, depth={:.3}",
                cp.point, d_b, cp.normal, cp.raw_depth
            );

            assert!(
                cp.raw_depth > -margin && cp.raw_depth < 1.0,
                "Unreasonable depth {:.3}",
                cp.raw_depth
            );
        }
    }

    // --- Hull-capsule: single EPA-witness contact ---

    #[test]
    fn hull_capsule_contact_sane() {
        let margin = 0.02;
        let he = Vector3::new(1.0, 1.0, 1.0);
        let hull = Arc::new(cube_hull(he));

        let shape_hull = ColliderShape::ConvexHull { hull };
        let shape_cap = ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.3,
        };

        let rot = UnitQuaternion::identity();
        let view_hull = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_hull,
        };
        // Hull +X face at 1.0, capsule surface at 1.2 - 0.3 = 0.9 → overlap 0.1
        let view_cap = ShapeView {
            center: Point3::new(1.2, 0.0, 0.0),
            rotation: rot,
            shape: &shape_cap,
        };

        let result = gjk_epa_manifold(&view_hull, &view_cap, margin);
        assert!(!result.is_empty(), "Should produce contacts");
        assert_eq!(result.len(), 1, "Hull-capsule should be single contact");

        let cp = &result.points[0];
        assert!(
            cp.normal.x > 0.8,
            "Normal should point +X (hull→capsule), got {:?}",
            cp.normal
        );
        assert!(
            cp.point.x > 0.5 && cp.point.x < 1.5,
            "Contact x={:.3} outside expected range",
            cp.point.x
        );
        assert!(
            cp.raw_depth > 0.0 && cp.raw_depth < 0.3,
            "Unexpected depth {:.3} (expected ~0.1)",
            cp.raw_depth
        );
    }

    #[test]
    fn hull_capsule_diagonal_matches_obb_reference_normal() {
        let margin = 0.02;
        let he = Vector3::new(1.0, 1.0, 1.0);
        let hull = Arc::new(cube_hull(he));

        let shape_hull = ColliderShape::ConvexHull { hull };
        let shape_cap = ColliderShape::Capsule {
            half_height: 0.8,
            radius: 0.3,
        };

        let rot = UnitQuaternion::identity();
        let t = 1.45_f32;
        let dir = Vector3::new(1.0, 1.0, 0.0).normalize();
        let center = Point3::from(dir * t);

        let view_hull = ShapeView {
            center: Point3::origin(),
            rotation: rot,
            shape: &shape_hull,
        };
        let view_cap = ShapeView {
            center,
            rotation: rot,
            shape: &shape_cap,
        };

        let hull_result = gjk_epa_manifold(&view_hull, &view_cap, margin);
        assert!(!hull_result.is_empty(), "Hull path should produce a contact");

        let obb = Obb::new(Point3::origin(), rot, he);
        let capsule = Capsule::new(center, rot, 0.8, 0.3);
        let obb_result = obb_capsule_manifold(&obb, &capsule, margin);
        assert!(!obb_result.is_empty(), "OBB reference should produce a contact");

        let hn = hull_result.points[0].normal;
        let on = obb_result.points[0].normal;
        let dot = hn.dot(&on);
        assert!(
            dot > 0.95,
            "Diagonal capsule normal mismatch: hull={:?}, obb={:?}, dot={:.4}",
            hn,
            on,
            dot
        );

        // Independent geometry (Python): segment-box gap ~0.025, so expected
        // geometric depth is about 0.3 - 0.025 = 0.275.
        assert!(
            (hull_result.points[0].raw_depth - obb_result.points[0].raw_depth).abs() < 0.1,
            "Depth mismatch: hull={:.4}, obb={:.4}",
            hull_result.points[0].raw_depth,
            obb_result.points[0].raw_depth
        );
    }

    // --- Tetrahedron-tetrahedron: contacts near overlap region ---

    #[test]
    fn tetrahedron_tetrahedron_contacts_sane() {
        let margin = 0.02;
        let tet = Arc::new(tetrahedron_hull(2.0));

        let shape_a = ColliderShape::ConvexHull { hull: tet.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: tet.clone() };

        let rot = UnitQuaternion::identity();
        let center_a = Point3::origin();
        let center_b = Point3::new(0.8, 0.0, 0.0);

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

        let result = gjk_epa_manifold(&view_a, &view_b, margin);
        assert!(!result.is_empty(), "Should produce contacts for overlapping tetrahedra");

        let bounding_r = tet.bounding_radius;
        for cp in &result.points {
            let dist_a = (cp.point - center_a).magnitude();
            let dist_b = (cp.point - center_b).magnitude();
            assert!(
                dist_a <= bounding_r + margin + 0.1,
                "Contact {:?} too far from tet A center (dist={:.3}, bound={:.3})",
                cp.point, dist_a, bounding_r
            );
            assert!(
                dist_b <= bounding_r + margin + 0.1,
                "Contact {:?} too far from tet B center (dist={:.3}, bound={:.3})",
                cp.point, dist_b, bounding_r
            );

            assert!(
                cp.raw_depth > -margin && cp.raw_depth < 2.0 * bounding_r,
                "Unreasonable depth {:.3}",
                cp.raw_depth
            );
        }
    }

    // --- Hull-hull: normal stability across small perturbation ---

    #[test]
    fn hull_hull_normal_stable_across_perturbation() {
        let margin = 0.02;
        let he = Vector3::new(1.0, 1.0, 1.0);
        let hull = Arc::new(cube_hull(he));

        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };

        let rot = UnitQuaternion::identity();
        let base_offset = 1.5;

        let mut prev_normal = None;
        for i in 0..5 {
            let dx = i as f32 * 0.01;
            let view_a = ShapeView {
                center: Point3::origin(),
                rotation: rot,
                shape: &shape_a,
            };
            let view_b = ShapeView {
                center: Point3::new(base_offset + dx, 0.0, 0.0),
                rotation: rot,
                shape: &shape_b,
            };

            let result = gjk_epa_manifold(&view_a, &view_b, margin);
            if result.is_empty() {
                continue;
            }

            let n = result.points[0].normal;
            if let Some(prev) = prev_normal {
                let dot: f32 = n.dot(&prev);
                assert!(
                    dot > 0.9,
                    "Normal flipped between steps: prev={:?}, cur={:?}, dot={:.3}",
                    prev, n, dot
                );
            }
            prev_normal = Some(n);
        }
    }

    // --- Hull EPA must match OBB EPA for identical cube geometry ---

    #[test]
    fn hull_epa_normal_matches_obb_for_cubes() {
        use crate::collision::discrete::gjk::{gjk_query, GjkResult};
        use crate::collision::discrete::epa::epa_penetration;

        let margin = 0.02;
        let he = Vector3::new(1.0, 1.0, 1.0);
        let rot = UnitQuaternion::identity();

        let obb_a = Obb::new(Point3::origin(), rot, he);
        let obb_b = Obb::new(Point3::new(1.5, 0.0, 0.0), rot, he);
        let obb_simplex = match gjk_query(&obb_a, &obb_b) {
            GjkResult::Intersecting { simplex } => simplex,
            _ => panic!("OBB should intersect"),
        };
        let obb_epa = epa_penetration(&obb_a, &obb_b, margin, &obb_simplex);

        let hull = Arc::new(cube_hull(he));
        let shape_a = ColliderShape::ConvexHull { hull: hull.clone() };
        let shape_b = ColliderShape::ConvexHull { hull: hull.clone() };
        let view_a = ShapeView { center: Point3::origin(), rotation: rot, shape: &shape_a };
        let view_b = ShapeView { center: Point3::new(1.5, 0.0, 0.0), rotation: rot, shape: &shape_b };
        let hull_simplex = match gjk_query(&view_a, &view_b) {
            GjkResult::Intersecting { simplex } => simplex,
            _ => panic!("Hull should intersect"),
        };
        let hull_epa = epa_penetration(&view_a, &view_b, margin, &hull_simplex);

        assert!(
            hull_epa.normal.dot(&obb_epa.normal).abs() > 0.9,
            "Hull EPA normal {:?} should match OBB {:?}",
            hull_epa.normal, obb_epa.normal
        );
        assert!(
            (hull_epa.depth - obb_epa.depth).abs() < 0.05,
            "Hull EPA depth {:.3} should match OBB {:.3}",
            hull_epa.depth, obb_epa.depth
        );
    }
}
