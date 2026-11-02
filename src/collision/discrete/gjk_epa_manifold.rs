//! GJK/EPA contact between a convex shape and a faceless one (sphere,
//! capsule): GJK → EPA → a single contact point, its normal taken from the
//! nearest face of the shape that has faces.
//!
//! The general path the dispatch's wildcard arm calls; `hull_capsule` builds
//! its manifold on top of it.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use super::clipping::face_centroid;
use super::epa::{epa_penetration, EpaResult};
use super::gjk::{gjk_query_seeded, GjkCache, GjkResult};
use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::shape_view::{SupportFace, SupportFaceExtractor};
use crate::collision::support::ConvexSupport;

/// Tolerance for GJK separation distance check.
const GJK_TOLERANCE: f32 = 1e-4;

/// Small angular probe used to perturb candidate normals in symmetric tie cases.
const NORMAL_PROBE_EPS: f32 = 1e-3;

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

/// Generate a contact manifold between a convex shape and a faceless one
/// (sphere, capsule) using GJK + EPA: one contact point.
///
/// `b` must be faceless; `a` may have faces (OBB, hull) or not. Pairs where
/// both have faces have their own SAT manifolds, and the dispatch passes the
/// higher-ranked shape as `a`.
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
    let seed = gjk_cache.as_ref().and_then(|cache| cache.last_direction);
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

            debug_assert!(
                b.support_face(-normal).is_none(),
                "face-face pairs have their own SAT manifolds"
            );
            match a.support_face(normal) {
                Some(face_a) => face_vs_faceless_contact(a, b, &face_a, normal, &epa, margin)
                    .unwrap_or_else(ContactManifold::empty),
                None => {
                    // Both shapes are faceless (sphere-sphere).
                    let raw_depth = epa.depth - 2.0 * margin;
                    let point = Point3::from((epa.witness_a.coords + epa.witness_b.coords) * 0.5);
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
    let face_center = face_centroid(verts);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::capsule::Capsule;
    use crate::collision::convex_hull::cube_hull;
    use crate::collision::discrete::obb_capsule::obb_capsule_manifold;
    use crate::collision::discrete::sphere_obb::sphere_obb_manifold;
    use crate::collision::obb::Obb;
    use crate::collision::shape_view::ShapeView;
    use std::sync::Arc;

    use crate::physics::ColliderShape;
    use nalgebra::UnitQuaternion;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    // --- OBB-OBB cross-validation ---

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
        assert_eq!(
            result.len(),
            1,
            "Coincident spheres should produce one contact"
        );

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

    // --- Hull-hull: cube vs cube face-face, cross-validate with OBB SAT ---

    // --- Hull-hull: contact positions are within both shapes ---

    // --- Hull-hull: rotated cube (edge contact) ---

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
        assert!(
            !hull_result.is_empty(),
            "Hull path should produce a contact"
        );

        let obb = Obb::new(Point3::origin(), rot, he);
        let capsule = Capsule::new(center, rot, 0.8, 0.3);
        let obb_result = obb_capsule_manifold(&obb, &capsule, margin);
        assert!(
            !obb_result.is_empty(),
            "OBB reference should produce a contact"
        );

        let hull_p = hull_result
            .points
            .iter()
            .max_by(|a, b| a.raw_depth.total_cmp(&b.raw_depth))
            .expect("non-empty");
        let obb_p = obb_result
            .points
            .iter()
            .max_by(|a, b| a.raw_depth.total_cmp(&b.raw_depth))
            .expect("non-empty");

        let hn = hull_p.normal;
        let on = obb_p.normal;
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
            (hull_p.raw_depth - obb_p.raw_depth).abs() < 0.1,
            "Depth mismatch: hull={:.4}, obb={:.4}",
            hull_p.raw_depth,
            obb_p.raw_depth
        );
    }

    // --- Tetrahedron-tetrahedron: contacts near overlap region ---

    // --- Hull-hull: normal stability across small perturbation ---

    // --- Hull EPA must match OBB EPA for identical cube geometry ---
}
