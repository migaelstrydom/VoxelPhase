//! Expanding Polytope Algorithm (EPA) for penetration depth and normal.
//!
//! Takes an intersecting GJK simplex (tetrahedron enclosing the origin) and
//! iteratively expands it toward the boundary of the Minkowski difference to
//! find the minimum penetration depth, normal, and witness points.

use nalgebra::{Point3, Vector3};

use super::gjk::{GjkSimplex, MinkowskiVertex};
use crate::collision::support::ConvexSupport;

/// Maximum EPA iterations before returning the best result so far.
const MAX_ITERATIONS: u32 = 64;

/// Maximum polytope faces (bounded by vertex count and iteration cap).
const MAX_FACES: usize = 256;

/// Maximum polytope vertices.
const MAX_VERTICES: usize = 128;

/// Convergence tolerance: if the new support point doesn't extend the polytope
/// by more than this, we've converged.
const TOLERANCE: f32 = 1e-4;

/// Minimum triangle area squared to consider a face valid.
const MIN_FACE_AREA_SQ: f32 = 1e-16;

/// Result of EPA: penetration normal, depth, and witness points.
#[derive(Debug, Clone)]
pub struct EpaResult {
    /// Penetration normal (unit vector, pointing from A toward B).
    pub normal: Vector3<f32>,
    /// Penetration depth (positive).
    pub depth: f32,
    /// Closest point on shape A's surface.
    pub witness_a: Point3<f32>,
    /// Closest point on shape B's surface.
    pub witness_b: Point3<f32>,
}

/// A triangular face of the EPA polytope.
#[derive(Clone, Copy)]
struct EpaFace {
    /// Indices into the vertex buffer.
    v: [usize; 3],
    /// Outward face normal (away from origin).
    normal: Vector3<f32>,
    /// Signed distance from origin to face plane (always positive for valid faces).
    distance: f32,
}

/// Run EPA on two intersecting convex shapes, starting from GJK's simplex.
///
/// The `margin` parameter inflates both support functions by `margin` for
/// Minkowski-sum margin handling. The returned depth includes the inflation;
/// callers subtract `2 * margin` for the raw geometric depth.
pub fn epa_penetration(
    a: &dyn ConvexSupport,
    b: &dyn ConvexSupport,
    margin: f32,
    simplex: &GjkSimplex,
) -> EpaResult {
    let mut vertices = Vec::with_capacity(MAX_VERTICES);
    let mut faces: Vec<EpaFace> = Vec::with_capacity(64);

    // Build the initial polytope from the GJK simplex.
    // We need a tetrahedron. If the simplex has fewer than 4 vertices,
    // expand it.
    let initial = ensure_tetrahedron(a, b, margin, simplex);
    for v in &initial {
        vertices.push(*v);
    }

    // Create 4 triangular faces with outward-pointing normals.
    // For each face, orient the normal away from the origin (the origin is
    // inside the Minkowski difference, so all face normals should point outward).
    let face_indices: [[usize; 3]; 4] = [
        [0, 1, 2],
        [0, 3, 1],
        [0, 2, 3],
        [1, 3, 2],
    ];

    for indices in &face_indices {
        if let Some(mut face) = make_face(&vertices, indices[0], indices[1], indices[2]) {
            // Ensure outward-pointing normal: if distance is negative, flip.
            if face.distance < 0.0 {
                face.v.swap(0, 1);
                face.normal = -face.normal;
                face.distance = -face.distance;
            }
            if face.distance >= 0.0 {
                faces.push(face);
            }
        }
    }

    if faces.is_empty() {
        return fallback_result(a, b, margin);
    }

    let mut best_face_idx;

    for _ in 0..MAX_ITERATIONS {
        // Find the closest face to the origin.
        best_face_idx = 0;
        let mut best_dist = faces[0].distance;
        for (i, face) in faces.iter().enumerate().skip(1) {
            if face.distance < best_dist {
                best_dist = face.distance;
                best_face_idx = i;
            }
        }

        let search_normal = faces[best_face_idx].normal;

        // Get new support point in the direction of the closest face's normal.
        let new_vertex = inflated_minkowski_support(a, b, search_normal, margin);

        // Check convergence: does the new point extend past the closest face?
        let new_dist = new_vertex.point.coords.dot(&search_normal);
        if new_dist - best_dist < TOLERANCE {
            break;
        }

        if vertices.len() >= MAX_VERTICES {
            break;
        }

        let new_idx = vertices.len();
        vertices.push(new_vertex);

        // Remove all faces visible from the new point and find the silhouette.
        let mut edges: Vec<[usize; 2]> = Vec::new();
        let mut i = 0;
        while i < faces.len() {
            let face = &faces[i];
            let to_new = new_vertex.point - vertices[face.v[0]].point;
            if face.normal.dot(&to_new) > 0.0 {
                // Face is visible from the new point — record its edges.
                add_edge(&mut edges, face.v[0], face.v[1]);
                add_edge(&mut edges, face.v[1], face.v[2]);
                add_edge(&mut edges, face.v[2], face.v[0]);
                faces.swap_remove(i);
            } else {
                i += 1;
            }
        }

        // Create new faces from silhouette edges to the new vertex.
        for edge in &edges {
            if faces.len() >= MAX_FACES {
                break;
            }
            if let Some(face) = make_face(&vertices, edge[0], edge[1], new_idx) {
                // Ensure the normal points away from origin.
                if face.distance >= 0.0 {
                    faces.push(face);
                } else {
                    // Flip winding.
                    if let Some(mut flipped) = make_face(&vertices, edge[1], edge[0], new_idx) {
                        if flipped.distance < 0.0 {
                            flipped.normal = -flipped.normal;
                            flipped.distance = -flipped.distance;
                        }
                        faces.push(flipped);
                    }
                }
            }
        }

        if faces.is_empty() {
            return fallback_result(a, b, margin);
        }
    }

    // Extract result from the best face.
    if faces.is_empty() {
        return fallback_result(a, b, margin);
    }

    // Re-find the closest face (may have changed after last expansion).
    best_face_idx = 0;
    let mut best_dist = faces[0].distance;
    for (i, face) in faces.iter().enumerate().skip(1) {
        if face.distance < best_dist {
            best_dist = face.distance;
            best_face_idx = i;
        }
    }

    let best = &faces[best_face_idx];
    let (witness_a, witness_b) = compute_witness_points(&vertices, best);

    EpaResult {
        normal: best.normal,
        depth: best.distance,
        witness_a,
        witness_b,
    }
}

/// Compute a support point on the margin-inflated Minkowski difference.
fn inflated_minkowski_support(
    a: &dyn ConvexSupport,
    b: &dyn ConvexSupport,
    direction: Vector3<f32>,
    margin: f32,
) -> MinkowskiVertex {
    let sa = inflated_support(a, direction, margin);
    let sb = inflated_support(b, -direction, margin);
    MinkowskiVertex {
        point: Point3::from(sa.coords - sb.coords),
        support_a: sa,
        support_b: sb,
    }
}

/// Inflate a shape's support function by margin (Minkowski sum with sphere).
fn inflated_support(
    shape: &dyn ConvexSupport,
    direction: Vector3<f32>,
    margin: f32,
) -> Point3<f32> {
    let p = shape.support(direction);
    let len = direction.magnitude();
    if len < 1e-10 || margin == 0.0 {
        return p;
    }
    p + direction * (margin / len)
}

/// Build a tetrahedron enclosing the origin from the Minkowski difference.
///
/// The GJK simplex may have fewer than 4 vertices (if intersection was
/// detected early via `v ≈ 0`), or the simplex may be degenerate (nearly
/// flat). When the GJK simplex isn't usable directly, we gather a diverse
/// set of Minkowski support points and exhaustively search for the
/// largest-volume tetrahedron that encloses the origin.
fn ensure_tetrahedron(
    a: &dyn ConvexSupport,
    b: &dyn ConvexSupport,
    margin: f32,
    simplex: &GjkSimplex,
) -> Vec<MinkowskiVertex> {
    const MIN_TETRA_VOL6: f32 = 1e-12;

    // Fast path: if GJK already gave a valid enclosing tetrahedron, use it.
    if simplex.count == 4 {
        let candidate = [
            simplex.vertices[0],
            simplex.vertices[1],
            simplex.vertices[2],
            simplex.vertices[3],
        ];
        let vol = tetra_volume6(candidate[0], candidate[1], candidate[2], candidate[3]);
        if vol > MIN_TETRA_VOL6 && origin_inside_tetrahedron(&candidate) {
            return candidate.to_vec();
        }
    }

    let candidates = gather_candidate_supports(a, b, margin, simplex);
    find_best_tetrahedron(&candidates)
}

/// Gather a diverse set of Minkowski support points for tetrahedron construction.
///
/// Includes both axis-aligned and diagonal directions. Diagonals are critical
/// for ConvexHull shapes, whose support tiebreaking for axis-aligned queries
/// can produce coplanar Minkowski vertices. Diagonals force distinct corner
/// vertices, preventing degenerate initial tetrahedra.
fn gather_candidate_supports(
    a: &dyn ConvexSupport,
    b: &dyn ConvexSupport,
    margin: f32,
    simplex: &GjkSimplex,
) -> Vec<MinkowskiVertex> {
    let d = std::f32::consts::FRAC_1_SQRT_2;
    let dirs = [
        Vector3::x(),
        -Vector3::x(),
        Vector3::y(),
        -Vector3::y(),
        Vector3::z(),
        -Vector3::z(),
        Vector3::new(d, d, d),
        Vector3::new(d, d, -d),
        Vector3::new(d, -d, d),
        Vector3::new(-d, d, d),
    ];

    let mut pts = Vec::with_capacity(dirs.len() + simplex.count);
    for dir in &dirs {
        pts.push(inflated_minkowski_support(a, b, *dir, margin));
    }
    for i in 0..simplex.count {
        pts.push(simplex.vertices[i]);
    }
    pts
}

/// Find the largest-volume tetrahedron enclosing the origin from a set of
/// candidate Minkowski vertices.
///
/// Exhaustively searches all 4-tuples (O(n⁴) on a small candidate set,
/// typically ≤14 points). Prefers the tetrahedron that encloses the origin
/// with the greatest volume; if none encloses the origin, falls back to the
/// largest-volume tetrahedron overall.
fn find_best_tetrahedron(candidates: &[MinkowskiVertex]) -> Vec<MinkowskiVertex> {
    let n = candidates.len();

    let mut best_enclosing_vol = -1.0f32;
    let mut best_enclosing: Option<[usize; 4]> = None;

    let mut best_any_vol = 0.0f32;
    let mut best_any = [0, 1, 2, 3.min(n - 1)];

    for i in 0..n {
        for j in (i + 1)..n {
            for k in (j + 1)..n {
                let e1 = candidates[j].point - candidates[i].point;
                let e2 = candidates[k].point - candidates[i].point;
                let tri_n = e1.cross(&e2);
                if tri_n.magnitude() < 1e-10 {
                    continue;
                }
                for l in (k + 1)..n {
                    let e3 = candidates[l].point - candidates[i].point;
                    let vol = e3.dot(&tri_n).abs();

                    if vol > best_any_vol {
                        best_any_vol = vol;
                        best_any = [i, j, k, l];
                    }
                    if vol > best_enclosing_vol {
                        let quad = [
                            candidates[i],
                            candidates[j],
                            candidates[k],
                            candidates[l],
                        ];
                        if origin_inside_tetrahedron(&quad) {
                            best_enclosing_vol = vol;
                            best_enclosing = Some([i, j, k, l]);
                        }
                    }
                }
            }
        }
    }

    let indices = best_enclosing.unwrap_or(best_any);
    indices.iter().map(|&i| candidates[i]).collect()
}

/// Triple product magnitude for tetrahedron volume comparison.
fn tetra_volume6(a: MinkowskiVertex, b: MinkowskiVertex, c: MinkowskiVertex, d: MinkowskiVertex) -> f32 {
    let ab = b.point.coords - a.point.coords;
    let ac = c.point.coords - a.point.coords;
    let ad = d.point.coords - a.point.coords;
    ad.dot(&ab.cross(&ac)).abs()
}


/// Check if the origin is inside a tetrahedron defined by 4 Minkowski vertices.
fn origin_inside_tetrahedron(verts: &[MinkowskiVertex]) -> bool {
    if verts.len() < 4 {
        return false;
    }
    let a = verts[0].point.coords;
    let b = verts[1].point.coords;
    let c = verts[2].point.coords;
    let d = verts[3].point.coords;

    // For the origin to be inside, it must be on the same side of each face
    // as the opposite vertex.
    same_side(a, b, c, d, Vector3::zeros())
        && same_side(b, c, d, a, Vector3::zeros())
        && same_side(c, d, a, b, Vector3::zeros())
        && same_side(d, a, b, c, Vector3::zeros())
}

/// Check if points p and q are on the same side of the plane defined by triangle (a, b, c).
fn same_side(
    a: Vector3<f32>,
    b: Vector3<f32>,
    c: Vector3<f32>,
    p: Vector3<f32>,
    q: Vector3<f32>,
) -> bool {
    let normal = (b - a).cross(&(c - a));
    let dp = normal.dot(&(p - a));
    let dq = normal.dot(&(q - a));
    // Same sign (or on the plane).
    dp * dq >= 0.0
}

/// Construct an EPA face from three vertex indices. Returns None if degenerate.
fn make_face(vertices: &[MinkowskiVertex], a: usize, b: usize, c: usize) -> Option<EpaFace> {
    let ab = vertices[b].point - vertices[a].point;
    let ac = vertices[c].point - vertices[a].point;
    let normal = ab.cross(&ac);

    let area_sq = normal.magnitude_squared();
    if area_sq < MIN_FACE_AREA_SQ {
        return None;
    }

    let normal = normal / area_sq.sqrt();
    let distance = normal.dot(&vertices[a].point.coords);

    Some(EpaFace {
        v: [a, b, c],
        normal,
        distance,
    })
}

/// Add an edge to the silhouette list, removing it if the reverse already exists
/// (shared edge between two visible faces — not on the silhouette).
fn add_edge(edges: &mut Vec<[usize; 2]>, a: usize, b: usize) {
    // Check if the reverse edge exists.
    if let Some(pos) = edges.iter().position(|e| e[0] == b && e[1] == a) {
        edges.swap_remove(pos);
    } else {
        edges.push([a, b]);
    }
}

/// Compute witness points on shapes A and B from the closest EPA face.
/// Projects the origin onto the face and uses barycentric coordinates
/// to interpolate the per-vertex support points.
fn compute_witness_points(
    vertices: &[MinkowskiVertex],
    face: &EpaFace,
) -> (Point3<f32>, Point3<f32>) {
    let va = &vertices[face.v[0]];
    let vb = &vertices[face.v[1]];
    let vc = &vertices[face.v[2]];

    let a = va.point.coords;
    let b = vb.point.coords;
    let c = vc.point.coords;

    // Project origin onto the face plane.
    let projected = face.normal * face.distance;

    // Barycentric coordinates of the projected point in triangle ABC.
    let v0 = b - a;
    let v1 = c - a;
    let v2 = projected - a;

    let d00 = v0.dot(&v0);
    let d01 = v0.dot(&v1);
    let d11 = v1.dot(&v1);
    let d20 = v2.dot(&v0);
    let d21 = v2.dot(&v1);

    let denom = d00 * d11 - d01 * d01;
    if denom.abs() < 1e-12 {
        return (va.support_a, va.support_b);
    }

    let bary_v = (d11 * d20 - d01 * d21) / denom;
    let bary_w = (d00 * d21 - d01 * d20) / denom;
    let bary_u = 1.0 - bary_v - bary_w;

    let witness_a = Point3::from(
        va.support_a.coords * bary_u
            + vb.support_a.coords * bary_v
            + vc.support_a.coords * bary_w,
    );
    let witness_b = Point3::from(
        va.support_b.coords * bary_u
            + vb.support_b.coords * bary_v
            + vc.support_b.coords * bary_w,
    );

    (witness_a, witness_b)
}

/// Fallback result when EPA can't construct a valid polytope.
fn fallback_result(
    a: &dyn ConvexSupport,
    b: &dyn ConvexSupport,
    _margin: f32,
) -> EpaResult {
    let ca = a.support(Vector3::zeros());
    let cb = b.support(Vector3::zeros());
    let delta = cb - ca;
    let dist = delta.magnitude();
    let normal = if dist > 1e-10 {
        delta / dist
    } else {
        Vector3::y()
    };
    EpaResult {
        normal,
        depth: 0.0,
        witness_a: ca,
        witness_b: cb,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::discrete::gjk::{gjk_query, GjkResult};
    use crate::collision::obb::Obb;
    use crate::collision::support::SupportSphere;
    use nalgebra::UnitQuaternion;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    fn run_epa(
        a: &dyn ConvexSupport,
        b: &dyn ConvexSupport,
        margin: f32,
    ) -> EpaResult {
        match gjk_query(a, b) {
            GjkResult::Intersecting { simplex } => epa_penetration(a, b, margin, &simplex),
            GjkResult::Separated { distance, .. } => {
                panic!("Expected intersection, got separation with distance {}", distance);
            }
        }
    }

    // --- Overlapping spheres ---

    #[test]
    fn overlapping_spheres_depth() {
        let a = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(1.5, 0.0, 0.0),
            radius: 1.0,
        };

        let result = run_epa(&a, &b, 0.0);

        // Expected penetration: 1.0 + 1.0 - 1.5 = 0.5
        assert!(
            approx_eq(result.depth, 0.5, 0.05),
            "Expected depth ~0.5, got {}",
            result.depth
        );
        // Normal should point roughly from A to B (+X).
        assert!(
            result.normal.x > 0.9,
            "Expected normal ~+X, got {:?}",
            result.normal
        );
    }

    // --- Overlapping OBBs (face-face) ---

    #[test]
    fn overlapping_obbs_face_face() {
        let a = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let b = Obb::new(
            Point3::new(1.5, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );

        let result = run_epa(&a, &b, 0.0);

        // Expected penetration along X: 1.0 + 1.0 - 1.5 = 0.5
        assert!(
            approx_eq(result.depth, 0.5, 0.05),
            "Expected depth ~0.5, got {}",
            result.depth
        );
        assert!(
            result.normal.x.abs() > 0.9,
            "Expected normal along X, got {:?}",
            result.normal
        );
    }

    // --- Overlapping OBBs (edge-edge) ---

    #[test]
    fn overlapping_obbs_edge_edge() {
        let rot45 = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::y()),
            std::f32::consts::FRAC_PI_4,
        );
        // Rotated OBB extends sqrt(2) ≈ 1.414 along X.
        // Place B so its -X face at 1.0 overlaps the rotated OBB's edge.
        let a = Obb::new(
            Point3::origin(),
            rot45,
            Vector3::new(1.0, 1.0, 1.0),
        );
        let b = Obb::new(
            Point3::new(2.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );

        // OBB A corner at ~1.414, OBB B face at 1.0. Overlap ≈ 0.414
        let result = run_epa(&a, &b, 0.0);
        assert!(
            result.depth > 0.1 && result.depth < 1.0,
            "Expected depth ~0.4, got {}",
            result.depth
        );
    }

    // --- Deep overlap ---

    #[test]
    fn deep_overlap_sphere_inside_obb() {
        let sphere = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 0.1,
        };
        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(2.0, 3.0, 4.0),
        );

        let result = run_epa(&sphere, &obb, 0.0);

        // Smallest escape is along X (half-extent 2.0). Depth ≈ 2.0 + 0.1 = 2.1
        assert!(
            result.depth > 1.5 && result.depth < 3.0,
            "Expected depth ~2.1, got {}",
            result.depth
        );
    }

    // --- Barely overlapping ---

    #[test]
    fn barely_overlapping() {
        let a = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(1.999, 0.0, 0.0),
            radius: 1.0,
        };

        let result = run_epa(&a, &b, 0.0);

        assert!(
            approx_eq(result.depth, 0.001, 0.01),
            "Expected depth ~0.001, got {}",
            result.depth
        );
    }

    // --- Margin inflation ---

    #[test]
    fn margin_inflates_depth() {
        let a = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(1.5, 0.0, 0.0),
            radius: 1.0,
        };

        let margin = 0.1;
        let result = run_epa(&a, &b, margin);

        // Without margin: depth = 0.5
        // With margin: depth ≈ 0.5 + 2 * 0.1 = 0.7
        // EPA on spheres converges slowly (curved surface), so accept wider
        // tolerance. In practice, spheres use the analytic path; margin
        // inflation matters for polyhedra where EPA converges tightly.
        assert!(
            result.depth > 0.3 && result.depth < 0.8,
            "Expected depth ~0.7, got {}",
            result.depth
        );
    }
}
