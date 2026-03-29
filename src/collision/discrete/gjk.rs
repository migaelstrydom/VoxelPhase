//! Gilbert-Johnson-Keerthi (GJK) distance/intersection algorithm.
//!
//! Operates purely on `ConvexSupport` trait objects. Given two convex shapes,
//! determines whether they intersect or computes the closest distance and
//! witness points.

use nalgebra::{Point3, Vector3};

use crate::collision::support::ConvexSupport;

/// Maximum GJK iterations before bailing out.
const MAX_ITERATIONS: u32 = 32;

/// Convergence tolerance: if a new support point doesn't make progress beyond
/// this threshold, we declare separation.
const TOLERANCE: f32 = 1e-6;

/// Tolerance for degenerate simplex detection (near-zero magnitude).
const DEGENERATE_TOLERANCE: f32 = 1e-10;

/// Result of a GJK query between two convex shapes.
#[derive(Debug)]
pub enum GjkResult {
    /// The shapes are separated. `distance` is the minimum distance between
    /// their surfaces, and `closest_a`/`closest_b` are the witness points.
    Separated {
        distance: f32,
        closest_a: Point3<f32>,
        closest_b: Point3<f32>,
    },
    /// The shapes overlap. The `simplex` is a tetrahedron (4 vertices) on
    /// the Minkowski difference that encloses the origin.
    Intersecting { simplex: GjkSimplex },
}

/// Persistent warm-start cache for GJK direction seeding.
#[derive(Debug, Clone, Copy, Default)]
pub struct GjkCache {
    /// Last successful search direction used to seed the next query.
    pub last_direction: Option<Vector3<f32>>,
}

/// A vertex on the Minkowski difference A ⊖ B, tracking the original support
/// points on each shape for witness point reconstruction.
#[derive(Debug, Clone, Copy)]
pub struct MinkowskiVertex {
    /// Point on the Minkowski difference: `support_a - support_b`.
    pub point: Point3<f32>,
    /// Support point on shape A.
    pub support_a: Point3<f32>,
    /// Support point on shape B.
    pub support_b: Point3<f32>,
}

/// GJK simplex: 1 to 4 vertices on the Minkowski difference.
#[derive(Debug, Clone)]
pub struct GjkSimplex {
    pub vertices: [MinkowskiVertex; 4],
    pub count: usize,
}

impl GjkSimplex {
    fn new() -> Self {
        Self {
            vertices: [MinkowskiVertex {
                point: Point3::origin(),
                support_a: Point3::origin(),
                support_b: Point3::origin(),
            }; 4],
            count: 0,
        }
    }

    fn push(&mut self, v: MinkowskiVertex) {
        debug_assert!(self.count < 4);
        self.vertices[self.count] = v;
        self.count += 1;
    }
}

/// Compute a support point on the Minkowski difference A ⊖ B.
fn minkowski_support(
    a: &dyn ConvexSupport,
    b: &dyn ConvexSupport,
    direction: Vector3<f32>,
) -> MinkowskiVertex {
    let sa = a.support(direction);
    let sb = b.support(-direction);
    MinkowskiVertex {
        point: Point3::from(sa.coords - sb.coords),
        support_a: sa,
        support_b: sb,
    }
}

/// Run the GJK algorithm to determine intersection or minimum distance
/// between two convex shapes.
///
/// Uses the distance-aware variant: tracks the closest point on the simplex
/// to the origin, using it both as the search direction and for convergence
/// testing. Returns `Separated` with witness points when the shapes don't
/// overlap, or `Intersecting` with the enclosing simplex for EPA.
pub fn gjk_query(a: &dyn ConvexSupport, b: &dyn ConvexSupport) -> GjkResult {
    gjk_query_seeded(a, b, None)
}

/// Run GJK with an optional warm-start direction.
///
/// When `initial_direction` is provided and non-degenerate, it is used as the
/// first support direction. Otherwise, GJK falls back to the center-biased
/// heuristic from shape supports at zero direction.
pub fn gjk_query_seeded(
    a: &dyn ConvexSupport,
    b: &dyn ConvexSupport,
    initial_direction: Option<Vector3<f32>>,
) -> GjkResult {
    // Initial search direction: arbitrary, but biased toward center-to-center.
    let center_biased = a.support(Vector3::zeros()).coords - b.support(Vector3::zeros()).coords;
    let direction = if let Some(seed) = initial_direction {
        if seed.magnitude_squared() > DEGENERATE_TOLERANCE {
            seed
        } else if center_biased.magnitude_squared() > DEGENERATE_TOLERANCE {
            center_biased
        } else {
            Vector3::x()
        }
    } else if center_biased.magnitude_squared() > DEGENERATE_TOLERANCE {
        center_biased
    } else {
        Vector3::x()
    };

    let mut simplex = GjkSimplex::new();

    // First support point.
    let v0 = minkowski_support(a, b, direction);
    simplex.push(v0);

    // `v` is the closest point on the current simplex to the origin.
    let mut v = v0.point.coords;

    if v.magnitude_squared() < DEGENERATE_TOLERANCE {
        return GjkResult::Intersecting { simplex };
    }

    for _ in 0..MAX_ITERATIONS {
        // Search direction: from closest point toward origin.
        let d = -v;

        let w = minkowski_support(a, b, d);

        // Convergence check: the new support point can't improve the closest
        // distance by more than `v·v - v·w`. If that's below tolerance, we've
        // found the minimum distance.
        let vv = v.magnitude_squared();
        let vw = v.dot(&w.point.coords);
        if vv - vw <= TOLERANCE * vv.max(1.0) {
            let (closest_a, closest_b, distance) = extract_closest_points(&simplex);
            return GjkResult::Separated {
                distance,
                closest_a,
                closest_b,
            };
        }

        simplex.push(w);

        // Process the simplex: find the closest feature to the origin,
        // reduce the simplex to that feature, and update `v`.
        match simplex.count {
            2 => process_line_simplex(&mut simplex, &mut v),
            3 => process_triangle_simplex(&mut simplex, &mut v),
            4 => {
                if process_tetrahedron_simplex(&mut simplex, &mut v) {
                    return GjkResult::Intersecting { simplex };
                }
            }
            _ => unreachable!(),
        }

        if v.magnitude_squared() < DEGENERATE_TOLERANCE {
            return GjkResult::Intersecting { simplex };
        }
    }

    // Max iterations: if we have a tetrahedron, assume intersection.
    if simplex.count == 4 {
        GjkResult::Intersecting { simplex }
    } else {
        let (closest_a, closest_b, distance) = extract_closest_points(&simplex);
        GjkResult::Separated {
            distance,
            closest_a,
            closest_b,
        }
    }
}

/// Process a 2-vertex simplex (line segment). Finds the closest point on the
/// segment to the origin, reduces the simplex if needed, and updates `v`.
fn process_line_simplex(simplex: &mut GjkSimplex, v: &mut Vector3<f32>) {
    let a = simplex.vertices[1].point; // newest
    let b = simplex.vertices[0].point;

    let ab = b - a;
    let ao = -a.coords;

    let t = ao.dot(&ab);
    if t <= 0.0 {
        // Closest to vertex A.
        simplex.vertices[0] = simplex.vertices[1];
        simplex.count = 1;
        *v = a.coords;
    } else {
        let ab_sq = ab.magnitude_squared();
        if t >= ab_sq {
            // Closest to vertex B.
            simplex.count = 1;
            *v = b.coords;
        } else {
            // Closest to interior of segment.
            let frac = t / ab_sq;
            *v = a.coords + ab * frac;
        }
    }
}

/// Process a 3-vertex simplex (triangle). Finds the closest point on the
/// triangle to the origin, reduces the simplex, and updates `v`.
///
/// Uses the Voronoi region method from Christer Ericson's "Real-Time Collision
/// Detection" (Section 5.1.5). Vertices are stored as [oldest..newest] with
/// A=vertices[2] being the most recently added.
fn process_triangle_simplex(simplex: &mut GjkSimplex, v: &mut Vector3<f32>) {
    // Copy vertices to locals so we can rearrange the simplex without aliasing.
    let va = simplex.vertices[2]; // A (newest)
    let vb = simplex.vertices[1]; // B
    let vc = simplex.vertices[0]; // C

    let a = va.point.coords;
    let b = vb.point.coords;
    let c = vc.point.coords;

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let d1 = ab.dot(&ao);
    let d2 = ac.dot(&ao);

    // Vertex A region.
    if d1 <= 0.0 && d2 <= 0.0 {
        simplex.vertices[0] = va;
        simplex.count = 1;
        *v = a;
        return;
    }

    let bo = -b;
    let d3 = ab.dot(&bo);
    let d4 = ac.dot(&bo);

    // Edge AB region.
    let vc_bary = d1 * d4 - d3 * d2;
    if vc_bary <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        simplex.vertices[0] = vb;
        simplex.vertices[1] = va;
        simplex.count = 2;
        *v = a + ab * t;
        return;
    }

    let co = -c;
    let d5 = ab.dot(&co);
    let d6 = ac.dot(&co);

    // Edge AC region.
    let vb_bary = d5 * d2 - d1 * d6;
    if vb_bary <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        simplex.vertices[0] = vc;
        simplex.vertices[1] = va;
        simplex.count = 2;
        *v = a + ac * t;
        return;
    }

    // Vertex B region.
    if d3 >= 0.0 && d4 <= d3 {
        simplex.vertices[0] = vb;
        simplex.count = 1;
        *v = b;
        return;
    }

    // Vertex C region.
    if d6 >= 0.0 && d5 <= d6 {
        simplex.vertices[0] = vc;
        simplex.count = 1;
        *v = c;
        return;
    }

    // Edge BC region.
    let va_bary = d3 * d6 - d5 * d4;
    if va_bary <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        simplex.vertices[0] = vb;
        simplex.vertices[1] = vc;
        simplex.count = 2;
        *v = b + (c - b) * t;
        return;
    }

    // Interior of triangle — keep all three vertices.
    let denom = va_bary + vb_bary + vc_bary;
    if denom.abs() < DEGENERATE_TOLERANCE {
        *v = a;
        return;
    }
    let u = va_bary / denom;
    let frac_v = vb_bary / denom;
    let w = 1.0 - u - frac_v;
    *v = a * w + b * u + c * frac_v;
}

/// Process a 4-vertex simplex (tetrahedron). Returns true if the origin is
/// enclosed. Otherwise reduces to the closest feature and updates `v`.
fn process_tetrahedron_simplex(simplex: &mut GjkSimplex, v: &mut Vector3<f32>) -> bool {
    let a = simplex.vertices[3].point; // newest
    let b = simplex.vertices[2].point;
    let c = simplex.vertices[1].point;
    let d = simplex.vertices[0].point;

    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let ao = -a.coords;

    // Compute face normals. We need them oriented outward (away from the
    // fourth vertex).
    let mut abc = ab.cross(&ac);
    if abc.dot(&ad) > 0.0 {
        abc = -abc;
    }
    let mut acd = ac.cross(&ad);
    if acd.dot(&ab) > 0.0 {
        acd = -acd;
    }
    let mut adb = ad.cross(&ab);
    if adb.dot(&ac) > 0.0 {
        adb = -adb;
    }

    let in_front_abc = abc.dot(&ao) > 0.0;
    let in_front_acd = acd.dot(&ao) > 0.0;
    let in_front_adb = adb.dot(&ao) > 0.0;

    if !in_front_abc && !in_front_acd && !in_front_adb {
        return true;
    }

    // Find the closest face and reduce to that triangle.
    // We test each visible face and pick the closest.
    let mut best_dist_sq = f32::MAX;
    let mut best_v = *v;
    let mut best_simplex = simplex.clone();

    if in_front_abc {
        let mut s = GjkSimplex::new();
        s.vertices[0] = simplex.vertices[0 /* placeholder */];
        // Rearrange: face ABC → [C, B, A] so newest is at index 2.
        s.vertices[0] = simplex.vertices[1]; // C
        s.vertices[1] = simplex.vertices[2]; // B
        s.vertices[2] = simplex.vertices[3]; // A
        s.count = 3;
        let mut sv = Vector3::zeros();
        process_triangle_simplex(&mut s, &mut sv);
        let dsq = sv.magnitude_squared();
        if dsq < best_dist_sq {
            best_dist_sq = dsq;
            best_v = sv;
            best_simplex = s;
        }
    }

    if in_front_acd {
        let mut s = GjkSimplex::new();
        s.vertices[0] = simplex.vertices[0]; // D
        s.vertices[1] = simplex.vertices[1]; // C
        s.vertices[2] = simplex.vertices[3]; // A
        s.count = 3;
        let mut sv = Vector3::zeros();
        process_triangle_simplex(&mut s, &mut sv);
        let dsq = sv.magnitude_squared();
        if dsq < best_dist_sq {
            best_dist_sq = dsq;
            best_v = sv;
            best_simplex = s;
        }
    }

    if in_front_adb {
        let mut s = GjkSimplex::new();
        s.vertices[0] = simplex.vertices[2]; // B
        s.vertices[1] = simplex.vertices[0]; // D
        s.vertices[2] = simplex.vertices[3]; // A
        s.count = 3;
        let mut sv = Vector3::zeros();
        process_triangle_simplex(&mut s, &mut sv);
        let dsq = sv.magnitude_squared();
        if dsq < best_dist_sq {
            best_v = sv;
            best_simplex = s;
        }
    }

    *simplex = best_simplex;
    *v = best_v;
    false
}

/// Extract closest points on shapes A and B from the current simplex.
/// Returns (closest_a, closest_b, distance).
fn extract_closest_points(simplex: &GjkSimplex) -> (Point3<f32>, Point3<f32>, f32) {
    match simplex.count {
        1 => {
            let v = &simplex.vertices[0];
            (v.support_a, v.support_b, v.point.coords.magnitude())
        }
        2 => closest_on_line_segment(
            &simplex.vertices[0],
            &simplex.vertices[1],
        ),
        3 => closest_on_triangle(
            &simplex.vertices[0],
            &simplex.vertices[1],
            &simplex.vertices[2],
        ),
        _ => {
            // Shouldn't happen for separated case, but handle gracefully.
            let v = &simplex.vertices[0];
            (v.support_a, v.support_b, v.point.coords.magnitude())
        }
    }
}

/// Closest point to the origin on a line segment, with barycentric interpolation
/// of the witness points.
fn closest_on_line_segment(
    v0: &MinkowskiVertex,
    v1: &MinkowskiVertex,
) -> (Point3<f32>, Point3<f32>, f32) {
    let ab = v1.point - v0.point;
    let ao = Point3::origin() - v0.point;
    let ab_sq = ab.magnitude_squared();

    if ab_sq < DEGENERATE_TOLERANCE {
        return (v0.support_a, v0.support_b, v0.point.coords.magnitude());
    }

    let t = (ao.dot(&ab) / ab_sq).clamp(0.0, 1.0);
    let closest_a = Point3::from(v0.support_a.coords.lerp(&v1.support_a.coords, t));
    let closest_b = Point3::from(v0.support_b.coords.lerp(&v1.support_b.coords, t));
    let closest_mink = Point3::from(v0.point.coords.lerp(&v1.point.coords, t));
    let distance = closest_mink.coords.magnitude();

    (closest_a, closest_b, distance)
}

/// Closest point to the origin on a triangle, with barycentric interpolation.
fn closest_on_triangle(
    v0: &MinkowskiVertex,
    v1: &MinkowskiVertex,
    v2: &MinkowskiVertex,
) -> (Point3<f32>, Point3<f32>, f32) {
    let a = v0.point.coords;
    let b = v1.point.coords;
    let c = v2.point.coords;

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let d1 = ab.dot(&ao);
    let d2 = ac.dot(&ao);
    // Vertex region A.
    if d1 <= 0.0 && d2 <= 0.0 {
        return (v0.support_a, v0.support_b, a.magnitude());
    }

    // Vertex region B.
    let bo = -b;
    let d3_ = ab.dot(&bo);
    let d4_ = ac.dot(&bo);
    if d3_ >= 0.0 && d4_ <= d3_ {
        return (v1.support_a, v1.support_b, b.magnitude());
    }

    // Edge region AB.
    let vc = d1 * d4_ - d3_ * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3_ <= 0.0 {
        let t = d1 / (d1 - d3_);
        let closest_a = Point3::from(v0.support_a.coords.lerp(&v1.support_a.coords, t));
        let closest_b = Point3::from(v0.support_b.coords.lerp(&v1.support_b.coords, t));
        let closest_mink = a + ab * t;
        return (closest_a, closest_b, closest_mink.magnitude());
    }

    // Vertex region C.
    let co = -c;
    let d5_ = ab.dot(&co);
    let d6_ = ac.dot(&co);
    if d6_ >= 0.0 && d5_ <= d6_ {
        return (v2.support_a, v2.support_b, c.magnitude());
    }

    // Edge region AC.
    let vb = d5_ * d2 - d1 * d6_;
    if vb <= 0.0 && d2 >= 0.0 && d6_ <= 0.0 {
        let t = d2 / (d2 - d6_);
        let closest_a = Point3::from(v0.support_a.coords.lerp(&v2.support_a.coords, t));
        let closest_b = Point3::from(v0.support_b.coords.lerp(&v2.support_b.coords, t));
        let closest_mink = a + ac * t;
        return (closest_a, closest_b, closest_mink.magnitude());
    }

    // Edge region BC.
    let va = d3_ * d6_ - d5_ * d4_;
    if va <= 0.0 {
        let bc = c - b;
        let bo_dot_bc = bo.dot(&bc);
        let bc_sq = bc.magnitude_squared();
        if bo_dot_bc >= 0.0 && bo_dot_bc <= bc_sq && bc_sq > DEGENERATE_TOLERANCE {
            let t = bo_dot_bc / bc_sq;
            let closest_a = Point3::from(v1.support_a.coords.lerp(&v2.support_a.coords, t));
            let closest_b_pt = Point3::from(v1.support_b.coords.lerp(&v2.support_b.coords, t));
            let closest_mink = b + bc * t;
            return (closest_a, closest_b_pt, closest_mink.magnitude());
        }
    }

    // Interior of triangle.
    let denom = va + vb + vc;
    if denom.abs() < DEGENERATE_TOLERANCE {
        return (v0.support_a, v0.support_b, a.magnitude());
    }
    let u = va / denom;
    let v = vb / denom;
    let w = 1.0 - u - v;

    let closest_a = Point3::from(v0.support_a.coords * w + v1.support_a.coords * u + v2.support_a.coords * v);
    let closest_b = Point3::from(v0.support_b.coords * w + v1.support_b.coords * u + v2.support_b.coords * v);
    let closest_mink = a * w + b * u + c * v;
    (closest_a, closest_b, closest_mink.magnitude())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::obb::Obb;
    use crate::collision::support::SupportSphere;
    use nalgebra::UnitQuaternion;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    // --- Separated spheres ---

    #[test]
    fn separated_spheres() {
        let a = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(5.0, 0.0, 0.0),
            radius: 1.0,
        };

        match gjk_query(&a, &b) {
            GjkResult::Separated {
                distance,
                closest_a,
                closest_b,
            } => {
                // Distance between surfaces: 5 - 1 - 1 = 3
                assert!(
                    approx_eq(distance, 3.0, 0.05),
                    "Expected distance ~3.0, got {}",
                    distance
                );
                // Closest point on A should be near (1, 0, 0).
                assert!(
                    (closest_a - Point3::new(1.0, 0.0, 0.0)).magnitude() < 0.1,
                    "Closest A: {:?}",
                    closest_a
                );
                // Closest point on B should be near (4, 0, 0).
                assert!(
                    (closest_b - Point3::new(4.0, 0.0, 0.0)).magnitude() < 0.1,
                    "Closest B: {:?}",
                    closest_b
                );
            }
            GjkResult::Intersecting { .. } => panic!("Expected Separated, got Intersecting"),
        }
    }

    // --- Overlapping spheres ---

    #[test]
    fn overlapping_spheres() {
        let a = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(1.0, 0.0, 0.0),
            radius: 1.0,
        };

        match gjk_query(&a, &b) {
            GjkResult::Intersecting { simplex } => {
                assert!(simplex.count >= 1, "Simplex should have at least 1 vertex");
            }
            GjkResult::Separated { distance, .. } => {
                panic!("Expected Intersecting, got Separated with distance {}", distance);
            }
        }
    }

    // --- Separated OBBs ---

    #[test]
    fn separated_obbs() {
        let a = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let b = Obb::new(
            Point3::new(5.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );

        match gjk_query(&a, &b) {
            GjkResult::Separated { distance, .. } => {
                // Gap: 5 - 1 - 1 = 3
                assert!(
                    approx_eq(distance, 3.0, 0.05),
                    "Expected distance ~3.0, got {}",
                    distance
                );
            }
            GjkResult::Intersecting { .. } => panic!("Expected Separated, got Intersecting"),
        }
    }

    // --- Overlapping OBBs ---

    #[test]
    fn overlapping_obbs() {
        let a = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let b = Obb::new(
            Point3::new(1.5, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );

        match gjk_query(&a, &b) {
            GjkResult::Intersecting { .. } => {}
            GjkResult::Separated { distance, .. } => {
                panic!("Expected Intersecting, got Separated with distance {}", distance);
            }
        }
    }

    // --- Sphere vs OBB cross-validation ---

    #[test]
    fn sphere_vs_obb_separated() {
        let sphere = SupportSphere {
            center: Point3::new(3.0, 0.0, 0.0),
            radius: 0.5,
        };
        let obb = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );

        match gjk_query(&sphere, &obb) {
            GjkResult::Separated { distance, .. } => {
                // Sphere surface at 2.5, OBB face at 1.0 → gap = 1.5
                assert!(
                    approx_eq(distance, 1.5, 0.05),
                    "Expected distance ~1.5, got {}",
                    distance
                );
            }
            GjkResult::Intersecting { .. } => panic!("Expected Separated"),
        }
    }

    // --- Capsule vs OBB ---

    #[test]
    fn capsule_vs_obb_separated() {
        use crate::collision::capsule::Capsule;

        let capsule = Capsule::new(
            Point3::new(5.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            1.0,
            0.3,
        );
        let obb = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );

        match gjk_query(&capsule, &obb) {
            GjkResult::Separated { distance, .. } => {
                // Capsule center at x=5, closest point on capsule surface at x=4.7
                // OBB face at x=1. Gap = 3.7
                assert!(
                    approx_eq(distance, 3.7, 0.1),
                    "Expected distance ~3.7, got {}",
                    distance
                );
            }
            GjkResult::Intersecting { .. } => panic!("Expected Separated"),
        }
    }

    // --- Degenerate: coincident shapes ---

    #[test]
    fn coincident_spheres() {
        let a = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };

        match gjk_query(&a, &b) {
            GjkResult::Intersecting { .. } => {}
            GjkResult::Separated { distance, .. } => {
                panic!("Expected Intersecting, got Separated with distance {}", distance);
            }
        }
    }

    // --- Degenerate: zero-radius sphere (point) vs OBB ---

    #[test]
    fn point_vs_obb() {
        let point = SupportSphere {
            center: Point3::new(3.0, 0.0, 0.0),
            radius: 0.0,
        };
        let obb = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );

        match gjk_query(&point, &obb) {
            GjkResult::Separated { distance, .. } => {
                // Point at (3,0,0), OBB face at x=1. Gap = 2.0
                assert!(
                    approx_eq(distance, 2.0, 0.05),
                    "Expected distance ~2.0, got {}",
                    distance
                );
            }
            GjkResult::Intersecting { .. } => panic!("Expected Separated"),
        }
    }

    // --- Iteration cap: long thin OBB vs distant point ---

    #[test]
    fn long_thin_obb_terminates() {
        let thin_obb = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(100.0, 0.01, 0.01),
        );
        let point = SupportSphere {
            center: Point3::new(200.0, 0.0, 0.0),
            radius: 0.0,
        };

        // Should terminate within 32 iterations and report separation.
        match gjk_query(&thin_obb, &point) {
            GjkResult::Separated { distance, .. } => {
                assert!(
                    approx_eq(distance, 100.0, 0.5),
                    "Expected distance ~100.0, got {}",
                    distance
                );
            }
            GjkResult::Intersecting { .. } => panic!("Expected Separated"),
        }
    }

    // --- Barely touching spheres (boundary case) ---

    #[test]
    fn barely_touching_spheres() {
        let a = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(2.0, 0.0, 0.0),
            radius: 1.0,
        };

        // Exactly touching — could report either way. Just verify it terminates
        // and gives a reasonable result.
        match gjk_query(&a, &b) {
            GjkResult::Separated { distance, .. } => {
                assert!(
                    distance < 0.1,
                    "Distance should be near zero, got {}",
                    distance
                );
            }
            GjkResult::Intersecting { .. } => {
                // Also acceptable for exactly touching shapes.
            }
        }
    }

    // --- Rotated OBB ---

    #[test]
    fn rotated_obb_separated() {
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::y()),
            std::f32::consts::FRAC_PI_4,
        );
        let a = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            rot,
            Vector3::new(1.0, 1.0, 1.0),
        );
        let b = Obb::new(
            Point3::new(5.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );

        match gjk_query(&a, &b) {
            GjkResult::Separated { distance, .. } => {
                // Rotated OBB extends further along X by sqrt(2) ≈ 1.414.
                // Gap ≈ 5 - 1.414 - 1.0 = 2.586
                assert!(
                    distance > 2.0 && distance < 3.5,
                    "Expected distance ~2.6, got {}",
                    distance
                );
            }
            GjkResult::Intersecting { .. } => panic!("Expected Separated"),
        }
    }
}
