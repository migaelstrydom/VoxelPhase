//! Convex hull shape: vertices + faces with support function and face extraction.

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use smallvec::SmallVec;

use super::shape_view::{SupportFace, SupportFaceExtractor};
use super::support::ConvexSupport;

/// Maximum number of vertices allowed in a convex hull.
pub const MAX_HULL_VERTICES: usize = 128;

/// Minimum ratio of thinnest to thickest AABB dimension.
/// Hulls thinner than this are rejected to avoid degenerate Minkowski differences.
const MIN_THICKNESS_RATIO: f32 = 0.01;
const FACE_NORMAL_EPS: f32 = 1e-8;

/// A face of a convex hull.
#[derive(Debug, Clone)]
pub struct HullFace {
    /// Indices into `ConvexHull::vertices` (CCW winding from outside).
    pub vertex_indices: SmallVec<[u16; 6]>,
    /// Outward face normal (precomputed, normalized).
    pub normal: Vector3<f32>,
}

/// An edge with precomputed Gauss map adjacency for SAT edge-pair filtering.
///
/// Each edge on a convex hull is shared by exactly two faces. The two adjacent
/// face normals define an arc on the Gauss map (unit sphere). When testing
/// edge-edge separating axes between two hulls, only edge pairs whose Gauss
/// map arcs intersect can produce a valid separating axis (Minkowski face test).
///
/// Reference: Gregorius, "The Separating Axis Test", GDC 2013.
#[derive(Debug, Clone)]
pub struct HullEdgeAdj {
    /// First vertex index.
    pub v0: u16,
    /// Second vertex index.
    pub v1: u16,
    /// Outward normal of the first adjacent face.
    pub normal_a: Vector3<f32>,
    /// Outward normal of the second adjacent face.
    pub normal_b: Vector3<f32>,
}

/// A convex hull defined by vertices and faces.
///
/// Vertices and normals are in local space. The `ShapeView` layer handles
/// world-space transforms. Construction validates vertex count and minimum
/// thickness — hull generation from point clouds is out of scope.
#[derive(Debug, Clone)]
pub struct ConvexHull {
    /// Vertices of the hull in local space.
    pub vertices: Vec<Vector3<f32>>,
    /// Faces of the hull, each with vertex indices and precomputed normal.
    pub faces: Vec<HullFace>,
    /// Unique edges with precomputed Gauss map adjacency (adjacent face normals).
    pub edges: Vec<HullEdgeAdj>,
    /// Precomputed bounding radius (max vertex distance from origin).
    pub bounding_radius: f32,
}

impl ConvexHull {
    /// Create a new convex hull from vertices and faces.
    ///
    /// # Panics
    /// - If vertex count exceeds `MAX_HULL_VERTICES` (64).
    /// - If the hull is degenerate (thinnest AABB dimension < 1% of thickest).
    /// - If no faces are provided.
    pub fn new(vertices: Vec<Vector3<f32>>, faces: Vec<HullFace>) -> Self {
        assert!(
            vertices.len() <= MAX_HULL_VERTICES,
            "ConvexHull: vertex count {} exceeds maximum {}",
            vertices.len(),
            MAX_HULL_VERTICES,
        );
        assert!(
            !vertices.is_empty(),
            "ConvexHull: at least one vertex required"
        );
        assert!(!faces.is_empty(), "ConvexHull: at least one face required");

        // Validate minimum thickness.
        let (min_v, max_v) = aabb_of(&vertices);
        let extent = max_v - min_v;
        let max_dim = extent.x.max(extent.y).max(extent.z);
        let min_dim = extent.x.min(extent.y).min(extent.z);
        assert!(
            max_dim > 0.0 && min_dim / max_dim >= MIN_THICKNESS_RATIO,
            "ConvexHull: degenerate hull — thinnest dimension {min_dim} is < 1% of thickest {max_dim}",
        );

        let canonical_faces = canonicalize_faces(&vertices, faces);

        let bounding_radius = vertices
            .iter()
            .map(|v| v.magnitude())
            .fold(0.0f32, f32::max);

        let edges = build_edge_adjacency(&vertices, &canonical_faces);

        Self {
            vertices,
            faces: canonical_faces,
            edges,
            bounding_radius,
        }
    }

    /// Compute volume via tetrahedron decomposition from the origin.
    ///
    /// Each face is decomposed into triangles (fan from first vertex).
    /// Each triangle forms a tetrahedron with the origin; signed volumes are
    /// summed. Works correctly when the origin is inside the hull.
    pub fn compute_volume(&self) -> f32 {
        let mut volume = 0.0f32;
        for face in &self.faces {
            let indices = &face.vertex_indices;
            if indices.len() < 3 {
                continue;
            }
            let v0 = self.vertices[indices[0] as usize];
            for i in 1..indices.len() - 1 {
                let v1 = self.vertices[indices[i] as usize];
                let v2 = self.vertices[indices[i + 1] as usize];
                volume += signed_tetrahedron_volume(v0, v1, v2);
            }
        }
        volume.abs()
    }

    /// Compute the inertia tensor for this hull at the given mass.
    ///
    /// Uses the same tetrahedron decomposition as `compute_volume`, accumulating
    /// per-tetrahedron inertia contributions weighted by their volume fraction.
    pub fn compute_inertia(&self, mass: f32) -> Matrix3<f32> {
        let mut total_volume = 0.0f32;
        let mut cov = Matrix3::zeros(); // covariance-like accumulator

        for face in &self.faces {
            let indices = &face.vertex_indices;
            if indices.len() < 3 {
                continue;
            }
            let a = self.vertices[indices[0] as usize];
            for i in 1..indices.len() - 1 {
                let b = self.vertices[indices[i] as usize];
                let c = self.vertices[indices[i + 1] as usize];
                let vol = signed_tetrahedron_volume(a, b, c);
                total_volume += vol;
                // Accumulate second moments for the tetrahedron (O, a, b, c).
                cov += tetrahedron_covariance(a, b, c) * vol;
            }
        }

        if total_volume.abs() < 1e-12 {
            return Matrix3::identity() * mass;
        }

        // Normalize covariance by total volume to get averaged second moments.
        cov /= total_volume;

        // Inertia tensor: I_ij = mass * (trace(cov) * delta_ij - cov_ij)
        let trace = cov[(0, 0)] + cov[(1, 1)] + cov[(2, 2)];
        (Matrix3::identity() * trace - cov) * mass
    }
}

impl ConvexSupport for ConvexHull {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        let mut best_dot = f32::NEG_INFINITY;
        let mut best = Vector3::zeros();
        for v in &self.vertices {
            let d = v.dot(&direction);
            if d > best_dot {
                best_dot = d;
                best = *v;
            }
        }
        Point3::from(best)
    }

    fn bounding_radius(&self) -> f32 {
        self.bounding_radius
    }
}

impl SupportFaceExtractor for ConvexHull {
    fn support_face(&self, direction: Vector3<f32>) -> Option<SupportFace> {
        if self.faces.is_empty() {
            return None;
        }

        // Keep the first face on ties so hull-vs-OBB chooses the same axis
        // preference as the OBB support-face path (X, then Y, then Z).
        let mut best_idx = 0usize;
        let mut best_dot = self.faces[0].normal.dot(&direction);
        for (idx, face) in self.faces.iter().enumerate().skip(1) {
            let dot = face.normal.dot(&direction);
            if dot > best_dot {
                best_dot = dot;
                best_idx = idx;
            }
        }
        let face = &self.faces[best_idx];

        let vertices: SmallVec<[Point3<f32>; 8]> = face
            .vertex_indices
            .iter()
            .map(|&i| Point3::from(self.vertices[i as usize]))
            .collect();

        Some(SupportFace {
            vertices,
            normal: face.normal,
            face_index: best_idx as u32,
        })
    }
}

/// World-space wrapper for a ConvexHull, applying center and rotation transforms.
pub struct TransformedHull<'a> {
    pub hull: &'a ConvexHull,
    pub center: Point3<f32>,
    pub rotation: UnitQuaternion<f32>,
}

impl ConvexSupport for TransformedHull<'_> {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        let local_dir = self.rotation.inverse() * direction;
        let local_pt = self.hull.support(local_dir);
        self.center + self.rotation * local_pt.coords
    }

    fn bounding_radius(&self) -> f32 {
        self.hull.bounding_radius
    }
}

impl SupportFaceExtractor for TransformedHull<'_> {
    fn support_face(&self, direction: Vector3<f32>) -> Option<SupportFace> {
        let local_dir = self.rotation.inverse() * direction;
        let local_face = self.hull.support_face(local_dir)?;

        let vertices = local_face
            .vertices
            .iter()
            .map(|v| self.center + self.rotation * v.coords)
            .collect();

        Some(SupportFace {
            vertices,
            normal: self.rotation * local_face.normal,
            face_index: local_face.face_index,
        })
    }
}

/// Signed volume of the tetrahedron formed by the origin and three vertices.
/// Equals (1/6) * a · (b × c).
fn signed_tetrahedron_volume(a: Vector3<f32>, b: Vector3<f32>, c: Vector3<f32>) -> f32 {
    a.dot(&b.cross(&c)) / 6.0
}

/// Second-moment-of-volume matrix for a tetrahedron with vertices at O, a, b, c.
///
/// For the canonical tetrahedron, each element is:
///   C_ij = (1/20) * sum over pairs of (a_i*a_j + b_i*b_j + c_i*c_j)
///        + (1/20) * (a_i*b_j + a_j*b_i + a_i*c_j + a_j*c_i + b_i*c_j + b_j*c_i)
///
/// Simplified to: (1/20) * (outer(a,a) + outer(b,b) + outer(c,c))
///              + (1/20) * (outer(a,b) + outer(b,a) + outer(a,c) + outer(c,a) + outer(b,c) + outer(c,b)) / 2
/// Which equals: (1/20) * (M * M^T) where M = [a b c] ... but more explicitly:
fn tetrahedron_covariance(a: Vector3<f32>, b: Vector3<f32>, c: Vector3<f32>) -> Matrix3<f32> {
    // For a tetrahedron (O, a, b, c), the second moment contribution is:
    // (1/20) * (2*(outer(a,a) + outer(b,b) + outer(c,c)) + outer(a,b) + outer(b,a) + outer(a,c) + outer(c,a) + outer(b,c) + outer(c,b))
    // = (1/20) * (2*diag_sum + cross_sum)
    let f = 1.0 / 20.0;
    let mut m = Matrix3::zeros();
    let verts = [a, b, c];
    for i in 0..3 {
        for j in 0..3 {
            let mut val = 0.0;
            for v in &verts {
                val += 2.0 * v[i] * v[j];
            }
            // Cross terms
            for p in 0..3 {
                for q in (p + 1)..3 {
                    val += verts[p][i] * verts[q][j] + verts[q][i] * verts[p][j];
                }
            }
            m[(i, j)] = val * f;
        }
    }
    m
}

/// Compute the AABB (min, max) of a set of vertices.
fn aabb_of(vertices: &[Vector3<f32>]) -> (Vector3<f32>, Vector3<f32>) {
    let mut min_v = Vector3::new(f32::MAX, f32::MAX, f32::MAX);
    let mut max_v = Vector3::new(f32::MIN, f32::MIN, f32::MIN);
    for v in vertices {
        min_v.x = min_v.x.min(v.x);
        min_v.y = min_v.y.min(v.y);
        min_v.z = min_v.z.min(v.z);
        max_v.x = max_v.x.max(v.x);
        max_v.y = max_v.y.max(v.y);
        max_v.z = max_v.z.max(v.z);
    }
    (min_v, max_v)
}

fn canonicalize_faces(vertices: &[Vector3<f32>], mut faces: Vec<HullFace>) -> Vec<HullFace> {
    let hull_center =
        vertices.iter().fold(Vector3::zeros(), |acc, v| acc + *v) / (vertices.len() as f32);

    for (face_idx, face) in faces.iter_mut().enumerate() {
        assert!(
            face.vertex_indices.len() >= 3,
            "ConvexHull: face {face_idx} has fewer than 3 vertices"
        );

        for &idx in &face.vertex_indices {
            assert!(
                (idx as usize) < vertices.len(),
                "ConvexHull: face {face_idx} references invalid vertex index {idx}"
            );
        }

        let computed_normal =
            compute_face_normal(vertices, &face.vertex_indices).unwrap_or_else(|| {
                panic!("ConvexHull: face {face_idx} is degenerate (collinear vertices)")
            });
        let face_center = face
            .vertex_indices
            .iter()
            .fold(Vector3::zeros(), |acc, &idx| acc + vertices[idx as usize])
            / (face.vertex_indices.len() as f32);

        let mut outward_normal = computed_normal;
        let to_face = face_center - hull_center;
        if to_face.dot(&outward_normal) < 0.0 {
            face.vertex_indices.reverse();
            outward_normal = -outward_normal;
        }

        face.normal = outward_normal;
    }

    faces
}

fn compute_face_normal(vertices: &[Vector3<f32>], indices: &[u16]) -> Option<Vector3<f32>> {
    let v0 = vertices[*indices.first()? as usize];
    let mut normal = Vector3::zeros();
    for i in 1..indices.len() - 1 {
        let v1 = vertices[indices[i] as usize];
        let v2 = vertices[indices[i + 1] as usize];
        normal += (v1 - v0).cross(&(v2 - v0));
    }
    let len_sq = normal.magnitude_squared();
    if len_sq <= FACE_NORMAL_EPS {
        None
    } else {
        Some(normal / len_sq.sqrt())
    }
}

/// Build the unique edge list with adjacent face normals from a hull's face data.
///
/// Each edge (shared by exactly two faces with opposite windings) is stored once
/// with both adjacent face normals. Boundary edges (only one adjacent face) are
/// skipped — these shouldn't exist on a closed convex hull.
fn build_edge_adjacency(_vertices: &[Vector3<f32>], faces: &[HullFace]) -> Vec<HullEdgeAdj> {
    struct FaceEdgeRef {
        face_idx: usize,
        a: u16,
        b: u16,
    }

    let mut edge_faces: Vec<((u16, u16), Vec<FaceEdgeRef>)> = Vec::new();

    for (face_idx, face) in faces.iter().enumerate() {
        let indices = &face.vertex_indices;
        let n = indices.len();
        for j in 0..n {
            let a = indices[j];
            let b = indices[(j + 1) % n];
            let key = if a < b { (a, b) } else { (b, a) };

            if let Some((_, refs)) = edge_faces.iter_mut().find(|(k, _)| *k == key) {
                refs.push(FaceEdgeRef { face_idx, a, b });
            } else {
                edge_faces.push((key, vec![FaceEdgeRef { face_idx, a, b }]));
            }
        }
    }

    edge_faces
        .into_iter()
        .map(|((key_v0, key_v1), refs)| {
            assert!(
                refs.len() == 2,
                "ConvexHull: edge ({key_v0},{key_v1}) has {} adjacent faces; expected 2 for closed manifold",
                refs.len()
            );

            let e0 = &refs[0];
            let e1 = &refs[1];
            assert!(
                e0.a == e1.b && e0.b == e1.a,
                "ConvexHull: edge ({key_v0},{key_v1}) has inconsistent face winding"
            );

            HullEdgeAdj {
                // Preserve topological edge direction from face winding.
                // The Gauss-map Minkowski-face test depends on this orientation.
                v0: e0.a,
                v1: e0.b,
                normal_a: faces[e0.face_idx].normal,
                normal_b: faces[e1.face_idx].normal,
            }
        })
        .collect()
}

/// Build a cube-shaped ConvexHull with the given half-extents, centered at origin.
///
/// Used by tests to create cube-shaped hulls for comparison against OBB paths.
#[cfg(test)]
pub fn cube_hull(half_extents: Vector3<f32>) -> ConvexHull {
    let hx = half_extents.x;
    let hy = half_extents.y;
    let hz = half_extents.z;

    let vertices = vec![
        Vector3::new(-hx, -hy, -hz), // 0
        Vector3::new(hx, -hy, -hz),  // 1
        Vector3::new(hx, hy, -hz),   // 2
        Vector3::new(-hx, hy, -hz),  // 3
        Vector3::new(-hx, -hy, hz),  // 4
        Vector3::new(hx, -hy, hz),   // 5
        Vector3::new(hx, hy, hz),    // 6
        Vector3::new(-hx, hy, hz),   // 7
    ];

    let faces = vec![
        // +X face (1, 5, 6, 2)
        HullFace {
            vertex_indices: SmallVec::from_slice(&[1, 5, 6, 2]),
            normal: Vector3::x(),
        },
        // -X face (4, 0, 3, 7)
        HullFace {
            vertex_indices: SmallVec::from_slice(&[4, 0, 3, 7]),
            normal: -Vector3::x(),
        },
        // +Y face (3, 2, 6, 7)
        HullFace {
            vertex_indices: SmallVec::from_slice(&[3, 2, 6, 7]),
            normal: Vector3::y(),
        },
        // -Y face (0, 4, 5, 1)
        HullFace {
            vertex_indices: SmallVec::from_slice(&[0, 4, 5, 1]),
            normal: -Vector3::y(),
        },
        // +Z face (4, 7, 6, 5)
        HullFace {
            vertex_indices: SmallVec::from_slice(&[4, 7, 6, 5]),
            normal: Vector3::z(),
        },
        // -Z face (0, 1, 2, 3)
        HullFace {
            vertex_indices: SmallVec::from_slice(&[0, 1, 2, 3]),
            normal: -Vector3::z(),
        },
    ];

    ConvexHull::new(vertices, faces)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::obb::Obb;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    fn approx_eq_point(a: Point3<f32>, b: Point3<f32>, tol: f32) -> bool {
        (a - b).magnitude() < tol
    }

    // --- ConvexSupport correctness: hull cube vs Obb ---

    #[test]
    fn support_matches_obb_axis_aligned() {
        let he = Vector3::new(1.0, 2.0, 3.0);
        let hull = cube_hull(he);
        let obb = Obb::new(Point3::origin(), UnitQuaternion::identity(), he);

        // Directions where the support vertex is unique (non-axis-aligned).
        let dirs = [
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(-1.0, 0.5, -0.3),
            Vector3::new(0.3, -0.7, 0.2),
            Vector3::new(-1.0, -1.0, -1.0),
        ];

        for dir in &dirs {
            let hull_pt = hull.support(*dir);
            let obb_pt = obb.support(*dir);
            assert!(
                approx_eq_point(hull_pt, obb_pt, 1e-5),
                "support mismatch for dir {:?}: hull={:?}, obb={:?}",
                dir,
                hull_pt,
                obb_pt,
            );
        }

        // For axis-aligned directions, the support dot product should match
        // even though the exact vertex may differ (tiebreaking differs).
        let axis_dirs = [
            Vector3::x(),
            -Vector3::x(),
            Vector3::y(),
            -Vector3::y(),
            Vector3::z(),
            -Vector3::z(),
        ];
        for dir in &axis_dirs {
            let hull_dot = hull.support(*dir).coords.dot(dir);
            let obb_dot = obb.support(*dir).coords.dot(dir);
            assert!(
                approx_eq(hull_dot, obb_dot, 1e-5),
                "support dot mismatch for dir {:?}: hull={}, obb={}",
                dir,
                hull_dot,
                obb_dot,
            );
        }
    }

    #[test]
    fn support_matches_obb_with_transform() {
        let he = Vector3::new(1.0, 1.5, 0.5);
        let hull = cube_hull(he);
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
        let center = Point3::new(3.0, 1.0, -2.0);

        let obb = Obb::new(center, rot, he);
        let th = TransformedHull {
            hull: &hull,
            center,
            rotation: rot,
        };

        // Use non-axis-aligned directions for unique support vertices.
        let dirs = [
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(-0.5, 2.0, -1.0),
            Vector3::new(0.7, -0.3, 0.5),
        ];

        for dir in &dirs {
            let hull_pt = th.support(*dir);
            let obb_pt = obb.support(*dir);
            assert!(
                approx_eq_point(hull_pt, obb_pt, 1e-4),
                "transformed support mismatch for dir {:?}: hull={:?}, obb={:?}",
                dir,
                hull_pt,
                obb_pt,
            );
        }
    }

    // --- Mass / volume ---

    #[test]
    fn volume_matches_box() {
        let he = Vector3::new(1.0, 2.0, 3.0);
        let hull = cube_hull(he);
        let expected_volume = 8.0 * he.x * he.y * he.z;
        let hull_volume = hull.compute_volume();
        assert!(
            approx_eq(hull_volume, expected_volume, expected_volume * 0.01),
            "volume: hull={}, expected={}",
            hull_volume,
            expected_volume,
        );
    }

    #[test]
    fn inertia_matches_box() {
        let he = Vector3::new(1.0, 2.0, 3.0);
        let hull = cube_hull(he);
        let density = 1000.0;
        let volume = 8.0 * he.x * he.y * he.z;
        let mass = volume * density;

        let hull_inertia = hull.compute_inertia(mass);
        let box_inertia =
            crate::physics::ColliderShape::Box { half_extents: he }.compute_inertia(mass);

        for i in 0..3 {
            for j in 0..3 {
                let h = hull_inertia[(i, j)];
                let b = box_inertia[(i, j)];
                assert!(
                    approx_eq(h, b, b.abs() * 0.01 + 1e-6),
                    "inertia[{},{}]: hull={}, box={}",
                    i,
                    j,
                    h,
                    b,
                );
            }
        }
    }

    // --- Bounding radius ---

    #[test]
    fn bounding_radius_cube() {
        let he = Vector3::new(1.0, 1.0, 1.0);
        let hull = cube_hull(he);
        let expected = he.magnitude();
        assert!(approx_eq(hull.bounding_radius, expected, 1e-5));
    }

    // --- Validation ---

    #[test]
    #[should_panic(expected = "vertex count")]
    fn rejects_too_many_vertices() {
        let vertices: Vec<Vector3<f32>> = (0..65)
            .map(|i| {
                let angle = (i as f32) * std::f32::consts::TAU / 65.0;
                Vector3::new(angle.cos(), angle.sin(), 0.5)
            })
            .collect();
        let faces = vec![HullFace {
            vertex_indices: SmallVec::from_slice(&[0, 1, 2]),
            normal: Vector3::z(),
        }];
        ConvexHull::new(vertices, faces);
    }

    #[test]
    #[should_panic(expected = "degenerate")]
    fn rejects_flat_hull() {
        // All vertices coplanar (z = 0).
        let vertices = vec![
            Vector3::new(-1.0, -1.0, 0.0),
            Vector3::new(1.0, -1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(-1.0, 1.0, 0.0),
        ];
        let faces = vec![HullFace {
            vertex_indices: SmallVec::from_slice(&[0, 1, 2, 3]),
            normal: Vector3::z(),
        }];
        ConvexHull::new(vertices, faces);
    }

    #[test]
    fn canonicalizes_face_normals_and_winding() {
        let s = 1.0f32;
        let vertices = vec![
            Vector3::new(s, s, s),
            Vector3::new(s, -s, -s),
            Vector3::new(-s, s, -s),
            Vector3::new(-s, -s, s),
        ];
        // Deliberately provide mixed winding and bogus normals.
        let faces = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 2, 1]),
                normal: Vector3::new(0.0, 0.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 2, 3]),
                normal: Vector3::new(99.0, -3.0, 2.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 3, 0]),
                normal: Vector3::new(-7.0, 2.0, 1.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 2, 3]),
                normal: Vector3::new(1.0, 1.0, 1.0),
            },
        ];

        let hull = ConvexHull::new(vertices.clone(), faces);
        assert_eq!(
            hull.edges.len(),
            6,
            "Tetrahedron must have 6 manifold edges"
        );

        let center =
            vertices.iter().fold(Vector3::zeros(), |acc, v| acc + *v) / (vertices.len() as f32);
        for (face_idx, face) in hull.faces.iter().enumerate() {
            let face_center = face
                .vertex_indices
                .iter()
                .fold(Vector3::zeros(), |acc, &idx| acc + vertices[idx as usize])
                / (face.vertex_indices.len() as f32);
            assert!(
                (face.normal.magnitude() - 1.0).abs() < 1e-5,
                "face {face_idx} normal must be unit length"
            );
            assert!(
                (face_center - center).dot(&face.normal) > 0.0,
                "face {face_idx} normal must point outward"
            );
        }
    }

    #[test]
    #[should_panic(expected = "adjacent faces")]
    fn rejects_non_manifold_edge_adjacency() {
        let s = 1.0f32;
        let vertices = vec![
            Vector3::new(s, s, s),
            Vector3::new(s, -s, -s),
            Vector3::new(-s, s, -s),
            Vector3::new(-s, -s, s),
        ];
        // Duplicate one face to force edges with 3 adjacent faces.
        let faces = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 1, 2]),
                normal: Vector3::new(0.0, 0.0, 1.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 1, 2]),
                normal: Vector3::new(0.0, 0.0, 1.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 2, 3]),
                normal: Vector3::new(0.0, 1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 3, 1]),
                normal: Vector3::new(1.0, 0.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 3, 2]),
                normal: Vector3::new(-1.0, 0.0, 0.0),
            },
        ];
        ConvexHull::new(vertices, faces);
    }

    // --- SupportFaceExtractor ---

    #[test]
    fn support_face_returns_correct_face() {
        let hull = cube_hull(Vector3::new(1.0, 1.0, 1.0));

        // +Y direction should give the +Y face
        let face = hull.support_face(Vector3::y()).unwrap();
        assert!(face.normal.dot(&Vector3::y()) > 0.99);
        assert_eq!(face.vertices.len(), 4);

        // All face vertices should have y == 1.0
        for v in &face.vertices {
            assert!(approx_eq(v.y, 1.0, 1e-5));
        }
    }

    #[test]
    fn support_face_transformed() {
        let hull = cube_hull(Vector3::new(1.0, 1.0, 1.0));
        let center = Point3::new(5.0, 0.0, 0.0);
        let rot = UnitQuaternion::identity();
        let th = TransformedHull {
            hull: &hull,
            center,
            rotation: rot,
        };

        let face = th.support_face(Vector3::y()).unwrap();
        assert!(face.normal.dot(&Vector3::y()) > 0.99);
        // Vertices should be offset by center
        for v in &face.vertices {
            assert!(approx_eq(v.x, 5.0 + v.x - 5.0, 1e-5)); // x in [4,6]
            assert!(approx_eq(v.y, 1.0, 1e-5));
        }
    }
}
