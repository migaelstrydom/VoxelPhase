//! General convex shape vs FilteredPatch manifold generation.
//!
//! For each ContactFace in the patch, projects the convex shape onto the face
//! plane along the face normal, clips support faces, and produces up to 4
//! contacts per patch via ContactReducer.
//!
//! Uses direct face-normal projection rather than GJK/EPA. For mesh contacts
//! the contact normal is the face normal, and penetration depth is computed by
//! projecting the shape's deepest support point onto the face plane. This avoids
//! GJK's numerical instability when a small convex shape meets a large flat
//! polygon (the Minkowski difference has extreme aspect ratio, causing simplex
//! refinement failures).

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::{ContactManifold, ContactPoint};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::mesh::seam_filter::{ContactFace, FilteredPatch};
use crate::collision::shape_view::{SupportFace, SupportFaceExtractor};
use crate::collision::support::ConvexSupport;

/// Maximum contacts in the final manifold.
const MAX_MANIFOLD_POINTS: usize = 4;
/// Small tolerance for rejecting backfacing mesh contacts.
const BACKFACE_EPSILON: f32 = 1e-4;
/// Keep near-touching contacts single-point until penetration is clearly positive.
const SHALLOW_PENETRATION_EPSILON: f32 = 1e-3;

/// A convex polygon (ContactFace) wrapped for GJK/EPA support queries.
struct SupportPolygon<'a> {
    face: &'a ContactFace,
}

impl ConvexSupport for SupportPolygon<'_> {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        let verts = &self.face.vertices;
        let mut best = verts[0];
        let mut best_dot = verts[0].coords.dot(&direction);
        for v in verts.iter().skip(1) {
            let d = v.coords.dot(&direction);
            if d > best_dot {
                best_dot = d;
                best = *v;
            }
        }
        best
    }

    fn bounding_radius(&self) -> f32 {
        let verts = &self.face.vertices;
        let center =
            Point3::from(verts.iter().map(|v| v.coords).sum::<Vector3<f32>>() / verts.len() as f32);
        verts
            .iter()
            .map(|v| (v - center).magnitude())
            .fold(0.0f32, f32::max)
    }
}

impl SupportFaceExtractor for SupportPolygon<'_> {
    fn support_face(&self, _direction: Vector3<f32>) -> Option<SupportFace> {
        Some(SupportFace {
            vertices: SmallVec::from_slice(&self.face.vertices),
            normal: self.face.normal,
            face_index: 0,
        })
    }
}

/// Generate a contact manifold for a convex shape against a filtered mesh patch.
///
/// Runs direct face-normal projection per face, clips support faces,
/// and reduces to 4 contacts.
/// The normal points from the terrain surface toward the shape.
///
/// # Arguments
/// * `shape` — world-space convex shape view
/// * `patch` — seam-filtered mesh patch
/// * `margin` — contact margin for speculative contacts
pub fn gjk_patch_manifold<S: ConvexSupport + SupportFaceExtractor>(
    shape: &S,
    patch: &FilteredPatch,
    margin: f32,
) -> ContactManifold {
    let mut all_points: SmallVec<[ContactPoint; 4]> = SmallVec::new();

    for face in &patch.faces {
        if face.vertices.len() < 3 {
            continue;
        }

        let face_contacts = gjk_epa_vs_face(shape, face, margin);
        all_points.extend(face_contacts);
    }

    if all_points.is_empty() {
        return ContactManifold::empty();
    }

    if all_points.len() > MAX_MANIFOLD_POINTS {
        let reducer = ContactReducer::new(MAX_MANIFOLD_POINTS);
        let reduced = reducer.reduce(&all_points);
        return ContactManifold::from_vec(reduced.into_iter().collect());
    }

    ContactManifold::from_vec(all_points)
}

/// Generate contacts between a convex shape and a single mesh face.
///
/// Uses direct face-normal projection rather than GJK/EPA. For mesh contacts
/// the contact normal is the face normal (the terrain surface pushes the shape
/// outward), and penetration depth is computed by projecting the shape's deepest
/// point onto the face plane. This avoids GJK's numerical issues when a small
/// convex shape collides with a large flat polygon (extreme Minkowski difference
/// aspect ratio causes simplex refinement failures).
///
/// Returns contact points for this face (0 to ~4).
fn gjk_epa_vs_face<S: ConvexSupport + SupportFaceExtractor>(
    shape: &S,
    face: &ContactFace,
    margin: f32,
) -> SmallVec<[ContactPoint; 4]> {
    let normal = face.normal;
    let face_point = face.vertices[0];

    // One-sided terrain: reject only when the entire shape is behind the face.
    // This uses exact support points, avoiding center-estimation jitter.
    let highest = shape.support(normal);
    let highest_signed_dist = (highest - face_point).dot(&normal);
    if highest_signed_dist < -BACKFACE_EPSILON {
        return SmallVec::new();
    }

    // Find the shape's deepest point along the face normal.
    let deepest = shape.support(-normal);
    let deepest_signed_dist = (deepest - face_point).dot(&normal);

    // Positive signed_dist means the deepest point is above the face plane.
    // Contact exists when deepest_signed_dist < margin.
    if deepest_signed_dist > margin {
        return SmallVec::new();
    }

    let raw_depth = -deepest_signed_dist;
    if raw_depth <= SHALLOW_PENETRATION_EPSILON {
        // Near-touching contact: keep a single point to avoid premature face-face
        // manifolds while transitioning from edge/vertex to stable penetration.
        let projected = project_onto_face_plane(deepest, face);
        if !point_in_convex_polygon(&projected, &face.vertices, &normal) {
            return SmallVec::new();
        }
        let mut contacts = SmallVec::new();
        contacts.push(ContactPoint::new(
            projected,
            normal,
            raw_depth,
            face.feature_id,
        ));
        return contacts;
    }

    // Extract support faces for clipping to produce multi-point manifolds.
    let shape_face = shape.support_face(-normal);
    let polygon = SupportPolygon { face };
    let face_support = polygon.support_face(normal);

    let clipped = match (shape_face, face_support) {
        (Some(sf), Some(ff)) => clip_shape_face_against_mesh_face(&sf, &ff, normal, margin, face),
        _ => SmallVec::new(),
    };

    if !clipped.is_empty() {
        return clipped;
    }

    // Face clipping produced nothing (edge/vertex contact, faceless shape,
    // or all clipped points above margin). Fall back to a single contact
    // at the deepest point's projection, but only if it lands inside the
    // face polygon — otherwise the adjacent face should handle it.
    let projected = project_onto_face_plane(deepest, face);
    if point_in_convex_polygon(&projected, &face.vertices, &normal) {
        let mut contacts = SmallVec::new();
        contacts.push(ContactPoint::new(
            projected,
            normal,
            raw_depth,
            face.feature_id,
        ));
        contacts
    } else {
        SmallVec::new()
    }
}

/// Clip the shape's support face against the mesh face's edges,
/// then project onto the face plane and emit contacts.
fn clip_shape_face_against_mesh_face(
    shape_face: &SupportFace,
    mesh_face_support: &SupportFace,
    normal: Vector3<f32>,
    margin: f32,
    face: &ContactFace,
) -> SmallVec<[ContactPoint; 4]> {
    let mesh_verts = &mesh_face_support.vertices;
    let face_normal = face.normal;

    // Clip shape face against mesh face edges (Sutherland-Hodgman).
    let clipped = clip_against_face_edges(&shape_face.vertices, mesh_verts, &face_normal);

    let face_point = face.vertices[0];

    if clipped.is_empty() {
        // Clipping failed — try projecting shape face vertices into the mesh face.
        return project_shape_verts_into_face(&shape_face.vertices, face, normal, margin);
    }

    let mut contacts: SmallVec<[ContactPoint; 4]> = SmallVec::new();

    for (i, p) in clipped.iter().enumerate() {
        let signed_dist = (p - face_point).dot(&face_normal);
        let raw_depth = -signed_dist;

        if raw_depth < -margin {
            continue;
        }

        let projected = p - face_normal * signed_dist;
        let feature_id = face.feature_id.with_vertex(i as u32);
        contacts.push(ContactPoint::new(projected, normal, raw_depth, feature_id));
    }

    // If all clipped points were above the margin threshold, return empty.
    // The caller falls back to a single deepest-point contact.
    contacts
}

/// Fallback: project shape face vertices onto the mesh face plane and keep
/// those that land inside the polygon.
fn project_shape_verts_into_face(
    shape_verts: &SmallVec<[Point3<f32>; 8]>,
    face: &ContactFace,
    normal: Vector3<f32>,
    margin: f32,
) -> SmallVec<[ContactPoint; 4]> {
    let face_point = face.vertices[0];
    let face_normal = face.normal;
    let mut contacts: SmallVec<[ContactPoint; 4]> = SmallVec::new();

    for (i, v) in shape_verts.iter().enumerate() {
        let signed_dist = (v - face_point).dot(&face_normal);
        let raw_depth = -signed_dist;

        if raw_depth < -margin {
            continue;
        }

        let projected = v - face_normal * signed_dist;
        if point_in_convex_polygon(&projected, &face.vertices, &face_normal) {
            contacts.push(ContactPoint::new(
                projected,
                normal,
                raw_depth,
                face.feature_id.with_vertex(i as u32),
            ));
        }
    }

    contacts
}

/// Clip subject polygon against the edge half-planes of a clip polygon.
fn clip_against_face_edges(
    subject: &SmallVec<[Point3<f32>; 8]>,
    clip_verts: &SmallVec<[Point3<f32>; 8]>,
    face_normal: &Vector3<f32>,
) -> SmallVec<[Point3<f32>; 8]> {
    if clip_verts.len() < 3 {
        return subject.clone();
    }

    let centroid = compute_centroid(clip_verts);
    let mut clipped = subject.clone();

    for i in 0..clip_verts.len() {
        if clipped.is_empty() {
            break;
        }
        let a = clip_verts[i];
        let b = clip_verts[(i + 1) % clip_verts.len()];
        let edge = b - a;
        let candidate_inward = edge.cross(face_normal);

        let inward = if (centroid - a).dot(&candidate_inward) >= 0.0 {
            candidate_inward
        } else {
            -candidate_inward
        };

        clipped = clip_polygon_by_plane(&clipped, a, inward);
    }

    clipped
}

/// Clip a convex polygon against a half-plane, keeping the inside.
fn clip_polygon_by_plane(
    polygon: &[Point3<f32>],
    plane_point: Point3<f32>,
    plane_normal: Vector3<f32>,
) -> SmallVec<[Point3<f32>; 8]> {
    let mut out: SmallVec<[Point3<f32>; 8]> = SmallVec::new();
    if polygon.is_empty() {
        return out;
    }

    for i in 0..polygon.len() {
        let p1 = polygon[i];
        let p2 = polygon[(i + 1) % polygon.len()];
        let d1 = (p1 - plane_point).dot(&plane_normal);
        let d2 = (p2 - plane_point).dot(&plane_normal);

        if d1 >= 0.0 && d2 >= 0.0 {
            out.push(p2);
        } else if d1 >= 0.0 {
            let t = d1 / (d1 - d2);
            out.push(p1 + (p2 - p1) * t);
        } else if d2 >= 0.0 {
            let t = d1 / (d1 - d2);
            out.push(p1 + (p2 - p1) * t);
            out.push(p2);
        }
    }

    out
}

/// Project a point onto a face's plane along the face normal.
fn project_onto_face_plane(point: Point3<f32>, face: &ContactFace) -> Point3<f32> {
    let signed_dist = (point - face.vertices[0]).dot(&face.normal);
    point - face.normal * signed_dist
}

/// Compute centroid of a polygon.
fn compute_centroid(verts: &[Point3<f32>]) -> Point3<f32> {
    let sum: Vector3<f32> = verts.iter().map(|v| v.coords).sum();
    Point3::from(sum / verts.len() as f32)
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
    use crate::collision::capsule::Capsule;
    use crate::collision::contact::FeatureId;
    use crate::collision::mesh::capsule_patch::capsule_patch_manifold;
    use crate::collision::mesh::obb_patch::obb_patch_manifold;
    use crate::collision::mesh::sphere_patch::sphere_patch_manifold;
    use crate::collision::obb::Obb;
    use crate::collision::shape_view::ShapeView;
    use crate::physics::ColliderShape;
    use nalgebra::UnitQuaternion;

    /// Step terrain: an upper level at y=1 for x>=0, a vertical wall at x=0,
    /// and a lower level at y=0 for x<0. Mirrors `StepGeometry` from the
    /// physics bench harness, merged into quads.
    fn step_patch() -> FilteredPatch {
        FilteredPatch {
            faces: SmallVec::from_vec(vec![
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-4.0, 0.0, -4.0),
                        Point3::new(0.0, 0.0, -4.0),
                        Point3::new(0.0, 0.0, 4.0),
                        Point3::new(-4.0, 0.0, 4.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(0.0, 0.0, -4.0),
                        Point3::new(0.0, 1.0, -4.0),
                        Point3::new(0.0, 1.0, 4.0),
                        Point3::new(0.0, 0.0, 4.0),
                    ]),
                    normal: -Vector3::x(),
                    feature_id: FeatureId::from_face(1),
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(0.0, 1.0, -4.0),
                        Point3::new(4.0, 1.0, -4.0),
                        Point3::new(4.0, 1.0, 4.0),
                        Point3::new(0.0, 1.0, 4.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(2),
                },
            ]),
            boundary_edges: SmallVec::new(),
        }
    }

    /// Build a box-shaped convex hull with the given half extents.
    fn box_hull(half_extents: Vector3<f32>) -> ConvexHull {
        use crate::collision::convex_hull::HullFace;

        let h = half_extents;
        let vertices = vec![
            Vector3::new(-h.x, -h.y, -h.z),
            Vector3::new(h.x, -h.y, -h.z),
            Vector3::new(h.x, h.y, -h.z),
            Vector3::new(-h.x, h.y, -h.z),
            Vector3::new(-h.x, -h.y, h.z),
            Vector3::new(h.x, -h.y, h.z),
            Vector3::new(h.x, h.y, h.z),
            Vector3::new(-h.x, h.y, h.z),
        ];
        let faces = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[4, 5, 6, 7]),
                normal: Vector3::z(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 0, 3, 2]),
                normal: -Vector3::z(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[5, 1, 2, 6]),
                normal: Vector3::x(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 4, 7, 3]),
                normal: -Vector3::x(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[3, 7, 6, 2]),
                normal: Vector3::y(),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 1, 5, 4]),
                normal: -Vector3::y(),
            },
        ];
        ConvexHull::new(vertices, faces)
    }

    /// Regression: a hull lying diagonally across a step edge overlaps the
    /// step corner, but its extreme support points along each face normal
    /// land outside that face's polygon — the bottom-most vertex hangs over
    /// the lower level, the right-most vertex sticks up above the upper
    /// level. The overlapping middle of the hull must still produce contacts,
    /// otherwise the body sinks through the step edge.
    #[test]
    fn hull_straddling_step_edge_produces_contacts() {
        let hull = std::sync::Arc::new(box_hull(Vector3::new(0.4, 1.2, 0.4)));
        let shape = ColliderShape::ConvexHull { hull };
        let view = ShapeView {
            center: Point3::new(0.2, 1.35, 0.0),
            rotation: UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                -std::f32::consts::FRAC_PI_4,
            ),
            shape: &shape,
        };

        let manifold = gjk_patch_manifold(&view, &step_patch(), 0.02);

        assert!(
            !manifold.is_empty(),
            "hull overlapping the step corner produced no contacts"
        );
        for cp in manifold.points.iter() {
            assert!(
                cp.raw_depth < 0.6,
                "contact depth {:.3} far exceeds the actual overlap",
                cp.raw_depth
            );
        }
    }

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
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        }
    }

    fn ramp_patch(slope_degrees: f32) -> FilteredPatch {
        // Ramp tilted around the X axis: rises in +Z direction.
        // At z=-5 the surface is at y=0, at z=+5 it's at y=rise.
        let angle = slope_degrees.to_radians();
        let rise = 10.0 * angle.tan();
        // Normal = (-sin(angle) in Z, cos(angle) in Y) rotated about X.
        let normal = Vector3::new(0.0, angle.cos(), -angle.sin()).normalize();
        FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-5.0, 0.0, -5.0),
                        Point3::new(5.0, 0.0, -5.0),
                        Point3::new(5.0, rise, 5.0),
                        Point3::new(-5.0, rise, 5.0),
                    ]),
                    normal,
                    feature_id: FeatureId::from_face(0),
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        }
    }

    // --- OBB cross-validation ---

    #[test]
    fn obb_cross_validation_flat() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let he = Vector3::new(0.5, 0.5, 0.5);
        let center = Point3::new(0.0, 0.4, 0.0);
        let rot = UnitQuaternion::identity();

        let shape = ColliderShape::Box { half_extents: he };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let gjk_result = gjk_patch_manifold(&view, &patch, margin);
        let obb = Obb::new(center, rot, he);
        let obb_result = obb_patch_manifold(&obb, &patch, margin);

        assert!(!gjk_result.is_empty(), "GJK patch should produce contacts");
        assert!(!obb_result.is_empty(), "OBB patch should produce contacts");

        // Normal direction (dot > 0.99).
        let gjk_n = gjk_result.points[0].normal;
        let obb_n = obb_result.points[0].normal;
        assert!(
            gjk_n.dot(&obb_n) > 0.99,
            "Normal mismatch: GJK {:?} vs OBB {:?}",
            gjk_n,
            obb_n,
        );

        // Depth (±0.05).
        let gjk_d = gjk_result.points[0].raw_depth;
        let obb_d = obb_result.points[0].raw_depth;
        assert!(
            (gjk_d - obb_d).abs() < 0.05,
            "Depth mismatch: GJK {} vs OBB {}",
            gjk_d,
            obb_d,
        );

        // Contact count (within ±1).
        let count_diff = (gjk_result.len() as i32 - obb_result.len() as i32).abs();
        assert!(
            count_diff <= 1,
            "Count mismatch: GJK {} vs OBB {}",
            gjk_result.len(),
            obb_result.len(),
        );

        // Contact positions (within 0.1 of nearest OBB contact).
        for gp in &gjk_result.points {
            let min_dist = obb_result
                .points
                .iter()
                .map(|op| (gp.point - op.point).magnitude())
                .fold(f32::INFINITY, f32::min);
            assert!(
                min_dist < 0.15,
                "GJK contact {:?} too far from any OBB contact (min_dist={})",
                gp.point,
                min_dist,
            );
        }
    }

    // --- Sphere cross-validation ---

    #[test]
    fn sphere_cross_validation_flat() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let center = Point3::new(0.0, 0.4, 0.0);
        let shape = ColliderShape::Sphere { radius: 0.5 };
        let view = ShapeView {
            center,
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };

        let gjk_result = gjk_patch_manifold(&view, &patch, margin);
        let sphere_result = sphere_patch_manifold(center, 0.5, &patch, margin);

        assert!(!gjk_result.is_empty(), "GJK patch should produce contacts");
        assert!(
            !sphere_result.is_empty(),
            "Sphere patch should produce contacts"
        );

        let gjk_n = gjk_result.points[0].normal;
        let sph_n = sphere_result.points[0].normal;
        assert!(gjk_n.dot(&sph_n) > 0.99, "Normal mismatch");

        let gjk_d = gjk_result.points[0].raw_depth;
        let sph_d = sphere_result.points[0].raw_depth;
        assert!(
            (gjk_d - sph_d).abs() < 0.05,
            "Depth mismatch: GJK {} vs Sphere {}",
            gjk_d,
            sph_d,
        );
    }

    // --- Capsule cross-validation ---

    #[test]
    fn capsule_cross_validation_flat() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let center = Point3::new(0.0, 1.0, 0.0);
        let rot = UnitQuaternion::identity();
        let shape = ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let gjk_result = gjk_patch_manifold(&view, &patch, margin);
        let capsule = Capsule::new(center, rot, 1.0, 0.5);
        let (seg_a, seg_b) = capsule.segment_endpoints();
        let cap_result = capsule_patch_manifold(seg_a, seg_b, 0.5, &patch, margin);

        assert!(!gjk_result.is_empty(), "GJK patch should produce contacts");
        assert!(
            !cap_result.is_empty(),
            "Capsule patch should produce contacts"
        );

        // Normal direction.
        let gjk_n = gjk_result.points[0].normal;
        let cap_n = cap_result.points[0].normal;
        assert!(
            gjk_n.dot(&cap_n) > 0.99,
            "Normal mismatch: GJK {:?} vs Capsule {:?}",
            gjk_n,
            cap_n,
        );

        // Depth (±0.05).
        let gjk_d = gjk_result.points[0].raw_depth;
        let cap_d = cap_result.points[0].raw_depth;
        assert!(
            (gjk_d - cap_d).abs() < 0.05,
            "Depth mismatch: GJK {} vs Capsule {}",
            gjk_d,
            cap_d,
        );
    }

    // --- Ramp geometry ---

    #[test]
    fn obb_on_ramp() {
        // 30° ramp: at z=0 the surface is at y ≈ 2.89.
        // Ramp normal is (0, 0.866, -0.5).
        // Place box center above the surface along the ramp normal.
        let patch = ramp_patch(30.0);
        let margin = 0.02;

        // Surface height at z=0: interpolated = rise/2 ≈ 2.89.
        // Box half-extent 0.5, so place center slightly into the surface
        // along the ramp normal direction.
        let ramp_normal = patch.faces[0].normal;
        let surface_point = Point3::new(0.0, 2.89, 0.0);
        let center = surface_point + ramp_normal * 0.4;

        let he = Vector3::new(0.5, 0.5, 0.5);
        let rot = UnitQuaternion::identity();
        let shape = ColliderShape::Box { half_extents: he };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &patch, margin);

        assert!(!result.is_empty(), "Should have contacts on ramp");
        let n = result.points[0].normal;
        assert!(
            n.dot(&ramp_normal) > 0.95,
            "Normal should align with ramp: got {:?}, expected {:?}",
            n,
            ramp_normal,
        );
    }

    // --- Step geometry ---

    #[test]
    fn obb_straddling_step() {
        let step_patch = FilteredPatch {
            faces: SmallVec::from_vec(vec![
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-5.0, 0.0, -5.0),
                        Point3::new(0.0, 0.0, -5.0),
                        Point3::new(0.0, 0.0, 5.0),
                        Point3::new(-5.0, 0.0, 5.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(0.0, 0.5, -5.0),
                        Point3::new(5.0, 0.5, -5.0),
                        Point3::new(5.0, 0.5, 5.0),
                        Point3::new(0.0, 0.5, 5.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(1),
                },
            ]),
            boundary_edges: SmallVec::new(),
        };
        let margin = 0.02;

        let he = Vector3::new(1.0, 0.5, 0.5);
        // Center at y=0.5 → bottom at y=0.0, touching both faces
        // (face 0 at y=0, face 1 at y=0.5). Box extends x=-1 to x=1,
        // straddling the step edge at x=0.
        let center = Point3::new(0.0, 0.5, 0.0);
        let rot = UnitQuaternion::identity();
        let shape = ColliderShape::Box { half_extents: he };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &step_patch, margin);

        // Should produce contacts on both faces.
        assert!(
            result.len() >= 2,
            "Should have contacts on both step faces, got {}",
            result.len(),
        );
    }

    // --- Tetrahedron hull helper ---

    use crate::collision::convex_hull::{ConvexHull, HullFace};
    use std::sync::Arc;

    /// Build a regular tetrahedron ConvexHull with the given edge length,
    /// centered at the origin. Same geometry as the spawnable.
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

        // Build faces with outward normals using opposite-vertex check.
        let face_defs: [(usize, usize, usize, usize); 4] = [
            (1, 2, 3, 0), // base, opposite = top
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

    fn flat_face() -> ContactFace {
        ContactFace {
            vertices: SmallVec::from_vec(vec![
                Point3::new(-10.0, 0.0, -10.0),
                Point3::new(10.0, 0.0, -10.0),
                Point3::new(10.0, 0.0, 10.0),
                Point3::new(-10.0, 0.0, 10.0),
            ]),
            normal: Vector3::y(),
            feature_id: FeatureId::from_face(0),
        }
    }

    /// Small polygon (2x2) centered at origin for isolating EPA from
    /// large-polygon GJK numerics.
    fn small_flat_face() -> ContactFace {
        ContactFace {
            vertices: SmallVec::from_vec(vec![
                Point3::new(-1.0, 0.0, -1.0),
                Point3::new(1.0, 0.0, -1.0),
                Point3::new(1.0, 0.0, 1.0),
                Point3::new(-1.0, 0.0, 1.0),
            ]),
            normal: Vector3::y(),
            feature_id: FeatureId::from_face(0),
        }
    }

    // ===== Layer 1: GJK/EPA directly =====
    // Verifies GJK+EPA correctness for tetrahedron vs small polygon.
    // Large polygons trigger a known GJK numerical issue (extreme Minkowski
    // difference aspect ratio), which is why gjk_epa_vs_face uses direct
    // face-normal projection instead of GJK.

    #[test]
    fn layer1_epa_tetrahedron_small_polygon_normal() {
        let hull = tetrahedron_hull(1.0);
        let face = small_flat_face();
        let polygon = SupportPolygon { face: &face };

        use crate::collision::convex_hull::TransformedHull;
        use crate::collision::discrete::epa::epa_penetration;
        use crate::collision::discrete::gjk::{gjk_query, GjkResult};

        let center = Point3::new(0.0, 0.15, 0.0);
        let transformed = TransformedHull {
            hull: &hull,
            center,
            rotation: UnitQuaternion::identity(),
        };

        let result = gjk_query(&transformed, &polygon);
        match result {
            GjkResult::Intersecting { simplex } => {
                let epa = epa_penetration(&transformed, &polygon, 0.0, &simplex);
                let alignment = epa.normal.dot(&Vector3::y()).abs();
                assert!(
                    alignment > 0.9,
                    "EPA normal should align with Y axis, got {:?} (alignment={})",
                    epa.normal,
                    alignment,
                );
            }
            GjkResult::Separated { distance, .. } => {
                panic!(
                    "Expected intersection with small polygon, got separation dist={}",
                    distance,
                );
            }
        }
    }

    // ===== Layer 2: gjk_epa_vs_face =====
    // Tests the per-face contact generation including clipping.

    #[test]
    fn layer2_tetrahedron_vs_face_intersecting() {
        let hull = tetrahedron_hull(1.0);
        let face = flat_face();
        let margin = 0.02;

        use crate::collision::convex_hull::TransformedHull;
        let center = Point3::new(0.0, 0.15, 0.0);
        let rot = UnitQuaternion::identity();
        let transformed = TransformedHull {
            hull: &hull,
            center,
            rotation: rot,
        };

        let contacts = gjk_epa_vs_face(&transformed, &face, margin);

        assert!(
            !contacts.is_empty(),
            "Should produce contacts for penetrating tetrahedron",
        );

        for (i, cp) in contacts.iter().enumerate() {
            let alignment = cp.normal.dot(&Vector3::y());
            assert!(
                alignment > 0.9,
                "Contact {} normal should point up, got {:?} (alignment={})",
                i,
                cp.normal,
                alignment,
            );
            // Contact point should be on or near the y=0 plane.
            assert!(
                cp.point.y.abs() < 0.1,
                "Contact {} point should be near y=0 plane, got y={}",
                i,
                cp.point.y,
            );
        }
    }

    #[test]
    fn layer2_tetrahedron_vs_face_margin_only() {
        let hull = tetrahedron_hull(1.0);
        let face = flat_face();
        let margin = 0.02;

        use crate::collision::convex_hull::TransformedHull;
        // Place tetrahedron just above the plane within margin range.
        // Hull bottom at y_base ≈ -0.204. Center at y=0.22 → bottom at ~0.016.
        let center = Point3::new(0.0, 0.22, 0.0);
        let rot = UnitQuaternion::identity();
        let transformed = TransformedHull {
            hull: &hull,
            center,
            rotation: rot,
        };

        let contacts = gjk_epa_vs_face(&transformed, &face, margin);

        assert!(
            !contacts.is_empty(),
            "Should produce margin contacts for tetrahedron just above plane",
        );

        for (i, cp) in contacts.iter().enumerate() {
            let alignment = cp.normal.dot(&Vector3::y());
            assert!(
                alignment > 0.9,
                "Margin contact {} normal should point up, got {:?} (alignment={})",
                i,
                cp.normal,
                alignment,
            );
        }
    }

    #[test]
    fn layer2_tetrahedron_vs_face_margin_only_single_contact() {
        let hull = tetrahedron_hull(1.0);
        let face = flat_face();
        let margin = 0.02;

        use crate::collision::convex_hull::TransformedHull;
        let transformed = TransformedHull {
            hull: &hull,
            center: Point3::new(0.0, 0.22, 0.0),
            rotation: UnitQuaternion::identity(),
        };

        let contacts = gjk_epa_vs_face(&transformed, &face, margin);
        assert_eq!(
            contacts.len(),
            1,
            "Margin-only contact should collapse to a single point"
        );
        assert!(
            contacts[0].raw_depth <= 0.0,
            "Expected margin-only raw_depth <= 0, got {}",
            contacts[0].raw_depth
        );
    }

    #[test]
    fn layer2_tetrahedron_vs_face_penetrating_allows_multi_contacts() {
        let hull = tetrahedron_hull(1.0);
        let face = flat_face();
        let margin = 0.02;

        use crate::collision::convex_hull::TransformedHull;
        let transformed = TransformedHull {
            hull: &hull,
            center: Point3::new(0.0, 0.15, 0.0),
            rotation: UnitQuaternion::identity(),
        };

        let contacts = gjk_epa_vs_face(&transformed, &face, margin);
        assert!(
            contacts.len() >= 2,
            "Penetrating tetrahedron should keep multi-point manifold candidates, got {}",
            contacts.len(),
        );
        assert!(
            contacts.iter().all(|cp| cp.raw_depth > 0.0),
            "Penetrating contacts should have positive raw depth"
        );
    }

    #[test]
    fn layer2_tetrahedron_vs_face_shallow_penetration_single_contact() {
        let hull = tetrahedron_hull(1.0);
        let face = flat_face();
        let margin = 0.02;

        use crate::collision::convex_hull::TransformedHull;
        // Resting center is about 0.204; this is a tiny positive penetration.
        let transformed = TransformedHull {
            hull: &hull,
            center: Point3::new(0.0, 0.2036, 0.0),
            rotation: UnitQuaternion::identity(),
        };

        let contacts = gjk_epa_vs_face(&transformed, &face, margin);
        assert_eq!(
            contacts.len(),
            1,
            "Shallow penetration should stay single-point near transition",
        );
        assert!(
            contacts[0].raw_depth > 0.0
                && contacts[0].raw_depth <= SHALLOW_PENETRATION_EPSILON + 5e-4,
            "Expected tiny positive raw depth near threshold, got {}",
            contacts[0].raw_depth
        );
    }

    #[test]
    fn layer2_tetrahedron_vs_face_fully_below_plane_rejected() {
        let hull = tetrahedron_hull(1.0);
        let face = flat_face();
        let margin = 0.02;

        use crate::collision::convex_hull::TransformedHull;
        // Top vertex is still below y=0, so the whole shape is behind the face.
        let center = Point3::new(0.0, -1.0, 0.0);
        let transformed = TransformedHull {
            hull: &hull,
            center,
            rotation: UnitQuaternion::identity(),
        };

        let contacts = gjk_epa_vs_face(&transformed, &face, margin);
        assert!(
            contacts.is_empty(),
            "Fully below-plane shape should be rejected by one-sided backface gate",
        );
    }

    #[test]
    fn layer2_tetrahedron_vs_face_partial_crossing_kept() {
        let hull = tetrahedron_hull(1.0);
        let face = flat_face();
        let margin = 0.02;

        use crate::collision::convex_hull::TransformedHull;
        // Center is below y=0, but the top vertex is above the plane.
        let center = Point3::new(0.0, -0.4, 0.0);
        let transformed = TransformedHull {
            hull: &hull,
            center,
            rotation: UnitQuaternion::identity(),
        };

        let contacts = gjk_epa_vs_face(&transformed, &face, margin);
        assert!(
            !contacts.is_empty(),
            "Partially crossing shape should produce contacts even with center below plane",
        );
        for cp in contacts {
            assert!(
                cp.normal.dot(&Vector3::y()) > 0.9,
                "Expected upward terrain normal, got {:?}",
                cp.normal,
            );
        }
    }

    // ===== Layer 3: gjk_patch_manifold (full pipeline) =====
    // Tests the complete manifold generation through the patch API.

    #[test]
    fn layer3_tetrahedron_patch_flat_intersecting() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let hull = Arc::new(tetrahedron_hull(1.0));
        let shape = ColliderShape::ConvexHull { hull };
        let view = ShapeView {
            center: Point3::new(0.0, 0.15, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &patch, margin);

        assert!(
            !result.is_empty(),
            "Should produce contacts for penetrating tetrahedron",
        );

        for (i, cp) in result.points.iter().enumerate() {
            let alignment = cp.normal.dot(&Vector3::y());
            assert!(
                alignment > 0.9,
                "Contact {} normal should point up, got {:?} (alignment={})",
                i,
                cp.normal,
                alignment,
            );
        }
    }

    #[test]
    fn layer3_tetrahedron_patch_flat_resting() {
        let patch = large_flat_patch();
        let margin = 0.02;

        // Resting position: bottom of tetrahedron exactly at y=0.
        // y_base ≈ -0.204, so center at 0.204.
        let hull = Arc::new(tetrahedron_hull(1.0));
        let shape = ColliderShape::ConvexHull { hull };
        let view = ShapeView {
            center: Point3::new(0.0, 0.204, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &patch, margin);

        assert!(
            !result.is_empty(),
            "Should produce margin contacts at resting height",
        );

        for (i, cp) in result.points.iter().enumerate() {
            let alignment = cp.normal.dot(&Vector3::y());
            assert!(
                alignment > 0.9,
                "Resting contact {} normal should point up, got {:?} (alignment={})",
                i,
                cp.normal,
                alignment,
            );
        }
    }

    #[test]
    fn layer3_tetrahedron_patch_rotated() {
        let patch = large_flat_patch();
        let margin = 0.02;

        // Rotate tetrahedron 45° around X — changes which vertex/edge is lowest.
        let hull = Arc::new(tetrahedron_hull(1.0));
        let shape = ColliderShape::ConvexHull { hull };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.785);
        let view = ShapeView {
            center: Point3::new(0.0, 0.3, 0.0),
            rotation: rot,
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &patch, margin);

        assert!(
            !result.is_empty(),
            "Should produce contacts for rotated tetrahedron"
        );

        for (i, cp) in result.points.iter().enumerate() {
            let alignment = cp.normal.dot(&Vector3::y());
            assert!(
                alignment > 0.9,
                "Rotated tetrahedron contact {} normal should point up, got {:?} (alignment={})",
                i,
                cp.normal,
                alignment,
            );
        }
    }

    #[test]
    fn layer3_tetrahedron_patch_near_plane_height_sweep_normal_stable() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let hull = Arc::new(tetrahedron_hull(1.0));
        let shape = ColliderShape::ConvexHull { hull };
        let rotation = UnitQuaternion::identity();

        // Resting center for this tetra is ~0.204 (bottom touches y=0).
        // Sweep tiny offsets across the near-plane band where jitter previously appeared.
        let center_offsets = [-0.012, -0.008, -0.004, 0.0, 0.004, 0.008, 0.012];

        for offset in center_offsets {
            let view = ShapeView {
                center: Point3::new(0.0, 0.204 + offset, 0.0),
                rotation,
                shape: &shape,
            };

            let result = gjk_patch_manifold(&view, &patch, margin);
            assert!(
                !result.is_empty(),
                "Expected contacts in near-plane sweep at offset {}",
                offset,
            );

            for (i, cp) in result.points.iter().enumerate() {
                let alignment = cp.normal.dot(&Vector3::y());
                assert!(
                    alignment > 0.95,
                    "Offset {} contact {} normal should stay upward, got {:?} (alignment={})",
                    offset,
                    i,
                    cp.normal,
                    alignment,
                );
            }
        }
    }

    #[test]
    fn layer3_tetrahedron_patch_rotated_near_plane_height_sweep_normal_stable() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let hull = Arc::new(tetrahedron_hull(1.0));
        let shape = ColliderShape::ConvexHull { hull };
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.785);

        // Find the rotated hull's local minimum Y so we can sweep around resting height.
        let lowest_local_y = {
            let hull_ref = match &shape {
                ColliderShape::ConvexHull { hull } => hull,
                _ => unreachable!(),
            };
            hull_ref
                .vertices
                .iter()
                .map(|v| (rotation * *v).y)
                .fold(f32::INFINITY, f32::min)
        };
        let resting_center_y = -lowest_local_y;
        let center_offsets = [-0.012, -0.008, -0.004, 0.0, 0.004, 0.008, 0.012];

        for offset in center_offsets {
            let view = ShapeView {
                center: Point3::new(0.0, resting_center_y + offset, 0.0),
                rotation,
                shape: &shape,
            };

            let result = gjk_patch_manifold(&view, &patch, margin);
            assert!(
                !result.is_empty(),
                "Expected contacts in rotated near-plane sweep at offset {}",
                offset,
            );

            for (i, cp) in result.points.iter().enumerate() {
                let alignment = cp.normal.dot(&Vector3::y());
                assert!(
                    alignment > 0.95,
                    "Offset {} rotated contact {} normal should stay upward, got {:?} (alignment={})",
                    offset,
                    i,
                    cp.normal,
                    alignment,
                );
            }
        }
    }

    // --- Edge-to-face rotation sweep ---
    // Rotates a tetrahedron from edge-on to face-on contact, checking for
    // phantom contacts at every angle. The tetrahedron sits with one edge
    // touching the ground and rotates about that edge's axis.

    #[test]
    fn layer3_tetrahedron_edge_to_face_rotation_no_phantom_contacts() {
        let patch = large_flat_patch();
        let margin = 0.02;
        let hull_raw = tetrahedron_hull(1.0);

        // The tetrahedron's base edge v1–v2 lies roughly along X.
        // We'll rotate about the X axis so this edge stays on the ground
        // while the shape tilts from edge-on toward face-on.

        // Find the lowest two vertices in the default orientation to determine
        // the resting-on-edge height for each rotation.
        let steps = 90;
        let mut failures: Vec<String> = Vec::new();

        for step in 0..=steps {
            // Sweep from ~0° (base down, face contact) to ~90° (edge contact
            // with body tilted away). Negative X rotation tilts base edge
            // toward the viewer.
            let angle = (step as f32 / steps as f32) * std::f32::consts::FRAC_PI_2;
            let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);

            // Find the lowest Y after rotation to place the shape resting on the plane.
            let lowest_y = hull_raw
                .vertices
                .iter()
                .map(|v| (rot * *v).y)
                .fold(f32::INFINITY, f32::min);
            let center_y = -lowest_y;

            let hull = Arc::new(hull_raw.clone());
            let shape = ColliderShape::ConvexHull { hull };
            let view = ShapeView {
                center: Point3::new(0.0, center_y, 0.0),
                rotation: rot,
                shape: &shape,
            };

            let result = gjk_patch_manifold(&view, &patch, margin);

            for (i, cp) in result.points.iter().enumerate() {
                // All contact normals must point upward.
                let alignment = cp.normal.dot(&Vector3::y());
                if alignment < 0.9 {
                    failures.push(format!(
                        "step={} angle={:.3}rad: contact {} normal {:?} (alignment={:.4})",
                        step, angle, i, cp.normal, alignment,
                    ));
                }

                // Contact points must be on or very near the y=0 plane.
                if cp.point.y.abs() > 0.1 {
                    failures.push(format!(
                        "step={} angle={:.3}rad: contact {} point y={:.4} (should be near 0)",
                        step, angle, i, cp.point.y,
                    ));
                }

                // Depth must be reasonable (not a phantom deep contact).
                if cp.raw_depth > 0.1 {
                    failures.push(format!(
                        "step={} angle={:.3}rad: contact {} raw_depth={:.4} (phantom?)",
                        step, angle, i, cp.raw_depth,
                    ));
                }
            }
        }

        assert!(
            failures.is_empty(),
            "Phantom contacts during edge-to-face rotation:\n{}",
            failures.join("\n"),
        );
    }

    #[test]
    fn layer3_tetrahedron_edge_to_face_rotation_z_axis_no_phantom() {
        let patch = large_flat_patch();
        let margin = 0.02;
        let hull_raw = tetrahedron_hull(1.0);

        let steps = 90;
        let mut failures: Vec<String> = Vec::new();

        for step in 0..=steps {
            let angle = (step as f32 / steps as f32) * std::f32::consts::FRAC_PI_2;
            let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle);

            let lowest_y = hull_raw
                .vertices
                .iter()
                .map(|v| (rot * *v).y)
                .fold(f32::INFINITY, f32::min);
            let center_y = -lowest_y;

            let hull = Arc::new(hull_raw.clone());
            let shape = ColliderShape::ConvexHull { hull };
            let view = ShapeView {
                center: Point3::new(0.0, center_y, 0.0),
                rotation: rot,
                shape: &shape,
            };

            let result = gjk_patch_manifold(&view, &patch, margin);

            for (i, cp) in result.points.iter().enumerate() {
                if cp.normal.dot(&Vector3::y()) < 0.9 {
                    failures.push(format!(
                        "step={} angle={:.3}rad: contact {} normal {:?}",
                        step, angle, i, cp.normal,
                    ));
                }
                if cp.point.y.abs() > 0.1 {
                    failures.push(format!(
                        "step={} angle={:.3}rad: contact {} point y={:.4}",
                        step, angle, i, cp.point.y,
                    ));
                }
                if cp.raw_depth > 0.1 {
                    failures.push(format!(
                        "step={} angle={:.3}rad: contact {} raw_depth={:.4}",
                        step, angle, i, cp.raw_depth,
                    ));
                }
            }
        }

        assert!(
            failures.is_empty(),
            "Phantom contacts during Z-axis rotation:\n{}",
            failures.join("\n"),
        );
    }

    /// Same sweep but with the tetrahedron slightly above the plane (within
    /// margin), simulating the "almost resting" case where phantoms appear.
    #[test]
    fn layer3_tetrahedron_edge_to_face_rotation_margin_hover_no_phantom() {
        let patch = large_flat_patch();
        let margin = 0.02;
        let hull_raw = tetrahedron_hull(1.0);

        let steps = 90;
        let mut failures: Vec<String> = Vec::new();

        for step in 0..=steps {
            let angle = (step as f32 / steps as f32) * std::f32::consts::FRAC_PI_2;
            let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);

            let lowest_y = hull_raw
                .vertices
                .iter()
                .map(|v| (rot * *v).y)
                .fold(f32::INFINITY, f32::min);
            // Hover 0.01 above the plane (within margin).
            let center_y = -lowest_y + 0.01;

            let hull = Arc::new(hull_raw.clone());
            let shape = ColliderShape::ConvexHull { hull };
            let view = ShapeView {
                center: Point3::new(0.0, center_y, 0.0),
                rotation: rot,
                shape: &shape,
            };

            let result = gjk_patch_manifold(&view, &patch, margin);

            for (i, cp) in result.points.iter().enumerate() {
                if cp.normal.dot(&Vector3::y()) < 0.9 {
                    failures.push(format!(
                        "hover step={} angle={:.3}rad: contact {} normal {:?}",
                        step, angle, i, cp.normal,
                    ));
                }
                if cp.point.y.abs() > 0.1 {
                    failures.push(format!(
                        "hover step={} angle={:.3}rad: contact {} point y={:.4}",
                        step, angle, i, cp.point.y,
                    ));
                }
                // For margin-hover, any positive raw_depth is suspect.
                if cp.raw_depth > margin + 0.01 {
                    failures.push(format!(
                        "hover step={} angle={:.3}rad: contact {} raw_depth={:.4} (phantom?)",
                        step, angle, i, cp.raw_depth,
                    ));
                }
            }
        }

        assert!(
            failures.is_empty(),
            "Phantom contacts during margin-hover rotation:\n{}",
            failures.join("\n"),
        );
    }

    // --- No contact ---

    #[test]
    fn shape_above_patch_no_contact() {
        let patch = large_flat_patch();
        let shape = ColliderShape::Box {
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let view = ShapeView {
            center: Point3::new(0.0, 5.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &patch, 0.02);
        assert!(result.is_empty(), "Should have no contacts when far above");
    }

    // --- Contact count limit ---

    #[test]
    fn manifold_reduced_to_four() {
        let patch = large_flat_patch();
        let margin = 0.02;

        let he = Vector3::new(0.5, 0.5, 0.5);
        let center = Point3::new(0.0, 0.5, 0.0);
        let rot = UnitQuaternion::identity();
        let shape = ColliderShape::Box { half_extents: he };
        let view = ShapeView {
            center,
            rotation: rot,
            shape: &shape,
        };

        let result = gjk_patch_manifold(&view, &patch, margin);

        assert!(
            result.len() <= MAX_MANIFOLD_POINTS,
            "Should be reduced to at most {}, got {}",
            MAX_MANIFOLD_POINTS,
            result.len(),
        );
    }
}
