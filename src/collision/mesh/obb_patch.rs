//! OBB vs FilteredPatch manifold generation.
//!
//! Which faces may push the OBB when its centre has crossed some of them is
//! [`solid_side`](super::solid_side)'s call; a ridge coming up through a face
//! no face contact clips is [`crease_contacts`](super::crease_contacts)'s.
//!
//! For each merged face in the filtered patch, tests OBB overlap via
//! half-extent projection. For the face with deepest penetration, clips
//! the OBB's support face against the merged polygon and projects the
//! clipped points onto the contact plane. Produces a multi-point manifold
//! directly, replacing the old per-triangle + coplanar_stabilizer approach.
//!
//! A face is pushed out along whichever axis clears it soonest: its own
//! normal, or the normal of one of the box's faces it comes up through. A
//! slab lying across a crater's lip touches the lip's steep walls only at
//! their top edge; the walls' planes run metres into the slab, while the
//! edge is in by millimetres. Pushed along the walls' normals the slab
//! was shoved sideways every frame and walked off across flat ground:
//!
//! ```text
//!     ┌──────────────────────────┐   slab
//!     └────────────▲─────────────┘
//!     ▓▓▓▓▓▓▓▓▓▓▓▓╱ ╲             ↑ : the slab's own face clears the lip
//!     ▓▓▓▓▓▓▓▓▓▓▓╱   ╲            ↖ : the wall's normal, metres deep
//! ```

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::{ContactManifold, ContactPoint};
use crate::collision::contact_reducer::ContactReducer;
use crate::collision::discrete::clipping::{clip_polygon, obb_face, ClipPolygon, ObbFace};
use crate::collision::mesh::crease_contacts::{crease_edge_contacts, SolidPlane};
use crate::collision::mesh::crease_edges::{ridges_of, Ridge};
use crate::collision::mesh::seam_filter::{ContactFace, FilteredPatch};
use crate::collision::mesh::solid_side::pushing_faces;
use crate::collision::obb::Obb;
use crate::collision::segment::segment_segment_closest_points;

/// Maximum contacts in the final manifold.
const MAX_MANIFOLD_POINTS: usize = 4;
/// Small tolerance for rejecting backfacing mesh contacts.
const BACKFACE_EPSILON: f32 = 1e-4;

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

    let pushing = pushing_faces(
        &patch.faces,
        obb.center,
        |normal| obb.project_half_extent(normal),
        contact_margin,
    );

    // Collect contacts from ALL overlapping faces (individual triangles).
    // Each triangle is always convex so clipping is always correct.
    for (face, _) in patch.faces.iter().zip(&pushing).filter(|(_, &p)| p) {
        if face.vertices.len() < 3 {
            continue;
        }
        let Some(overlap) = test_obb_face_overlap(obb, face, contact_margin) else {
            continue;
        };

        let normal = overlap.normal;
        let ridges: SmallVec<[Ridge; 3]> = ridges_of(patch, face).collect();
        if let Some(own) = OwnFacePush::shortest(obb, face, &ridges) {
            if own.depth + OWN_AXIS_PREFERENCE < overlap.depth {
                own.contacts_from_face(face, contact_margin, &mut all_points);
                continue;
            }
        }
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

    crease_edge_contacts(
        &SolidPlane::of_obb(obb),
        |u| obb.center.coords.dot(u) + obb.project_half_extent(u),
        patch,
        contact_margin,
        &mut all_points,
    );

    // No face or crease contacts — try boundary edges as a fallback.
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
    depth: f32,
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

    Some(FaceOverlap { normal, depth })
}

/// How much sooner, in metres, one of the OBB's own faces must clear a mesh
/// face than the mesh face's normal does to be the axis it is pushed along.
/// Where the two agree, as for a box lying flat on flat ground, the mesh
/// face's normal is kept.
const OWN_AXIS_PREFERENCE: f32 = 1.0e-3;

/// How far, as a cosine, a push may lean into a face it leaves along a ridge
/// of: the ridge between a vertical wall and the ground above it lets a box
/// lying across it be lifted straight up.
const RIDGE_TOLERANCE: f32 = 1.0e-3;

/// The OBB face most aligned with a mesh face's `-normal`, for clipping.
struct SupportFace {
    vertices: [Point3<f32>; 4],
}

/// Pushing the OBB out of a mesh face along the inward normal of one of its
/// own faces.
struct OwnFacePush {
    /// The OBB face the mesh face comes up through.
    face: ObbFace,
    /// How far the furthest of the mesh face and its ridges is inside that
    /// face's plane.
    depth: f32,
}

impl OwnFacePush {
    /// The shortest such push, among those that leave `face` over one of
    /// its `ridges` into neither face either side of it, and clear the
    /// whole straight run of every ridge it has.
    ///
    /// Only a ridge offers one. Where a face meets its neighbours flush, a
    /// push along a box face clears it only by sliding the box off it onto
    /// the next one: a slab resting on flat ground met a wall at its edge.
    /// A push must clear the ridges' runs as well as the face: a thin
    /// pillar's upright edges come in short pieces, and a box skewered on one
    /// climbed it a piece at a time.
    fn shortest(obb: &Obb, face: &ContactFace, ridges: &[Ridge]) -> Option<Self> {
        let leaves = |push: Vector3<f32>| {
            push.dot(&face.normal) >= -RIDGE_TOLERANCE
                && ridges
                    .iter()
                    .any(|ridge| push.dot(&ridge.normal_b) >= -RIDGE_TOLERANCE)
        };
        let obstacles = || {
            face.vertices
                .iter()
                .chain(ridges.iter().flat_map(|ridge| &ridge.run))
        };
        (0..3)
            .flat_map(|axis| [1.0, -1.0].map(|sign| obb_face(obb, axis, sign)))
            .filter(|own| leaves(-own.normal))
            .map(|own| {
                let depth = obstacles()
                    .map(|v| -(v - own.center).dot(&own.normal))
                    .fold(f32::NEG_INFINITY, f32::max);
                Self { face: own, depth }
            })
            .min_by(|a, b| a.depth.total_cmp(&b.depth))
    }

    /// Contacts for this push: the part of `face` within the OBB face's
    /// outline, at each corner that comes within `margin` of it.
    fn contacts_from_face(
        &self,
        face: &ContactFace,
        margin: f32,
        out: &mut SmallVec<[ContactPoint; 4]>,
    ) {
        let own = &self.face;
        let mut inside = ClipPolygon::from_slice(&face.vertices);
        for (tangent, half) in [(own.tangent_u, own.half_u), (own.tangent_v, own.half_v)] {
            for side in [1.0, -1.0] {
                inside = clip_polygon(
                    &inside,
                    own.center + tangent * (half * side),
                    -tangent * side,
                );
            }
        }
        for (vertex, point) in inside.iter().enumerate() {
            let depth = -(point - own.center).dot(&own.normal);
            if depth < -margin {
                continue;
            }
            out.push(
                ContactPoint::new(
                    *point,
                    -own.normal,
                    depth,
                    face.feature_id.with_vertex(vertex as u32),
                )
                .on(face.surface),
            );
        }
    }
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

    SupportFace {
        vertices: obb_face(obb, best_axis, sign).vertices,
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
    use crate::collision::mesh::seam_filter::{filter_patch, ContactEdge, FilteredPatch};
    use crate::collision::mesh_patch::{MeshPatch, PatchTriangle};
    use crate::collision::{SurfaceId, Triangle};
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

    /// A wall of a crater's lip, `steep` from the horizontal, with its top
    /// edge just inside the bottom of a wide slab lying over the lip, where
    /// it meets the flat ground: a ridge.
    fn crater_wall_under(slab_bottom: f32, steep_degrees: f32) -> FilteredPatch {
        let lip = slab_bottom + 0.005;
        let (sin, cos) = steep_degrees.to_radians().sin_cos();
        let wall = Vector3::new(sin, cos, 0.0);
        let (near, far) = (Point3::new(0.0, lip, -0.5), Point3::new(0.0, lip, 0.5));
        FilteredPatch {
            faces: SmallVec::from_vec(vec![
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(-1.0, lip, -0.5),
                        Point3::new(-1.0, lip, 0.5),
                        far,
                        near,
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(0),
                    surface: SurfaceId::UNSPECIFIED,
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        near,
                        far,
                        Point3::new(0.5 * cos, lip - 0.5 * sin, 0.0),
                    ]),
                    normal: wall,
                    feature_id: FeatureId::from_face(1),
                    surface: SurfaceId::UNSPECIFIED,
                },
            ]),
            boundary_edges: SmallVec::from_vec(vec![ContactEdge {
                a: near,
                b: far,
                feature_id: FeatureId::from_face(1),
                normal_a: wall,
                normal_b: Some(Vector3::y()),
                surface: SurfaceId::UNSPECIFIED,
            }]),
        }
    }

    #[test]
    fn a_slab_across_a_crater_lip_is_pushed_straight_up() {
        let slab = Obb::new(
            Point3::new(-3.0, 0.2, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(9.0, 0.2, 9.0),
        );
        for steep in [33.0, 60.0, 85.0, 90.5, 120.0] {
            let m = obb_patch_manifold(&slab, &crater_wall_under(0.0, steep), 0.02);

            assert!(!m.is_empty(), "the lip is 5 mm into the slab");
            for c in &m.points {
                assert!(
                    c.normal.y > 0.999,
                    "a {steep}° wall pushed along {:?}, not up",
                    c.normal
                );
                assert!(
                    (c.raw_depth - 0.005).abs() < 1e-4,
                    "a {steep}° wall's depth {} is not the lip's 5 mm",
                    c.raw_depth
                );
            }
        }
    }

    /// Where faces meet flush there is no ridge to leave by: a slab sunk a
    /// little into flat ground is pushed up, never along the ground towards
    /// a triangle just past its edge.
    #[test]
    fn a_slab_on_flat_ground_meets_no_wall_at_its_edge() {
        let slab = Obb::new(
            Point3::new(0.0, 0.195, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(2.0, 0.2, 2.0),
        );
        let past_edge = FilteredPatch {
            faces: SmallVec::from_elem(
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, -0.5),
                        Point3::new(2.5, 0.0, 0.0),
                    ]),
                    // A hair off level, as meshed ground is: enough to turn
                    // the slab's side towards it.
                    normal: Vector3::new(-1.0e-4, 1.0, 0.0).normalize(),
                    feature_id: FeatureId::from_face(0),
                    surface: SurfaceId::UNSPECIFIED,
                },
                1,
            ),
            boundary_edges: SmallVec::new(),
        };
        for c in &obb_patch_manifold(&slab, &past_edge, 0.02).points {
            assert!(c.normal.y > 0.999, "pushed along {:?}, not up", c.normal);
        }
    }

    /// A crater's radial crease: a short ridge sloping 16° down from the
    /// rim, on the crater's wall, the pair of faces a crater dug beside a
    /// temple step left under it. The rim is at the origin.
    fn radial_crease() -> FilteredPatch {
        let top = Point3::origin();
        let low = Point3::new(-0.454, -0.129, 0.0);
        let triangles = [
            (
                Triangle::new(low, Point3::new(-0.401, -0.5, 0.5), top),
                [None, None, Some(1)],
            ),
            (
                Triangle::new(Point3::new(-0.454, 0.0, -0.454), low, top),
                [None, Some(0), None],
            ),
        ];
        let patch = MeshPatch {
            triangles: triangles
                .into_iter()
                .map(|(triangle, neighbors)| PatchTriangle {
                    triangle,
                    neighbors,
                    surface: SurfaceId::UNSPECIFIED,
                })
                .collect(),
        };
        filter_patch(&patch, 0.98)
    }

    /// The crease ends at the rim, so the step lying over it clears it by
    /// rising. Taken as a line running on up the slope, it ran up into the
    /// step, and only a push 3 m along the step's length cleared it.
    #[test]
    fn a_slab_over_a_short_sloping_crease_is_pushed_up() {
        for (into_slab, case) in [(0.006, "into the slab"), (-0.009, "just under it")] {
            let step = Obb::new(
                Point3::new(8.5, 0.167 - into_slab, 10.8),
                UnitQuaternion::identity(),
                Vector3::new(9.4, 0.167, 14.0),
            );
            let m = obb_patch_manifold(&step, &radial_crease(), 0.02);

            assert!(
                !m.is_empty(),
                "the crease's top is {case}, within the margin"
            );
            for c in &m.points {
                assert!(
                    c.normal.y > 0.999,
                    "{case}: pushed along {:?}, not up",
                    c.normal
                );
                assert!(c.raw_depth < 0.01, "{case}: pushed {} m", c.raw_depth);
            }
        }
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
    /// a face it was generated from, pushing along that face's normal or one
    /// of the box's own, and stepping back along the normal by the reported
    /// depth lands on the box. A depth measured against a face's infinite
    /// plane, beyond its polygon, fails one or the other.
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
            let own_axis = obb
                .axes()
                .iter()
                .any(|axis| axis.dot(&normal).abs() > 0.999);
            let on_a_face = patch.faces.iter().any(|face| {
                (own_axis || face.normal.dot(&normal) > 0.999)
                    && (cp.point - face.vertices[0]).dot(&face.normal).abs() < TOLERANCE
                    && point_in_convex_polygon(&cp.point, &face.vertices, &face.normal)
            });
            assert!(
                on_a_face,
                "contact {i} at {:?} lies outside every face, or pushes along neither its face's normal nor the box's",
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
