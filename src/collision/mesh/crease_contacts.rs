//! Contacts from convex crease edges that run into a faceted shape through a
//! face neither of the crease's own faces would clip against.
//!
//! A face contact clips the shape's face most aligned with the terrain face.
//! On a ridge steeper than 45°, that is a side of a box for both slopes,
//! while the apex comes up through its bottom: neither slope's contact sees
//! it, and the box sank onto the ridge untouched. The crease is pushed out of
//! the shape along whichever of the shape's face normals clears it soonest,
//! of those within the crease's outward cone — so a ridge lifts what rests
//! across it.
//!
//! A crease edge is one piece of a crease that runs on past it, so only a
//! push across the crease clears it: sliding along it meets the next piece.
//! Each face normal is taken square to the edge before it is measured, or a
//! face tilted along a pillar's short vertical edges would clear each piece
//! soonest, and walk the shape up the pillar.
//!
//! ```text
//!         ┌───────┐
//!         │   ●   │      push ↑ : clears the apex soonest,
//!         │   ▲   │               and lies between both slopes' normals
//!         └──╱─╲──┘
//!           ╱   ╲
//! ```
//!
//! Round shapes do not need it: their edge contacts find the apex directly.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::ContactPoint;
use crate::collision::mesh::crease_edges::{convex_creases, ConvexCrease};
use crate::collision::mesh::seam_filter::FilteredPatch;
use crate::collision::obb::Obb;

/// Shortest a face normal, taken square to a crease, may be and still give a
/// direction: shorter, the face is all but square to the crease itself.
const SQUARE_EPSILON: f32 = 1e-3;

/// One face of a convex shape, as a plane: inside is `normal · x ≤ offset`.
#[derive(Debug, Clone, Copy)]
pub struct SolidPlane {
    /// Outward unit normal, world space.
    pub normal: Vector3<f32>,
    /// `normal · x` for every point on the face.
    pub offset: f32,
}

impl SolidPlane {
    /// The plane through `point` facing `normal`.
    pub fn through(point: Point3<f32>, normal: Vector3<f32>) -> Self {
        Self {
            normal,
            offset: normal.dot(&point.coords),
        }
    }

    /// The six faces of an OBB.
    pub fn of_obb(obb: &Obb) -> SmallVec<[SolidPlane; 8]> {
        let half = [obb.half_extents.x, obb.half_extents.y, obb.half_extents.z];
        let mut planes = SmallVec::new();
        for (axis, extent) in obb.axes().iter().zip(half) {
            for normal in [*axis, -*axis] {
                planes.push(Self::through(obb.center + normal * extent, normal));
            }
        }
        planes
    }
}

/// Add crease-edge contacts for a convex shape to `out`: the shape given as
/// its face `planes`, and `extent(u)`, the furthest `u · x` over its points.
pub fn crease_edge_contacts(
    planes: &[SolidPlane],
    extent: impl Fn(&Vector3<f32>) -> f32,
    patch: &FilteredPatch,
    margin: f32,
    out: &mut SmallVec<[ContactPoint; 4]>,
) {
    if planes.is_empty() {
        return;
    }
    for ConvexCrease { edge, normal_b } in convex_creases(patch) {
        let Some(clipped) = clip_segment(planes, edge.a, edge.b, margin) else {
            continue;
        };
        let crease = Crease {
            point: edge.a,
            along: (edge.b - edge.a).normalize(),
            normal_a: edge.normal_a,
            normal_b,
        };
        let Some(push) = shortest_push(planes, &extent, &crease) else {
            continue;
        };
        if push.plane == support_plane(planes, &edge.normal_a)
            || push.plane == support_plane(planes, &normal_b)
        {
            continue;
        }

        let depth = push.distance;
        if depth < -margin {
            continue;
        }
        for (vertex, point) in clipped.iter().enumerate() {
            out.push(
                ContactPoint::new(
                    *point,
                    -push.outward,
                    depth,
                    edge.feature_id.with_vertex(vertex as u32),
                )
                .on(edge.surface),
            );
        }
    }
}

/// A convex crease edge, as a line: the crease runs on past the edge's ends.
struct Crease {
    /// A point on the crease.
    point: Point3<f32>,
    /// Unit direction the crease runs in.
    along: Vector3<f32>,
    /// Outward normal of the face on one side.
    normal_a: Vector3<f32>,
    /// Outward normal of the face on the other.
    normal_b: Vector3<f32>,
}

/// The move that pushes a crease out of a shape.
struct CreasePush {
    /// The shape face the push is taken from.
    plane: usize,
    /// That face's normal, square to the crease: the shape moves along its
    /// negation.
    outward: Vector3<f32>,
    /// How far the shape must move for the crease to clear it.
    distance: f32,
}

/// The face, among those whose normal faces into both of the crease's faces,
/// that clears the crease in the shortest move across it.
fn shortest_push(
    planes: &[SolidPlane],
    extent: &impl Fn(&Vector3<f32>) -> f32,
    crease: &Crease,
) -> Option<CreasePush> {
    planes
        .iter()
        .enumerate()
        .filter_map(|(index, plane)| {
            let across = plane.normal - crease.along * plane.normal.dot(&crease.along);
            let outward = across.try_normalize(SQUARE_EPSILON)?;
            let faces_in =
                outward.dot(&crease.normal_a) < 0.0 && outward.dot(&crease.normal_b) < 0.0;
            faces_in.then(|| CreasePush {
                plane: index,
                outward,
                distance: extent(&outward) - outward.dot(&crease.point.coords),
            })
        })
        .min_by(|a, b| a.distance.total_cmp(&b.distance))
}

/// The face a face contact against `normal` clips: the one facing most
/// directly into it.
fn support_plane(planes: &[SolidPlane], normal: &Vector3<f32>) -> usize {
    planes
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            a.normal
                .dot(normal)
                .partial_cmp(&b.normal.dot(normal))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(index, _)| index)
        .unwrap_or(0)
}

/// The part of segment `a`–`b` inside the shape, grown by `margin`, if any.
fn clip_segment(
    planes: &[SolidPlane],
    a: Point3<f32>,
    b: Point3<f32>,
    margin: f32,
) -> Option<[Point3<f32>; 2]> {
    let (mut t0, mut t1) = (0.0f32, 1.0f32);
    for plane in planes {
        // Signed distance outside the grown plane at each end.
        let start = plane.normal.dot(&a.coords) - plane.offset - margin;
        let delta = plane.normal.dot(&(b - a));
        if delta.abs() < 1e-9 {
            if start > 0.0 {
                return None;
            }
            continue;
        }
        let t = -start / delta;
        if delta > 0.0 {
            t1 = t1.min(t);
        } else {
            t0 = t0.max(t);
        }
        if t0 > t1 {
            return None;
        }
    }
    Some([a + (b - a) * t0, a + (b - a) * t1])
}
