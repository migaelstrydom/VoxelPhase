//! Which of a patch's faces may push a shape, when the shape's centre has
//! crossed some of them.
//!
//! A mesh has no inside, so a face whose plane a shape's centre has crossed
//! is ambiguous: the shape may be a little too deep in the ground, or most of
//! the way through a thin wall and looking at its far side. Such a face still
//! pushes while the centre is within the shape's own reach behind it, unless
//! a face it stands back to back with would push the shape out a shorter way
//! — so a shape always leaves solid on the side its centre is on:
//!
//! ```text
//!     back to back: one pushes          face to face: both push
//!       ← │▓▓▓▓│ →                        ▓▓▓│ →    ← │▓▓▓
//!    ┌────┼──┐ │                          ▓▓▓│ ┌────┐ │▓▓▓
//!    │  ● │  │ │   centre left of the     ▓▓▓│ │ ●  │ │▓▓▓
//!    └────┼──┘ │   midline: push ←        ▓▓▓│ └────┘ │▓▓▓
//! ```
//!
//! Every mesh contact path asks this once per patch and skips the faces it
//! rules out; faces the centre is in front of are never ruled out. Nor is a
//! face further behind the centre than the shape reaches ever let in, even
//! by a path whose own test allows a contact margin past it: that face is the
//! far side of something the shape is already through, and a sphere resting
//! against a wall thinner than the margin was pushed into it by one. The only
//! thing a path supplies about its shape is how far it reaches along a
//! normal.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::mesh::seam_filter::ContactFace;

/// Tolerance for calling a point behind a plane.
const BEHIND_EPSILON: f32 = 1e-4;

/// For each of `faces`, whether it may push a shape centred at `centre`.
///
/// `reach(n)` is how far the shape extends from its centre in the direction
/// `-n`: towards a face whose outward normal is `n`.
pub fn pushing_faces(
    faces: &[ContactFace],
    centre: Point3<f32>,
    reach: impl Fn(&Vector3<f32>) -> f32,
    margin: f32,
) -> SmallVec<[bool; 16]> {
    // Nothing is ruled out while the centre is in front of every face, and
    // that is nearly always: settle it without asking the shape its reach.
    let crossed_any = faces.iter().any(|face| {
        face.vertices.len() >= 3 && (centre - face.vertices[0]).dot(&face.normal) < -BEHIND_EPSILON
    });
    if !crossed_any {
        return SmallVec::from_elem(true, faces.len());
    }

    let sides: SmallVec<[FaceSide; 16]> = faces
        .iter()
        .map(|face| FaceSide::of(face, centre, &reach, margin))
        .collect();
    let candidates: SmallVec<[Option<Candidate>; 16]> = sides
        .iter()
        .map(|side| match side {
            FaceSide::Within(candidate) => Some(*candidate),
            _ => None,
        })
        .collect();

    (0..faces.len())
        .map(|index| match &sides[index] {
            FaceSide::Through => false,
            FaceSide::Apart => true,
            FaceSide::Within(candidate) => {
                !candidate.centre_behind || !is_outpushed(index, faces, &candidates)
            }
        })
        .collect()
}

/// Where a shape stands relative to one face.
enum FaceSide {
    /// Within reach of the face's plane, on either side of it.
    Within(Candidate),
    /// Further behind the plane than the shape reaches.
    Through,
    /// In front of the plane and clear of it.
    Apart,
}

impl FaceSide {
    fn of(
        face: &ContactFace,
        centre: Point3<f32>,
        reach: &impl Fn(&Vector3<f32>) -> f32,
        margin: f32,
    ) -> Self {
        if face.vertices.len() < 3 {
            return Self::Apart;
        }
        let signed_dist = (centre - face.vertices[0]).dot(&face.normal);
        let reach = reach(&face.normal);
        let push = reach - signed_dist;
        if signed_dist < -reach - BEHIND_EPSILON {
            Self::Through
        } else if push < -margin {
            Self::Apart
        } else {
            Self::Within(Candidate {
                push,
                centre_behind: signed_dist < -BEHIND_EPSILON,
            })
        }
    }
}

/// A face the shape is within reach of.
#[derive(Clone, Copy)]
struct Candidate {
    /// How far the shape must move along the face normal to clear its plane.
    push: f32,
    /// Whether the shape's centre is behind the face's plane.
    centre_behind: bool,
}

/// Whether another candidate face, back to back with this one, would push
/// the shape out of the solid between them a shorter way.
///
/// Back to back means each face lies behind the other's plane: the two sides
/// of something solid, which the shape can leave by only one of. Faces that
/// face each other bound a gap instead, and both push. Pushes less than a
/// right angle apart never oppose, so they are summed as ever: the two slopes
/// of a roof both lift what rests across its ridge.
///
/// Equal pushes — the centre exactly on the midline — go to the lower index,
/// so one of the two always survives.
fn is_outpushed(index: usize, faces: &[ContactFace], candidates: &[Option<Candidate>]) -> bool {
    let (face, candidate) = (&faces[index], candidates[index].as_ref());
    let Some(candidate) = candidate else {
        return false;
    };
    candidates.iter().enumerate().any(|(other_index, other)| {
        let Some(other) = other else {
            return false;
        };
        let other_face = &faces[other_index];
        other_index != index
            && face.normal.dot(&other_face.normal) < 0.0
            && is_behind(face, other_face)
            && is_behind(other_face, face)
            && (other.push < candidate.push
                || (other.push == candidate.push && other_index < index))
    })
}

/// Whether `face`'s centroid lies on or behind `plane_of`'s plane.
fn is_behind(face: &ContactFace, plane_of: &ContactFace) -> bool {
    (centroid(face) - plane_of.vertices[0]).dot(&plane_of.normal) <= BEHIND_EPSILON
}

fn centroid(face: &ContactFace) -> Point3<f32> {
    let sum = face
        .vertices
        .iter()
        .fold(Vector3::zeros(), |acc, v| acc + v.coords);
    Point3::from(sum / face.vertices.len() as f32)
}
