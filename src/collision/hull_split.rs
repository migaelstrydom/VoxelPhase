//! Cutting a convex hull in two with a plane.
//!
//! ```text
//!        ╱                      ╱
//!   ┌───╱──┐              ┌───╱  ┌──┐
//!   │  ╱   │   plane      │  ╱   │  │   two hulls, each convex,
//!   │ ╱    │  ────────▶   │ ╱    │  │   together the original
//!   └╱─────┘              └╱     └──┘
//!    ╱                     ╱
//! ```
//!
//! A convex body cut by a plane gives two convex bodies, which is the whole
//! reason a fracture made of plane cuts is worth having: every piece is a
//! shape the collision engine takes as it is, with no hulling step and no
//! risk of a concave chunk.
//!
//! The work is bookkeeping rather than mathematics. Each of the hull's faces
//! is a polygon that the plane cuts into at most two polygons, and the pair
//! of points where it cut becomes one edge of the new face that caps each
//! half. The cap's edges arrive in no particular order, so they are sorted by
//! angle about the cap's own centre — cheaper than chaining them end to end
//! and, on a convex hull, the same answer.
//!
//! [`ConvexHull::new`] panics on a hull it cannot use, so everything built
//! here is checked first and a cut that would produce a wafer, a sliver or
//! too many vertices is refused instead.

use nalgebra::Vector3;
use smallvec::SmallVec;

use super::convex_hull::{ConvexHull, HullFace, MAX_HULL_VERTICES};

/// Distance from the plane within which a vertex counts as on it.
///
/// A vertex a hair off the plane makes a wafer-thin sliver of a face, which
/// the hull builder rejects as degenerate; snapping it to the plane makes the
/// same cut without the sliver.
const ON_PLANE: f32 = 1e-5;

/// Vertices closer together than this are the same vertex.
const COINCIDENT: f32 = 1e-5;

/// Thinnest an accepted piece may be, as a share of its longest dimension.
/// Above the hull builder's own 1% limit, so a piece that passes here is one
/// it will take and one the solver can hold.
const MIN_THICKNESS: f32 = 0.04;

/// An oriented plane: the points `x` with `normal · x = offset`.
#[derive(Debug, Clone, Copy)]
pub struct Plane {
    /// Unit normal. The half the normal points into is the `front`.
    pub normal: Vector3<f32>,
    /// Signed distance from the origin along the normal.
    pub offset: f32,
}

impl Plane {
    /// The plane with unit `normal` passing through `point`.
    pub fn through(point: Vector3<f32>, normal: Vector3<f32>) -> Self {
        Self {
            normal,
            offset: normal.dot(&point),
        }
    }

    /// How far `point` is in front of the plane.
    pub fn distance(&self, point: Vector3<f32>) -> f32 {
        self.normal.dot(&point) - self.offset
    }
}

/// The two halves of a hull cut by a plane: the one the plane's normal points
/// into, and the one behind it.
#[derive(Debug)]
pub struct HullHalves {
    pub front: ConvexHull,
    pub back: ConvexHull,
}

/// Cut `hull` in two along `plane`.
///
/// `None` if the plane misses the hull, or if either half would be a piece
/// the physics engine cannot hold: too thin for its length, too few faces, or
/// past the hull vertex limit. A refusal is the caller's cue to move the
/// plane, not an error.
pub fn split_hull(hull: &ConvexHull, plane: Plane) -> Option<HullHalves> {
    let distances: Vec<f32> = hull
        .vertices
        .iter()
        .map(|v| {
            let d = plane.distance(*v);
            if d.abs() <= ON_PLANE {
                0.0
            } else {
                d
            }
        })
        .collect();

    // A plane that leaves every vertex on one side has not cut anything, and
    // one that only grazes a face gives a half with no interior.
    if !distances.iter().any(|d| *d > 0.0) || !distances.iter().any(|d| *d < 0.0) {
        return None;
    }

    let mut front = HalfBuilder::default();
    let mut back = HalfBuilder::default();

    for face in &hull.faces {
        let loop_of = |side: Side| -> SmallVec<[Vector3<f32>; 8]> {
            let mut kept = SmallVec::new();
            let count = face.vertex_indices.len();
            for k in 0..count {
                let i = face.vertex_indices[k] as usize;
                let j = face.vertex_indices[(k + 1) % count] as usize;
                let (di, dj) = (distances[i], distances[j]);
                if side.keeps(di) {
                    kept.push(hull.vertices[i]);
                }
                // An edge that crosses the plane contributes the crossing
                // point to both halves. An edge that only touches it does
                // not: its endpoint is on the plane and already kept.
                if di * dj < 0.0 {
                    let t = di / (di - dj);
                    kept.push(hull.vertices[i] + (hull.vertices[j] - hull.vertices[i]) * t);
                }
            }
            kept
        };
        front.add_face(loop_of(Side::Front), &plane);
        back.add_face(loop_of(Side::Back), &plane);
    }

    Some(HullHalves {
        front: front.build(plane.normal)?,
        back: back.build(-plane.normal)?,
    })
}

/// Which half of the cut a builder is collecting.
#[derive(Clone, Copy)]
enum Side {
    Front,
    Back,
}

impl Side {
    /// Whether a vertex at signed distance `d` belongs to this half. A vertex
    /// exactly on the plane belongs to both: it is a corner of the cap.
    fn keeps(self, d: f32) -> bool {
        match self {
            Side::Front => d >= 0.0,
            Side::Back => d <= 0.0,
        }
    }
}

/// Collects the faces of one half, deduplicating their vertices as it goes.
#[derive(Default)]
struct HalfBuilder {
    vertices: Vec<Vector3<f32>>,
    faces: Vec<HullFace>,
    /// The vertices that landed on the cutting plane, which together are the
    /// face that caps this half.
    cap: Vec<u16>,
}

impl HalfBuilder {
    /// Add one clipped face, given as its corners in loop order. A face left
    /// with fewer than three distinct corners was cut away entirely.
    fn add_face(&mut self, corners: SmallVec<[Vector3<f32>; 8]>, plane: &Plane) {
        let mut indices: SmallVec<[u16; 6]> = SmallVec::new();
        for corner in corners {
            let index = self.intern(corner);
            if !indices.contains(&index) {
                indices.push(index);
            }
            if plane.distance(corner).abs() <= ON_PLANE && !self.cap.contains(&index) {
                self.cap.push(index);
            }
        }
        if indices.len() >= 3 {
            self.faces.push(HullFace {
                vertex_indices: indices,
                normal: Vector3::zeros(),
            });
        }
    }

    /// The index of `vertex`, adding it if it is new.
    fn intern(&mut self, vertex: Vector3<f32>) -> u16 {
        match self
            .vertices
            .iter()
            .position(|v| (v - vertex).magnitude() <= COINCIDENT)
        {
            Some(index) => index as u16,
            None => {
                self.vertices.push(vertex);
                (self.vertices.len() - 1) as u16
            }
        }
    }

    /// Close the half with its cap and build the hull, or refuse a piece the
    /// engine cannot hold. `outward` is the cap's outward normal, which is
    /// the only ordering this builder cannot work out for itself.
    fn build(mut self, outward: Vector3<f32>) -> Option<ConvexHull> {
        if self.cap.len() >= 3 {
            let ordered = order_around(&self.vertices, &self.cap, outward);
            self.faces.push(HullFace {
                vertex_indices: ordered,
                normal: Vector3::zeros(),
            });
        }
        if self.faces.len() < 4 || self.vertices.len() < 4 {
            return None;
        }
        if self.vertices.len() > MAX_HULL_VERTICES || !thick_enough(&self.vertices) {
            return None;
        }
        Some(ConvexHull::new(self.vertices, self.faces))
    }
}

/// Sort the cap's corners into a loop, by angle about their own centre in the
/// plane of the cap. Only a convex polygon can be ordered this way, which is
/// exactly what the cross-section of a convex hull is.
fn order_around(
    vertices: &[Vector3<f32>],
    cap: &[u16],
    outward: Vector3<f32>,
) -> SmallVec<[u16; 6]> {
    let centre = cap
        .iter()
        .fold(Vector3::zeros(), |acc, i| acc + vertices[*i as usize])
        / cap.len() as f32;
    let spoke = (vertices[cap[0] as usize] - centre).normalize();
    let side = outward.cross(&spoke);

    let mut ordered: SmallVec<[u16; 6]> = cap.iter().copied().collect();
    ordered.sort_by(|a, b| {
        let angle = |i: &u16| {
            let arm = vertices[*i as usize] - centre;
            arm.dot(&side).atan2(arm.dot(&spoke))
        };
        angle(a).total_cmp(&angle(b))
    });
    ordered
}

/// Whether a point cloud is stout enough to be a collider: its thinnest
/// dimension is a reasonable share of its longest, measured along the cloud's
/// own axes rather than the world's.
///
/// Measuring along the world's axes would pass a wafer lying at forty-five
/// degrees, whose axis-aligned box is a cube.
fn thick_enough(vertices: &[Vector3<f32>]) -> bool {
    let centre = vertices.iter().fold(Vector3::zeros(), |a, v| a + v) / vertices.len() as f32;
    let mut longest = 0.0f32;
    let mut axis = Vector3::x();
    for v in vertices {
        let arm = v - centre;
        if arm.magnitude() > longest {
            longest = arm.magnitude();
            axis = arm;
        }
    }
    if longest <= 0.0 {
        return false;
    }
    let axis = axis / longest;
    // The thinnest spread across any direction square to the long axis is
    // bounded below by the spread across two of them, which is enough to
    // catch a wafer without a full principal-axis decomposition.
    let side = if axis.x.abs() < 0.9 {
        axis.cross(&Vector3::x()).normalize()
    } else {
        axis.cross(&Vector3::y()).normalize()
    };
    let other = axis.cross(&side);
    [axis, side, other]
        .iter()
        .map(|dir| {
            let (min, max) = vertices.iter().fold((f32::MAX, f32::MIN), |(lo, hi), v| {
                let d = dir.dot(v);
                (lo.min(d), hi.max(d))
            });
            max - min
        })
        .fold(f32::MAX, f32::min)
        >= longest * 2.0 * MIN_THICKNESS
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::convex_hull::cube_hull;

    fn unit_cube() -> ConvexHull {
        cube_hull(Vector3::repeat(0.5))
    }

    /// The property that matters most: what goes in comes out. A cut that
    /// loses or double-counts volume is a cut that makes pieces which do not
    /// fit back where they came from.
    #[test]
    fn the_halves_together_are_the_whole() {
        let cube = unit_cube();
        let halves = split_hull(&cube, Plane::through(Vector3::zeros(), Vector3::x()))
            .expect("a cube cut down the middle");
        let total = halves.front.compute_volume() + halves.back.compute_volume();
        assert!(
            (total - cube.compute_volume()).abs() < 1e-4,
            "{total} from a volume of {}",
            cube.compute_volume()
        );
    }

    #[test]
    fn an_angled_cut_also_keeps_the_volume() {
        let cube = unit_cube();
        let normal = Vector3::new(0.9, 0.3, 0.32).normalize();
        let halves = split_hull(&cube, Plane::through(Vector3::new(0.05, 0.0, 0.0), normal))
            .expect("an angled cut");
        let total = halves.front.compute_volume() + halves.back.compute_volume();
        assert!((total - 1.0).abs() < 1e-4, "{total}");
        // Off-centre: the halves are not the same size, and the front is the
        // side the normal points into.
        assert!(halves.front.compute_volume() < halves.back.compute_volume());
    }

    /// A corner cut off a cube is a tetrahedron: four faces, four corners.
    #[test]
    fn a_corner_cut_gives_a_wedge_and_the_rest() {
        let cube = unit_cube();
        let normal = Vector3::repeat(1.0).normalize();
        let halves = split_hull(&cube, Plane::through(Vector3::repeat(0.3), normal))
            .expect("a corner comes off");
        assert_eq!(
            halves.front.vertices.len(),
            4,
            "the corner is a tetrahedron"
        );
        assert_eq!(halves.front.faces.len(), 4);
        let total = halves.front.compute_volume() + halves.back.compute_volume();
        assert!((total - 1.0).abs() < 1e-4, "{total}");
    }

    #[test]
    fn every_face_of_both_halves_faces_outwards() {
        let cube = unit_cube();
        let normal = Vector3::new(0.8, 0.5, 0.33).normalize();
        let halves =
            split_hull(&cube, Plane::through(Vector3::new(0.1, 0.1, 0.0), normal)).expect("a cut");
        for half in [&halves.front, &halves.back] {
            let centre = half.vertices.iter().fold(Vector3::zeros(), |a, v| a + v)
                / half.vertices.len() as f32;
            for face in &half.faces {
                let corner = half.vertices[face.vertex_indices[0] as usize];
                assert!(
                    face.normal.dot(&(corner - centre)) > 0.0,
                    "a face points into its own hull"
                );
                // A cap ordered wrongly folds into a bowtie, whose corners
                // are not all in the face's own plane.
                let plane = Plane::through(corner, face.normal);
                for &i in &face.vertex_indices {
                    assert!(plane.distance(half.vertices[i as usize]).abs() < 1e-4);
                }
            }
        }
    }

    #[test]
    fn a_plane_that_misses_the_hull_cuts_nothing() {
        let cube = unit_cube();
        assert!(split_hull(
            &cube,
            Plane::through(Vector3::new(2.0, 0.0, 0.0), Vector3::x())
        )
        .is_none());
        // Grazing the far face is not a cut either: one half would be flat.
        assert!(split_hull(
            &cube,
            Plane::through(Vector3::new(0.5, 0.0, 0.0), Vector3::x())
        )
        .is_none());
    }

    #[test]
    fn a_cut_that_would_shave_off_a_wafer_is_refused() {
        let cube = unit_cube();
        assert!(
            split_hull(
                &cube,
                Plane::through(Vector3::new(0.49, 0.0, 0.0), Vector3::x())
            )
            .is_none(),
            "a 1 cm wafer off a 1 m cube should be refused"
        );
    }

    /// A wafer lying at an angle has a cubic bounding box, so the thinness
    /// test has to measure along the piece's own axes.
    #[test]
    fn a_diagonal_wafer_is_refused_though_its_bounding_box_is_stout() {
        let cube = unit_cube();
        let normal = Vector3::new(1.0, 1.0, 0.0).normalize();
        let corner = Vector3::new(0.5, 0.5, 0.0);
        assert!(split_hull(&cube, Plane::through(corner * 0.98, normal)).is_none());
    }

    /// Cutting twice is how a block becomes three pieces, and the second cut
    /// works on a hull that is no longer a box.
    #[test]
    fn a_piece_of_a_cut_can_be_cut_again() {
        let cube = unit_cube();
        let first = split_hull(&cube, Plane::through(Vector3::zeros(), Vector3::x()))
            .expect("the first cut");
        let second = split_hull(
            &first.back,
            Plane::through(Vector3::zeros(), Vector3::new(0.2, 0.97, 0.0).normalize()),
        )
        .expect("the second cut");
        let total = first.front.compute_volume()
            + second.front.compute_volume()
            + second.back.compute_volume();
        assert!((total - 1.0).abs() < 1e-4, "three pieces make {total}");
    }
}
