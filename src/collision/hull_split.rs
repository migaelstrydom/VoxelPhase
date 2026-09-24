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

/// Sine of the angle below which a face's corner counts as lying on the
/// straight line between its neighbours.
const COLLINEAR: f32 = 1e-4;

/// Smallest fan sum a face may have and still be built: the hull builder
/// rejects a face whose corners sum to less than `1e-4` as collinear, and
/// does it with an assertion. Ten times that, so a face that passes here is
/// one it takes without argument — and still a thousandth of the smallest
/// face any piece worth keeping has.
const MIN_FACE_SPAN: f32 = 1e-3;

/// The hull builder's own limit, which it asserts rather than reports: the
/// shortest side of a hull's axis-aligned box must be at least this share of
/// the longest. Kept a little above the builder's 1% so a piece that passes
/// here is one it takes without argument.
const BUILDER_THICKNESS_RATIO: f32 = 0.02;

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
    let cut = Cut::new(&hull.vertices, plane)?;
    Some(HullHalves {
        front: cut
            .keep(&hull.faces, Side::Front)
            .finish(plane.normal)?
            .build(),
        back: cut
            .keep(&hull.faces, Side::Back)
            .finish(-plane.normal)?
            .build(),
    })
}

/// A convex solid as its corners and face loops, checked to be one the hull
/// builder will take but not yet built into a [`ConvexHull`].
///
/// Building is the expensive step — the builder works out every face's normal
/// and pairs up the two faces of every edge — and a run of cuts needs none of
/// that until the last one. Trimming a draft cut after cut and building it
/// once costs a fraction of building a hull after every cut.
pub struct HullDraft {
    /// Corners, each one used by at least one face.
    vertices: Vec<Vector3<f32>>,
    /// Face loops. Their normals are left for the builder to work out.
    faces: Vec<HullFace>,
}

impl HullDraft {
    /// The draft of a hull that has already been built.
    pub fn of(hull: &ConvexHull) -> Self {
        Self {
            vertices: hull.vertices.clone(),
            faces: hull.faces.clone(),
        }
    }

    /// The part of the draft behind `plane`, with everything in front of it
    /// cut away.
    ///
    /// The half-space form of [`split_hull`], for a cut whose other side is
    /// not wanted as a body — chamfering an edge throws away a sliver that no
    /// builder would accept as a piece, and refusing the cut on those grounds
    /// would refuse every chamfer. `None` if the plane misses the draft or if
    /// what is left is not a piece the builder will take.
    pub fn trim(&self, plane: Plane) -> Option<Self> {
        Cut::new(&self.vertices, plane)?
            .keep(&self.faces, Side::Back)
            .finish(-plane.normal)
    }

    /// Build the hull. Every draft is one the builder takes, so this cannot
    /// fail.
    pub fn build(self) -> ConvexHull {
        ConvexHull::new(self.vertices, self.faces)
    }
}

/// A plane laid across a solid: the signed distance of each of its vertices
/// from the plane, snapped to zero within [`ON_PLANE`].
struct Cut<'a> {
    /// The solid's vertices, indexed as its faces index them.
    vertices: &'a [Vector3<f32>],
    /// Signed distance of each vertex from the plane.
    distances: Vec<f32>,
}

impl<'a> Cut<'a> {
    /// `None` if the plane leaves every vertex on one side, where there is
    /// nothing to cut.
    fn new(vertices: &'a [Vector3<f32>], plane: Plane) -> Option<Self> {
        let distances: Vec<f32> = vertices
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

        // A plane that leaves every vertex on one side has not cut anything,
        // and one that only grazes a face gives a half with no interior.
        if !distances.iter().any(|d| *d > 0.0) || !distances.iter().any(|d| *d < 0.0) {
            return None;
        }
        Some(Self {
            vertices,
            distances,
        })
    }

    /// Clip every face to one side of the plane, collecting what is left.
    fn keep(&self, faces: &[HullFace], side: Side) -> HalfBuilder {
        let mut half = HalfBuilder::new(self.vertices.len());
        for face in faces {
            let mut corners: SmallVec<[u16; 8]> = SmallVec::new();
            let count = face.vertex_indices.len();
            for k in 0..count {
                let i = face.vertex_indices[k] as usize;
                let j = face.vertex_indices[(k + 1) % count] as usize;
                let (di, dj) = (self.distances[i], self.distances[j]);
                if side.keeps(di) {
                    corners.push(half.original(i, self.vertices[i], di == 0.0));
                }
                // An edge that crosses the plane contributes the crossing
                // point to both halves. An edge that only touches it does
                // not: its endpoint is on the plane and already kept.
                if di * dj < 0.0 {
                    let t = di / (di - dj);
                    let (from, to) = (self.vertices[i], self.vertices[j]);
                    corners.push(half.on_plane(from + (to - from) * t));
                }
            }
            half.add_face(corners);
        }
        half
    }
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
struct HalfBuilder {
    vertices: Vec<Vector3<f32>>,
    faces: Vec<HullFace>,
    /// Where each vertex of the solid being cut has landed in this half, once
    /// a face has kept it.
    placed: Vec<Option<u16>>,
    /// The vertices that landed on the cutting plane, which together are the
    /// face that caps this half.
    cap: Vec<u16>,
}

impl HalfBuilder {
    /// An empty half of a solid with `source_vertices` vertices.
    fn new(source_vertices: usize) -> Self {
        Self {
            vertices: Vec::new(),
            faces: Vec::new(),
            placed: vec![None; source_vertices],
            cap: Vec::new(),
        }
    }

    /// Add one clipped face, given as its corners in loop order. A face left
    /// with fewer than three distinct corners was cut away entirely.
    fn add_face(&mut self, corners: SmallVec<[u16; 8]>) {
        let mut indices: SmallVec<[u16; 6]> = SmallVec::new();
        for index in corners {
            if !indices.contains(&index) {
                indices.push(index);
            }
        }
        if indices.len() >= 3 {
            self.faces.push(HullFace {
                vertex_indices: indices,
                normal: Vector3::zeros(),
            });
        }
    }

    /// The index in this half of vertex `source` of the solid being cut.
    ///
    /// A vertex of the solid is distinct from every other, so it only needs
    /// looking up by where it came from — unless it sits on the plane, where
    /// a crossing point may already have landed on top of it.
    fn original(&mut self, source: usize, position: Vector3<f32>, on_plane: bool) -> u16 {
        if let Some(index) = self.placed[source] {
            return index;
        }
        let index = if on_plane {
            self.on_plane(position)
        } else {
            self.push(position)
        };
        self.placed[source] = Some(index);
        index
    }

    /// The index of a point on the cutting plane, welded to any point already
    /// there. Only the cap can hold a match: every vertex off the plane is
    /// more than [`ON_PLANE`] from it, which is further than [`COINCIDENT`].
    fn on_plane(&mut self, position: Vector3<f32>) -> u16 {
        let existing =
            self.cap.iter().copied().find(|&index| {
                (self.vertices[index as usize] - position).magnitude() <= COINCIDENT
            });
        existing.unwrap_or_else(|| {
            let index = self.push(position);
            self.cap.push(index);
            index
        })
    }

    fn push(&mut self, position: Vector3<f32>) -> u16 {
        self.vertices.push(position);
        (self.vertices.len() - 1) as u16
    }

    /// Close the half with its cap and check it, or refuse a piece the engine
    /// cannot hold. `outward` is the cap's outward normal, which is the only
    /// ordering this builder cannot work out for itself.
    fn finish(mut self, outward: Vector3<f32>) -> Option<HullDraft> {
        if self.cap.len() >= 3 {
            let ordered = order_around(&self.vertices, &self.cap, outward);
            self.faces.push(HullFace {
                vertex_indices: ordered,
                normal: Vector3::zeros(),
            });
        }
        let (vertices, faces) = tidy(self.vertices, self.faces)?;
        if faces.len() < 4 || vertices.len() < 4 || vertices.len() > MAX_HULL_VERTICES {
            return None;
        }
        if !closed(&faces) {
            return None;
        }
        if !thick_enough(&vertices) || !stout_enough(&vertices) {
            return None;
        }
        Some(HullDraft { vertices, faces })
    }
}

/// Clean up the faces of a half so the hull builder will take them: it
/// panics on a face whose corners are in a line, and counts every vertex
/// towards its own limits whether a face uses it or not.
///
/// Two things need doing. A cut that runs near a corner leaves faces with
/// corners strung along a straight edge, which are redundant on a polygon
/// and fatal to a hull; and a face cut away entirely leaves its corners
/// behind in the vertex list with nothing referring to them.
fn tidy(
    vertices: Vec<Vector3<f32>>,
    faces: Vec<HullFace>,
) -> Option<(Vec<Vector3<f32>>, Vec<HullFace>)> {
    let kept: Vec<HullFace> = faces
        .into_iter()
        .filter_map(|face| {
            let corners = straighten(&vertices, face.vertex_indices)?;
            Some(HullFace {
                vertex_indices: corners,
                normal: Vector3::zeros(),
            })
        })
        .collect();

    // Renumber onto the vertices some face still uses.
    let mut moved_to: Vec<Option<u16>> = vec![None; vertices.len()];
    let mut compact = Vec::with_capacity(vertices.len());
    for face in &kept {
        for &index in &face.vertex_indices {
            if moved_to[index as usize].is_none() {
                moved_to[index as usize] = Some(compact.len() as u16);
                compact.push(vertices[index as usize]);
            }
        }
    }
    let renumbered = kept
        .into_iter()
        .map(|face| HullFace {
            vertex_indices: face
                .vertex_indices
                .iter()
                .map(|i| moved_to[*i as usize].expect("a face's own corner was kept"))
                .collect(),
            normal: face.normal,
        })
        .collect();
    Some((compact, renumbered))
}

/// Drop the corners of one face that sit on the straight line between their
/// neighbours, and refuse the face if what is left is not a polygon.
///
/// Repeated to a fixed point: taking one corner out can leave the next one
/// in line with its new neighbours.
fn straighten(
    vertices: &[Vector3<f32>],
    mut loop_: SmallVec<[u16; 6]>,
) -> Option<SmallVec<[u16; 6]>> {
    let mut dropped = true;
    while dropped && loop_.len() > 3 {
        dropped = false;
        for k in 0..loop_.len() {
            let count = loop_.len();
            let previous = vertices[loop_[(k + count - 1) % count] as usize];
            let corner = vertices[loop_[k] as usize];
            let next = vertices[loop_[(k + 1) % count] as usize];
            let (arriving, leaving) = (corner - previous, next - corner);
            let turn = arriving.cross(&leaving).magnitude();
            if turn <= COLLINEAR * arriving.magnitude() * leaving.magnitude() {
                loop_.remove(k);
                dropped = true;
                break;
            }
        }
    }
    (loop_.len() >= 3 && area_of(vertices, &loop_) > MIN_FACE_SPAN).then_some(loop_)
}

/// The fan sum of a face — twice its area — which is zero when its corners
/// are in a line. The same sum the hull builder takes for the face normal,
/// so comparing against its threshold is comparing like with like.
fn area_of(vertices: &[Vector3<f32>], loop_: &[u16]) -> f32 {
    let first = vertices[loop_[0] as usize];
    let mut area = Vector3::zeros();
    for k in 1..loop_.len() - 1 {
        let a = vertices[loop_[k] as usize] - first;
        let b = vertices[loop_[k + 1] as usize] - first;
        area += a.cross(&b);
    }
    area.magnitude()
}

/// Whether the faces make a closed surface: every edge shared by exactly two
/// of them.
///
/// The hull builder demands this and asserts it. It can fail here for a
/// reason that is nobody's mistake: a cut that passes exactly through an
/// existing corner leaves a face with a corner in the middle of one of its
/// edges, and dropping that corner — which must happen, or the face is
/// degenerate — leaves the neighbouring face with an edge that no longer
/// matches. Such a piece is refused, and the caller moves the plane and
/// tries again. Better a cut that does not happen than a hull with a hole.
fn closed(faces: &[HullFace]) -> bool {
    let mut edges: Vec<(u16, u16)> = faces
        .iter()
        .flat_map(|face| {
            let corners = &face.vertex_indices;
            (0..corners.len()).map(move |k| {
                let (a, b) = (corners[k], corners[(k + 1) % corners.len()]);
                (a.min(b), a.max(b))
            })
        })
        .collect();
    edges.sort_unstable();
    // Sorted, a closed surface lists every edge exactly twice in a row.
    !edges.is_empty() && edges.chunk_by(|a, b| a == b).all(|run| run.len() == 2)
}

/// Whether the hull builder's own thinness rule will accept this cloud: the
/// shortest side of its axis-aligned box against the longest. Checked here
/// because the builder does not check, it asserts.
fn stout_enough(vertices: &[Vector3<f32>]) -> bool {
    let (mut low, mut high) = (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN));
    for v in vertices {
        low = low.inf(v);
        high = high.sup(v);
    }
    let extent = high - low;
    let longest = extent.x.max(extent.y).max(extent.z);
    let shortest = extent.x.min(extent.y).min(extent.z);
    longest > 0.0 && shortest / longest >= BUILDER_THICKNESS_RATIO
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
    // From the corner furthest out, so the spoke is never a zero vector:
    // a corner sitting on the cap's own centre would give one, and every
    // angle measured against it would be a NaN.
    let spoke = cap
        .iter()
        .map(|i| vertices[*i as usize] - centre)
        .max_by(|a, b| a.magnitude_squared().total_cmp(&b.magnitude_squared()))
        .map(|arm| arm.normalize())
        .unwrap_or_else(Vector3::x);
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

    /// Every plane that can be aimed at a hull, twice over, and none of them
    /// may panic the hull builder.
    ///
    /// Written after one did: a grenade in an igloo found a cut whose face
    /// came out with all its corners in a line. A cut that cannot be made is
    /// a `None`, never a crash, and the only way to be sure of that is to
    /// try a great many of them.
    #[test]
    fn no_cut_anywhere_can_panic_the_hull_builder() {
        let mut state = 0x1234_5678u32;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            (state >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        let block = cube_hull(Vector3::new(0.3, 0.15, 0.2));
        let mut cuts = 0;
        for _ in 0..4000 {
            let normal = Vector3::new(next(), next(), next());
            if normal.magnitude() < 1e-3 {
                continue;
            }
            let plane = Plane::through(
                Vector3::new(next() * 0.7, next() * 0.35, next() * 0.5),
                normal.normalize(),
            );
            let Some(halves) = split_hull(&block, plane) else {
                continue;
            };
            cuts += 1;
            let total = halves.front.compute_volume() + halves.back.compute_volume();
            assert!(
                (total - block.compute_volume()).abs() < 1e-3,
                "a cut lost volume: {total}"
            );
            // And again, on a piece that is no longer a box.
            let normal = Vector3::new(next(), next(), next());
            if normal.magnitude() < 1e-3 {
                continue;
            }
            let centre = halves.front.centroid();
            let again = Plane::through(
                centre + Vector3::new(next() * 0.2, next() * 0.2, next() * 0.2),
                normal.normalize(),
            );
            if let Some(twice) = split_hull(&halves.front, again) {
                let total = twice.front.compute_volume() + twice.back.compute_volume();
                assert!(
                    (total - halves.front.compute_volume()).abs() < 1e-3,
                    "a second cut lost volume: {total}"
                );
            }
        }
        assert!(cuts > 500, "only {cuts} of the planes cut anything");
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
