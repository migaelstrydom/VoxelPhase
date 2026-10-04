//! A boulder's collision shape: convex bricks fitted to its surface, or, for a
//! shape too intricate for one body, the parts to cut it into.
//!
//! The eye sees the fragment's true marching-cubes mesh; the solver sees up to
//! `max_bricks` convex bricks. The fragment's samples are partitioned into
//! cells, and each cell's brick is the 26-sided hull (box, edge bevels, corner
//! bevels) of what it holds: its samples, the mesh vertices on their edges,
//! and the midpoints to its neighbour cells' samples, where it meets them.
//!
//! ```text
//!   Occupancy + mesh vertices ──▶ one cell around every sample
//!     │  1. split every cell whose brick holds air or ground, however many
//!     │     cells that takes, each where its halves' boxes shrink most
//!     ├── more than max_bricks cells ──▶ Shape::Parts: the split tree cut
//!     │                                  into subtrees of at most
//!     │                                  max_bricks cells, one body each
//!     │  2. while the budget lasts, split cells whose brick swells past
//!     │     max_overcover × their solid
//!     ▼
//!   Shape::Whole(Bricks): hulls on the lattice's axes, centres in the world
//! ```
//!
//! Terrain takes any shape, and some (a hollow dome, a ring) need more convex
//! pieces than any one body should carry. Bricks never hold air, so such a
//! shape is not covered over: it comes down as several bodies, cut where its
//! own convex pieces meet.
//!
//! Fitting the mesh, not the samples, matters: the surface runs close to the
//! outermost samples, and bricks built from whole sample cubes hold a resting
//! boulder half a voxel off the ground. Every face stands in from the surface
//! by `inset`, so neighbouring bricks never overlap, and none starts touching
//! the ground the fragment broke from across a break face, whose new surface
//! lies where the fragment's break face does.
//!
//! A brick is convex and the fragment need not be: a piece that broke off
//! around a knuckle of ground would have its brick reach into the ground,
//! and the solver would throw it out on its first step. The ground's samples
//! and the midpoints to their neighbours stand for its surface, and a brick
//! holding one is split. A cell one sample across that still holds one is cut
//! back with a plane through each, facing away from its samples.

use std::collections::HashSet;

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::collision::convex_hull::cube_hull;
use crate::collision::hull_split::{HullDraft, Plane};
use crate::collision::ConvexHull;
use crate::terrain::{Occupancy, Sample};

/// Thinnest a brick's box may be against its longest side. Well above the
/// hull builder's 1% limit, so a long thin cell still builds.
const MIN_ASPECT: f32 = 0.05;
/// Thinnest a brick's box may be at all, in voxels.
const MIN_EXTENT: f32 = 0.25;

/// One convex piece of a boulder's collision shape.
pub struct Brick {
    /// The brick around its own centre, on the lattice's axes, in metres.
    pub hull: ConvexHull,
    /// Where its centre was in the world at the blast.
    pub centre: Point3<f32>,
}

/// What a fragment is shaped as.
pub enum Shape {
    /// One body, of these bricks.
    Whole(Bricks),
    /// More convex pieces than one body carries: these sets of the
    /// fragment's sample indices, each to be cut out and shaped as a body of
    /// its own.
    Parts(Vec<Vec<[usize; 3]>>),
}

/// A boulder's collision shape.
pub struct Bricks {
    pub bricks: Vec<Brick>,
    /// Turns the lattice's axes, which every brick is built on, into the
    /// world's.
    pub rotation: UnitQuaternion<f32>,
}

impl Bricks {
    /// Total volume of the bricks, in m³.
    pub fn volume(&self) -> f32 {
        self.bricks.iter().map(|b| b.hull.compute_volume()).sum()
    }
}

/// How finely a fragment is cut into bricks.
#[derive(Debug, Clone, Copy)]
pub struct BrickShaper {
    /// Most bricks one body is given.
    pub max_bricks: usize,
    /// A cell is split while its brick's volume is more than this many times
    /// the solid it holds.
    pub max_overcover: f32,
    /// How far each brick's faces stand in from what they enclose, in
    /// voxels: the gap a boulder starts with from the ground it broke from.
    pub inset: f32,
}

impl Default for BrickShaper {
    fn default() -> Self {
        Self {
            max_bricks: 16,
            max_overcover: 1.3,
            inset: 0.1,
        }
    }
}

/// A point on the fragment's surface, in sample indices, and the sample whose
/// edge it lies on.
#[derive(Clone, Copy)]
struct SurfacePoint {
    at: Vector3<f32>,
    owner: [usize; 3],
}

/// Some of a fragment's samples, the surface on their edges, and the brick
/// around them.
struct Cell {
    samples: Vec<[usize; 3]>,
    surface: Vec<SurfacePoint>,
    brick: Brick,
    /// The brick's centre, in sample indices.
    centre: Vector3<f32>,
    /// Brick volume over the volume of its samples.
    overcover: f32,
    /// Points of what the fragment broke from that the brick holds, in
    /// sample indices.
    intrusions: Vec<Vector3<f32>>,
    /// Whether the brick holds an air sample: spans a hole or a gap.
    holds_air: bool,
    /// What no brick may hold, near enough to the cell that its brick, or
    /// its children's, could.
    nearby: Forbidden,
    /// The cell's place in the split tree: near (`false`) or far (`true`)
    /// at each split above it.
    path: Vec<bool>,
}

/// What no brick may hold, in sample indices.
#[derive(Default)]
struct Forbidden {
    /// What the fragment broke from, stood for by each of its samples and the
    /// midpoint to each neighbour that is not one of them.
    ground: Vec<Vector3<f32>>,
    /// Every air sample in a hole, gap or hollow of the fragment
    /// (`Occupancy::encloses`), so that a brick does not span one. Air
    /// outside a convex surface is left out: every brick's bevels skim some.
    air: Vec<Vector3<f32>>,
}

impl Forbidden {
    /// The points within `margin` of the box from `lo` to `hi`.
    fn near(&self, lo: Vector3<f32>, hi: Vector3<f32>, margin: f32) -> Self {
        let inside =
            |p: &&Vector3<f32>| (0..3).all(|a| p[a] >= lo[a] - margin && p[a] <= hi[a] + margin);
        Self {
            ground: self.ground.iter().filter(inside).copied().collect(),
            air: self.air.iter().filter(inside).copied().collect(),
        }
    }
}

/// How far past its samples a cell's brick can reach, in voxels: to the mesh
/// vertices on their edges, never a whole voxel out.
const REACH: f32 = 1.0;

impl BrickShaper {
    /// The shape of the fragment whose samples are `occupancy` and whose
    /// mesh has its vertices at `surface`, world positions at the blast.
    /// A fragment with no samples is a whole of no bricks.
    pub fn shape(
        &self,
        occupancy: &Occupancy,
        surface: impl IntoIterator<Item = Point3<f32>>,
    ) -> Shape {
        let [nx, ny, nz] = occupancy.dims();
        let samples: Vec<[usize; 3]> = (0..nx)
            .flat_map(|x| (0..ny).flat_map(move |y| (0..nz).map(move |z| [x, y, z])))
            .filter(|&index| occupancy.is_solid(index))
            .collect();
        let surface: Vec<SurfacePoint> = surface
            .into_iter()
            .filter_map(|world| {
                let at = occupancy.index_position(world);
                owner_of(occupancy, at).map(|owner| SurfacePoint { at, owner })
            })
            .collect();
        let mut cells = Vec::new();
        if !samples.is_empty() {
            let forbidden = forbidden_points(occupancy);
            cells.push(self.cell(samples, surface, Vec::new(), occupancy, &forbidden));
        }

        let splittable = |c: &&Cell| longest_span(c).1 >= 2;
        while let Some(worst) = cells
            .iter()
            .position(|c| (!c.intrusions.is_empty() || c.holds_air) && splittable(&c))
        {
            let cell = cells.swap_remove(worst);
            cells.extend(self.split(cell, occupancy));
        }
        if cells.len() > self.max_bricks {
            let mut parts = Vec::new();
            group(cells, 0, self.max_bricks, &mut parts);
            return Shape::Parts(parts);
        }

        while cells.len() < self.max_bricks {
            let swollen = cells
                .iter()
                .enumerate()
                .filter(|(_, c)| c.overcover > self.max_overcover && longest_span(c).1 >= 3)
                .max_by(|(_, a), (_, b)| excess(a).total_cmp(&excess(b)))
                .map(|(i, _)| i);
            let Some(worst) = swollen else {
                break;
            };
            let cell = cells.swap_remove(worst);
            cells.extend(self.split(cell, occupancy));
        }

        Shape::Whole(Bricks {
            bricks: cells
                .into_iter()
                .map(|c| self.cut_back(c, occupancy.spacing()))
                .collect(),
            rotation: occupancy.rotation(),
        })
    }

    /// `cell` cut in two where its halves' boxes shrink most.
    fn split(&self, cell: Cell, occupancy: &Occupancy) -> [Cell; 2] {
        let (axis, mid) = best_split(&cell.samples);
        let (near, far): (Vec<_>, Vec<_>) = cell.samples.into_iter().partition(|s| s[axis] <= mid);
        let (near_surface, far_surface): (Vec<_>, Vec<_>) =
            cell.surface.into_iter().partition(|p| p.owner[axis] <= mid);
        let path = |side: bool| {
            let mut path = cell.path.clone();
            path.push(side);
            path
        };
        [
            self.cell(near, near_surface, path(false), occupancy, &cell.nearby),
            self.cell(far, far_surface, path(true), occupancy, &cell.nearby),
        ]
    }

    /// `cell`'s brick, cut back off every point of the ground it holds.
    fn cut_back(&self, cell: Cell, spacing: f32) -> Brick {
        if cell.intrusions.is_empty() {
            return cell.brick;
        }
        let samples =
            cell.samples.iter().map(as_vector).sum::<Vector3<f32>>() / cell.samples.len() as f32;
        let mut draft = HullDraft::of(&cell.brick.hull);
        for point in &cell.intrusions {
            let Some(normal) = (point - samples).try_normalize(1e-6) else {
                continue;
            };
            let plane = Plane {
                normal,
                offset: (normal.dot(&(point - cell.centre)) - self.inset) * spacing,
            };
            if let Some(trimmed) = draft.trim(plane) {
                draft = trimmed;
            }
        }
        Brick {
            hull: draft.build(),
            centre: cell.brick.centre,
        }
    }

    /// The cell holding `samples`, which must not be empty, and the surface
    /// on their edges.
    fn cell(
        &self,
        samples: Vec<[usize; 3]>,
        surface: Vec<SurfacePoint>,
        path: Vec<bool>,
        occupancy: &Occupancy,
        forbidden: &Forbidden,
    ) -> Cell {
        let lo = Vector3::from_fn(|a, _| samples.iter().map(|s| s[a]).min().unwrap_or(0) as f32);
        let hi = Vector3::from_fn(|a, _| samples.iter().map(|s| s[a]).max().unwrap_or(0) as f32);
        let nearby = forbidden.near(lo, hi, REACH);
        let (brick, centre) = self.brick(&samples, &surface, occupancy);
        let solid = samples.len() as f32 * occupancy.spacing().powi(3);
        let overcover = brick.hull.compute_volume() / solid;
        let spacing = occupancy.spacing();
        let held = |p: &&Vector3<f32>| holds(&brick.hull, (*p - centre) * spacing);
        let intrusions = nearby.ground.iter().filter(held).copied().collect();
        let holds_air = nearby.air.iter().any(|p| held(&p));
        Cell {
            samples,
            surface,
            brick,
            centre,
            overcover,
            intrusions,
            holds_air,
            nearby,
            path,
        }
    }

    /// The 26-sided hull of what a cell holds, stood in by `inset`, and its
    /// centre in sample indices.
    fn brick(
        &self,
        samples: &[[usize; 3]],
        surface: &[SurfacePoint],
        occupancy: &Occupancy,
    ) -> (Brick, Vector3<f32>) {
        let mut points: Vec<Vector3<f32>> = surface.iter().map(|p| p.at).collect();
        points.extend(samples.iter().map(as_vector));
        // Where the cell meets another, its brick reaches halfway to the
        // other's samples, and the two meet there.
        let own: HashSet<[usize; 3]> = samples.iter().copied().collect();
        for s in samples {
            for neighbour in neighbours(occupancy, *s) {
                if occupancy.is_solid(neighbour) && !own.contains(&neighbour) {
                    points.push((as_vector(s) + as_vector(&neighbour)) / 2.0);
                }
            }
        }
        // How far the brick reaches along unit `n`, in sample indices.
        let reach = |n: &Vector3<f32>| {
            points
                .iter()
                .map(|p| n.dot(p))
                .fold(f32::NEG_INFINITY, f32::max)
                - self.inset
        };

        let low = -Vector3::from_fn(|a, _| reach(&-Vector3::ith(a, 1.0)));
        let high = Vector3::from_fn(|a, _| reach(&Vector3::ith(a, 1.0)));
        let centre = (low + high) / 2.0;
        let extent = (high - low) / 2.0;
        let floor = (extent.max() * MIN_ASPECT).max(MIN_EXTENT);
        let extent = extent.map(|e| e.max(floor));

        let spacing = occupancy.spacing();
        let mut draft = HullDraft::of(&cube_hull(extent * spacing));
        for normal in bevels() {
            let plane = Plane {
                normal,
                offset: (reach(&normal) - normal.dot(&centre)) * spacing,
            };
            // A bevel that misses the box, or that would leave a hull the
            // builder refuses, is left uncut: the brick only grows.
            if let Some(trimmed) = draft.trim(plane) {
                draft = trimmed;
            }
        }
        let brick = Brick {
            hull: draft.build(),
            centre: occupancy.world_position(centre),
        };
        (brick, centre)
    }
}

/// Whether `hull` holds `point`, strictly.
fn holds(hull: &ConvexHull, point: Vector3<f32>) -> bool {
    hull.faces.iter().all(|face| {
        let on = hull.vertices[face.vertex_indices[0] as usize];
        face.normal.dot(&(point - on)) < 0.0
    })
}

/// `cells`, which share their first `depth` splits, gathered into sets of
/// samples of at most `max` cells each: the split tree cut into subtrees, so
/// each set is a lump of neighbouring cells.
fn group(cells: Vec<Cell>, depth: usize, max: usize, out: &mut Vec<Vec<[usize; 3]>>) {
    if cells.len() <= max {
        out.push(cells.into_iter().flat_map(|c| c.samples).collect());
        return;
    }
    let (near, far): (Vec<Cell>, Vec<Cell>) = cells.into_iter().partition(|c| !c.path[depth]);
    for half in [near, far] {
        if !half.is_empty() {
            group(half, depth + 1, max, out);
        }
    }
}

/// What no brick of the fragment whose samples are `occupancy` may hold.
fn forbidden_points(occupancy: &Occupancy) -> Forbidden {
    let [nx, ny, nz] = occupancy.dims();
    let mut forbidden = Forbidden {
        ground: Vec::new(),
        air: Vec::new(),
    };
    for x in 0..nx {
        for y in 0..ny {
            for z in 0..nz {
                let o = [x, y, z];
                match occupancy.sample(o) {
                    Sample::Solid => {}
                    Sample::Air => {
                        if occupancy.encloses(o) {
                            forbidden.air.push(as_vector(&o));
                        }
                    }
                    Sample::Obstacle => {
                        forbidden.ground.push(as_vector(&o));
                        for n in neighbours(occupancy, o) {
                            if occupancy.sample(n) != Sample::Obstacle {
                                forbidden.ground.push((as_vector(&o) + as_vector(&n)) / 2.0);
                            }
                        }
                    }
                }
            }
        }
    }
    forbidden
}

fn as_vector(s: &[usize; 3]) -> Vector3<f32> {
    Vector3::new(s[0] as f32, s[1] as f32, s[2] as f32)
}

/// The face neighbours of `s` inside the block.
fn neighbours(occupancy: &Occupancy, s: [usize; 3]) -> impl Iterator<Item = [usize; 3]> {
    let dims = occupancy.dims();
    (0..3).flat_map(move |axis| {
        [-1i64, 1].into_iter().filter_map(move |step| {
            let along = s[axis] as i64 + step;
            (0..dims[axis] as i64).contains(&along).then(|| {
                let mut n = s;
                n[axis] = along as usize;
                n
            })
        })
    })
}

/// The solid sample nearest `at`, among the corners of the lattice cell it is
/// in. A marching-cubes vertex lies on an edge between one solid sample and
/// one air sample, so this is the sample it was placed against.
fn owner_of(occupancy: &Occupancy, at: Vector3<f32>) -> Option<[usize; 3]> {
    let dims = occupancy.dims();
    let corner = |pick: usize| -> Option<[usize; 3]> {
        let mut c = [0usize; 3];
        for axis in 0..3 {
            let v = if pick >> axis & 1 == 0 {
                at[axis].floor()
            } else {
                at[axis].ceil()
            };
            if v < 0.0 || v >= dims[axis] as f32 {
                return None;
            }
            c[axis] = v as usize;
        }
        Some(c)
    };
    (0..8)
        .filter_map(corner)
        .filter(|&c| occupancy.is_solid(c))
        .min_by(|a, b| {
            (as_vector(a) - at)
                .norm_squared()
                .total_cmp(&(as_vector(b) - at).norm_squared())
        })
}

/// The twenty unit directions that bevel a box: its twelve edges and eight
/// corners.
fn bevels() -> impl Iterator<Item = Vector3<f32>> {
    (-1..=1)
        .flat_map(|x| (-1..=1).flat_map(move |y| (-1..=1).map(move |z| [x, y, z])))
        .filter(|d: &[i32; 3]| d.iter().filter(|&&c| c != 0).count() >= 2)
        .map(|[x, y, z]| Vector3::new(x as f32, y as f32, z as f32).normalize())
}

/// Where to cut `samples` in two: the axis, and the last index along it that
/// goes to the near half. The cut that leaves the two halves' boxes smallest
/// in total, so a deck is parted from what hangs under it, and a slab from
/// either side of a hole, before anything is cut in the middle.
fn best_split(samples: &[[usize; 3]]) -> (usize, usize) {
    let boxed = |half: &mut dyn Iterator<Item = &[usize; 3]>| -> usize {
        let mut lo = [usize::MAX; 3];
        let mut hi = [0; 3];
        let mut any = false;
        for s in half {
            any = true;
            for a in 0..3 {
                lo[a] = lo[a].min(s[a]);
                hi[a] = hi[a].max(s[a]);
            }
        }
        if any {
            (0..3).map(|a| hi[a] - lo[a] + 1).product()
        } else {
            0
        }
    };
    // (total box volume, distance off centre, axis, last near index)
    let mut best = (usize::MAX, usize::MAX, 0, 0);
    for axis in 0..3 {
        let lo = samples.iter().map(|s| s[axis]).min().unwrap_or(0);
        let hi = samples.iter().map(|s| s[axis]).max().unwrap_or(0);
        for mid in lo..hi {
            let near = boxed(&mut samples.iter().filter(|s| s[axis] <= mid));
            let far = boxed(&mut samples.iter().filter(|s| s[axis] > mid));
            // Ties go to the cut nearest the middle, so a solid block is
            // halved rather than shaved.
            let off_centre = (2 * mid + 1).abs_diff(lo + hi);
            if (near + far, off_centre) < (best.0, best.1) {
                best = (near + far, off_centre, axis, mid);
            }
        }
    }
    (best.2, best.3)
}

/// The axis along which `cell`'s samples spread furthest, and how many
/// samples they span along it.
fn longest_span(cell: &Cell) -> (usize, usize) {
    (0..3)
        .map(|axis| {
            let lo = cell.samples.iter().map(|s| s[axis]).min().unwrap_or(0);
            let hi = cell.samples.iter().map(|s| s[axis]).max().unwrap_or(0);
            (axis, hi - lo + 1)
        })
        .max_by_key(|&(_, span)| span)
        .unwrap_or((0, 0))
}

/// How much more than its samples' volume `cell`'s brick covers, in samples.
fn excess(cell: &Cell) -> f32 {
    cell.samples.len() as f32 * (cell.overcover - 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    const SPACING: f32 = 0.5;

    fn occupancy(dims: [usize; 3], solid: impl Fn([usize; 3]) -> bool) -> Occupancy {
        Occupancy::new(
            dims,
            SPACING,
            Point3::new(3.0, -2.0, 7.0),
            UnitQuaternion::from_euler_angles(0.0, 0.7, 0.0),
            |index| {
                if solid(index) {
                    Sample::Solid
                } else {
                    Sample::Air
                }
            },
        )
    }

    fn samples(occupancy: &Occupancy) -> Vec<[usize; 3]> {
        let [nx, ny, nz] = occupancy.dims();
        (0..nx)
            .flat_map(|x| (0..ny).flat_map(move |y| (0..nz).map(move |z| [x, y, z])))
            .filter(|&s| occupancy.is_solid(s))
            .collect()
    }

    /// A stand-in for the mesh: a vertex halfway along every edge from a
    /// solid sample to air, as marching cubes places it where the densities
    /// are ±1.
    fn surface(occupancy: &Occupancy) -> Vec<Point3<f32>> {
        samples(occupancy)
            .into_iter()
            .flat_map(|s| {
                neighbours(occupancy, s)
                    .filter(|&n| !occupancy.is_solid(n))
                    .map(move |n| occupancy.world_position((as_vector(&s) + as_vector(&n)) / 2.0))
                    .collect::<Vec<_>>()
            })
            .collect()
    }

    fn shape(shaper: &BrickShaper, occupancy: &Occupancy) -> Bricks {
        match shaper.shape(occupancy, surface(occupancy)) {
            Shape::Whole(bricks) => bricks,
            Shape::Parts(parts) => panic!("cut into {} parts", parts.len()),
        }
    }

    /// How far `point` is outside `brick`: its distance past the face it is
    /// furthest past, negative inside.
    fn outside(bricks: &Bricks, brick: &Brick, point: Point3<f32>) -> f32 {
        let at = local(bricks, brick, point);
        let hull = &brick.hull;
        hull.faces
            .iter()
            .map(|face| {
                let on = hull.vertices[face.vertex_indices[0] as usize];
                face.normal.dot(&(at - on))
            })
            .fold(f32::MIN, f32::max)
    }

    /// `point`, a world position, in `brick`'s own frame.
    fn local(bricks: &Bricks, brick: &Brick, point: Point3<f32>) -> Vector3<f32> {
        bricks
            .rotation
            .inverse_transform_vector(&(point - brick.centre))
    }

    fn contains(hull: &ConvexHull, point: Vector3<f32>) -> bool {
        hull.faces.iter().all(|face| {
            let on = hull.vertices[face.vertex_indices[0] as usize];
            face.normal.dot(&(point - on)) <= 1e-4
        })
    }

    /// Every sample inside a brick, the surface no further outside one than
    /// the inset, no two bricks overlapping, within the budget, and the
    /// bricks' volume no more than 30% over the samples'. How far short of
    /// it they may fall is held by the surface check: the inset comes off
    /// every face, most of a strand one sample thick.
    fn assert_sound(occupancy: &Occupancy, shaper: &BrickShaper, bricks: &Bricks) {
        let samples = samples(occupancy);
        assert!(!bricks.bricks.is_empty());
        assert!(bricks.bricks.len() <= shaper.max_bricks);
        for s in &samples {
            let at = occupancy.world_position(Vector3::new(s[0] as f32, s[1] as f32, s[2] as f32));
            let holders = bricks
                .bricks
                .iter()
                .filter(|b| contains(&b.hull, local(bricks, b, at)))
                .count();
            assert_eq!(holders, 1, "sample {s:?} is in {holders} bricks");
        }
        for point in surface(occupancy) {
            let past = bricks
                .bricks
                .iter()
                .map(|b| outside(bricks, b, point))
                .fold(f32::MAX, f32::min);
            assert!(
                past <= shaper.inset * SPACING + 1e-3,
                "surface at {point:?} is {past} m outside every brick"
            );
        }
        // Bricks are built on the lattice's axes, so boxes on those axes
        // apart mean bricks apart.
        let boxes: Vec<(Vector3<f32>, Vector3<f32>)> = bricks
            .bricks
            .iter()
            .map(|b| {
                let c = bricks.rotation.inverse_transform_vector(&b.centre.coords);
                let lo = b
                    .hull
                    .vertices
                    .iter()
                    .fold(Vector3::repeat(f32::MAX), |m, v| m.inf(v));
                let hi = b
                    .hull
                    .vertices
                    .iter()
                    .fold(Vector3::repeat(f32::MIN), |m, v| m.sup(v));
                (c + lo, c + hi)
            })
            .collect();
        for (i, a) in boxes.iter().enumerate() {
            for b in &boxes[i + 1..] {
                let apart = (0..3).any(|k| a.1[k] <= b.0[k] + 1e-4 || b.1[k] <= a.0[k] + 1e-4);
                assert!(apart, "bricks overlap: {a:?} {b:?}");
            }
        }
        let solid = samples.len() as f32 * SPACING.powi(3);
        let ratio = bricks.volume() / solid;
        assert!(
            ratio <= 1.3,
            "bricks hold {ratio} of the solid: {} samples, {} bricks",
            samples.len(),
            bricks.bricks.len()
        );
    }

    /// Shape `occupancy`; where it comes back in parts, cut each out as the
    /// planner does, the others its obstacles, and shape it in turn. Every
    /// body that results is sound, and the parts take every sample once.
    fn assert_sound_in_parts(occupancy: &Occupancy, shaper: &BrickShaper) {
        match shaper.shape(occupancy, surface(occupancy)) {
            Shape::Whole(bricks) => assert_sound(occupancy, shaper, &bricks),
            Shape::Parts(parts) => {
                let mut taken: Vec<[usize; 3]> = parts.iter().flatten().copied().collect();
                taken.sort_unstable();
                let mut all = samples(occupancy);
                all.sort_unstable();
                assert_eq!(taken, all, "the parts do not take every sample once");
                for part in &parts {
                    let own: HashSet<[usize; 3]> = part.iter().copied().collect();
                    let cut = Occupancy::new(
                        occupancy.dims(),
                        SPACING,
                        occupancy.world_position(Vector3::zeros()),
                        occupancy.rotation(),
                        |s| match occupancy.sample(s) {
                            Sample::Solid if own.contains(&s) => Sample::Solid,
                            Sample::Air => Sample::Air,
                            _ => Sample::Obstacle,
                        },
                    );
                    assert_sound_in_parts(&cut, shaper);
                }
            }
        }
    }

    #[test]
    fn a_block_is_one_brick() {
        let block = occupancy([10, 8, 9], |[x, y, z]| {
            (2..8).contains(&x) && (2..6).contains(&y) && (2..7).contains(&z)
        });
        let shaper = BrickShaper::default();
        let bricks = shape(&shaper, &block);
        assert_eq!(bricks.bricks.len(), 1);
        assert_sound(&block, &shaper, &bricks);
    }

    #[test]
    fn an_l_is_split_where_it_turns() {
        // Its arms are thin enough that even its convex hull, the best one
        // brick can do, is nearly twice its volume.
        let l = occupancy([18, 18, 6], |[x, y, z]| {
            (2..4).contains(&z) && (2..16).contains(&x) && (2..16).contains(&y) && (x < 4 || y < 4)
        });
        let shaper = BrickShaper::default();
        let bricks = shape(&shaper, &l);
        assert!(bricks.bricks.len() >= 2, "{} bricks", bricks.bricks.len());
        assert_sound(&l, &shaper, &bricks);
    }

    /// Air samples a brick holds deeper than the inset: empty space the
    /// solver would treat as rock.
    fn air_held(occupancy: &Occupancy, shaper: &BrickShaper, bricks: &Bricks) -> usize {
        let [nx, ny, nz] = occupancy.dims();
        (0..nx)
            .flat_map(|x| (0..ny).flat_map(move |y| (0..nz).map(move |z| [x, y, z])))
            .filter(|&s| occupancy.sample(s) == Sample::Air)
            .filter(|s| {
                let at = occupancy.world_position(as_vector(s));
                bricks
                    .bricks
                    .iter()
                    .any(|b| outside(bricks, b, at) < -shaper.inset * SPACING)
            })
            .count()
    }

    #[test]
    fn a_hole_through_a_slab_stays_open() {
        let slab = occupancy([20, 6, 20], |[x, y, z]| {
            let hole = (8..12).contains(&x) && (8..12).contains(&z);
            (2..18).contains(&x) && (2..4).contains(&y) && (2..18).contains(&z) && !hole
        });
        let shaper = BrickShaper::default();
        let bricks = shape(&shaper, &slab);
        assert_eq!(
            air_held(&slab, &shaper, &bricks),
            0,
            "{} bricks",
            bricks.bricks.len()
        );
        assert_sound(&slab, &shaper, &bricks);
    }

    #[test]
    fn stalactites_under_a_slab_are_not_boxed_in() {
        let slab = occupancy([20, 12, 20], |[x, y, z]| {
            let deck = (2..18).contains(&x) && (8..10).contains(&y) && (2..18).contains(&z);
            let spike = |cx: usize, cz: usize, len: usize| {
                (cx..cx + 2).contains(&x) && (cz..cz + 2).contains(&z) && (8 - len..8).contains(&y)
            };
            deck || spike(4, 4, 5) || spike(12, 5, 4) || spike(6, 13, 6) || spike(14, 14, 3)
        });
        let shaper = BrickShaper::default();
        let bricks = shape(&shaper, &slab);
        assert_eq!(
            air_held(&slab, &shaper, &bricks),
            0,
            "{} bricks",
            bricks.bricks.len()
        );
        assert_sound(&slab, &shaper, &bricks);
    }

    /// A hollow dome, a shell two samples thick, needs more convex pieces
    /// than one body carries: it is cut into parts that take every sample
    /// once, each a lump of neighbours.
    #[test]
    fn a_hollow_dome_comes_down_in_parts() {
        let dome = occupancy([26, 15, 26], |[x, y, z]| {
            let d = Vector3::new(x as f32, y as f32, z as f32) - Vector3::new(12.5, 2.0, 12.5);
            (2..14).contains(&y) && (9.0..=11.0).contains(&d.norm())
        });
        let shaper = BrickShaper::default();
        let Shape::Parts(parts) = shaper.shape(&dome, surface(&dome)) else {
            panic!("a dome is shaped as one body");
        };
        assert!(parts.len() >= 2);
        let mut taken: Vec<[usize; 3]> = parts.iter().flatten().copied().collect();
        taken.sort_unstable();
        let before = taken.len();
        taken.dedup();
        assert_eq!(taken.len(), before, "a sample is in two parts");
        assert_eq!(taken, {
            let mut all = samples(&dome);
            all.sort_unstable();
            all
        });
        for part in &parts {
            let lo = [0, 1, 2].map(|a| part.iter().map(|s| s[a]).min().unwrap());
            let hi = [0, 1, 2].map(|a| part.iter().map(|s| s[a]).max().unwrap());
            let spread = (0..3).map(|a| hi[a] - lo[a]).max().unwrap();
            assert!(spread < 26, "a part spans the dome: {lo:?}..{hi:?}");
        }
    }

    #[test]
    fn a_ball_is_bevelled_not_boxed() {
        let ball = occupancy([13, 13, 13], |[x, y, z]| {
            let d = Vector3::new(x as f32, y as f32, z as f32) - Vector3::repeat(6.0);
            d.norm() <= 4.5
        });
        let shaper = BrickShaper {
            max_bricks: 1,
            ..BrickShaper::default()
        };
        let bricks = shape(&shaper, &ball);
        let solid = samples(&ball).len() as f32 * SPACING.powi(3);
        // Its box would be 9.8³ samples, 1.6 × the ball's volume.
        assert!(
            bricks.volume() < 1.25 * solid,
            "{}",
            bricks.volume() / solid
        );
    }

    /// Seeded lumpy blobs: every one shapes without a refused hull, and
    /// soundly.
    #[test]
    fn random_lumps_shape_soundly() {
        let mut rng = StdRng::seed_from_u64(7);
        let shaper = BrickShaper::default();
        for _ in 0..300 {
            let dims = [0; 3].map(|_| rng.gen_range(6..22));
            let lumps: Vec<(Vector3<f32>, f32)> = (0..rng.gen_range(1..5))
                .map(|_| {
                    let centre = Vector3::from_fn(|a, _| rng.gen_range(2.0..dims[a] as f32 - 2.0));
                    (centre, rng.gen_range(1.5..6.0))
                })
                .collect();
            let lump = occupancy(dims, |[x, y, z]| {
                let p = Vector3::new(x as f32, y as f32, z as f32);
                (1..dims[0] - 1).contains(&x)
                    && (1..dims[1] - 1).contains(&y)
                    && (1..dims[2] - 1).contains(&z)
                    && lumps.iter().any(|(c, r)| (p - c).norm() <= *r)
            });
            if samples(&lump).is_empty() {
                continue;
            }
            assert_sound_in_parts(&lump, &shaper);
        }
    }
}
