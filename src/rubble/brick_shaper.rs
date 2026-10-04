//! A boulder's collision shape: a few convex bricks fitted to its surface.
//!
//! The eye sees the fragment's true marching-cubes mesh; the solver sees up to
//! a dozen convex bricks. The fragment's samples are partitioned into cells,
//! and each cell's brick is the 26-sided hull (box, edge bevels, corner
//! bevels) of what it holds: its samples, the mesh vertices on their edges,
//! and the midpoints to its neighbour cells' samples, where it meets them. A
//! cell whose brick swells well past the solid it holds is split in two along
//! its longest axis, the worst cell first, until every brick fits or the
//! budget is spent.
//!
//! ```text
//!   Occupancy + mesh vertices ──▶ one cell around every sample
//!                   │  brick: box ∩ 20 bevel planes, each through the
//!                   │         farthest point that way, stood in by `inset`
//!                   │  worst overcover > max_overcover and budget left?
//!                   │     └── split at the longest axis' midpoint, repeat
//!                   ▼
//!   Bricks: hulls on the lattice's axes, centres in the world at the blast
//! ```
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
//! and the midpoints to their neighbours stand for its surface; a cell whose
//! brick holds one of them is split first, whatever its overcover, and one
//! still holding one when the budget is spent is cut back with a plane
//! through each, facing away from the cell's samples.

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
    /// Most bricks one boulder is given.
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
            max_bricks: 12,
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
}

impl BrickShaper {
    /// The bricks of the fragment whose samples are `occupancy` and whose
    /// mesh has its vertices at `surface`, world positions at the blast.
    /// Empty for a fragment with no samples.
    pub fn shape(
        &self,
        occupancy: &Occupancy,
        surface: impl IntoIterator<Item = Point3<f32>>,
    ) -> Bricks {
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
        let ground = ground_points(occupancy);
        let mut cells = Vec::new();
        if !samples.is_empty() {
            cells.push(self.cell(samples, surface, occupancy, &ground));
        }

        while cells.len() < self.max_bricks {
            let intruded = cells
                .iter()
                .enumerate()
                .filter(|(_, c)| !c.intrusions.is_empty() && longest_span(c).1 >= 2)
                .max_by_key(|(_, c)| c.samples.len());
            let swollen = || {
                cells
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| c.overcover > self.max_overcover && longest_span(c).1 >= 3)
                    .max_by(|(_, a), (_, b)| excess(a).total_cmp(&excess(b)))
            };
            let Some(worst) = intruded.or_else(swollen).map(|(i, _)| i) else {
                break;
            };
            let cell = cells.swap_remove(worst);
            let (axis, _) = longest_span(&cell);
            let lo = cell.samples.iter().map(|s| s[axis]).min().unwrap_or(0);
            let hi = cell.samples.iter().map(|s| s[axis]).max().unwrap_or(0);
            let mid = lo + (hi - lo) / 2;
            let (near, far): (Vec<_>, Vec<_>) =
                cell.samples.into_iter().partition(|s| s[axis] <= mid);
            let (near_surface, far_surface): (Vec<_>, Vec<_>) =
                cell.surface.into_iter().partition(|p| p.owner[axis] <= mid);
            cells.push(self.cell(near, near_surface, occupancy, &ground));
            cells.push(self.cell(far, far_surface, occupancy, &ground));
        }

        Bricks {
            bricks: cells
                .into_iter()
                .map(|c| self.cut_back(c, occupancy.spacing()))
                .collect(),
            rotation: occupancy.rotation(),
        }
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
        occupancy: &Occupancy,
        ground: &[Vector3<f32>],
    ) -> Cell {
        let (brick, centre) = self.brick(&samples, &surface, occupancy);
        let solid = samples.len() as f32 * occupancy.spacing().powi(3);
        let overcover = brick.hull.compute_volume() / solid;
        let spacing = occupancy.spacing();
        let intrusions = ground
            .iter()
            .filter(|p| holds(&brick.hull, (*p - centre) * spacing))
            .copied()
            .collect();
        Cell {
            samples,
            surface,
            brick,
            centre,
            overcover,
            intrusions,
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

/// Points standing for the surface of what the fragment broke from, in
/// sample indices: each of its samples, and the midpoint to each neighbour
/// that is not one of them.
fn ground_points(occupancy: &Occupancy) -> Vec<Vector3<f32>> {
    let [nx, ny, nz] = occupancy.dims();
    let mut points = Vec::new();
    for x in 0..nx {
        for y in 0..ny {
            for z in 0..nz {
                let o = [x, y, z];
                if occupancy.sample(o) != Sample::Obstacle {
                    continue;
                }
                points.push(as_vector(&o));
                for n in neighbours(occupancy, o) {
                    if occupancy.sample(n) != Sample::Obstacle {
                        points.push((as_vector(&o) + as_vector(&n)) / 2.0);
                    }
                }
            }
        }
    }
    points
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
        shaper.shape(occupancy, surface(occupancy))
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
    /// bricks' volume no more than 30% over the samples'. It may fall
    /// further short: the inset is taken off every face, a large share of a
    /// lump a few samples across.
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
            (0.6..=1.3).contains(&ratio),
            "bricks hold {ratio} of the solid"
        );
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
            assert_sound(&lump, &shaper, &shape(&shaper, &lump));
        }
    }
}
