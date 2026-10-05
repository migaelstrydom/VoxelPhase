//! Stamping a fragment back into the field where it came to rest.
//!
//! A fragment keeps its samples on the lattice it was cut from. Turned to the
//! pose it landed in and resampled at the target lattice's sample points, that
//! block is a density field like any other, and is unioned in the way every
//! authored feature is (`csg::union_solid`).
//!
//! ```text
//!   for every grid sample p in the fragment's box at its new pose:
//!       q = to_fragment · p               a point of the fragment's block
//!       d = trilinear(block, q) + DEPOSIT_BIAS
//!       union: keep max(existing, d), the fragment's material where d wins
//!   weld: each piece of rock that bears joined to bearing ground
//! ```
//!
//! Resampling at an angle loses what the lattice cannot hold: the surface
//! moves by about a tenth of a voxel, and a sharp tip is blunted (the
//! measurements are in `docs/terrain_rubble/DESIGN.md`, Part 5). Trilinear
//! blending of a clamped distance also cuts into convex features, the same
//! amount for every shape and pose, which [`DEPOSIT_BIAS`] puts back.

use std::collections::HashSet;

use nalgebra::{Isometry3, Point3};

use super::chunk_grid::ChunkGrid;
use super::csg::SURFACE_BAND;
use super::fragment::{Fragment, BEARING_DENSITY};
use super::voxel::{Voxel, VoxelMaterial};
use super::voxel_block::VoxelBlock;
use crate::collision::AABB;

/// What a resampled density gains, in voxels. Trilinear interpolation of a
/// clamped distance draws a solid 5–6.5% smaller than it was, a curved one
/// more than a flat one per area. No constant gives every shape its volume
/// back; this one leaves a rock 1.5% short and a slab 2% over (the test
/// below), where the 0.055 of the design's Python study left the slab 3.5%
/// over.
pub(super) const DEPOSIT_BIAS: f32 = 0.045;

/// How far a weld reaches from a rock to the ground, in samples on each axis.
/// A rock resting on terrain touches it within a voxel, but may lie on a skin
/// too weak to bear, deposited before it; one further from any rests on
/// something else, a body or rock yet to be deposited. Over `rubble_viewer`'s
/// scenarios no weld raised more than two samples.
const WELD_REACH: i32 = 3;

/// Union `fragment` into `grid`, `to_fragment` taking the grid's local
/// positions to its block's, and weld it to the ground it rests on. Returns
/// the grid-local box around every sample it changed, or `None` if it changed
/// none.
pub(super) fn deposit(
    grid: &mut ChunkGrid,
    fragment: &Fragment,
    to_fragment: &Isometry3<f32>,
) -> Option<AABB> {
    let block = fragment.voxels();
    let material = fragment.material();
    let step = grid.voxel_size();
    let reach = to_grid_box(block, &to_fragment.inverse());
    let lo = (reach.min.coords / step).map(|v| v.floor() as i32);
    let hi = (reach.max.coords / step).map(|v| v.ceil() as i32);

    let mut changed = Vec::new();
    let mut rock = HashSet::new();
    for x in lo.x..=hi.x {
        for y in lo.y..=hi.y {
            for z in lo.z..=hi.z {
                let p = sample_at([x, y, z], step);
                let Some(resampled) = resample(block, to_fragment * p) else {
                    continue;
                };
                let existing = grid.get(p);
                let Some(voxel) = resampled.union(existing, material) else {
                    continue;
                };
                grid.set(p, voxel);
                changed.push([x, y, z]);
                if voxel.density >= BEARING_DENSITY && existing.density < BEARING_DENSITY {
                    rock.insert([x, y, z]);
                }
            }
        }
    }
    changed.extend(weld(grid, &rock, material));
    changed
        .into_iter()
        .map(|c| {
            let p = sample_at(c, step);
            AABB::new(p, p)
        })
        .reduce(|a, b| a.merged(&b))
}

/// Join every piece of `rock`, the samples a deposit made bear, to bearing
/// ground: through a face it touches, or a piece already joined, or else
/// along the shortest lattice path to the nearest within [`WELD_REACH`],
/// each sample on it raised to bear. Support only travels through bearing
/// samples (`BEARING_DENSITY`), and a rock resting on the ground touches it
/// at a few points that no lattice sample need land near: unwelded, it would
/// stand free of the ground it lies on, and the next blast near it would lift
/// it again. Returns the samples it raised.
fn weld(grid: &mut ChunkGrid, rock: &HashSet<[i32; 3]>, material: VoxelMaterial) -> Vec<[i32; 3]> {
    let step = grid.voxel_size();
    let mut pieces = face_connected(rock);
    let mut joined: HashSet<[i32; 3]> = HashSet::new();
    let mut raised = Vec::new();
    loop {
        let anchor = |grid: &ChunkGrid, joined: &HashSet<[i32; 3]>, c: [i32; 3]| {
            (!rock.contains(&c) || joined.contains(&c))
                && grid.get(sample_at(c, step)).density >= BEARING_DENSITY
        };
        let touching = pieces.iter().position(|piece| {
            piece
                .iter()
                .any(|&c| face_neighbours(c).any(|n| anchor(grid, &joined, n)))
        });
        if let Some(k) = touching {
            joined.extend(pieces.swap_remove(k));
            continue;
        }
        let nearest = pieces
            .iter()
            .enumerate()
            .flat_map(|(k, piece)| piece.iter().map(move |&c| (k, c)))
            .flat_map(|(k, c)| within_reach(c).map(move |g| (k, c, g)))
            .filter(|&(_, _, g)| anchor(grid, &joined, g))
            .min_by_key(|&(_, c, g)| (0..3).map(|a| c[a].abs_diff(g[a])).sum::<u32>());
        let Some((k, from, to)) = nearest else {
            break;
        };
        for c in path_between(from, to) {
            let p = sample_at(c, step);
            let existing = grid.get(p);
            if existing.density < BEARING_DENSITY {
                let kept = if existing.is_solid() {
                    existing.material
                } else {
                    material
                };
                grid.set(
                    p,
                    Voxel {
                        density: BEARING_DENSITY,
                        material: kept,
                    },
                );
                raised.push(c);
            }
        }
        joined.extend(pieces.swap_remove(k));
    }
    raised
}

/// `samples` in face-connected pieces.
fn face_connected(samples: &HashSet<[i32; 3]>) -> Vec<Vec<[i32; 3]>> {
    let mut seen = HashSet::new();
    let mut pieces = Vec::new();
    for &start in samples {
        if !seen.insert(start) {
            continue;
        }
        let mut piece = vec![start];
        let mut next = 0;
        while next < piece.len() {
            let c = piece[next];
            next += 1;
            for n in face_neighbours(c) {
                if samples.contains(&n) && seen.insert(n) {
                    piece.push(n);
                }
            }
        }
        pieces.push(piece);
    }
    pieces
}

fn face_neighbours(c: [i32; 3]) -> impl Iterator<Item = [i32; 3]> {
    (0..6).map(move |k| {
        let mut n = c;
        n[k / 2] += if k % 2 == 0 { -1 } else { 1 };
        n
    })
}

/// The samples within [`WELD_REACH`] of `c` on every axis.
fn within_reach(c: [i32; 3]) -> impl Iterator<Item = [i32; 3]> {
    let r = WELD_REACH;
    (-r..=r).flat_map(move |x| {
        (-r..=r).flat_map(move |y| (-r..=r).map(move |z| [c[0] + x, c[1] + y, c[2] + z]))
    })
}

/// The samples strictly between `from` and `to` on a lattice path that
/// closes the vertical first, then across.
fn path_between(from: [i32; 3], to: [i32; 3]) -> Vec<[i32; 3]> {
    let mut path = Vec::new();
    let mut at = from;
    for axis in [1, 0, 2] {
        while at[axis] != to[axis] {
            at[axis] += (to[axis] - at[axis]).signum();
            if at != to {
                path.push(at);
            }
        }
    }
    path
}

fn sample_at(c: [i32; 3], step: f32) -> Point3<f32> {
    Point3::new(c[0] as f32, c[1] as f32, c[2] as f32) * step
}

/// The fragment's field at one point of its block.
struct Resampled {
    /// Blended density, bias included, clamped to a voxel either way.
    density: f32,
    /// The densest solid corner's material, if any corner is solid.
    material: Option<VoxelMaterial>,
}

impl Resampled {
    /// The voxel the union writes over `existing`, or `None` where the
    /// fragment is no denser. A deposit never lowers a density, and never
    /// turns indestructible ground into the fragment's material.
    fn union(&self, existing: Voxel, fallback: VoxelMaterial) -> Option<Voxel> {
        let mut density = self.density;
        // A sample on the iso-surface degenerates marching cubes (see
        // `SURFACE_BAND`). An authored write pushes it into the solid; here
        // that would make a speck of solid wherever the rock's field only
        // just reaches, so it goes to the air.
        if density.abs() < SURFACE_BAND {
            density = -SURFACE_BAND;
        }
        if density <= existing.density {
            return None;
        }
        let material = if density <= 0.0 {
            VoxelMaterial::Air
        } else if existing.is_solid() && existing.material.is_indestructible() {
            existing.material
        } else {
            self.material.unwrap_or(fallback)
        };
        Some(Voxel { density, material })
    }
}

/// The block's field at `q`, a point in its own positions: the eight samples
/// around it blended. `None` outside the block, or where all eight are air at
/// its fullest, which a union could only write over air as air.
fn resample(block: &VoxelBlock, q: Point3<f32>) -> Option<Resampled> {
    let lattice = block.lattice();
    let dims = lattice.dims();
    let first = lattice.position(0, 0, 0);
    let mut cell = [0usize; 3];
    let mut t = [0.0f32; 3];
    for a in 0..3 {
        if dims[a] < 2 {
            return None;
        }
        let f = (q[a] - first[a]) / lattice.spacing();
        if !(0.0..=(dims[a] - 1) as f32).contains(&f) {
            return None;
        }
        cell[a] = (f.floor() as usize).min(dims[a] - 2);
        t[a] = f - cell[a] as f32;
    }

    let mut density = 0.0;
    let mut densest: Option<Voxel> = None;
    for corner in 0..8 {
        let offset = [corner & 1, (corner >> 1) & 1, (corner >> 2) & 1];
        let voxel = block.get(
            cell[0] + offset[0],
            cell[1] + offset[1],
            cell[2] + offset[2],
        );
        let weight: f32 = (0..3)
            .map(|a| if offset[a] == 1 { t[a] } else { 1.0 - t[a] })
            .product();
        density += weight * voxel.density;
        if voxel.is_solid() && densest.is_none_or(|d| voxel.density > d.density) {
            densest = Some(voxel);
        }
    }
    if density <= -1.0 + f32::EPSILON {
        return None;
    }

    Some(Resampled {
        density: (density + DEPOSIT_BIAS).clamp(-1.0, 1.0),
        material: densest.map(|v| v.material),
    })
}

/// The grid-local box around the block, `to_grid` taking its positions to the
/// grid's.
fn to_grid_box(block: &VoxelBlock, to_grid: &Isometry3<f32>) -> AABB {
    let b = block.lattice().bounds();
    (0..8)
        .map(|k| {
            let corner = Point3::new(
                if k & 1 == 0 { b.min.x } else { b.max.x },
                if k & 2 == 0 { b.min.y } else { b.max.y },
                if k & 4 == 0 { b.min.z } else { b.max.z },
            );
            let at = to_grid * corner;
            AABB::new(at, at)
        })
        .reduce(|a, b| a.merged(&b))
        .unwrap_or_else(AABB::empty)
}

#[cfg(test)]
mod tests {
    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    use super::*;
    use crate::terrain::csg::union_solid;
    use crate::terrain::fragment::Search;
    use crate::terrain::fragment_mesh::FragmentMesh;

    /// Every test lattice has metre voxels, so distances read in voxels.
    const STEP: f32 = 1.0;

    /// Where the test shapes are built, off the lattice on every axis.
    const BUILT_AT: Point3<f32> = Point3::new(10.3, 10.6, 10.2);

    /// A shape, by its signed distance from [`BUILT_AT`].
    struct Shape {
        name: &'static str,
        sdf: fn(Vector3<f32>) -> f32,
        /// How far the shape reaches from its centre.
        reach: f32,
        /// The worst mean and 99th-percentile surface error allowed, in
        /// voxels: twice what was measured.
        mean: f32,
        p99: f32,
    }

    fn shapes() -> [Shape; 3] {
        [
            Shape {
                name: "rock",
                sdf: |v| v.norm() - 3.0 + 0.3 * (1.7 * v.x).sin() * (1.3 * v.y).cos(),
                reach: 4.0,
                mean: 0.1,
                p99: 0.3,
            },
            Shape {
                name: "shard",
                sdf: |v| {
                    [
                        Vector3::new(1.0, 0.2, 0.1),
                        Vector3::new(-0.8, 0.5, 0.3),
                        Vector3::new(0.1, -1.0, 0.4),
                        Vector3::new(0.2, 0.9, -0.6),
                        Vector3::new(-0.3, -0.4, -1.0),
                        Vector3::new(0.4, 0.1, 1.0),
                        Vector3::new(-0.6, -0.7, 0.2),
                    ]
                    .iter()
                    .map(|n| v.dot(&n.normalize()) - 2.6)
                    .fold(f32::MIN, f32::max)
                },
                reach: 7.0,
                mean: 0.1,
                p99: 0.4,
            },
            Shape {
                name: "shelf",
                sdf: |v| {
                    let q = v.abs() - Vector3::new(4.0, 1.2, 3.0);
                    q.map(|c| c.max(0.0)).norm() + q.max().min(0.0)
                },
                reach: 6.0,
                mean: 0.1,
                p99: 0.3,
            },
        ]
    }

    /// The shape built on an empty lattice and lifted out whole, where it
    /// was built.
    fn fragment_of(shape: &Shape) -> Fragment {
        let mut grid = ChunkGrid::new(STEP);
        let r = shape.reach.ceil() as i32 + 2;
        let c = BUILT_AT.coords.map(|v| v.round() as i32);
        for x in c.x - r..=c.x + r {
            for y in c.y - r..=c.y + r {
                for z in c.z - r..=c.z + r {
                    let p = Point3::new(x as f32, y as f32, z as f32) * STEP;
                    let sdf = (shape.sdf)(p - BUILT_AT);
                    union_solid(&mut grid, p, sdf, STEP, VoxelMaterial::Rock);
                }
            }
        }
        let reach = Vector3::repeat(shape.reach + 1.0);
        let region = AABB::new(BUILT_AT - reach, BUILT_AT + reach);
        Fragment::of_region(&grid, &region, Isometry3::identity())
    }

    /// A random turn about the shape's centre, then a move to somewhere off
    /// the lattice well away from where it was built.
    fn random_move(rng: &mut StdRng) -> Isometry3<f32> {
        let turn = UnitQuaternion::from_euler_angles(
            rng.gen_range(-3.1..3.1),
            rng.gen_range(-1.5..1.5),
            rng.gen_range(-3.1..3.1),
        );
        let to = Point3::new(
            40.0 + rng.gen::<f32>(),
            30.0 + rng.gen::<f32>(),
            40.0 + rng.gen::<f32>(),
        );
        Translation3::from(to.coords) * turn * Translation3::from(-BUILT_AT.coords)
    }

    /// The volume a closed mesh encloses.
    fn volume(mesh: &FragmentMesh) -> f32 {
        mesh.indices
            .chunks_exact(3)
            .map(|t| {
                let [a, b, c] = [0, 1, 2].map(|k| mesh.vertices[t[k] as usize].pos);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum::<f32>()
            .abs()
    }

    /// The grid's field at a point, as the deposit's trilinear blend reads a
    /// block: in voxels, near enough a distance within the surface band.
    fn field(grid: &ChunkGrid, p: Point3<f32>) -> f32 {
        let f = p.coords / STEP;
        let cell = f.map(f32::floor);
        let t = f - cell;
        (0..8)
            .map(|corner| {
                let offset = Vector3::new(
                    (corner & 1) as f32,
                    ((corner >> 1) & 1) as f32,
                    ((corner >> 2) & 1) as f32,
                );
                let weight: f32 = (0..3)
                    .map(|a| if offset[a] == 1.0 { t[a] } else { 1.0 - t[a] })
                    .product();
                weight * grid.get(Point3::from((cell + offset) * STEP)).density
            })
            .sum()
    }

    fn percentile(mut values: Vec<f32>, p: f32) -> f32 {
        values.sort_by(f32::total_cmp);
        values[((values.len() - 1) as f32 * p) as usize]
    }

    /// Deposited at random poses, every test shape keeps its volume to
    /// within 2% on average and 4% at worst, and its surface moves no more
    /// than the measured table allows, both ways: the old surface lies on
    /// the new field's zero, and the new surface on the old field's.
    #[test]
    fn deposits_keep_volume_and_surface_at_any_pose() {
        let mut rng = StdRng::seed_from_u64(7);
        let mut failures = Vec::new();
        for shape in shapes() {
            let fragment = fragment_of(&shape);
            let before = fragment.mesh();
            let wanted = volume(&before);
            let mut changes = Vec::new();
            let mut errors = Vec::new();
            for _ in 0..20 {
                let moved = random_move(&mut rng);
                let mut grid = ChunkGrid::new(STEP);
                let changed = deposit(&mut grid, &fragment, &moved.inverse())
                    .expect("a deposit into air changes samples");

                let pad = Vector3::repeat(2.0 * STEP);
                let region = AABB::new(changed.min - pad, changed.max + pad);
                let after = Fragment::of_region(&grid, &region, Isometry3::identity()).mesh();
                changes.push(volume(&after) / wanted - 1.0);

                for v in &before.vertices {
                    errors.push(field(&grid, moved * (before.origin + v.pos)).abs());
                }
                let back = moved.inverse();
                for v in &after.vertices {
                    let q = back * (after.origin + v.pos);
                    let old = resample(fragment.voxels(), q).map_or(-1.0, |r| r.density);
                    errors.push((old - DEPOSIT_BIAS).abs());
                }
            }
            let mean_change = changes.iter().sum::<f32>() / changes.len() as f32;
            let worst_change = changes.iter().fold(0.0f32, |w, c| w.max(c.abs()));
            let mean = errors.iter().sum::<f32>() / errors.len() as f32;
            let p99 = percentile(errors, 0.99);
            println!(
                "{}: volume {:+.2}% mean, {:.2}% worst; surface {mean:.3} mean, {p99:.3} p99",
                shape.name,
                100.0 * mean_change,
                100.0 * worst_change
            );
            let checks = [
                (mean_change.abs() < 0.02, "volume drifts"),
                (worst_change < 0.04, "a pose loses volume"),
                (mean < shape.mean, "surface moves"),
                (p99 < shape.p99, "surface moves in places"),
            ];
            failures.extend(
                checks
                    .into_iter()
                    .filter(|(ok, _)| !ok)
                    .map(|(_, what)| format!("{}: {what}", shape.name)),
            );
        }
        assert!(failures.is_empty(), "{failures:?}");
    }

    /// A ball lying a little clear of the ground, as on a skin too weak to
    /// bear, shares no bearing sample with it: deposited, it is welded to
    /// the ground, and nothing that bears stands free.
    #[test]
    fn a_ball_lying_on_the_ground_is_welded_to_it() {
        let ball = Shape {
            name: "ball",
            sdf: |v| v.norm() - 2.3,
            reach: 3.0,
            mean: 0.0,
            p99: 0.0,
        };
        let fragment = fragment_of(&ball);
        let mut rng = StdRng::seed_from_u64(3);
        for _ in 0..10 {
            let mut grid = ChunkGrid::new(STEP);
            for x in 30..=50 {
                for z in 30..=50 {
                    for y in 20..=30 {
                        let p = Point3::new(x as f32, y as f32, z as f32);
                        // Bedrock under the dirt holds it up.
                        let material = if y < 23 {
                            VoxelMaterial::Bedrock
                        } else {
                            VoxelMaterial::Dirt
                        };
                        union_solid(&mut grid, p, p.y - 26.3, STEP, material);
                    }
                }
            }
            let rest = Point3::new(
                40.0 + rng.gen::<f32>(),
                26.3 + 2.3 + 0.4,
                40.0 + rng.gen::<f32>(),
            );
            let turn = UnitQuaternion::from_euler_angles(rng.gen(), rng.gen(), rng.gen());
            let moved: Isometry3<f32> =
                Translation3::from(rest.coords) * turn * Translation3::from(-BUILT_AT.coords);
            deposit(&mut grid, &fragment, &moved.inverse()).expect("it changes samples");
            let loose = Search::audit(&grid).loose_samples();
            assert_eq!(
                loose.bearing, 0,
                "the ball at {rest} stands free: {loose:?}"
            );
        }
    }

    /// Deposited into the ground, a fragment raises densities and lowers
    /// none, and leaves indestructible ground as it was made.
    #[test]
    fn a_deposit_never_lowers_a_density() {
        let shape = &shapes()[0];
        let fragment = fragment_of(shape);
        let mut grid = ChunkGrid::new(STEP);
        for x in 30..=50 {
            for z in 30..=50 {
                for y in 20..=30 {
                    let p = Point3::new(x as f32, y as f32, z as f32);
                    let material = if x < 40 {
                        VoxelMaterial::Bedrock
                    } else {
                        VoxelMaterial::Dirt
                    };
                    union_solid(&mut grid, p, p.y - 28.4, STEP, material);
                }
            }
        }
        let before: Vec<(Point3<f32>, Voxel)> = (30..=50)
            .flat_map(|x| (20..=36).flat_map(move |y| (30..=50).map(move |z| (x, y, z))))
            .map(|(x, y, z)| Point3::new(x as f32, y as f32, z as f32))
            .map(|p| (p, grid.get(p)))
            .collect();

        let moved = Translation3::new(29.7, 19.0, 29.9);
        deposit(&mut grid, &fragment, &moved.inverse().into()).expect("it changes samples");

        let mut raised = 0;
        for (p, was) in before {
            let now = grid.get(p);
            assert!(now.density >= was.density, "{p} lowered");
            if was.is_solid() && was.material.is_indestructible() {
                assert_eq!(now.material, was.material, "{p} made destructible");
            }
            raised += usize::from(now.density > was.density);
        }
        assert!(raised > 50, "the rock was not stamped in: {raised} raised");
    }
}
