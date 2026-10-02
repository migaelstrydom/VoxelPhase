//! Terrain a blast has cut loose from the ground.
//!
//! A carve removes a sphere and asks nothing about what is left. Where crater
//! rims meet, the solid between them is a cusp thinner than a voxel; where a
//! blast goes off below the surface, the layer above it can be left one
//! sample thick. Marching cubes draws both faithfully — as strips hanging in
//! the air and shelves with no thickness — because nothing ever lets them fall.
//!
//! ```text
//!   Crater::survey ──▶ the samples around the crater, before the carve
//!   carve
//!   Search::run ──▶ the same samples, after it; for each of the two:
//!               │  classify: air / weak / bearing / fixed
//!               │  flood from the block's faces and fixed samples, through
//!               │  bearing samples only ──▶ grounded
//!               │  weak samples touching grounded ones ──▶ the lip, grounded
//!               │  the rest, 6-connected ──▶ standing free
//!               ▼
//!           a piece standing free after the carve falls if, before it, it
//!           was held up — or it is not the largest piece of what stood free
//!               ▼
//!           Search ──lift──▶ grid samples set to air
//!                  ──into_fragments──▶ Vec<Fragment>  (world pose, own block)
//! ```
//!
//! Comparing with the field before the carve is what keeps authored floating
//! terrain up. An island over the ground is held up by nothing the search can
//! see, before the blast as after it; only what the blast cuts off it falls.
//!
//! `docs/TERRAIN_RUBBLE_DESIGN.md` Part 1 has the reasoning behind each rule.

use std::collections::VecDeque;

use nalgebra::{Isometry3, Point3, Translation3, Vector3};

use super::chunk_grid::ChunkGrid;
use super::frame::SegmentFrame;
use super::voxel::{Voxel, VoxelMaterial};
use super::voxel_block::{SampleLattice, VoxelBlock, VoxelSource};
use crate::collision::AABB;

/// The least density a sample needs to hold anything up.
///
/// A density is the distance to the surface in voxels, so a sample below this
/// is a rind: marching cubes still draws a closed surface around it, but one
/// thinner than half this fraction of a voxel on either side.
pub const BEARING_DENSITY: f32 = 0.25;

/// How far past the crater the search reads, as a multiple of its radius,
/// before the clamp below. Anything reaching further than this is held up by
/// whatever it reaches, so this is the size of the largest structure a blast
/// can bring down. Measured on `test_arena` at 1 m voxels: 1.5 costs 0.08 ms a
/// grenade, 4 costs 0.25 ms, and the 32-voxel ceiling everywhere 5.8 ms.
const MARGIN_PER_RADIUS: f32 = 3.0;

/// Bounds on the margin, in voxels. The floor keeps a small crater's search
/// from ending on its own rim; the ceiling caps the cost at fine resolutions.
const MIN_MARGIN_VOXELS: f32 = 6.0;
const MAX_MARGIN_VOXELS: f32 = 32.0;

/// Drawn thickness, in voxels, below which a solid reads as paper: the
/// surfaces marching cubes puts either side of a sample less than half a
/// voxel apart.
const PAPER_THIN: f32 = 0.5;

/// Air samples around a fragment's own block: one so its marching cubes
/// closes, one more for the central differences its normals take.
const FRAGMENT_PADDING: usize = 2;

/// What a sample can do for the samples around it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Role {
    Air,
    /// Solid, but too thin to hold anything up.
    Weak,
    /// Weak, and drawn so thin it reads as paper: not even kept as a lip.
    Paper,
    Bearing,
    /// A material no charge removes: always grounded, never lifted.
    Fixed,
}

impl Role {
    fn is_solid(self) -> bool {
        self != Role::Air
    }

    fn bears(self) -> bool {
        matches!(self, Role::Bearing | Role::Fixed)
    }
}

/// Flat indexing over a block's samples, `z` contiguous like `VoxelBlock`.
#[derive(Debug, Clone, Copy)]
struct Lattice3 {
    dims: [usize; 3],
}

impl Lattice3 {
    fn len(&self) -> usize {
        self.dims[0] * self.dims[1] * self.dims[2]
    }

    fn index(&self, [x, y, z]: [usize; 3]) -> usize {
        (x * self.dims[1] + y) * self.dims[2] + z
    }

    fn coords(&self, i: usize) -> [usize; 3] {
        let z = i % self.dims[2];
        let y = (i / self.dims[2]) % self.dims[1];
        let x = i / (self.dims[1] * self.dims[2]);
        [x, y, z]
    }

    fn on_boundary(&self, i: usize) -> bool {
        let c = self.coords(i);
        (0..3).any(|a| c[a] == 0 || c[a] + 1 == self.dims[a])
    }

    /// The samples sharing a face with `i`: six, fewer at the block's faces.
    fn neighbours(&self, i: usize) -> impl Iterator<Item = usize> + '_ {
        let c = self.coords(i);
        (0..3).flat_map(move |axis| {
            let below = (c[axis] > 0).then(|| {
                let mut n = c;
                n[axis] -= 1;
                self.index(n)
            });
            let above = (c[axis] + 1 < self.dims[axis]).then(|| {
                let mut n = c;
                n[axis] += 1;
                self.index(n)
            });
            below.into_iter().chain(above)
        })
    }
}

/// The samples around a crater as they were before it was carved.
pub(super) struct Crater {
    before: VoxelBlock,
    undercut: Undercut,
}

impl Crater {
    /// Read the samples a crater of `radius` at grid-local `center` will be
    /// searched over. Call before carving it.
    pub(super) fn survey(grid: &ChunkGrid, center: Point3<f32>, radius: f32) -> Self {
        let step = grid.voxel_size();
        let margin =
            (MARGIN_PER_RADIUS * radius).clamp(MIN_MARGIN_VOXELS * step, MAX_MARGIN_VOXELS * step);
        let region = AABB::from_center_half_extents(center, Vector3::repeat(radius + margin));
        Self {
            before: read_box(grid, &region),
            undercut: Undercut {
                center,
                reach: radius + 2.0 * step,
            },
        }
    }
}

/// The most times [`cut_loose`] searches one crater. Lifting a piece can strip
/// the last neighbour from a sample beside it and leave that paper-thin in
/// turn, so a crater is searched again until nothing more comes away. Over
/// `rubble_viewer`'s scenarios no blast lifted anything on a third pass; the
/// cap bounds a pathological field.
const MAX_PASSES: usize = 4;

/// What [`cut_loose`] lifted out of the grid.
pub(super) struct CutLoose {
    /// Grid-local box around every lifted sample, or `None` if none were.
    pub lifted: Option<AABB>,
    pub fragments: Vec<Fragment>,
}

/// Lift out of `grid` everything `crater`, now carved into it, cut loose, as
/// fragments placed in the world by `frame`.
pub(super) fn cut_loose(grid: &mut ChunkGrid, crater: &Crater, frame: &SegmentFrame) -> CutLoose {
    let mut lifted: Option<AABB> = None;
    let mut fragments = Vec::new();
    for _ in 0..MAX_PASSES {
        let search = Search::run(grid, crater);
        let Some(loose) = search.bounds() else {
            break;
        };
        search.lift(grid);
        lifted = Some(lifted.map_or(loose, |b| b.merged(&loose)));
        fragments.extend(search.into_fragments(frame));
    }
    CutLoose { lifted, fragments }
}

/// The samples a blast has cut loose, found but not yet lifted.
pub(super) struct Search {
    /// The samples read around the crater, as they were after the carve.
    block: VoxelBlock,
    /// Each loose piece, as flat indices into `block`.
    pieces: Vec<Vec<usize>>,
}

impl Search {
    /// Find what `crater`, now carved into `grid`, cut loose.
    pub(super) fn run(grid: &ChunkGrid, crater: &Crater) -> Self {
        let mut after = VoxelBlock::air(*crater.before.lattice());
        grid.fill_block(&mut after);
        let lattice = Lattice3 { dims: after.dims() };

        let undercut = crater.undercut;
        let (stood_free, _) = standing_free(&crater.before, &lattice, None);
        let (stand_free, roles) = standing_free(&after, &lattice, Some(undercut));
        let reached = |i: usize| {
            let [x, y, z] = lattice.coords(i);
            (after.lattice().position(x, y, z) - undercut.center).norm() <= undercut.reach
        };
        // A piece of an island goes on standing only if something in it bears,
        // or the blast never reached it: weak scraps the blast left are no
        // island, whatever they were part of.
        let can_stand = |piece: &[usize]| {
            piece.iter().any(|&i| roles[i].bears()) || !piece.iter().any(|&i| reached(i))
        };
        let pieces = fallen(stand_free, &stood_free, &lattice, can_stand);
        Self {
            block: after,
            pieces,
        }
    }

    /// Everything standing free anywhere in the grid, as an audit: a blast
    /// should never leave more of it than there was.
    ///
    /// The whole grid is read with a voxel of air around it, so nothing reaches
    /// the edge of the read and only fixed samples hold anything up. No sheet
    /// counts as undercut: with no blast to say which sheets are damage, every
    /// sheet is taken to be authored.
    pub(super) fn audit(grid: &ChunkGrid) -> Self {
        let block = read_whole(grid);
        let lattice = Lattice3 { dims: block.dims() };
        let (pieces, _) = standing_free(&block, &lattice, None);
        Self { block, pieces }
    }

    /// How many destructible samples anywhere in the grid are drawn thinner
    /// than [`PAPER_THIN`] across some axis: what reads as a sheet of paper.
    pub(super) fn paper_thin_samples(grid: &ChunkGrid) -> usize {
        let block = read_whole(grid);
        let lattice = Lattice3 { dims: block.dims() };
        (0..lattice.len())
            .map(|i| lattice.coords(i))
            .filter(|&c| {
                let voxel = block.get(c[0], c[1], c[2]);
                voxel.is_solid()
                    && !voxel.material.is_indestructible()
                    && drawn_thickness(&block, &lattice, c) < PAPER_THIN
            })
            .count()
    }

    /// How many samples are loose, over every piece.
    pub(super) fn loose_samples(&self) -> usize {
        self.pieces.iter().map(Vec::len).sum()
    }

    /// Grid-local box around every loose sample, or `None` if there are none.
    pub(super) fn bounds(&self) -> Option<AABB> {
        let lattice = Lattice3 {
            dims: self.block.dims(),
        };
        self.pieces
            .iter()
            .flatten()
            .map(|&i| {
                let [x, y, z] = lattice.coords(i);
                self.block.lattice().position(x, y, z)
            })
            .fold(None, |bounds: Option<AABB>, p| {
                let point = AABB::new(p, p);
                Some(bounds.map_or(point, |b| b.merged(&point)))
            })
    }

    /// Remove every loose sample from the grid.
    pub(super) fn lift(&self, grid: &mut ChunkGrid) {
        let lattice = Lattice3 {
            dims: self.block.dims(),
        };
        for &i in self.pieces.iter().flatten() {
            let [x, y, z] = lattice.coords(i);
            grid.set(self.block.lattice().position(x, y, z), Voxel::air());
        }
    }

    /// The loose pieces as fragments placed in the world by `frame`.
    pub(super) fn into_fragments(self, frame: &SegmentFrame) -> Vec<Fragment> {
        let pose =
            Isometry3::from_parts(Translation3::from(frame.origin().coords), frame.rotation());
        self.pieces
            .iter()
            .map(|piece| Fragment::cut(&self.block, piece, pose))
            .collect()
    }
}

/// Where a blast may have left sheets that hold nothing up. The carve changes
/// samples out to a voxel past its radius (a density is a clamped distance),
/// so a sample a voxel further out can have lost the neighbours either side of
/// it.
#[derive(Debug, Clone, Copy)]
struct Undercut {
    center: Point3<f32>,
    reach: f32,
}

/// Read every sample the grid holds, with a voxel of air around them; an empty
/// block for an empty grid.
fn read_whole(grid: &ChunkGrid) -> VoxelBlock {
    let step = grid.voxel_size();
    match grid.allocated_bounds() {
        Some(bounds) => {
            let pad = Vector3::repeat(step);
            read_box(grid, &AABB::new(bounds.min - pad, bounds.max + pad))
        }
        None => VoxelBlock::air(SampleLattice::new(Point3::origin(), [0; 3], step, [0; 3])),
    }
}

/// Read every sample in a grid-local box into one block.
fn read_box(grid: &ChunkGrid, region: &AABB) -> VoxelBlock {
    let step = grid.voxel_size();
    let lo = (region.min.coords / step).map(|v| v.floor() as i32);
    let hi = (region.max.coords / step).map(|v| v.ceil() as i32);
    let dims = [0, 1, 2].map(|a| (hi[a] - lo[a] + 1) as usize);

    let lattice = SampleLattice::new(Point3::origin(), [lo.x, lo.y, lo.z], step, dims);
    let mut block = VoxelBlock::air(lattice);
    grid.fill_block(&mut block);
    block
}

/// What each sample can bear.
///
/// A sample is a sheet sample when, on some axis, neither neighbour is solid:
/// a layer one sample thick. Near the crater that is a shelf the blast has
/// undercut, and it holds nothing up. Further out the same shape is authored
/// geometry — a thin deck — which this blast did not touch and must not drop.
///
/// Near the crater, a weak sample that marching cubes would draw paper-thin is
/// not kept even as a lip: a lip that thin is the very flap this module exists
/// to remove.
fn classify(block: &VoxelBlock, lattice: &Lattice3, undercut: Option<Undercut>) -> Vec<Role> {
    let solid = |c: [usize; 3]| block.get(c[0], c[1], c[2]).is_solid();

    (0..lattice.len())
        .map(|i| {
            let c = lattice.coords(i);
            let voxel = block.get(c[0], c[1], c[2]);
            if !voxel.is_solid() {
                return Role::Air;
            }
            if voxel.material.is_indestructible() {
                return Role::Fixed;
            }
            let near = undercut.is_some_and(|u| {
                (block.lattice().position(c[0], c[1], c[2]) - u.center).norm() <= u.reach
            });
            if voxel.density >= BEARING_DENSITY && !(near && is_sheet(c, lattice, &solid)) {
                return Role::Bearing;
            }
            if near && drawn_thickness(block, lattice, c) < PAPER_THIN {
                return Role::Paper;
            }
            Role::Weak
        })
        .collect()
}

/// Whether, on some axis, neither neighbour of `c` is solid. A neighbour off
/// the block counts as solid: the block's faces are not evidence of anything.
fn is_sheet(c: [usize; 3], lattice: &Lattice3, solid: &impl Fn([usize; 3]) -> bool) -> bool {
    (0..3).any(|axis| {
        let side = |up: bool| {
            let mut n = c;
            if up {
                if n[axis] + 1 >= lattice.dims[axis] {
                    return true;
                }
                n[axis] += 1;
            } else {
                if n[axis] == 0 {
                    return true;
                }
                n[axis] -= 1;
            }
            solid(n)
        };
        !side(false) && !side(true)
    })
}

/// How thick marching cubes draws the solid at `c` across its thinnest axis,
/// in voxels: on an axis with air on both sides, the distance between the two
/// surfaces it interpolates either side of the sample. A whole voxel or more
/// on any axis with solid on a side.
fn drawn_thickness(block: &VoxelBlock, lattice: &Lattice3, c: [usize; 3]) -> f32 {
    let density = |c: [usize; 3]| block.get(c[0], c[1], c[2]).density;
    let here = density(c);
    (0..3)
        .filter_map(|axis| {
            if c[axis] == 0 || c[axis] + 1 >= lattice.dims[axis] {
                return None;
            }
            let (mut lo, mut hi) = (c, c);
            lo[axis] -= 1;
            hi[axis] += 1;
            let (below, above) = (density(lo), density(hi));
            if below > 0.0 || above > 0.0 {
                return None;
            }
            Some(here / (here - below) + here / (here - above))
        })
        .fold(f32::INFINITY, f32::min)
}

/// Which samples are held up: those reached through bearing samples from the
/// block's faces or from a fixed sample, and the weak lip touching them.
fn ground(roles: &[Role], lattice: &Lattice3) -> Vec<bool> {
    let mut grounded = vec![false; roles.len()];
    let mut queue: VecDeque<usize> = (0..roles.len())
        .filter(|&i| {
            roles[i] == Role::Fixed || (roles[i] == Role::Bearing && lattice.on_boundary(i))
        })
        .collect();
    for &i in &queue {
        grounded[i] = true;
    }
    while let Some(i) = queue.pop_front() {
        for n in lattice.neighbours(i) {
            if !grounded[n] && roles[n].bears() {
                grounded[n] = true;
                queue.push_back(n);
            }
        }
    }

    // One step only: a shelf breaks a voxel out from the cliff, not flush with
    // it, and a weak strip keeps its root but not its length.
    let lip: Vec<usize> = (0..roles.len())
        .filter(|&i| roles[i] == Role::Weak)
        .filter(|&i| {
            lattice
                .neighbours(i)
                .any(|n| grounded[n] && roles[n].bears())
        })
        .collect();
    for i in lip {
        grounded[i] = true;
    }
    grounded
}

/// The solid samples of `block` that nothing holds up, as 6-connected pieces,
/// and the role of every sample.
fn standing_free(
    block: &VoxelBlock,
    lattice: &Lattice3,
    undercut: Option<Undercut>,
) -> (Vec<Vec<usize>>, Vec<Role>) {
    let roles = classify(block, lattice, undercut);
    let grounded = ground(&roles, lattice);
    (loose_pieces(&roles, &grounded, lattice), roles)
}

/// The solid samples not held up, split into 6-connected pieces.
///
/// A piece that reaches the block's faces continues past what was read, so it
/// is held up by whatever it reaches. Paper-thin samples join only each other:
/// a flap on the side of a floating island is its own piece, not part of the
/// island.
fn loose_pieces(roles: &[Role], grounded: &[bool], lattice: &Lattice3) -> Vec<Vec<usize>> {
    let mut seen = grounded.to_vec();
    let mut pieces = Vec::new();

    for start in 0..roles.len() {
        if seen[start] || !roles[start].is_solid() {
            continue;
        }
        let paper = roles[start] == Role::Paper;
        let mut piece = vec![start];
        let mut reaches_edge = false;
        seen[start] = true;
        let mut cursor = 0;
        while cursor < piece.len() {
            let i = piece[cursor];
            cursor += 1;
            reaches_edge |= lattice.on_boundary(i);
            for n in lattice.neighbours(i) {
                if !seen[n] && roles[n].is_solid() && (roles[n] == Role::Paper) == paper {
                    seen[n] = true;
                    piece.push(n);
                }
            }
        }
        if !reaches_edge {
            pieces.push(piece);
        }
    }
    pieces
}

/// Which of the pieces standing free after the carve fall.
///
/// A carve only removes solid, so every sample of a piece after it was solid
/// before it, and the piece lay within one thing before: held up, or standing
/// free already. Cut off what was held up, it falls. Cut out of what stood free
/// already — an authored island — the largest piece that `can_stand` goes on
/// standing as the island did, and the rest fall.
fn fallen(
    after: Vec<Vec<usize>>,
    before: &[Vec<usize>],
    lattice: &Lattice3,
    can_stand: impl Fn(&[usize]) -> bool,
) -> Vec<Vec<usize>> {
    const HELD: usize = usize::MAX;
    let mut origin = vec![HELD; lattice.len()];
    for (index, piece) in before.iter().enumerate() {
        for &i in piece {
            origin[i] = index;
        }
    }

    let mut fall = Vec::new();
    let mut largest_of: Vec<Option<Vec<usize>>> = vec![None; before.len()];
    for piece in after {
        if piece.iter().any(|&i| origin[i] == HELD) || !can_stand(&piece) {
            fall.push(piece);
            continue;
        }
        match origin[piece[0]] {
            HELD => unreachable!("checked above"),
            island => match &mut largest_of[island] {
                Some(kept) if kept.len() >= piece.len() => fall.push(piece),
                Some(kept) => fall.push(std::mem::replace(kept, piece)),
                slot @ None => *slot = Some(piece),
            },
        }
    }
    fall
}

/// A piece of terrain cut loose by a blast, lifted out of the field.
///
/// It keeps its own samples on the lattice it was cut from, so its surface can
/// be rebuilt exactly as it was, and where it was in the world when it broke.
pub struct Fragment {
    /// The fragment's samples and only those; every other sample is air.
    /// Padded by [`FRAGMENT_PADDING`] samples on every side.
    voxels: VoxelBlock,
    /// Takes the block's grid-local positions to the world, as they were at
    /// the moment of the blast.
    pose: Isometry3<f32>,
    /// How many of the block's samples belong to the fragment.
    samples: usize,
}

impl Fragment {
    /// Copy `piece`'s samples out of `region` into a block of their own.
    fn cut(region: &VoxelBlock, piece: &[usize], pose: Isometry3<f32>) -> Self {
        let lattice = Lattice3 {
            dims: region.dims(),
        };
        let coords: Vec<[usize; 3]> = piece.iter().map(|&i| lattice.coords(i)).collect();
        let lo = [0, 1, 2].map(|a| coords.iter().map(|c| c[a]).min().unwrap_or(0));
        let hi = [0, 1, 2].map(|a| coords.iter().map(|c| c[a]).max().unwrap_or(0));

        let source = region.lattice();
        let base = [0, 1, 2].map(|a| source.base()[a] + lo[a] as i32 - FRAGMENT_PADDING as i32);
        let dims = [0, 1, 2].map(|a| hi[a] - lo[a] + 1 + 2 * FRAGMENT_PADDING);
        let own = SampleLattice::new(Point3::origin(), base, source.spacing(), dims);

        let mut voxels = VoxelBlock::air(own);
        for c in &coords {
            let at = [0, 1, 2].map(|a| c[a] - lo[a] + FRAGMENT_PADDING);
            voxels.set(at[0], at[1], at[2], region.get(c[0], c[1], c[2]));
        }
        Self {
            voxels,
            pose,
            samples: piece.len(),
        }
    }

    /// How many lattice samples the fragment is made of.
    pub fn sample_count(&self) -> usize {
        self.samples
    }

    /// Edge length of one voxel of the lattice it was cut from.
    pub fn voxel_size(&self) -> f32 {
        self.voxels.lattice().spacing()
    }

    /// Volume of solid it carries: one voxel per sample.
    pub fn volume(&self) -> f32 {
        self.samples as f32 * self.voxel_size().powi(3)
    }

    /// Mean world position of its samples at the moment of the blast.
    pub fn world_centroid(&self) -> Point3<f32> {
        let sum = self
            .solid_samples()
            .fold(Vector3::zeros(), |sum, (p, _)| sum + p.coords);
        self.pose * Point3::from(sum / self.samples.max(1) as f32)
    }

    /// What most of it is made of.
    pub fn material(&self) -> VoxelMaterial {
        let mut counts: Vec<(VoxelMaterial, usize)> = Vec::new();
        for (_, voxel) in self.solid_samples() {
            match counts.iter_mut().find(|(m, _)| *m == voxel.material) {
                Some((_, n)) => *n += 1,
                None => counts.push((voxel.material, 1)),
            }
        }
        counts
            .into_iter()
            .max_by_key(|&(_, n)| n)
            .map_or(VoxelMaterial::Air, |(m, _)| m)
    }

    /// Grid-local position and voxel of every sample it is made of.
    fn solid_samples(&self) -> impl Iterator<Item = (Point3<f32>, Voxel)> + '_ {
        let [nx, ny, nz] = self.voxels.dims();
        (0..nx).flat_map(move |x| {
            (0..ny).flat_map(move |y| {
                (0..nz).filter_map(move |z| {
                    let voxel = self.voxels.get(x, y, z);
                    voxel
                        .is_solid()
                        .then(|| (self.voxels.lattice().position(x, y, z), voxel))
                })
            })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Half-width of the ground in every test: wider than any search reads, so
    /// the ground always reaches the search's faces.
    const GROUND_HALF_WIDTH: i32 = 32;

    fn put(grid: &mut ChunkGrid, at: [i32; 3], density: f32, material: VoxelMaterial) {
        let p = Point3::new(at[0] as f32, at[1] as f32, at[2] as f32);
        grid.set(p, Voxel { density, material });
    }

    /// Every sample in an inclusive box of lattice indices.
    fn cells(lo: [i32; 3], hi: [i32; 3]) -> impl Iterator<Item = [i32; 3]> {
        (lo[0]..=hi[0]).flat_map(move |x| {
            (lo[1]..=hi[1]).flat_map(move |y| (lo[2]..=hi[2]).map(move |z| [x, y, z]))
        })
    }

    fn fill(grid: &mut ChunkGrid, lo: [i32; 3], hi: [i32; 3], density: f32) {
        for c in cells(lo, hi) {
            put(grid, c, density, VoxelMaterial::Rock);
        }
    }

    fn clear(grid: &mut ChunkGrid, lo: [i32; 3], hi: [i32; 3]) {
        for c in cells(lo, hi) {
            grid.set(
                Point3::new(c[0] as f32, c[1] as f32, c[2] as f32),
                Voxel::air(),
            );
        }
    }

    /// Ground from y = -8 up to a top layer at y = 0 whose surface is half a
    /// voxel above it, the way a heightfield leaves it.
    fn ground(g: &mut ChunkGrid) {
        let w = GROUND_HALF_WIDTH;
        fill(g, [-w, -8, -w], [w, -1, w], 1.0);
        fill(g, [-w, 0, -w], [w, 0, w], 0.5);
    }

    /// What a crater at `center` cut loose from the field `build` makes.
    ///
    /// The field before the blast is the same one with the crater still solid:
    /// whatever the crater touches was joined through it.
    fn blast(build: impl Fn(&mut ChunkGrid), center: [f32; 3], radius: f32) -> Search {
        let filled = |g: &mut ChunkGrid| {
            build(g);
            let reach = radius.ceil() as i32;
            let c = center.map(|v| v.round() as i32);
            let lo = [c[0] - reach, c[1] - reach, c[2] - reach];
            let hi = [c[0] + reach, c[1] + reach, c[2] + reach];
            for at in cells(lo, hi) {
                let d = (0..3)
                    .map(|a| (at[a] as f32 - center[a]).powi(2))
                    .sum::<f32>();
                if d.sqrt() <= radius {
                    put(g, at, 1.0, VoxelMaterial::Rock);
                }
            }
        };
        blast_between(filled, &build, center, radius)
    }

    /// What a crater at `center` cut loose, given the field before and after.
    fn blast_between(
        before: impl Fn(&mut ChunkGrid),
        after: impl Fn(&mut ChunkGrid),
        center: [f32; 3],
        radius: f32,
    ) -> Search {
        let center = Point3::new(center[0], center[1], center[2]);
        let mut was = ChunkGrid::new(1.0);
        before(&mut was);
        let mut is = ChunkGrid::new(1.0);
        after(&mut is);
        Search::run(&is, &Crater::survey(&was, center, radius))
    }

    fn piece_sizes(s: &Search) -> Vec<usize> {
        let mut sizes: Vec<usize> = s.pieces.iter().map(Vec::len).collect();
        sizes.sort_unstable();
        sizes
    }

    #[test]
    fn a_crater_in_open_ground_cuts_nothing_loose() {
        let s = blast(
            |g| {
                ground(g);
                clear(g, [-2, -2, -2], [2, 0, 2]);
            },
            [0.0, 0.0, 0.0],
            2.5,
        );
        assert!(s.pieces.is_empty());
    }

    /// The screenshot's floating strip: a row of samples barely inside a
    /// surface, left attached to nothing.
    #[test]
    fn a_sliver_left_floating_is_cut_loose() {
        let s = blast(
            |g| {
                ground(g);
                for x in -3..=3 {
                    put(g, [x, 3, 0], 0.05, VoxelMaterial::Grass);
                }
            },
            [0.0, 2.0, 0.0],
            2.0,
        );
        assert_eq!(piece_sizes(&s), vec![7]);
    }

    #[test]
    fn a_block_left_floating_is_cut_loose() {
        let s = blast(
            |g| {
                ground(g);
                fill(g, [-1, 4, -1], [1, 6, 1], 1.0);
            },
            [0.0, 3.0, 0.0],
            2.0,
        );
        assert_eq!(piece_sizes(&s), vec![27]);
    }

    /// The screenshot's shelf: the top layer left one sample thick over a
    /// cavity the blast opened under it. It breaks off a voxel in from the rim.
    #[test]
    fn an_undercut_shelf_breaks_off_inside_a_lip() {
        let s = blast(
            |g| {
                ground(g);
                clear(g, [-3, -3, -3], [3, -1, 3]);
            },
            [0.0, -2.0, 0.0],
            5.0,
        );
        assert_eq!(piece_sizes(&s), vec![25], "the 5 x 5 inside the lip falls");
    }

    /// A sheet that the blast did not reach is authored geometry, not damage.
    #[test]
    fn a_thin_deck_away_from_the_blast_stands() {
        let s = blast(
            |g| {
                ground(g);
                fill(g, [-7, 1, -1], [-7, 2, 1], 1.0);
                fill(g, [-7, 3, -1], [-4, 3, 1], 1.0);
            },
            [0.0, 0.0, 0.0],
            2.0,
        );
        assert!(s.pieces.is_empty());
    }

    /// A bar two samples thick that a blast skimmed down to a sliver is no
    /// sheet, but too thin to bear: it keeps its root on the wall and loses
    /// the rest.
    #[test]
    fn a_thick_sliver_off_a_wall_falls() {
        let bar = |density: f32| {
            move |g: &mut ChunkGrid| {
                ground(g);
                fill(g, [-6, 1, 3], [-4, 6, 6], 1.0);
                for c in cells([-3, 4, 4], [3, 5, 5]) {
                    put(g, c, density, VoxelMaterial::Grass);
                }
            }
        };
        let s = blast_between(bar(1.0), bar(0.05), [0.0, 6.5, 4.5], 1.0);
        assert_eq!(piece_sizes(&s), vec![24]);
    }

    /// The same sliver, already that thin before a blast nearby, is part of
    /// the field as authored, and stays.
    #[test]
    fn a_sliver_the_blast_did_not_thin_stays() {
        let bar = |g: &mut ChunkGrid| {
            ground(g);
            fill(g, [-6, 1, 3], [-4, 6, 6], 1.0);
            for c in cells([-3, 4, 4], [3, 5, 5]) {
                put(g, c, 0.05, VoxelMaterial::Grass);
            }
        };
        assert!(blast_between(bar, bar, [0.0, 0.0, 0.0], 1.0)
            .pieces
            .is_empty());
    }

    /// A weak strip off a wall keeps the sample at its root, out of the
    /// blast's reach, and loses the rest.
    #[test]
    fn a_weak_strip_keeps_its_root() {
        let s = blast(
            |g| {
                ground(g);
                fill(g, [-3, 1, -3], [-1, 4, 3], 1.0);
                for x in 0..=4 {
                    put(g, [x, 3, 0], 0.1, VoxelMaterial::Grass);
                }
            },
            [3.0, 1.0, 0.0],
            1.0,
        );
        assert_eq!(piece_sizes(&s), vec![4]);
    }

    /// The same strip with the blast close enough to its root: a root drawn
    /// paper-thin is a flap, not a lip, and goes with the rest.
    #[test]
    fn a_paper_thin_root_is_no_lip() {
        let s = blast(
            |g| {
                ground(g);
                fill(g, [-3, 1, -3], [-1, 4, 3], 1.0);
                for x in 0..=4 {
                    put(g, [x, 3, 0], 0.1, VoxelMaterial::Grass);
                }
            },
            [1.0, 1.5, 0.0],
            1.5,
        );
        assert_eq!(piece_sizes(&s), vec![5]);
    }

    /// Two samples meeting along an edge carry no load between them.
    #[test]
    fn a_block_joined_only_along_an_edge_falls() {
        let s = blast(
            |g| {
                ground(g);
                fill(g, [-1, 1, -1], [1, 3, 1], 1.0);
                fill(g, [2, 4, -1], [4, 6, 1], 1.0);
            },
            [2.0, 3.0, 0.0],
            1.0,
        );
        assert_eq!(piece_sizes(&s), vec![27]);
    }

    /// Assumption A: what reaches the edge of the search is held up by
    /// whatever it reaches, however thin the cut below it.
    #[test]
    fn a_column_running_out_of_the_search_stands() {
        let s = blast(
            |g| {
                ground(g);
                fill(g, [-1, 3, -1], [1, 60, 1], 1.0);
            },
            [0.0, 2.0, 0.0],
            1.0,
        );
        assert!(s.pieces.is_empty());
    }

    /// The same for a bar too weak to bear but not paper-thin: it carries on
    /// past the search, and the search cannot tell what it hangs from there.
    #[test]
    fn a_weak_bar_running_out_of_the_search_stays() {
        let s = blast(
            |g| {
                ground(g);
                for c in cells([-3, 3, 0], [60, 4, 1]) {
                    put(g, c, 0.05, VoxelMaterial::Grass);
                }
            },
            [0.0, 2.0, 0.0],
            2.0,
        );
        assert!(s.pieces.is_empty());
    }

    /// An island over the ground stands on nothing before the blast as after
    /// it. A blast nowhere near it must not bring it down.
    #[test]
    fn an_island_the_blast_did_not_touch_stands() {
        let s = blast(
            |g| {
                ground(g);
                fill(g, [-2, 6, -2], [2, 7, 2], 1.0);
            },
            [5.0, 0.0, 0.0],
            1.5,
        );
        assert!(s.pieces.is_empty());
    }

    /// A blast at an island's edge chips it: the chip falls, the island stays.
    #[test]
    fn an_island_keeps_its_largest_piece_and_loses_the_chip() {
        let s = blast(
            |g| {
                ground(g);
                fill(g, [-1, 6, -2], [3, 8, 2], 1.0);
                fill(g, [6, 6, 0], [7, 7, 1], 1.0);
            },
            [5.0, 7.0, 0.0],
            1.5,
        );
        assert_eq!(piece_sizes(&s), vec![8]);
    }

    /// A flap the blast thinned to paper on the side of an island is its own
    /// piece: it falls, and the island it hung from stays.
    #[test]
    fn a_paper_flap_on_an_island_falls_alone() {
        let island = |flap: f32| {
            move |g: &mut ChunkGrid| {
                ground(g);
                fill(g, [-1, 6, -2], [3, 8, 2], 1.0);
                put(g, [4, 7, 0], flap, VoxelMaterial::Rock);
            }
        };
        let s = blast_between(island(1.0), island(0.05), [5.0, 7.0, 0.0], 1.0);
        assert_eq!(piece_sizes(&s), vec![1]);
    }

    /// A scrap of an island that the blast left too thin to bear is no island,
    /// even when it is all that stood free there.
    #[test]
    fn a_weak_scrap_the_blast_reached_falls() {
        let s = blast_between(
            |g: &mut ChunkGrid| {
                ground(g);
                put(g, [0, 4, 0], 0.1, VoxelMaterial::Grass);
                put(g, [0, 4, 1], 0.1, VoxelMaterial::Grass);
            },
            |g: &mut ChunkGrid| {
                ground(g);
                put(g, [0, 4, 0], 0.1, VoxelMaterial::Grass);
            },
            [0.0, 4.0, 2.0],
            1.0,
        );
        assert_eq!(piece_sizes(&s), vec![1]);
    }

    /// An island segment read whole has nothing to hang from at all; it holds
    /// itself up the same way.
    #[test]
    fn an_island_with_no_ground_keeps_its_largest_piece() {
        let s = blast(
            |g| {
                fill(g, [-1, -2, -2], [3, 2, 2], 1.0);
                fill(g, [6, 0, 0], [7, 1, 1], 1.0);
            },
            [5.0, 0.0, 0.0],
            1.5,
        );
        assert_eq!(piece_sizes(&s), vec![8]);
    }

    #[test]
    fn what_rests_on_bedrock_is_held_up() {
        let s = blast(
            |g| {
                for c in cells([-2, 0, -2], [2, 0, 2]) {
                    put(g, c, 1.0, VoxelMaterial::Bedrock);
                }
                fill(g, [-1, 1, -1], [1, 2, 1], 1.0);
            },
            [0.0, 3.0, 0.0],
            1.0,
        );
        assert!(s.pieces.is_empty());
    }

    /// Lifting removes exactly the loose samples and nothing else, and the
    /// fragments carry every one of them.
    #[test]
    fn lifting_moves_every_loose_sample_into_a_fragment() {
        let build = |g: &mut ChunkGrid| {
            ground(g);
            fill(g, [-1, 4, -1], [1, 6, 1], 1.0);
            for x in -3..=3 {
                put(g, [x, 8, 0], 0.05, VoxelMaterial::Grass);
            }
        };
        let s = blast(build, [0.0, 4.0, 0.0], 4.5);
        let mut g = ChunkGrid::new(1.0);
        build(&mut g);
        s.lift(&mut g);

        let mut untouched = ChunkGrid::new(1.0);
        ground(&mut untouched);
        for c in cells([-4, -2, -4], [4, 10, 4]) {
            let p = Point3::new(c[0] as f32, c[1] as f32, c[2] as f32);
            assert_eq!(g.get(p), untouched.get(p), "sample {c:?} after the lift");
        }
        let fragments = s.into_fragments(&SegmentFrame::identity());
        let carried: Vec<usize> = fragments
            .iter()
            .map(|f| f.solid_samples().count())
            .collect();
        let counted: Vec<usize> = fragments.iter().map(Fragment::sample_count).collect();
        assert_eq!(counted, carried);
        assert_eq!(carried.iter().sum::<usize>(), 27 + 7);
    }

    #[test]
    fn a_fragment_is_placed_where_its_segment_is() {
        let s = blast(
            |g| {
                ground(g);
                fill(g, [-1, 4, -1], [1, 6, 1], 1.0);
            },
            [0.0, 3.0, 0.0],
            2.0,
        );
        let frame = SegmentFrame::new(Point3::new(100.0, 0.0, -50.0), 1);
        let fragments = s.into_fragments(&frame);
        assert_eq!(fragments.len(), 1);
        let expected = frame.to_world(Point3::new(0.0, 5.0, 0.0));
        assert!((fragments[0].world_centroid() - expected).norm() < 1e-4);
        assert_eq!(fragments[0].material(), VoxelMaterial::Rock);
        assert_eq!(fragments[0].volume(), 27.0);
    }

    #[test]
    fn the_audit_finds_what_stands_free_anywhere() {
        let mut g = ChunkGrid::new(1.0);
        for c in cells([-8, -1, -8], [8, -1, 8]) {
            put(&mut g, c, 1.0, VoxelMaterial::Bedrock);
        }
        fill(&mut g, [-8, 0, -8], [8, 0, 8], 0.5);
        fill(&mut g, [-1, 4, -1], [1, 6, 1], 1.0);
        assert_eq!(Search::audit(&g).loose_samples(), 27);
    }
}
