//! Dense voxel samples over a regular lattice, and the bulk interface that
//! fills them.
//!
//! Meshing needs every voxel of a padded block at once, not one voxel at a
//! time. Reading them point-by-point makes the cost proportional to the number
//! of *sample points* — one sparse-octree descent each — when the octree
//! collapses uniform regions and could instead splat a whole leaf in a single
//! write. `VoxelSource::fill_block` is that bulk path, and `VoxelBlock` is the
//! flat destination it writes into.
//!
//! ```text
//!   SampleLattice        where the samples are (origin + index * spacing)
//!         │
//!         ▼
//!   VoxelBlock  ◄──fill_block──  VoxelSource  (ChunkGrid → Chunk → SVO)
//!         │
//!         ▼
//!   MarchingCubes::generate_range
//! ```

use std::ops::Range;

use nalgebra::Point3;

use super::voxel::Voxel;
use crate::collision::AABB;

/// A regular axis-aligned grid of sample points.
///
/// A sample is addressed by a local index `0..dims[axis]`, but its position is
/// derived from a *global* lattice index `base[axis] + local`. Expressing it
/// that way makes positions bit-identical between blocks that share a sample
/// plane, which is what lets neighbouring chunks mesh without a seam.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SampleLattice {
    /// Position of global lattice index `[0, 0, 0]`.
    origin: Point3<f32>,
    /// Global lattice index of this block's local index `[0, 0, 0]`.
    base: [i32; 3],
    /// Distance between adjacent samples, on every axis.
    spacing: f32,
    /// Number of samples per axis.
    dims: [usize; 3],
}

impl SampleLattice {
    pub fn new(origin: Point3<f32>, base: [i32; 3], spacing: f32, dims: [usize; 3]) -> Self {
        debug_assert!(spacing > 0.0, "sample spacing must be positive");
        Self {
            origin,
            base,
            spacing,
            dims,
        }
    }

    /// Number of samples per axis.
    pub fn dims(&self) -> [usize; 3] {
        self.dims
    }

    /// Global lattice index of local index `[0, 0, 0]`.
    pub fn base(&self) -> [i32; 3] {
        self.base
    }

    /// Distance between adjacent samples.
    pub fn spacing(&self) -> f32 {
        self.spacing
    }

    /// Position of one axis' local sample index.
    pub fn axis_position(&self, axis: usize, index: usize) -> f32 {
        self.origin[axis] + (self.base[axis] + index as i32) as f32 * self.spacing
    }

    /// Position of a local sample index.
    pub fn position(&self, x: usize, y: usize, z: usize) -> Point3<f32> {
        Point3::new(
            self.axis_position(0, x),
            self.axis_position(1, y),
            self.axis_position(2, z),
        )
    }

    /// Box enclosing every sample point in the lattice.
    ///
    /// Empty on any axis with no samples, which cannot enclose anything.
    pub fn bounds(&self) -> AABB {
        let last = |axis: usize| self.axis_position(axis, self.dims[axis].saturating_sub(1));
        let min = self.position(0, 0, 0);
        let max = Point3::new(last(0), last(1), last(2));
        AABB::new(min.inf(&max), min.sup(&max))
    }

    /// Samples inside `region`, treating it as half-open `[min, max)` per axis.
    ///
    /// This is the form that matches a floor-based owner lookup: adjacent boxes
    /// meeting on a shared plane each claim it exactly once.
    pub fn indices_in_half_open(&self, region: &AABB) -> BlockRange {
        self.range_per_axis(region, false)
    }

    /// Samples inside `region`, inclusive of both faces on every axis.
    pub fn indices_in_closed(&self, region: &AABB) -> BlockRange {
        self.range_per_axis(region, true)
    }

    fn range_per_axis(&self, region: &AABB, closed: bool) -> BlockRange {
        let axis_range = |axis: usize| {
            let lo = self.first_index_at_least(axis, region.min[axis]);
            let hi = if closed {
                self.first_index_above(axis, region.max[axis])
            } else {
                self.first_index_at_least(axis, region.max[axis])
            };
            lo..hi.max(lo)
        };
        BlockRange::new([axis_range(0), axis_range(1), axis_range(2)])
    }

    /// First index whose position is `>= value`, or `dims[axis]` if none is.
    ///
    /// Binary search rather than arithmetic on purpose: the result must agree
    /// exactly with a `>=` comparison against the same position expression, and
    /// dividing by `spacing` would not be guaranteed to round the same way.
    pub fn first_index_at_least(&self, axis: usize, value: f32) -> usize {
        self.partition(axis, |pos| pos < value)
    }

    /// First index whose position is `> value`, or `dims[axis]` if none is.
    fn first_index_above(&self, axis: usize, value: f32) -> usize {
        self.partition(axis, |pos| pos <= value)
    }

    /// First index for which `still_below` is false. Positions increase with
    /// index, so the predicate is monotonic and binary search applies.
    fn partition(&self, axis: usize, still_below: impl Fn(f32) -> bool) -> usize {
        let (mut lo, mut hi) = (0, self.dims[axis]);
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if still_below(self.axis_position(axis, mid)) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        lo
    }
}

/// A box of local sample indices, half-open on every axis.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlockRange {
    axes: [Range<usize>; 3],
}

impl BlockRange {
    pub fn new(axes: [Range<usize>; 3]) -> Self {
        Self { axes }
    }

    /// A range covering every sample of a lattice.
    pub fn all(lattice: &SampleLattice) -> Self {
        let d = lattice.dims();
        Self::new([0..d[0], 0..d[1], 0..d[2]])
    }

    pub fn axis(&self, axis: usize) -> Range<usize> {
        self.axes[axis].clone()
    }

    /// True if any axis is empty, and therefore the box contains no samples.
    pub fn is_empty(&self) -> bool {
        self.axes.iter().any(|r| r.start >= r.end)
    }

    /// The samples in both boxes.
    pub fn intersect(&self, other: &Self) -> Self {
        let axis = |a: usize| {
            let start = self.axes[a].start.max(other.axes[a].start);
            let end = self.axes[a].end.min(other.axes[a].end);
            start..end.max(start)
        };
        Self::new([axis(0), axis(1), axis(2)])
    }
}

/// Dense voxel samples over a `SampleLattice`.
///
/// Stored flat with `z` contiguous, so filling a run along `z` — the shape a
/// bulk octree fill produces — is a single slice write, and marching cubes'
/// eight cell corners are two short strides apart instead of three pointer
/// dereferences.
pub struct VoxelBlock {
    lattice: SampleLattice,
    /// Indexed `(x * dims[1] + y) * dims[2] + z`.
    voxels: Vec<Voxel>,
}

impl VoxelBlock {
    /// An all-air block covering `lattice`.
    pub fn air(lattice: SampleLattice) -> Self {
        let d = lattice.dims();
        Self {
            lattice,
            voxels: vec![Voxel::air(); d[0] * d[1] * d[2]],
        }
    }

    pub fn lattice(&self) -> &SampleLattice {
        &self.lattice
    }

    pub fn dims(&self) -> [usize; 3] {
        self.lattice.dims()
    }

    fn index(&self, x: usize, y: usize, z: usize) -> usize {
        let d = self.lattice.dims();
        (x * d[1] + y) * d[2] + z
    }

    pub fn get(&self, x: usize, y: usize, z: usize) -> Voxel {
        self.voxels[self.index(x, y, z)]
    }

    pub fn set(&mut self, x: usize, y: usize, z: usize, voxel: Voxel) {
        let i = self.index(x, y, z);
        self.voxels[i] = voxel;
    }

    /// Write `voxel` to every sample in `range`.
    pub fn fill(&mut self, range: &BlockRange, voxel: Voxel) {
        if range.is_empty() {
            return;
        }
        let z = range.axis(2);
        for x in range.axis(0) {
            for y in range.axis(1) {
                let start = self.index(x, y, z.start);
                let end = start + (z.end - z.start);
                self.voxels[start..end].fill(voxel);
            }
        }
    }
}

/// Something that can supply voxels for a whole block of samples at once.
///
/// Implementors write only the samples they cover, leaving the rest as the
/// caller initialised them — which is how unallocated space stays air without
/// anyone having to ask about it.
pub trait VoxelSource {
    fn fill_block(&self, block: &mut VoxelBlock);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::voxel::VoxelMaterial;

    fn lattice(dims: usize) -> SampleLattice {
        SampleLattice::new(Point3::origin(), [0, 0, 0], 1.0, [dims; 3])
    }

    #[test]
    fn positions_come_from_the_global_index() {
        let l = SampleLattice::new(Point3::origin(), [10, 0, -4], 0.5, [3; 3]);
        assert_eq!(l.position(0, 0, 0), Point3::new(5.0, 0.0, -2.0));
        assert_eq!(l.position(2, 1, 0), Point3::new(6.0, 0.5, -2.0));
    }

    #[test]
    fn half_open_range_claims_a_shared_plane_once() {
        let l = lattice(5);
        let lower = AABB::new(Point3::origin(), Point3::new(2.0, 2.0, 2.0));
        let upper = AABB::new(Point3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));

        assert_eq!(l.indices_in_half_open(&lower).axis(0), 0..2);
        assert_eq!(l.indices_in_half_open(&upper).axis(0), 2..4);
    }

    #[test]
    fn closed_range_includes_the_far_face() {
        let l = lattice(5);
        let region = AABB::new(Point3::origin(), Point3::new(2.0, 2.0, 2.0));
        assert_eq!(l.indices_in_closed(&region).axis(0), 0..3);
    }

    #[test]
    fn range_outside_the_lattice_is_empty() {
        let l = lattice(5);
        let region = AABB::new(Point3::new(10.0, 10.0, 10.0), Point3::new(12.0, 12.0, 12.0));
        assert!(l.indices_in_half_open(&region).is_empty());
    }

    #[test]
    fn fill_writes_only_the_requested_box() {
        let l = lattice(4);
        let mut block = VoxelBlock::air(l);
        let solid = Voxel::solid(VoxelMaterial::Rock);
        block.fill(&BlockRange::new([1..3, 1..3, 1..3]), solid);

        assert_eq!(block.get(1, 1, 1).density, solid.density);
        assert_eq!(block.get(2, 2, 2).density, solid.density);
        assert_eq!(block.get(0, 1, 1).density, Voxel::air().density);
        assert_eq!(block.get(3, 3, 3).density, Voxel::air().density);
    }

    #[test]
    fn intersect_narrows_to_the_overlap() {
        let a = BlockRange::new([0..5, 0..5, 0..5]);
        let b = BlockRange::new([3..9, 0..2, 6..9]);
        let c = a.intersect(&b);
        assert_eq!(c.axis(0), 3..5);
        assert_eq!(c.axis(1), 0..2);
        assert!(c.is_empty());
    }
}
