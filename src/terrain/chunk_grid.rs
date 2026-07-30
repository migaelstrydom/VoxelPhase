//! A sparse lattice of voxel chunks in a single coordinate frame.
//!
//! `ChunkGrid` is the voxel-storage half of the terrain: it owns the chunks,
//! converts between grid-local positions and chunk coordinates, and allocates
//! chunks on demand so cost is proportional to what was written rather than to
//! any declared world extent.
//!
//! ```text
//!    grid-local position  ──floor(p / extent)──▶  ChunkCoord  ──map──▶  Chunk
//!                         ◀──── chunk_bounds ────
//! ```
//!
//! # Coordinate frames
//!
//! Everything in this module is **grid-local**. The grid's `origin` is where
//! local `(0, 0, 0)` sits in world space; callers convert at the boundary. Today
//! there is exactly one grid and its origin is the world origin, but keeping the
//! frames distinct is what lets a later stage place several grids independently
//! without their contents changing.

use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;

use super::chunk::{Chunk, ChunkCoord, CHUNK_VOXELS};
use super::voxel::Voxel;
use crate::collision::AABB;

/// A sparse grid of fixed-size voxel chunks sharing one coordinate frame and
/// one voxel resolution.
pub struct ChunkGrid {
    /// Allocated chunks, keyed by lattice coordinate. Absent means all-air.
    chunks: FxHashMap<ChunkCoord, Chunk>,

    /// World position of grid-local `(0, 0, 0)`.
    origin: Point3<f32>,

    /// Edge length of one voxel, in world units.
    voxel_size: f32,
}

impl ChunkGrid {
    /// Create an empty grid whose local origin sits at `origin` in world space.
    pub fn new(origin: Point3<f32>, voxel_size: f32) -> Self {
        assert!(voxel_size > 0.0, "voxel_size must be positive");
        Self {
            chunks: FxHashMap::default(),
            origin,
            voxel_size,
        }
    }

    // === Frame and resolution ===

    /// World position of grid-local `(0, 0, 0)`.
    pub fn origin(&self) -> Point3<f32> {
        self.origin
    }

    /// Edge length of one voxel, in world units.
    pub fn voxel_size(&self) -> f32 {
        self.voxel_size
    }

    /// Edge length of one chunk, in world units.
    pub fn chunk_extent(&self) -> f32 {
        CHUNK_VOXELS as f32 * self.voxel_size
    }

    /// Convert a world position into this grid's local frame.
    pub fn to_local(&self, world: Point3<f32>) -> Point3<f32> {
        world - self.origin.coords
    }

    /// Convert a grid-local position into world space.
    pub fn to_world(&self, local: Point3<f32>) -> Point3<f32> {
        local + self.origin.coords
    }

    /// Convert a grid-local AABB into world space.
    pub fn aabb_to_world(&self, local: &AABB) -> AABB {
        AABB::new(self.to_world(local.min), self.to_world(local.max))
    }

    /// Convert a world AABB into this grid's local frame.
    pub fn aabb_to_local(&self, world: &AABB) -> AABB {
        AABB::new(self.to_local(world.min), self.to_local(world.max))
    }

    // === Coordinate conversion ===

    /// The chunk containing a grid-local position.
    ///
    /// Uses a floor, so negative coordinates land in the chunk below zero rather
    /// than being truncated toward it: local `-0.5` with a 32 m extent is chunk
    /// `-1`, not chunk `0`.
    pub fn coord_at(&self, local: Point3<f32>) -> ChunkCoord {
        let extent = self.chunk_extent();
        ChunkCoord::new(
            (local.x / extent).floor() as i32,
            (local.y / extent).floor() as i32,
            (local.z / extent).floor() as i32,
        )
    }

    /// Grid-local bounds of a chunk coordinate.
    pub fn chunk_bounds(&self, coord: ChunkCoord) -> AABB {
        let extent = self.chunk_extent();
        let min = Point3::new(
            coord.x as f32 * extent,
            coord.y as f32 * extent,
            coord.z as f32 * extent,
        );
        AABB::new(min, min + Vector3::new(extent, extent, extent))
    }

    /// World-space bounds of a chunk coordinate.
    pub fn chunk_world_bounds(&self, coord: ChunkCoord) -> AABB {
        self.aabb_to_world(&self.chunk_bounds(coord))
    }

    /// Grid-local position of a chunk's minimum corner, in voxel lattice indices.
    ///
    /// Sample index `i` on an axis denotes local position `i * voxel_size`.
    pub fn first_sample(&self, coord: ChunkCoord) -> [i32; 3] {
        let n = CHUNK_VOXELS as i32;
        [coord.x * n, coord.y * n, coord.z * n]
    }

    /// Inclusive range of chunk coordinates whose bounds intersect a grid-local
    /// AABB, as `(min, max)`.
    pub fn coord_range(&self, local: &AABB) -> (ChunkCoord, ChunkCoord) {
        (self.coord_at(local.min), self.coord_at(local.max))
    }

    /// Iterate the chunk coordinates whose bounds intersect a grid-local AABB.
    pub fn coords_in(&self, local: &AABB) -> impl Iterator<Item = ChunkCoord> {
        let (lo, hi) = self.coord_range(local);
        (lo.x..=hi.x).flat_map(move |x| {
            (lo.y..=hi.y).flat_map(move |y| (lo.z..=hi.z).map(move |z| ChunkCoord::new(x, y, z)))
        })
    }

    // === Voxel access ===

    /// Voxel at a grid-local position. Unallocated space reads as air.
    pub fn get(&self, local: Point3<f32>) -> Voxel {
        match self.chunks.get(&self.coord_at(local)) {
            Some(chunk) => chunk.voxel_at(local),
            None => Voxel::air(),
        }
    }

    /// Write a voxel at a grid-local position, allocating the chunk if needed.
    pub fn set(&mut self, local: Point3<f32>, voxel: Voxel) {
        let coord = self.coord_at(local);
        self.chunk_or_insert(coord).set_voxel(local, voxel);
    }

    // === Chunk access ===

    pub fn chunk(&self, coord: ChunkCoord) -> Option<&Chunk> {
        self.chunks.get(&coord)
    }

    pub fn chunk_mut(&mut self, coord: ChunkCoord) -> Option<&mut Chunk> {
        self.chunks.get_mut(&coord)
    }

    /// Get a chunk, allocating an empty one if it does not exist yet.
    pub fn chunk_or_insert(&mut self, coord: ChunkCoord) -> &mut Chunk {
        let bounds = self.chunk_bounds(coord);
        self.chunks
            .entry(coord)
            .or_insert_with(|| Chunk::new(coord, bounds))
    }

    pub fn chunks(&self) -> impl Iterator<Item = &Chunk> {
        self.chunks.values()
    }

    pub fn chunks_mut(&mut self) -> impl Iterator<Item = &mut Chunk> {
        self.chunks.values_mut()
    }

    /// Number of allocated chunks.
    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }

    /// Coordinates of all allocated chunks, sorted for deterministic iteration.
    pub fn coords(&self) -> Vec<ChunkCoord> {
        let mut coords: Vec<ChunkCoord> = self.chunks.keys().copied().collect();
        coords.sort_unstable();
        coords
    }

    /// Allocate the neighbours that own boundary cells of solid chunks.
    ///
    /// A marching-cubes cell straddling the minimum face of chunk `c` is owned
    /// by `c`'s negative-side neighbour, so without this the closing faces on a
    /// chunk's `-X`/`-Y`/`-Z` seams would simply be missing — most visibly the
    /// underside of the terrain. Only the negative octant is needed: cells on
    /// the *maximum* faces are already owned by `c` itself.
    ///
    /// Neighbours that turn out to contribute nothing are dropped again by
    /// [`Self::prune_vacant`] once meshing has run.
    pub fn allocate_seam_neighbours(&mut self) {
        let solid: Vec<ChunkCoord> = self
            .chunks
            .iter()
            .filter(|(_, chunk)| chunk.has_solid())
            .map(|(coord, _)| *coord)
            .collect();

        for coord in solid {
            for dx in -1..=0 {
                for dy in -1..=0 {
                    for dz in -1..=0 {
                        if (dx, dy, dz) != (0, 0, 0) {
                            self.chunk_or_insert(coord.offset(dx, dy, dz));
                        }
                    }
                }
            }
        }
    }

    /// Drop chunks that hold neither voxel data nor mesh geometry.
    pub fn prune_vacant(&mut self) {
        self.chunks.retain(|_, chunk| !chunk.is_vacant());
    }

    /// World-space union of all allocated chunk bounds.
    ///
    /// Bounds are derived from what is allocated, so they stay tight rather than
    /// reserving space the terrain never filled. Returns `None` for an empty grid.
    pub fn allocated_world_bounds(&self) -> Option<AABB> {
        let mut iter = self.chunks.keys();
        let first = self.chunk_world_bounds(*iter.next()?);
        Some(iter.fold(first, |acc, coord| {
            let b = self.chunk_world_bounds(*coord);
            AABB::new(
                Point3::new(
                    acc.min.x.min(b.min.x),
                    acc.min.y.min(b.min.y),
                    acc.min.z.min(b.min.z),
                ),
                Point3::new(
                    acc.max.x.max(b.max.x),
                    acc.max.y.max(b.max.y),
                    acc.max.z.max(b.max.z),
                ),
            )
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(voxel_size: f32) -> ChunkGrid {
        ChunkGrid::new(Point3::origin(), voxel_size)
    }

    #[test]
    fn chunk_extent_is_voxels_times_voxel_size() {
        assert_eq!(grid(1.0).chunk_extent(), 32.0);
        assert_eq!(grid(0.5).chunk_extent(), 16.0);
    }

    /// Every point inside a chunk must map back to that chunk, for negative
    /// coordinates as well as positive ones. Truncation instead of flooring
    /// would collapse chunks -1 and 0 onto each other.
    #[test]
    fn coord_round_trips_including_negatives() {
        let g = grid(1.0);
        for x in -3..=3 {
            for y in -3..=3 {
                for z in -3..=3 {
                    let coord = ChunkCoord::new(x, y, z);
                    let bounds = g.chunk_bounds(coord);

                    assert_eq!(g.coord_at(bounds.center()), coord, "centre of {coord:?}");
                    assert_eq!(g.coord_at(bounds.min), coord, "min corner of {coord:?}");

                    // The maximum corner belongs to the *next* chunk: bounds are
                    // half-open so adjacent chunks never claim the same voxel.
                    assert_eq!(
                        g.coord_at(bounds.max),
                        coord.offset(1, 1, 1),
                        "max corner of {coord:?}"
                    );

                    // Just inside the maximum face is still this chunk.
                    let inside_max = bounds.max - Vector3::repeat(0.001);
                    assert_eq!(g.coord_at(inside_max), coord, "inside max of {coord:?}");
                }
            }
        }
    }

    #[test]
    fn coord_at_floors_across_zero() {
        let g = grid(1.0);
        assert_eq!(g.coord_at(Point3::new(-0.5, 0.0, 0.0)).x, -1);
        assert_eq!(g.coord_at(Point3::new(-32.0, 0.0, 0.0)).x, -1);
        assert_eq!(g.coord_at(Point3::new(-32.5, 0.0, 0.0)).x, -2);
        assert_eq!(g.coord_at(Point3::new(0.0, 0.0, 0.0)).x, 0);
        assert_eq!(g.coord_at(Point3::new(31.9, 0.0, 0.0)).x, 0);
        assert_eq!(g.coord_at(Point3::new(32.0, 0.0, 0.0)).x, 1);
    }

    #[test]
    fn first_sample_is_the_chunk_min_in_voxel_indices() {
        let g = grid(0.5);
        let coord = ChunkCoord::new(-2, 1, 0);
        let [sx, sy, sz] = g.first_sample(coord);
        let bounds = g.chunk_bounds(coord);
        assert_eq!(sx as f32 * g.voxel_size(), bounds.min.x);
        assert_eq!(sy as f32 * g.voxel_size(), bounds.min.y);
        assert_eq!(sz as f32 * g.voxel_size(), bounds.min.z);
    }

    #[test]
    fn world_local_round_trip_with_offset_origin() {
        let g = ChunkGrid::new(Point3::new(10.0, -4.0, 2.5), 1.0);
        let world = Point3::new(-7.0, 3.0, 11.0);
        let local = g.to_local(world);
        assert_eq!(g.to_world(local), world);
        assert_eq!(local, Point3::new(-17.0, 7.0, 8.5));
    }

    #[test]
    fn coords_in_covers_the_query_box() {
        let g = grid(1.0);
        let aabb = AABB::new(Point3::new(-33.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let coords: Vec<_> = g.coords_in(&aabb).collect();
        assert_eq!(coords.len(), 3);
        assert!(coords.contains(&ChunkCoord::new(-2, 0, 0)));
        assert!(coords.contains(&ChunkCoord::new(-1, 0, 0)));
        assert!(coords.contains(&ChunkCoord::new(0, 0, 0)));
    }

    #[test]
    fn writes_allocate_only_the_touched_chunk() {
        let mut g = grid(1.0);
        assert_eq!(g.chunk_count(), 0);

        g.set(
            Point3::new(-1.0, -1.0, -1.0),
            Voxel::solid(crate::terrain::voxel::VoxelMaterial::Rock, 1),
        );
        assert_eq!(g.chunk_count(), 1);
        assert!(g.chunk(ChunkCoord::new(-1, -1, -1)).is_some());

        // Reading elsewhere must not allocate.
        assert_eq!(g.get(Point3::new(100.0, 100.0, 100.0)).density, -1.0);
        assert_eq!(g.chunk_count(), 1);
    }

    #[test]
    fn seam_neighbours_are_only_added_on_the_negative_side() {
        let mut g = grid(1.0);
        g.set(
            Point3::new(4.0, 4.0, 4.0),
            Voxel::solid(crate::terrain::voxel::VoxelMaterial::Rock, 1),
        );
        g.allocate_seam_neighbours();

        // The solid chunk plus the seven chunks in its negative octant.
        assert_eq!(g.chunk_count(), 8);
        assert!(g.chunk(ChunkCoord::new(-1, -1, -1)).is_some());
        assert!(g.chunk(ChunkCoord::new(1, 0, 0)).is_none());
    }

    #[test]
    fn prune_drops_chunks_with_no_content() {
        let mut g = grid(1.0);
        g.set(
            Point3::new(4.0, 4.0, 4.0),
            Voxel::solid(crate::terrain::voxel::VoxelMaterial::Rock, 1),
        );
        g.allocate_seam_neighbours();
        g.prune_vacant();
        assert_eq!(g.chunk_count(), 1);
    }
}
