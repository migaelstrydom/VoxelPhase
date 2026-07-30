//! A single voxel chunk: one SVO of voxel data plus its meshed output.
//!
//! A chunk is a cube of `CHUNK_VOXELS³` voxels at the owning grid's voxel
//! size. Its octree depth is fixed by the chunk definition (`log2(CHUNK_VOXELS)`)
//! rather than derived from any level-wide extent, so the voxel size a level
//! declares is the voxel size it gets.
//!
//! ```text
//!   Chunk
//!     ├── SparseVoxelOctree   voxel data, depth = log2(CHUNK_VOXELS)
//!     ├── MeshOctree          marching-cubes output for this chunk's cells
//!     └── dirty flag          set when voxels change, cleared on remesh
//! ```
//!
//! # Cell ownership
//!
//! A marching-cubes cell spans two adjacent sample planes, so a cell on a chunk
//! boundary could be emitted by either side. Ownership is fixed: chunk `c` emits
//! exactly the cells whose *minimum* corner sample lies in `c`, which is exactly
//! the cells inside `c`'s bounds. Every triangle therefore lies within its own
//! chunk and no cell is meshed twice — duplicate triangles at a seam would show
//! up as duplicate contacts in the physics solver.

use nalgebra::Point3;

use super::mesh_octree::{MeshOctree, TriangleRef};
use super::svo::SparseVoxelOctree;
use super::voxel::Voxel;
use crate::collision::AABB;

/// Edge length of a chunk in voxels. A chunk holds `CHUNK_VOXELS³` voxels.
pub const CHUNK_VOXELS: u32 = 32;

/// Octree depth needed to resolve a single voxel within a chunk.
pub const CHUNK_DEPTH: u32 = CHUNK_VOXELS.trailing_zeros();

/// Integer lattice coordinate of a chunk within a `ChunkGrid`.
///
/// Chunk `c` covers the grid-local half-open box
/// `[c * extent, (c + 1) * extent)` where `extent = CHUNK_VOXELS * voxel_size`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ChunkCoord {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl ChunkCoord {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    /// This coordinate offset by an integer delta.
    pub fn offset(self, dx: i32, dy: i32, dz: i32) -> Self {
        Self::new(self.x + dx, self.y + dy, self.z + dz)
    }
}

/// A triangle identified across the whole grid: which chunk owns it, and which
/// triangle it is within that chunk's mesh octree.
///
/// Mesh octrees are per-chunk, so a bare `TriangleRef` is only unique within one
/// chunk. Adjacency and physics patches span chunks and need the qualified form.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ChunkTriangleRef {
    pub chunk: ChunkCoord,
    pub triangle: TriangleRef,
}

/// One chunk of voxel terrain: storage plus its meshed output.
pub struct Chunk {
    /// Lattice coordinate of this chunk within its grid.
    coord: ChunkCoord,

    /// Grid-local bounds of this chunk.
    bounds: AABB,

    /// Voxel storage covering exactly `bounds`, at depth `CHUNK_DEPTH`.
    svo: SparseVoxelOctree,

    /// Marching-cubes output for the cells this chunk owns.
    mesh: MeshOctree,

    /// Set when voxels change; cleared once the mesh has been rebuilt.
    dirty: bool,
}

impl Chunk {
    /// Create an empty (all-air, unmeshed) chunk covering `bounds`.
    pub fn new(coord: ChunkCoord, bounds: AABB) -> Self {
        Self {
            coord,
            bounds,
            svo: SparseVoxelOctree::new(bounds, CHUNK_DEPTH),
            mesh: MeshOctree::new(bounds),
            dirty: true,
        }
    }

    pub fn coord(&self) -> ChunkCoord {
        self.coord
    }

    /// Grid-local bounds of this chunk.
    pub fn bounds(&self) -> &AABB {
        &self.bounds
    }

    /// Voxel at a grid-local position. Positions outside the chunk read as air.
    pub fn voxel_at(&self, local: Point3<f32>) -> Voxel {
        self.svo.get(local)
    }

    /// Write a voxel at a grid-local position, marking the chunk dirty.
    pub fn set_voxel(&mut self, local: Point3<f32>, voxel: Voxel) {
        self.svo.set(local, voxel);
        self.dirty = true;
    }

    /// Damage voxels within a sphere. Returns true if any voxel was destroyed.
    pub fn damage_sphere(&mut self, center: Point3<f32>, radius: f32, damage: u8) -> bool {
        let mut any_destroyed = false;
        self.svo.modify_sphere(center, radius, |_pos, voxel| {
            let after = voxel.apply_damage(damage);
            if after.material != voxel.material {
                any_destroyed = true;
            }
            after
        });
        if any_destroyed {
            self.dirty = true;
        }
        any_destroyed
    }

    /// Whether this chunk holds any solid voxel.
    ///
    /// A chunk without one emits no triangles of its own, but may still be
    /// meshed because a neighbour's samples close a surface across the seam.
    pub fn has_solid(&self) -> bool {
        self.svo.has_solid()
    }

    /// Whether this chunk holds neither voxel data nor mesh geometry, and can
    /// therefore be dropped from the grid without changing anything observable.
    pub fn is_vacant(&self) -> bool {
        self.svo.is_empty_air() && self.mesh.triangle_count() == 0
    }

    pub fn mesh(&self) -> &MeshOctree {
        &self.mesh
    }

    /// Install a freshly built mesh and clear the dirty flag.
    pub fn replace_mesh(&mut self, mesh: MeshOctree) {
        self.mesh = mesh;
        self.dirty = false;
    }

    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }
}
