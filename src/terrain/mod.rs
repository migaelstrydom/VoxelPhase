//! Terrain system with destructible voxel-based terrain.
//!
//! This module provides:
//! - Sparse Voxel Octree (SVO) for voxel storage within a chunk
//! - Marching Cubes mesh generation for smooth rendering
//! - Adaptive Mesh Octree for efficient rendering and collision queries
//! - Segments: independently placed chunk grids with their own frame,
//!   resolution and named anchors
//! - TerrainWorld: the engine-facing, world-space view across all segments
//! - Procedural terrain generation

mod adjacency;
mod anchor;
mod chunk;
mod chunk_grid;
mod csg;
pub use csg::SURFACE_BAND;
mod frame;
pub mod generation;
mod marching_cubes;
mod mesh_octree;
mod render_cache;
mod segment;
pub(crate) mod svo;
pub mod traversal;
mod voxel;
mod voxel_block;
mod world;

pub use anchor::{mate, outward, Anchor};
pub use chunk::{ChunkCoord, CHUNK_VOXELS};
pub use chunk_grid::ChunkGrid;
pub use frame::{SegmentFrame, YAW_STEP_DEGREES};
pub use segment::{Segment, SegmentState};
pub use world::{TerrainWorld, UpdateTimings};

// Voxel types for terrain modification
pub use voxel::{DurabilityConfig, VoxelMaterial};

pub use generation::generate_terrain;
