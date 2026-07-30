//! Terrain system with destructible voxel-based terrain.
//!
//! This module provides:
//! - Sparse Voxel Octree (SVO) for terrain data storage
//! - Marching Cubes mesh generation for smooth rendering
//! - Adaptive Mesh Octree for efficient rendering and collision queries
//! - TerrainManager for unified terrain handling
//! - Procedural terrain generation

mod adjacency;
mod chunk;
mod chunk_grid;
pub mod generation;
mod manager;
mod marching_cubes;
mod mesh_octree;
pub(crate) mod svo;
mod voxel;

pub use chunk::{ChunkCoord, CHUNK_VOXELS};
pub use chunk_grid::ChunkGrid;
pub use manager::{TerrainManager, UpdateTimings};

// Voxel types for terrain modification
pub use voxel::DurabilityConfig;

pub use generation::generate_terrain;
