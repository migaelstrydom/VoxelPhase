//! Terrain system with destructible voxel-based terrain.
//!
//! This module provides:
//! - Sparse Voxel Octree (SVO) for terrain data storage
//! - Marching Cubes mesh generation for smooth rendering
//! - Adaptive Mesh Octree for efficient rendering and collision queries
//! - TerrainManager for unified terrain handling
//! - Procedural terrain generation

pub mod generation;
mod manager;
mod marching_cubes;
mod mesh_octree;
mod svo;
mod voxel;

pub use manager::TerrainManager;

// Voxel types for terrain modification
pub use voxel::Voxel;

pub use generation::create_test_terrain;
