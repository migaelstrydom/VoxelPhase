//! Terrain system with destructible voxel-based terrain.
//!
//! This module provides:
//! - Sparse Voxel Octree (SVO) for terrain data storage
//! - Marching Cubes mesh generation for smooth rendering
//! - TerrainManager for unified terrain handling
//! - Procedural terrain generation

pub mod generation;
mod manager;
mod marching_cubes;
mod svo;
mod voxel;

// New unified terrain manager
pub use manager::TerrainManager;

pub use generation::create_test_terrain;
