//! Terrain manager that combines SVO storage with collision and rendering.
//!
//! This module provides incremental terrain updates:
//! - SVO stores voxel data with mesh region tracking
//! - Only dirty regions are rebuilt when terrain is modified
//! - TerrainCollider provides fast collision queries with partial updates

use nalgebra::Point3;
use std::time::Instant;

use crate::collision::{TerrainCollider, TerrainColliderStats, AABB};
use crate::rendering::vertex::Vertex;

use super::svo::{MeshRegionKey, SparseVoxelOctree};
use super::voxel::Voxel;

/// Unified terrain manager handling storage, collision, and rendering.
///
/// Uses incremental mesh rebuilding: when terrain is modified, only the
/// affected mesh regions are regenerated, providing significant speedup
/// for localized changes like explosions.
pub struct TerrainManager {
    /// Sparse voxel octree storing terrain data and mesh regions.
    svo: SparseVoxelOctree,

    /// Collision structure with spatial indexing.
    collider: TerrainCollider,

    /// Combined render mesh vertices (from all regions).
    render_vertices: Vec<Vertex>,

    /// Combined render mesh indices (from all regions).
    render_indices: Vec<u32>,
}

impl TerrainManager {
    /// Create a terrain manager from an existing SVO.
    ///
    /// Marks all regions dirty and performs initial mesh build.
    pub fn from_svo(mut svo: SparseVoxelOctree) -> Self {
        let bounds = *svo.bounds();
        let svo_depth = svo.max_depth();
        let collider_depth = (svo_depth - 1).max(3);

        // Mark all regions as dirty for initial build
        let t0 = Instant::now();
        svo.mark_all_regions_dirty();
        log::info!(
            "mark_all_regions_dirty: {:?} ({} regions)",
            t0.elapsed(),
            svo.dirty_region_count()
        );

        let mut manager = Self {
            svo,
            collider: TerrainCollider::new(bounds, collider_depth),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
        };

        // Perform initial build
        let t1 = Instant::now();
        manager.update();
        log::info!("initial mesh build (update): {:?}", t1.elapsed());

        manager
    }

    /// Modify terrain in a sphere (for explosions, digging, etc.)
    ///
    /// Only affected mesh regions will be rebuilt on next update.
    pub fn modify_sphere<F>(&mut self, center: Point3<f32>, radius: f32, modifier: F)
    where
        F: FnMut(Point3<f32>, Voxel) -> Voxel,
    {
        self.svo.modify_sphere(center, radius, modifier);

        // Mark affected mesh regions as dirty
        let affected = AABB::new(
            Point3::new(center.x - radius, center.y - radius, center.z - radius),
            Point3::new(center.x + radius, center.y + radius, center.z + radius),
        );
        self.svo.mark_regions_dirty(&affected);
    }

    /// Update terrain incrementally (only rebuilds dirty regions).
    ///
    /// Call this once per frame or after batch modifications.
    pub fn update(&mut self) {
        if !self.svo.has_dirty_regions() {
            return;
        }

        let t0 = Instant::now();
        let dirty_region_count = self.svo.dirty_region_count();

        // Rebuild dirty mesh regions in the SVO
        let rebuilt_keys = self.svo.rebuild_dirty_regions();

        // Combine all region meshes into render buffers
        self.combine_region_meshes();

        // Update collision for affected regions only
        self.update_collision_partial(&rebuilt_keys);

        log::debug!(
            "Terrain updated: {} regions rebuilt, {} total vertices, {} triangles, {:?} time elapsed",
            dirty_region_count,
            self.render_vertices.len(),
            self.collider.stats().total_triangles,
            t0.elapsed()
        );
    }

    /// Combine all mesh regions into unified render buffers.
    fn combine_region_meshes(&mut self) {
        self.render_vertices.clear();
        self.render_indices.clear();

        for mesh in self.svo.mesh_regions().values() {
            let index_offset = self.render_vertices.len() as u32;
            self.render_vertices.extend_from_slice(&mesh.vertices);
            self.render_indices
                .extend(mesh.indices.iter().map(|i| i + index_offset));
        }
    }

    /// Update collision data for rebuilt regions only.
    fn update_collision_partial(&mut self, rebuilt_keys: &[MeshRegionKey]) {
        for key in rebuilt_keys {
            let region_bounds = self.svo.region_bounds(*key);

            // Remove old triangles in this region
            self.collider.remove_triangles_in_region(&region_bounds);

            // Add new triangles if region has geometry
            if let Some(mesh) = self.svo.mesh_regions().get(key) {
                self.collider
                    .add_triangles_from_mesh(&mesh.vertices, &mesh.indices);
            }
        }
    }

    // === Collision queries (delegate to TerrainCollider) ===

    /// Query sphere collision against terrain.
    pub fn query_sphere_collision(
        &self,
        center: Point3<f32>,
        radius: f32,
    ) -> Vec<crate::collision::ContactPoint> {
        self.collider.query_sphere(center, radius)
    }

    /// Query swept sphere collision (continuous collision detection).
    pub fn query_swept_sphere(
        &self,
        start: Point3<f32>,
        end: Point3<f32>,
        radius: f32,
    ) -> Option<crate::collision::SweptContact> {
        self.collider.query_swept_sphere(start, end, radius)
    }

    /// Get collision statistics.
    pub fn collision_stats(&self) -> TerrainColliderStats {
        self.collider.stats()
    }

    // === Rendering data ===

    /// Check if there's any geometry to render.
    pub fn has_geometry(&self) -> bool {
        !self.render_vertices.is_empty()
    }

    /// Get render vertices.
    pub fn render_vertices(&self) -> &[Vertex] {
        &self.render_vertices
    }

    /// Get render indices.
    pub fn render_indices(&self) -> &[u32] {
        &self.render_indices
    }
}

#[cfg(test)]
impl TerrainManager {
    /// Create a new terrain manager with the given bounds and SVO depth.
    /// Test-only helper for creating terrain managers in tests.
    fn new(bounds: AABB, svo_depth: u32) -> Self {
        // Collision index depth should give reasonable region sizes
        // For a 64-unit world with depth 5, regions are 2 units each
        let collider_depth = (svo_depth - 1).max(3);

        Self {
            svo: SparseVoxelOctree::new(bounds, svo_depth),
            collider: TerrainCollider::new(bounds, collider_depth),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
        }
    }

    /// Set a voxel at a world position.
    /// Test-only helper for setting individual voxels in tests.
    fn set_voxel(&mut self, position: Point3<f32>, voxel: Voxel) {
        self.svo.set(position, voxel);
        // Mark the region containing this voxel as dirty
        let voxel_size = self.svo.voxel_size();
        let affected = AABB::new(
            position - nalgebra::Vector3::new(voxel_size, voxel_size, voxel_size),
            position + nalgebra::Vector3::new(voxel_size, voxel_size, voxel_size),
        );
        self.svo.mark_regions_dirty(&affected);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::voxel::VoxelMaterial;

    #[test]
    fn test_terrain_manager_basic() {
        let bounds = AABB::new(
            Point3::new(-16.0, -16.0, -16.0),
            Point3::new(16.0, 16.0, 16.0),
        );

        let mut manager = TerrainManager::new(bounds, 4);

        // Set some solid voxels
        for x in -5..=5 {
            for z in -5..=5 {
                manager.set_voxel(
                    Point3::new(x as f32, 0.0, z as f32),
                    Voxel::solid(VoxelMaterial::Rock),
                );
            }
        }

        manager.update();

        // Should have generated some geometry
        assert!(manager.has_geometry());
        assert!(manager.collider.triangle_count() > 0);
    }
}
