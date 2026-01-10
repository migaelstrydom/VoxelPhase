//! Terrain manager that combines SVO storage with collision and rendering.
//!
//! This replaces the chunk-based system with a cleaner architecture:
//! - SVO stores voxel data
//! - TerrainCollider provides fast collision queries
//! - Mesh data is generated for rendering

use nalgebra::Point3;

use crate::collision::{TerrainCollider, TerrainColliderStats, Triangle, AABB};
use crate::rendering::vertex::Vertex;

use super::marching_cubes::MarchingCubes;
use super::svo::SparseVoxelOctree;
use super::voxel::Voxel;

/// Unified terrain manager handling storage, collision, and rendering.
pub struct TerrainManager {
    /// Sparse voxel octree storing terrain data.
    svo: SparseVoxelOctree,

    /// Collision structure with spatial indexing.
    collider: TerrainCollider,

    /// Render mesh vertices.
    render_vertices: Vec<Vertex>,

    /// Render mesh indices.
    render_indices: Vec<u32>,

    /// Whether the mesh needs regeneration.
    dirty: bool,
}

impl TerrainManager {
    /// Create a new terrain manager with the given bounds and SVO depth.
    pub fn new(bounds: AABB, svo_depth: u32) -> Self {
        // Collision index depth should give reasonable region sizes
        // For a 64-unit world with depth 5, regions are 2 units each
        let collider_depth = (svo_depth - 1).max(3);

        Self {
            svo: SparseVoxelOctree::new(bounds, svo_depth),
            collider: TerrainCollider::new(bounds, collider_depth),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
            dirty: true,
        }
    }

    /// Create a terrain manager from an existing SVO.
    pub fn from_svo(svo: SparseVoxelOctree) -> Self {
        let bounds = *svo.bounds();
        let svo_depth = svo.max_depth();
        let collider_depth = (svo_depth - 1).max(3);

        let mut manager = Self {
            svo,
            collider: TerrainCollider::new(bounds, collider_depth),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
            dirty: true,
        };

        manager.rebuild_mesh();
        manager
    }

    /// Get the terrain bounds.
    pub fn bounds(&self) -> &AABB {
        self.svo.bounds()
    }

    /// Set a voxel at a world position.
    pub fn set_voxel(&mut self, position: Point3<f32>, voxel: Voxel) {
        self.svo.set(position, voxel);
        self.dirty = true;
    }

    /// Get a voxel at a world position.
    pub fn get_voxel(&self, position: Point3<f32>) -> Voxel {
        self.svo.get(position)
    }

    /// Modify terrain in a sphere (for explosions, digging, etc.)
    pub fn modify_sphere<F>(&mut self, center: Point3<f32>, radius: f32, modifier: F)
    where
        F: FnMut(Point3<f32>, Voxel) -> Voxel,
    {
        self.svo.modify_sphere(center, radius, modifier);
        self.dirty = true;
    }

    /// Update terrain if dirty (regenerates mesh and collision data).
    ///
    /// Call this once per frame or after batch modifications.
    pub fn update(&mut self) {
        if self.dirty {
            self.rebuild_mesh();
            self.dirty = false;
        }
    }

    /// Force immediate mesh regeneration.
    pub fn rebuild_mesh(&mut self) {
        let bounds = *self.svo.bounds();
        let resolution = 1 << self.svo.max_depth();

        // Sample voxels into a grid
        let grid = self.svo.sample_grid(&bounds, resolution + 1);

        // Generate mesh using marching cubes
        let marching_cubes = MarchingCubes::new();
        let voxel_size = bounds.size().x / resolution as f32;
        let mesh = marching_cubes.generate(&grid, bounds.min, voxel_size);

        // Update render data
        self.render_vertices = mesh.to_vertices();
        self.render_indices = mesh.render_indices().to_vec();

        // Update collision data
        self.collider.clear();
        self.collider.add_triangles(mesh.triangles());

        log::debug!(
            "Terrain rebuilt: {} vertices, {} triangles",
            self.render_vertices.len(),
            self.collider.triangle_count()
        );
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
