//! Terrain manager that combines SVO storage with adaptive mesh octree.
//!
//! This module provides:
//! - SVO for voxel data storage
//! - MeshOctree for efficient rendering and collision queries
//! - Incremental updates when terrain is modified

use nalgebra::{Point3, Vector3};
use std::time::Instant;

use crate::collision::{
    sphere_triangle_collision, swept_sphere_triangle, ContactPoint, SweptContact, AABB,
};
use crate::core::error::EngineResult;
use crate::physics::{StaticContact, StaticGeometry, SweptStaticContact};
use crate::rendering::vertex::Vertex;
use crate::resources::textures::{TextureHandle, TextureManager};

use super::mesh_octree::MeshOctree;
use super::svo::SparseVoxelOctree;
use super::voxel::Voxel;

/// Unified terrain manager handling storage, collision, and rendering.
///
/// Uses MeshOctree for adaptive mesh storage with complete spatial queries.
/// When terrain is modified, only affected regions are rebuilt.
pub struct TerrainManager {
    /// Sparse voxel octree storing terrain data.
    svo: SparseVoxelOctree,

    /// Adaptive mesh octree for rendering and collision.
    mesh: MeshOctree,

    /// Voxel size (minimum voxel size from SVO).
    voxel_size: f32,

    /// Regions that need mesh rebuilding (stored as AABBs).
    dirty_regions: Vec<AABB>,

    /// Cached render data (updated on each mesh rebuild).
    render_vertices: Vec<Vertex>,
    render_indices: Vec<u32>,

    /// Optional noise texture for terrain surface variation.
    texture: Option<TextureHandle>,
}

impl TerrainManager {
    /// Create a terrain manager from an existing SVO.
    ///
    /// Performs initial mesh build and generates procedural noise texture for surface variation.
    pub fn from_svo(
        svo: SparseVoxelOctree,
        texture_manager: &TextureManager,
    ) -> EngineResult<Self> {
        let bounds = *svo.bounds();
        let voxel_size = svo.voxel_size();

        let t0 = Instant::now();

        // Generate procedural noise texture for terrain surface variation
        let texture = texture_manager.create_noise_texture(512, 512, 5, 20.0, 42)?;
        log::info!("Generated terrain noise texture (512x512, 5 octaves, scale 20.0)");

        let mut manager = Self {
            svo,
            mesh: MeshOctree::new(bounds),
            voxel_size,
            dirty_regions: Vec::new(),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
            texture: Some(texture),
        };

        // Mark entire terrain as dirty for initial build
        manager.dirty_regions.push(bounds);

        // Perform initial build
        manager.update();
        log::info!(
            "initial mesh build: {:?}, {} triangles, {} leaves",
            t0.elapsed(),
            manager.mesh.triangle_count(),
            manager.mesh.leaf_count()
        );

        Ok(manager)
    }

    /// Modify terrain in a sphere (for explosions, digging, etc.)
    ///
    /// Only affected regions will be rebuilt on next update.
    pub fn modify_sphere<F>(&mut self, center: Point3<f32>, radius: f32, modifier: F)
    where
        F: FnMut(Point3<f32>, Voxel) -> Voxel,
    {
        self.svo.modify_sphere(center, radius, modifier);

        // Mark affected region as dirty (with some padding for mesh generation)
        let padding = self.voxel_size * 2.0;
        let affected = AABB::new(
            Point3::new(
                center.x - radius - padding,
                center.y - radius - padding,
                center.z - radius - padding,
            ),
            Point3::new(
                center.x + radius + padding,
                center.y + radius + padding,
                center.z + radius + padding,
            ),
        );
        self.dirty_regions.push(affected);
    }

    /// Update terrain incrementally (only rebuilds dirty regions).
    ///
    /// Call this once per frame or after batch modifications.
    pub fn update(&mut self) {
        if self.dirty_regions.is_empty() {
            return;
        }

        let t0 = Instant::now();
        let dirty_count = self.dirty_regions.len();

        let t1 = Instant::now();
        // Merge overlapping dirty regions to reduce redundant work
        let merged_regions = self.merge_dirty_regions();
        let merge_dirty_regions_time = t1.elapsed();

        // Rebuild each dirty region
        let t2 = Instant::now();
        for region in &merged_regions {
            self.rebuild_region(region);
        }
        self.dirty_regions.clear();
        let rebuild_dirty_regions_time = t2.elapsed();

        let t3 = Instant::now();
        // Update cached render data
        let (vertices, indices) = self.mesh.get_render_data();
        self.render_vertices = vertices;
        self.render_indices = indices;
        let update_render_data_time = t3.elapsed();

        log::debug!(
            "Terrain updated: {} dirty regions merged to {}, mesh now has {} triangles in {} leaves, {:?} merge time, {:?} rebuild time, {:?} update render data time, {:?} update time",
            dirty_count,
            merged_regions.len(),
            self.mesh.triangle_count(),
            self.mesh.leaf_count(),
            merge_dirty_regions_time,
            rebuild_dirty_regions_time,
            update_render_data_time,
            t0.elapsed()
        );
    }

    /// Merge overlapping dirty regions to reduce redundant rebuilds.
    fn merge_dirty_regions(&self) -> Vec<AABB> {
        if self.dirty_regions.is_empty() {
            return Vec::new();
        }

        let mut merged: Vec<AABB> = Vec::new();

        for region in &self.dirty_regions {
            let mut found_overlap = false;
            for existing in &mut merged {
                if existing.intersects(region) {
                    // Expand existing to include this region
                    *existing = AABB::new(
                        Point3::new(
                            existing.min.x.min(region.min.x),
                            existing.min.y.min(region.min.y),
                            existing.min.z.min(region.min.z),
                        ),
                        Point3::new(
                            existing.max.x.max(region.max.x),
                            existing.max.y.max(region.max.y),
                            existing.max.z.max(region.max.z),
                        ),
                    );
                    found_overlap = true;
                    break;
                }
            }
            if !found_overlap {
                merged.push(*region);
            }
        }

        merged
    }

    /// Rebuild mesh for a specific region from SVO voxel data.
    fn rebuild_region(&mut self, region: &AABB) {
        let bounds = self.svo.bounds();
        let voxel_size = self.voxel_size;

        // Snap region to voxel grid boundaries to ensure consistent sampling.
        // Without this, regenerated regions would sample at different positions
        // than the original terrain, causing seams at boundaries.
        let world_origin = bounds.min;
        let snapped = AABB::new(
            Point3::new(
                world_origin.x
                    + ((region.min.x - world_origin.x) / voxel_size).floor() * voxel_size,
                world_origin.y
                    + ((region.min.y - world_origin.y) / voxel_size).floor() * voxel_size,
                world_origin.z
                    + ((region.min.z - world_origin.z) / voxel_size).floor() * voxel_size,
            ),
            Point3::new(
                world_origin.x + ((region.max.x - world_origin.x) / voxel_size).ceil() * voxel_size,
                world_origin.y + ((region.max.y - world_origin.y) / voxel_size).ceil() * voxel_size,
                world_origin.z + ((region.max.z - world_origin.z) / voxel_size).ceil() * voxel_size,
            ),
        );

        // Clamp to SVO bounds
        let clamped = AABB::new(
            Point3::new(
                snapped.min.x.max(bounds.min.x),
                snapped.min.y.max(bounds.min.y),
                snapped.min.z.max(bounds.min.z),
            ),
            Point3::new(
                snapped.max.x.min(bounds.max.x),
                snapped.max.y.min(bounds.max.y),
                snapped.max.z.min(bounds.max.z),
            ),
        );

        // Generate mesh from voxel data
        let svo = &self.svo;
        self.mesh
            .generate_from_voxels(&clamped, voxel_size, |pos| svo.get(pos));
    }

    // === Collision queries ===

    /// Query sphere collision against terrain.
    ///
    /// Returns contact points for all triangles the sphere intersects.
    pub fn query_sphere_collision(&self, center: Point3<f32>, radius: f32) -> Vec<ContactPoint> {
        // Build query AABB
        let query = AABB::new(
            Point3::new(center.x - radius, center.y - radius, center.z - radius),
            Point3::new(center.x + radius, center.y + radius, center.z + radius),
        );

        // Get triangles from mesh octree
        let triangles = self.mesh.query_aabb(&query);

        // Test each triangle
        let mut contacts = Vec::new();
        for triangle in triangles {
            if let Some(contact) = sphere_triangle_collision(center, radius, &triangle) {
                contacts.push(contact);
            }
        }

        contacts
    }

    /// Query swept sphere collision (continuous collision detection).
    ///
    /// Returns the first collision along the sweep path, if any.
    pub fn query_swept_sphere(
        &self,
        start: Point3<f32>,
        end: Point3<f32>,
        radius: f32,
    ) -> Option<SweptContact> {
        // Build query AABB covering the entire sweep
        let min_x = start.x.min(end.x) - radius;
        let min_y = start.y.min(end.y) - radius;
        let min_z = start.z.min(end.z) - radius;
        let max_x = start.x.max(end.x) + radius;
        let max_y = start.y.max(end.y) + radius;
        let max_z = start.z.max(end.z) + radius;
        let query = AABB::new(
            Point3::new(min_x, min_y, min_z),
            Point3::new(max_x, max_y, max_z),
        );

        // Get triangles from mesh octree
        let triangles = self.mesh.query_aabb(&query);

        // Find earliest collision
        let mut earliest: Option<SweptContact> = None;
        for triangle in triangles {
            if let Some(contact) = swept_sphere_triangle(start, end, radius, &triangle) {
                if earliest.is_none() || contact.t < earliest.as_ref().unwrap().t {
                    earliest = Some(contact);
                }
            }
        }

        earliest
    }

    /// Query all triangles intersecting an AABB.
    ///
    /// Useful for custom collision detection or spatial queries.
    #[allow(dead_code)]
    pub fn query_triangles(&self, query: &AABB) -> Vec<crate::collision::Triangle> {
        self.mesh.query_aabb(query)
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

    /// Get the terrain texture handle, if set.
    pub fn texture(&self) -> Option<&TextureHandle> {
        self.texture.as_ref()
    }

    /// Get render data with frustum culling.
    ///
    /// `frustum_planes` should be 6 planes: left, right, bottom, top, near, far.
    /// Each plane is (normal, distance) where dot(normal, point) + distance >= 0 inside.
    #[allow(dead_code)]
    pub fn get_render_data_culled(
        &self,
        frustum_planes: &[(Vector3<f32>, f32); 6],
    ) -> (Vec<Vertex>, Vec<u32>) {
        self.mesh.get_render_data_frustum_culled(frustum_planes)
    }

    // === Statistics ===

    /// Get the total triangle count.
    pub fn triangle_count(&self) -> usize {
        self.mesh.triangle_count()
    }

    /// Get the number of mesh octree leaves.
    pub fn leaf_count(&self) -> usize {
        self.mesh.leaf_count()
    }
}

#[cfg(test)]
impl TerrainManager {
    /// Create a new terrain manager with the given bounds and SVO depth.
    /// Test-only helper for creating terrain managers in tests.
    fn new(bounds: AABB, svo_depth: u32) -> Self {
        Self {
            svo: SparseVoxelOctree::new(bounds, svo_depth),
            mesh: MeshOctree::new(bounds),
            voxel_size: bounds.size().x / (1 << svo_depth) as f32,
            dirty_regions: Vec::new(),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
            texture: None,
        }
    }

    /// Set a voxel at a world position.
    /// Test-only helper for setting individual voxels in tests.
    fn set_voxel(&mut self, position: Point3<f32>, voxel: Voxel) {
        self.svo.set(position, voxel);
        // Mark the region containing this voxel as dirty
        let padding = self.voxel_size * 2.0;
        let affected = AABB::new(
            position - Vector3::new(padding, padding, padding),
            position + Vector3::new(padding, padding, padding),
        );
        self.dirty_regions.push(affected);
    }
}

impl StaticGeometry for TerrainManager {
    fn query_sphere(&self, center: Point3<f32>, radius: f32) -> Vec<StaticContact> {
        self.query_sphere_collision(center, radius)
            .into_iter()
            .map(|cp| {
                // Compute contact point from center, normal, and depth
                let point = center - cp.normal * (radius - cp.depth);
                StaticContact::new(point, cp.normal, cp.depth)
            })
            .collect()
    }

    fn sweep_sphere(
        &self,
        start: Point3<f32>,
        end: Point3<f32>,
        radius: f32,
    ) -> Option<SweptStaticContact> {
        self.query_swept_sphere(start, end, radius)
            .map(|sc| SweptStaticContact::new(sc.t, sc.point, sc.normal))
    }

    fn query_triangles(&self, aabb: &crate::collision::AABB) -> Vec<crate::collision::Triangle> {
        self.mesh.query_aabb(aabb)
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
        assert!(manager.triangle_count() > 0);
    }

    #[test]
    fn test_sphere_collision() {
        let bounds = AABB::new(
            Point3::new(-16.0, -16.0, -16.0),
            Point3::new(16.0, 16.0, 16.0),
        );

        let mut manager = TerrainManager::new(bounds, 4);

        // Create a floor of solid voxels
        for x in -5..=5 {
            for z in -5..=5 {
                manager.set_voxel(
                    Point3::new(x as f32, 0.0, z as f32),
                    Voxel::solid(VoxelMaterial::Rock),
                );
            }
        }

        manager.update();
    }
}
