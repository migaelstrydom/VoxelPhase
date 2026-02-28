//! Terrain manager that combines SVO storage with adaptive mesh octree.
//!
//! This module provides:
//! - SVO for voxel data storage
//! - MeshOctree for efficient rendering and collision queries
//! - Incremental updates when terrain is modified
//! - Triangle adjacency map (edge-sharing neighbors), updated incrementally
//!   alongside mesh rebuilds so cost is proportional to the dirty region

use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;
use std::time::Instant;

use super::adjacency::AdjacencyMap;
use super::mesh_octree::MeshOctree;
use super::svo::SparseVoxelOctree;
use crate::collision::continuous::{swept_sphere_triangle, SweptContact};
use crate::collision::{MeshPatch, PatchTriangle, AABB};
use crate::core::error::EngineResult;
use crate::physics::StaticGeometry;
use crate::rendering::vertex::Vertex;
use crate::resources::textures::{TextureHandle, TextureManager};
use crate::sensing::{ProbeHit, ProbeTarget};

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

    /// Edge-based triangle adjacency map, rebuilt after each mesh update.
    adjacency: AdjacencyMap,
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

        // Expand mesh octree bounds by one voxel so marching cubes boundary
        // triangles (whose vertices sit up to half a voxel outside the SVO)
        // are accepted. This gives the terrain solid side and bottom faces,
        // producing a floating-island look instead of a paper-thin surface.
        let mesh_bounds = AABB::new(
            Point3::new(
                bounds.min.x - voxel_size,
                bounds.min.y - voxel_size,
                bounds.min.z - voxel_size,
            ),
            Point3::new(
                bounds.max.x + voxel_size,
                bounds.max.y + voxel_size,
                bounds.max.z + voxel_size,
            ),
        );

        let mut manager = Self {
            svo,
            mesh: MeshOctree::new(mesh_bounds),
            voxel_size,
            dirty_regions: Vec::new(),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
            texture: Some(texture),
            adjacency: AdjacencyMap::new(),
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

    /// Damage voxels within a sphere, reducing their health.
    ///
    /// Voxels whose health reaches zero are converted to air. Indestructible
    /// voxels (bedrock) are unaffected. Only marks the region dirty if at
    /// least one voxel was actually destroyed.
    pub fn damage_sphere(&mut self, center: Point3<f32>, radius: f32, damage: u8) {
        let mut any_destroyed = false;
        self.svo.modify_sphere(center, radius, |_pos, voxel| {
            let after = voxel.apply_damage(damage);
            if after.material != voxel.material {
                any_destroyed = true;
            }
            after
        });

        if any_destroyed {
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

        // Rebuild each dirty region with incremental adjacency updates.
        let t2 = Instant::now();
        let mut old_tris: Vec<(super::mesh_octree::TriangleRef, [Point3<f32>; 3])> = Vec::new();
        let mut new_tris: Vec<(super::mesh_octree::TriangleRef, [Point3<f32>; 3])> = Vec::new();
        let mut adjacency_elapsed = std::time::Duration::ZERO;

        for region in &merged_regions {
            let clamped = self.snap_region(region);

            // Collect triangles in the region before the mesh rebuild.
            old_tris.clear();
            self.mesh
                .collect_triangles_in_region(&clamped, &mut old_tris);

            // Rebuild the mesh for this region.
            let svo = &self.svo;
            self.mesh
                .generate_from_voxels(&clamped, self.voxel_size, |pos| svo.get(pos));

            // Collect triangles in the region after the mesh rebuild.
            new_tris.clear();
            self.mesh
                .collect_triangles_in_region(&clamped, &mut new_tris);

            // Patch adjacency for the affected region.
            let t_adj = Instant::now();
            self.adjacency
                .update_region(&old_tris, &new_tris, self.voxel_size * 0.01);
            adjacency_elapsed += t_adj.elapsed();
        }
        self.dirty_regions.clear();
        let rebuild_dirty_regions_time = t2.elapsed();
        let adjacency_time = adjacency_elapsed;

        let t3 = Instant::now();
        // Update cached render data
        let (vertices, indices) = self.mesh.get_render_data();
        self.render_vertices = vertices;
        self.render_indices = indices;
        let update_render_data_time = t3.elapsed();

        log::debug!(
            "Terrain updated: {} dirty regions merged to {}, mesh now has {} triangles in {} leaves, {:?} merge time, {:?} rebuild time, {:?} render data time, {:?} adjacency time, {:?} total",
            dirty_count,
            merged_regions.len(),
            self.mesh.triangle_count(),
            self.mesh.leaf_count(),
            merge_dirty_regions_time,
            rebuild_dirty_regions_time,
            update_render_data_time,
            adjacency_time,
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

    /// Snap a dirty region to voxel grid boundaries and clamp to SVO bounds.
    ///
    /// Without grid snapping, regenerated regions would sample at different
    /// positions than the original terrain, causing seams at boundaries.
    fn snap_region(&self, region: &AABB) -> AABB {
        let bounds = self.svo.bounds();
        let voxel_size = self.voxel_size;
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

        AABB::new(
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
        )
    }

    /// Get the world-space bounds of the terrain.
    pub fn bounds(&self) -> &AABB {
        self.svo.bounds()
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

    // === Adjacency ===

    /// Get the triangle adjacency map.
    #[allow(dead_code)]
    pub fn adjacency(&self) -> &AdjacencyMap {
        &self.adjacency
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
use crate::terrain::voxel::Voxel;

#[cfg(test)]
impl TerrainManager {
    /// Create a new terrain manager with the given bounds and SVO depth.
    /// Test-only helper for creating terrain managers in tests.
    fn new(bounds: AABB, svo_depth: u32) -> Self {
        let voxel_size = bounds.size().x / (1 << svo_depth) as f32;
        let mesh_bounds = AABB::new(
            Point3::new(
                bounds.min.x - voxel_size,
                bounds.min.y - voxel_size,
                bounds.min.z - voxel_size,
            ),
            Point3::new(
                bounds.max.x + voxel_size,
                bounds.max.y + voxel_size,
                bounds.max.z + voxel_size,
            ),
        );
        Self {
            svo: SparseVoxelOctree::new(bounds, svo_depth),
            mesh: MeshOctree::new(mesh_bounds),
            voxel_size,
            dirty_regions: Vec::new(),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
            texture: None,
            adjacency: AdjacencyMap::new(),
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
    fn query_region(&self, aabb: &AABB) -> MeshPatch {
        let results = self.mesh.query_aabb(aabb);

        // Build a lookup from TriangleRef → local index in the patch.
        let ref_to_index: FxHashMap<_, _> = results
            .iter()
            .enumerate()
            .map(|(i, (tri_ref, _))| (*tri_ref, i as u32))
            .collect();

        let triangles = results
            .iter()
            .map(|(tri_ref, triangle)| {
                // Look up adjacency for this triangle in the global map.
                let neighbors = if let Some(nbrs) = self.adjacency.neighbors(tri_ref) {
                    std::array::from_fn(|edge| {
                        nbrs.neighbors[edge].and_then(|nbr_ref| ref_to_index.get(&nbr_ref).copied())
                    })
                } else {
                    [None; 3]
                };
                PatchTriangle {
                    triangle: triangle.clone(),
                    neighbors,
                }
            })
            .collect();

        MeshPatch { triangles }
    }
}

impl ProbeTarget for TerrainManager {
    fn swept_probe(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
        radius: f32,
    ) -> Option<ProbeHit> {
        let end = origin + direction * length;
        let sweep_aabb = AABB::new(
            Point3::new(
                origin.x.min(end.x) - radius,
                origin.y.min(end.y) - radius,
                origin.z.min(end.z) - radius,
            ),
            Point3::new(
                origin.x.max(end.x) + radius,
                origin.y.max(end.y) + radius,
                origin.z.max(end.z) + radius,
            ),
        );
        let patch = self.query_region(&sweep_aabb);

        let mut earliest: Option<SweptContact> = None;
        for pt in &patch.triangles {
            if let Some(contact) = swept_sphere_triangle(origin, end, radius, &pt.triangle) {
                if earliest
                    .as_ref()
                    .map_or(true, |e: &SweptContact| contact.t < e.t)
                {
                    earliest = Some(contact);
                }
            }
        }

        earliest.map(|c| ProbeHit {
            t: c.t,
            point: c.point,
            normal: c.normal,
        })
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
                    Voxel::solid(VoxelMaterial::Rock, 1),
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
                    Voxel::solid(VoxelMaterial::Rock, 1),
                );
            }
        }

        manager.update();
    }
}
