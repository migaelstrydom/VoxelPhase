//! Terrain manager: the engine-facing view of a chunked voxel world.
//!
//! ```text
//!                    ┌──────────────────────────────────┐
//!                    │          TerrainManager          │
//!                    │  StaticGeometry + ProbeTarget    │
//!                    │  chunk dispatch, adjacency,      │
//!                    │  concatenated render buffers     │
//!                    └────────────────┬─────────────────┘
//!                                     │  world AABB → chunk range
//!                    ┌────────────────▼─────────────────┐
//!                    │             ChunkGrid            │
//!                    └───┬──────────────┬───────────────┘
//!                        ▼              ▼
//!                   ┌────────┐     ┌────────┐
//!                   │ Chunk  │     │ Chunk  │     sparse: allocated only
//!                   │ SVO    │     │ SVO    │     where something was written
//!                   │ Mesh   │     │ Mesh   │
//!                   └────────┘     └────────┘
//! ```
//!
//! Queries fan out over the chunks intersecting the query volume and the
//! results are unioned. Chunks own disjoint sets of marching-cubes cells, so
//! no triangle is ever returned twice — see `chunk.rs` for why that matters.
//!
//! Remeshing is whole-chunk: `damage_sphere` marks the chunks a change can
//! affect, `update` rebuilds exactly those, and adjacency is patched with just
//! their triangles so its cost stays proportional to the change.

use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;
use std::time::Instant;

use super::adjacency::AdjacencyMap;
use super::chunk::{ChunkCoord, ChunkTriangleRef, CHUNK_VOXELS};
use super::chunk_grid::ChunkGrid;
use super::mesh_octree::{MeshOctree, TriangleRef};
use crate::collision::ray_triangle::{ray_triangle, RayHit};
use crate::collision::{MeshPatch, PatchTriangle, AABB};
use crate::core::error::EngineResult;
use crate::physics::StaticGeometry;
use crate::rendering::vertex::Vertex;
use crate::resources::textures::{TextureHandle, TextureManager};
use crate::sensing::{ProbeHit, ProbeTarget};

/// Fraction of a voxel used as the vertex-matching tolerance when linking
/// triangles into the adjacency map.
const ADJACENCY_TOLERANCE_FACTOR: f32 = 0.01;

/// Unified terrain manager handling storage, collision, and rendering.
pub struct TerrainManager {
    /// Voxel chunks and their meshes.
    grid: ChunkGrid,

    /// Edge-based triangle adjacency, spanning chunk boundaries.
    adjacency: AdjacencyMap<ChunkTriangleRef>,

    /// World AABBs of the chunks rebuilt in the most recent `update()` call.
    /// Downstream systems (e.g. WaterSystem) read these to detect terrain changes.
    rebuilt_regions: Vec<AABB>,

    /// Cached render data (updated on each mesh rebuild).
    render_vertices: Vec<Vertex>,
    render_indices: Vec<u32>,

    /// Optional noise texture for terrain surface variation.
    texture: Option<TextureHandle>,

    /// World bounds of the allocated chunks, refreshed on each update.
    bounds: AABB,

    /// Cached mesh statistics, refreshed on each update.
    triangle_count: usize,
    leaf_count: usize,
}

impl TerrainManager {
    /// Axis-aligned neighbor offsets for the 6-neighbor voxel check.
    const NEIGHBOR_OFFSETS: [Vector3<f32>; 6] = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, -1.0),
    ];

    /// Create a terrain manager from a populated chunk grid.
    ///
    /// Performs the initial mesh build and generates the procedural noise
    /// texture used for surface variation.
    pub fn from_grid(grid: ChunkGrid, texture_manager: &TextureManager) -> EngineResult<Self> {
        let texture = texture_manager.create_noise_texture(512, 512, 5, 20.0, 42)?;
        log::info!("Generated terrain noise texture (512x512, 5 octaves, scale 20.0)");

        let mut manager = Self::from_grid_unmeshed(grid);
        manager.texture = Some(texture);

        let t0 = Instant::now();
        manager.update();
        log::info!(
            "initial mesh build: {:?}, {} chunks, {} triangles, {} leaves",
            t0.elapsed(),
            manager.grid.chunk_count(),
            manager.triangle_count,
            manager.leaf_count
        );

        Ok(manager)
    }

    /// Create an unmeshed, untextured manager. The caller must call `update()`.
    fn from_grid_unmeshed(mut grid: ChunkGrid) -> Self {
        // Cells straddling a chunk's minimum faces belong to the neighbour on
        // that side, so those neighbours have to exist before meshing.
        grid.allocate_seam_neighbours();

        let bounds = grid
            .allocated_world_bounds()
            .unwrap_or_else(|| AABB::new(Point3::origin(), Point3::origin()));

        Self {
            grid,
            adjacency: AdjacencyMap::new(),
            rebuilt_regions: Vec::new(),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
            texture: None,
            bounds,
            triangle_count: 0,
            leaf_count: 0,
        }
    }

    /// Damage voxels within a sphere, reducing their health.
    ///
    /// Voxels whose health reaches zero are converted to air. Indestructible
    /// voxels (bedrock) are unaffected. Only chunks that actually lost a voxel
    /// are marked for remeshing.
    pub fn damage_sphere(&mut self, center: Point3<f32>, radius: f32, damage: u8) {
        let local_center = self.grid.to_local(center);
        let voxel_size = self.grid.voxel_size();

        // A changed voxel affects the marching-cubes cells on both sides of its
        // sample, so the neighbouring chunk across a seam has to remesh too.
        let reach = radius + voxel_size;
        let affected = AABB::from_center_half_extents(local_center, Vector3::repeat(reach));

        let coords: Vec<ChunkCoord> = self.grid.coords_in(&affected).collect();
        let mut any_destroyed = false;
        for coord in &coords {
            if let Some(chunk) = self.grid.chunk_mut(*coord) {
                any_destroyed |= chunk.damage_sphere(local_center, radius, damage);
            }
        }

        if any_destroyed {
            for coord in &coords {
                if let Some(chunk) = self.grid.chunk_mut(*coord) {
                    chunk.mark_dirty();
                }
            }
        }
    }

    /// Update terrain incrementally, remeshing only the chunks marked dirty.
    ///
    /// Call this once per frame or after batch modifications.
    pub fn update(&mut self) {
        // Clear last frame's rebuilt regions so downstream systems see an empty
        // list on frames with no terrain changes.
        self.rebuilt_regions.clear();

        let dirty: Vec<ChunkCoord> = self
            .grid
            .coords()
            .into_iter()
            .filter(|c| self.grid.chunk(*c).is_some_and(|chunk| chunk.is_dirty()))
            .collect();

        if dirty.is_empty() {
            return;
        }

        let t0 = Instant::now();
        let mut adjacency_elapsed = std::time::Duration::ZERO;

        for coord in &dirty {
            adjacency_elapsed += self.remesh_chunk(*coord);
            self.rebuilt_regions
                .push(self.grid.chunk_world_bounds(*coord));
        }

        // A chunk allocated purely to own a seam cell may have produced nothing.
        self.grid.prune_vacant();

        let t1 = Instant::now();
        self.refresh_caches();
        let cache_time = t1.elapsed();

        log::debug!(
            "Terrain updated: {} chunks remeshed of {}, {} triangles in {} leaves, {:?} adjacency, {:?} render data, {:?} total",
            dirty.len(),
            self.grid.chunk_count(),
            self.triangle_count,
            self.leaf_count,
            adjacency_elapsed,
            cache_time,
            t0.elapsed()
        );
    }

    /// Rebuild one chunk's mesh and patch adjacency. Returns adjacency time.
    fn remesh_chunk(&mut self, coord: ChunkCoord) -> std::time::Duration {
        let local_bounds = self.grid.chunk_bounds(coord);
        let first_sample = self.grid.first_sample(coord);
        let voxel_size = self.grid.voxel_size();

        let mut mesh = MeshOctree::new(local_bounds);
        {
            let grid = &self.grid;
            mesh.generate_block(
                Point3::origin(),
                first_sample,
                CHUNK_VOXELS as usize,
                voxel_size,
                |local| grid.get(local),
            );
        }

        let mut old_tris = Vec::new();
        if let Some(chunk) = self.grid.chunk(coord) {
            chunk.mesh().collect_all_triangles_into(&mut old_tris);
        }
        let mut new_tris = Vec::new();
        mesh.collect_all_triangles_into(&mut new_tris);

        let t = Instant::now();
        let old_world = self.to_world_refs(coord, &old_tris);
        let new_world = self.to_world_refs(coord, &new_tris);
        self.adjacency.update_region(
            &old_world,
            &new_world,
            voxel_size * ADJACENCY_TOLERANCE_FACTOR,
        );
        let elapsed = t.elapsed();

        self.grid.chunk_or_insert(coord).replace_mesh(mesh);

        elapsed
    }

    /// Qualify a chunk's triangle refs with its coordinate and lift their
    /// vertex positions into world space, ready for the adjacency map.
    ///
    /// Adjacency matches triangles by quantised vertex position, so positions
    /// from different chunks must be expressed in one shared frame.
    fn to_world_refs(
        &self,
        coord: ChunkCoord,
        tris: &[(TriangleRef, [Point3<f32>; 3])],
    ) -> Vec<(ChunkTriangleRef, [Point3<f32>; 3])> {
        tris.iter()
            .map(|(tri_ref, positions)| {
                (
                    ChunkTriangleRef {
                        chunk: coord,
                        triangle: *tri_ref,
                    },
                    positions.map(|p| self.grid.to_world(p)),
                )
            })
            .collect()
    }

    /// Recompute the cached render buffers, bounds and statistics.
    fn refresh_caches(&mut self) {
        self.render_vertices.clear();
        self.render_indices.clear();
        self.triangle_count = 0;
        self.leaf_count = 0;

        let offset = self.grid.origin().coords;
        for coord in self.grid.coords() {
            let Some(chunk) = self.grid.chunk(coord) else {
                continue;
            };
            let mesh = chunk.mesh();
            let (vertices, indices) = mesh.get_render_data();
            let base = self.render_vertices.len() as u32;
            self.render_vertices
                .extend(vertices.into_iter().map(|mut v| {
                    v.pos.x += offset.x;
                    v.pos.y += offset.y;
                    v.pos.z += offset.z;
                    v
                }));
            self.render_indices
                .extend(indices.into_iter().map(|i| i + base));
            self.triangle_count += mesh.triangle_count();
            self.leaf_count += mesh.leaf_count();
        }

        if let Some(bounds) = self.grid.allocated_world_bounds() {
            self.bounds = bounds;
        }
    }

    /// Get the world-space bounds of the terrain.
    ///
    /// Derived from the allocated chunks, so an unfilled region of the level's
    /// declared extent does not inflate them.
    pub fn bounds(&self) -> &AABB {
        &self.bounds
    }

    // === Water system interface ===

    /// World-space size of the smallest voxel.
    pub fn voxel_size(&self) -> f32 {
        self.grid.voxel_size()
    }

    /// Whether the voxel at a world-space position is solid (density > 0).
    pub fn is_solid_at(&self, x: f32, y: f32, z: f32) -> bool {
        self.voxel_at(Point3::new(x, y, z)).density > 0.0
    }

    /// Voxel at a world-space position.
    fn voxel_at(&self, world: Point3<f32>) -> super::voxel::Voxel {
        self.grid.get(self.grid.to_local(world))
    }

    /// Get the highest solid terrain surface height at a given (x, z) position.
    ///
    /// Walks the column at voxel resolution, finding where density transitions
    /// from positive (solid) to negative (air). Returns the interpolated surface Y,
    /// or None if the column is entirely air.
    pub fn approx_surface_height_at(&self, x: f32, z: f32) -> Option<f32> {
        self.surface_heights_at(x, z).into_iter().next()
    }

    /// Get all solid surface heights in a column, sorted top-to-bottom.
    ///
    /// A surface is detected where density transitions from positive (solid, below)
    /// to negative (air, above). For sky islands with multiple terrain layers, this
    /// returns multiple heights. Used by the water system to find floors.
    pub fn surface_heights_at(&self, x: f32, z: f32) -> Vec<f32> {
        let bounds = &self.bounds;

        if x < bounds.min.x || x > bounds.max.x || z < bounds.min.z || z > bounds.max.z {
            return Vec::new();
        }

        let step = self.grid.voxel_size();
        let mut surfaces = Vec::new();

        // Walk the column from top to bottom at voxel resolution.
        // Detect sign transitions: solid (density > 0) below, air (density <= 0) above.
        let mut y = bounds.max.y;
        let mut prev_density = self.voxel_at(Point3::new(x, y, z)).density;

        y -= step;
        while y >= bounds.min.y {
            let density = self.voxel_at(Point3::new(x, y, z)).density;

            // Transition from solid to air (going upward): this Y is a surface.
            if density > 0.0 && prev_density <= 0.0 {
                // Linearly interpolate the exact surface Y between the two samples.
                let t = density / (density - prev_density);
                let surface_y = y + t * step;
                surfaces.push(surface_y);
            }

            prev_density = density;
            y -= step;
        }

        surfaces
    }

    /// Regions that were rebuilt in the most recent `update()` call.
    ///
    /// Downstream systems (e.g. WaterSystem) read these to detect terrain
    /// changes and invalidate cached floor levels.
    pub fn dirty_regions(&self) -> &[AABB] {
        &self.rebuilt_regions
    }

    // === Mesh-precise queries ===

    /// Whether a point is inside the terrain mesh.
    ///
    /// Uses a fast-path voxel neighborhood check: if the point's voxel and all
    /// 6 axis-aligned neighbors agree (all solid or all air), the answer is
    /// immediate. Only near the surface — where marching cubes interpolation
    /// differs from the voxel grid — does this fall back to a ray parity test
    /// against the actual triangle mesh.
    pub fn is_mesh_solid_at(&self, x: f32, y: f32, z: f32) -> bool {
        let pos = Point3::new(x, y, z);
        let center_solid = self.voxel_at(pos).density > 0.0;

        // Check 6-connected voxel neighbors for unanimity.
        let vs = self.grid.voxel_size();
        let all_agree = Self::NEIGHBOR_OFFSETS.iter().all(|offset| {
            let neighbor = Point3::new(x + offset.x * vs, y + offset.y * vs, z + offset.z * vs);
            (self.voxel_at(neighbor).density > 0.0) == center_solid
        });

        if all_agree {
            return center_solid;
        }

        // Near the surface — use ray parity test (cast +Y, count crossings).
        let ray_length = self.bounds.max.y - y + vs;
        if ray_length <= 0.0 {
            return false;
        }

        let hits = self.ray_cast_all(pos, Vector3::new(0.0, 1.0, 0.0), ray_length);
        hits.len() % 2 == 1
    }

    /// Get the highest mesh surface height at a given (x, z) position.
    ///
    /// Casts a vertical ray downward through the triangle mesh, returning the
    /// Y coordinate of the nearest hit. This accounts for marching cubes
    /// interpolation, unlike the voxel-based `approx_surface_height_at`.
    pub fn mesh_surface_height_at(&self, x: f32, z: f32) -> Option<f32> {
        let (origin, length) = self.downward_ray(x, z);
        self.ray_cast_all(origin, Vector3::new(0.0, -1.0, 0.0), length)
            .into_iter()
            .min_by(|a, b| a.t.total_cmp(&b.t))
            .map(|hit| hit.point.y)
    }

    /// Get all mesh surface heights in a column, sorted top-to-bottom.
    ///
    /// Like `surface_heights_at` but uses the actual triangle mesh instead of
    /// voxel density transitions.
    pub fn mesh_surface_heights_at(&self, x: f32, z: f32) -> Vec<f32> {
        let (origin, length) = self.downward_ray(x, z);
        let hits = self.ray_cast_all(origin, Vector3::new(0.0, -1.0, 0.0), length);

        // Filter to surfaces facing upward (normal.y > 0 means top-of-terrain).
        let mut heights: Vec<f32> = hits
            .iter()
            .filter(|h| h.normal.y > 0.0)
            .map(|h| h.point.y)
            .collect();
        heights.sort_by(|a, b| b.total_cmp(a));
        heights
    }

    /// Origin and length of a downward ray spanning the terrain at (x, z).
    fn downward_ray(&self, x: f32, z: f32) -> (Point3<f32>, f32) {
        let vs = self.grid.voxel_size();
        let origin = Point3::new(x, self.bounds.max.y + vs, z);
        let length = (self.bounds.max.y - self.bounds.min.y) + 2.0 * vs;
        (origin, length)
    }

    /// Cast a ray against every chunk it passes through, returning all hits in
    /// world space.
    fn ray_cast_all(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Vec<RayHit> {
        let local_origin = self.grid.to_local(origin);
        let end = local_origin + direction * length;
        let sweep = AABB::new(
            Point3::new(
                local_origin.x.min(end.x),
                local_origin.y.min(end.y),
                local_origin.z.min(end.z),
            ),
            Point3::new(
                local_origin.x.max(end.x),
                local_origin.y.max(end.y),
                local_origin.z.max(end.z),
            ),
        );

        let offset = self.grid.origin().coords;
        let mut hits = Vec::new();
        for coord in self.grid.coords_in(&sweep) {
            let Some(chunk) = self.grid.chunk(coord) else {
                continue;
            };
            hits.extend(
                chunk
                    .mesh()
                    .ray_cast_all(local_origin, direction, length)
                    .into_iter()
                    .map(|mut hit| {
                        hit.point += offset;
                        hit
                    }),
            );
        }
        hits
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
        let offset = self.grid.origin().coords;
        let mut vertices: Vec<Vertex> = Vec::new();
        let mut indices: Vec<u32> = Vec::new();
        for coord in self.grid.coords() {
            let Some(chunk) = self.grid.chunk(coord) else {
                continue;
            };
            let (v, i) = chunk.mesh().get_render_data_frustum_culled(frustum_planes);
            let base = vertices.len() as u32;
            vertices.extend(v.into_iter().map(|mut vert| {
                vert.pos.x += offset.x;
                vert.pos.y += offset.y;
                vert.pos.z += offset.z;
                vert
            }));
            indices.extend(i.into_iter().map(|idx| idx + base));
        }
        (vertices, indices)
    }

    // === Adjacency ===

    /// Get the triangle adjacency map.
    #[allow(dead_code)]
    pub fn adjacency(&self) -> &AdjacencyMap<ChunkTriangleRef> {
        &self.adjacency
    }

    // === Statistics ===

    /// Get the total triangle count.
    pub fn triangle_count(&self) -> usize {
        self.triangle_count
    }

    /// Get the number of mesh octree leaves.
    pub fn leaf_count(&self) -> usize {
        self.leaf_count
    }

    /// Number of allocated chunks.
    #[allow(dead_code)]
    pub fn chunk_count(&self) -> usize {
        self.grid.chunk_count()
    }
}

#[cfg(test)]
use crate::terrain::voxel::Voxel;

#[cfg(test)]
impl TerrainManager {
    /// Create an empty terrain manager at the given voxel size.
    /// Test-only helper; production code goes through `from_grid`.
    fn new(voxel_size: f32) -> Self {
        Self::from_grid_unmeshed(ChunkGrid::new(Point3::origin(), voxel_size))
    }

    /// Set a voxel at a world position.
    /// Test-only helper for setting individual voxels in tests.
    fn set_voxel(&mut self, position: Point3<f32>, voxel: Voxel) {
        let local = self.grid.to_local(position);
        self.grid.set(local, voxel);
        // A voxel on a chunk's minimum face is also a corner of the neighbour's
        // cells, so make sure that neighbour exists and will be meshed.
        self.grid.allocate_seam_neighbours();
    }
}

impl StaticGeometry for TerrainManager {
    fn query_region(&self, aabb: &AABB) -> MeshPatch {
        let local = self.grid.aabb_to_local(aabb);
        let offset = self.grid.origin().coords;

        let mut results: Vec<(ChunkTriangleRef, crate::collision::Triangle)> = Vec::new();
        for coord in self.grid.coords_in(&local) {
            let Some(chunk) = self.grid.chunk(coord) else {
                continue;
            };
            for (tri_ref, mut triangle) in chunk.mesh().query_aabb(&local) {
                triangle.v0 += offset;
                triangle.v1 += offset;
                triangle.v2 += offset;
                results.push((
                    ChunkTriangleRef {
                        chunk: coord,
                        triangle: tri_ref,
                    },
                    triangle,
                ));
            }
        }

        // Build a lookup from ChunkTriangleRef → local index in the patch.
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
                    triangle: *triangle,
                    neighbors,
                }
            })
            .collect();

        MeshPatch { triangles }
    }
}

impl ProbeTarget for TerrainManager {
    fn raycast(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Option<ProbeHit> {
        let end = origin + direction * length;
        let sweep_aabb = AABB::new(
            Point3::new(
                origin.x.min(end.x),
                origin.y.min(end.y),
                origin.z.min(end.z),
            ),
            Point3::new(
                origin.x.max(end.x),
                origin.y.max(end.y),
                origin.z.max(end.z),
            ),
        );
        let patch = self.query_region(&sweep_aabb);

        let mut earliest: Option<RayHit> = None;
        for pt in &patch.triangles {
            if let Some(hit) = ray_triangle(origin, direction, &pt.triangle, length) {
                if earliest.as_ref().map_or(true, |e: &RayHit| hit.t < e.t) {
                    earliest = Some(hit);
                }
            }
        }

        earliest.map(|h| ProbeHit {
            t: h.t / length,
            point: h.point,
            normal: h.normal,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::voxel::VoxelMaterial;

    /// Build a solid slab of terrain from `min` to `max` (world, inclusive) at
    /// the given voxel size, and mesh it.
    fn slab(voxel_size: f32, min: Point3<f32>, max: Point3<f32>) -> TerrainManager {
        let mut manager = TerrainManager::new(voxel_size);
        let mut x = min.x;
        while x <= max.x {
            let mut y = min.y;
            while y <= max.y {
                let mut z = min.z;
                while z <= max.z {
                    manager.set_voxel(Point3::new(x, y, z), Voxel::solid(VoxelMaterial::Rock, 1));
                    z += voxel_size;
                }
                y += voxel_size;
            }
            x += voxel_size;
        }
        manager.update();
        manager
    }

    #[test]
    fn generates_geometry_from_voxels() {
        let manager = slab(
            1.0,
            Point3::new(-5.0, 0.0, -5.0),
            Point3::new(5.0, 0.0, 5.0),
        );
        assert!(manager.has_geometry());
        assert!(manager.triangle_count() > 0);
    }

    /// A slab far smaller than any world extent must allocate only the chunks it
    /// touches, plus the seam neighbours that own its minimum-face cells.
    #[test]
    fn chunk_allocation_is_proportional_to_content() {
        let manager = slab(1.0, Point3::new(1.0, 1.0, 1.0), Point3::new(4.0, 2.0, 4.0));
        // One chunk holds the slab; the seven negative-octant neighbours own the
        // cells that close its underside and minimum faces. Four of those turn
        // out empty and are pruned.
        assert!(
            manager.chunk_count() <= 8,
            "a 4x2x4 voxel feature allocated {} chunks",
            manager.chunk_count()
        );
        assert!(manager.chunk_count() >= 1);
    }

    /// The declared voxel size is the voxel size that gets used — a regression
    /// test for the old `log2(world_size / voxel_size)` derivation, which
    /// produced voxels twice the declared size.
    #[test]
    fn voxel_size_is_as_declared() {
        for voxel_size in [0.25_f32, 0.5, 1.0, 2.0] {
            let manager = TerrainManager::new(voxel_size);
            assert_eq!(manager.voxel_size(), voxel_size);

            // A single voxel write must land in exactly one voxel cell: reading
            // one voxel away must see air.
            let mut manager = manager;
            manager.set_voxel(
                Point3::new(0.5 * voxel_size, 0.5 * voxel_size, 0.5 * voxel_size),
                Voxel::solid(VoxelMaterial::Rock, 1),
            );
            assert!(manager.is_solid_at(0.5 * voxel_size, 0.5 * voxel_size, 0.5 * voxel_size));
            assert!(
                !manager.is_solid_at(1.5 * voxel_size, 0.5 * voxel_size, 0.5 * voxel_size),
                "voxel at size {voxel_size} bled into its neighbour"
            );
        }
    }

    /// Terrain must not step or crack where it crosses a chunk seam. A flat
    /// slab spanning the boundary at x = 32 should read the same height on both
    /// sides.
    #[test]
    fn surface_height_is_continuous_across_a_chunk_seam() {
        let voxel_size = 1.0;
        let manager = slab(
            voxel_size,
            Point3::new(24.0, 0.0, 0.0),
            Point3::new(40.0, 2.0, 8.0),
        );

        let z = 4.5;
        let mut heights = Vec::new();
        for x in [28.5_f32, 30.5, 31.5, 32.5, 33.5, 35.5] {
            let h = manager
                .mesh_surface_height_at(x, z)
                .unwrap_or_else(|| panic!("no surface at x={x}"));
            heights.push(h);
        }

        let min = heights.iter().copied().fold(f32::INFINITY, f32::min);
        let max = heights.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        assert!(
            max - min <= voxel_size,
            "surface height jumped across the chunk seam: {heights:?}"
        );
    }

    /// A crack at a chunk seam is an unmatched mesh edge. A solid slab spanning
    /// two chunks is a closed surface, so every edge must be shared by exactly
    /// two triangles — no exceptions anywhere, seam included.
    #[test]
    fn mesh_has_no_open_edges_across_a_chunk_seam() {
        let manager = slab(
            1.0,
            Point3::new(24.0, 0.0, 0.0),
            Point3::new(40.0, 2.0, 8.0),
        );

        let open = manager
            .adjacency()
            .boundary_edge_count(manager.triangle_count());
        assert_eq!(
            open,
            0,
            "{open} open edges in a {}-triangle closed slab spanning a chunk seam",
            manager.triangle_count()
        );
    }

    /// Physics reads terrain through `query_region`. A triangle straddling a
    /// chunk seam must be reported exactly once — duplicates become duplicate
    /// contacts and phantom forces in the solver.
    #[test]
    fn seam_triangles_are_not_duplicated() {
        let manager = slab(
            1.0,
            Point3::new(24.0, 0.0, 0.0),
            Point3::new(40.0, 2.0, 8.0),
        );

        let query = AABB::new(Point3::new(30.0, -2.0, 2.0), Point3::new(34.0, 4.0, 6.0));
        let patch = manager.query_region(&query);
        assert!(!patch.triangles.is_empty(), "seam query returned nothing");

        let mut seen = std::collections::HashSet::new();
        for pt in &patch.triangles {
            let key = |p: Point3<f32>| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits());
            let id = (
                key(pt.triangle.v0),
                key(pt.triangle.v1),
                key(pt.triangle.v2),
            );
            assert!(seen.insert(id), "duplicate triangle across chunk seam");
        }
    }

    /// Adjacency must link triangles that meet across a chunk boundary, or the
    /// physics feature-aware pipeline sees false boundary edges at every seam.
    #[test]
    fn adjacency_links_across_a_chunk_seam() {
        let manager = slab(
            1.0,
            Point3::new(24.0, 0.0, 0.0),
            Point3::new(40.0, 2.0, 8.0),
        );

        // Query a thin box centred on the seam and count how many of the
        // returned triangles found a neighbour within the patch.
        let query = AABB::new(Point3::new(31.0, 1.5, 2.0), Point3::new(33.0, 3.0, 6.0));
        let patch = manager.query_region(&query);
        assert!(!patch.triangles.is_empty());

        let linked = patch
            .triangles
            .iter()
            .filter(|t| t.neighbors.iter().any(Option::is_some))
            .count();
        assert!(
            linked > 0,
            "no triangle in the seam patch has a neighbour ({} triangles)",
            patch.triangles.len()
        );
    }

    /// End-to-end over the primary fixture: the level generates, meshes, and
    /// allocates far fewer chunks than its declared extent could hold.
    #[test]
    fn test_arena_generates_sparsely_at_its_declared_voxel_size() {
        use crate::terrain::generation::generate_terrain;
        use crate::terrain::voxel::DurabilityConfig;

        let level = crate::level::load_level(std::path::Path::new("levels/test_arena.level.ron"))
            .expect("test_arena should load");

        let bounds = level.terrain.bounds.to_aabb();
        let mut grid = ChunkGrid::new(Point3::origin(), level.terrain.voxel_size);
        generate_terrain(
            &mut grid,
            &level.terrain,
            &DurabilityConfig::default(),
            &bounds,
        );

        let mut manager = TerrainManager::from_grid_unmeshed(grid);
        manager.update();

        assert_eq!(manager.voxel_size(), level.terrain.voxel_size);
        assert!(manager.has_geometry());

        // Sparsity is the property that allocation follows the content, not the
        // declared extent: quadrupling the empty headroom above the terrain must
        // not allocate a single extra chunk.
        let allocated = |extra_height: f32| {
            let extent = AABB::new(
                bounds.min,
                Point3::new(bounds.max.x, bounds.max.y + extra_height, bounds.max.z),
            );
            let mut g = ChunkGrid::new(Point3::origin(), level.terrain.voxel_size);
            generate_terrain(
                &mut g,
                &level.terrain,
                &DurabilityConfig::default(),
                &extent,
            );
            g.chunk_count()
        };
        let headroom = 4.0 * CHUNK_VOXELS as f32 * level.terrain.voxel_size;
        assert_eq!(
            allocated(headroom),
            allocated(0.0),
            "raising the ceiling by {headroom} m changed how many chunks were allocated"
        );

        // The terrain surface sits at base_height away from any feature.
        let h = manager
            .mesh_surface_height_at(20.0, 30.0)
            .expect("no surface at an unfeatured column");
        assert!(
            (h - level.terrain.base_height).abs() < level.terrain.voxel_size,
            "flat terrain surfaced at {h}, expected {}",
            level.terrain.base_height
        );
    }

    #[test]
    fn damage_removes_geometry_and_reports_dirty_regions() {
        let mut manager = slab(
            1.0,
            Point3::new(-5.0, 0.0, -5.0),
            Point3::new(5.0, 3.0, 5.0),
        );
        let before = manager.triangle_count();

        manager.damage_sphere(Point3::new(0.0, 2.0, 0.0), 3.0, 255);
        manager.update();

        assert!(!manager.dirty_regions().is_empty());
        assert_ne!(manager.triangle_count(), before);
        assert!(!manager.is_solid_at(0.0, 2.0, 0.0));
    }
}
