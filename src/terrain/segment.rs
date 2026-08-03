//! One independently placed area of voxel terrain.
//!
//! ```text
//!   ┌──────────────────────────────────────────────────────┐
//!   │  Segment  "start_plaza"                              │
//!   │    SegmentFrame   origin + 90°-multiple yaw          │
//!   │    ChunkGrid      voxels, entirely segment-local     │
//!   │    AdjacencyMap   triangle links within this segment │
//!   │    Anchors        named local frames                 │
//!   └──────────────────────────────────────────────────────┘
//!         ▲  world query ──to_local──▶ grid ──to_world──▶ world result
//! ```
//!
//! Every public method takes and returns **world** coordinates; the conversion
//! happens here so that neither the grid below nor [`TerrainWorld`] above has to
//! think about frames. Generation, meshing and voxel storage never see a world
//! coordinate at all, which is what makes a segment relocatable: moving one
//! cannot change its terrain.
//!
//! [`TerrainWorld`]: super::world::TerrainWorld

use nalgebra::{Point3, Vector3};
use std::time::{Duration, Instant};

use super::adjacency::AdjacencyMap;
use super::anchor::Anchor;
use super::chunk::{ChunkCoord, ChunkTriangleRef, CHUNK_VOXELS};
use super::chunk_grid::ChunkGrid;
use super::frame::SegmentFrame;
use super::mesh_octree::{MeshBuildTimings, MeshOctree, TriangleRef};
use super::render_cache::{build_chunk_render_data, ChunkRenderCache};
use super::voxel::Voxel;
use crate::collision::ray_triangle::RayHit;
use crate::collision::{Triangle, AABB};
use crate::rendering::vertex::Vertex;

/// Fraction of a voxel used as the vertex-matching tolerance when linking
/// triangles into the adjacency map.
const ADJACENCY_TOLERANCE_FACTOR: f32 = 0.01;

/// Where a segment is in its lifecycle.
///
/// Everything is `Active` today. The state exists so that streaming and
/// per-segment reset become a scheduling change rather than a redesign — see
/// "Seams to leave open" in `LEVEL_SEGMENTS_PLAN.md`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SegmentState {
    /// Voxels generated, not yet meshed.
    Loaded,
    /// Meshed and queryable, but excluded from render and physics.
    Meshed,
    /// Fully participating.
    #[default]
    Active,
}

/// Time spent remeshing one segment, split by phase.
#[derive(Debug, Clone, Copy, Default)]
pub struct SegmentTimings {
    pub chunks_dirtied: usize,
    /// Everything from sampling voxels to owning the new octree. `build` and
    /// `collect` are sub-phases of this, so they do not add to it.
    pub remesh: Duration,
    /// Mesh construction inside `remesh`, split by phase.
    pub build: MeshBuildTimings,
    /// Walking the old and new octrees to list their triangles for adjacency.
    pub collect: Duration,
    pub adjacency: Duration,
}

/// Wall-clock breakdown of one render-buffer concatenation.
#[derive(Debug, Clone, Copy, Default)]
pub struct ConcatTimings {
    /// Walking each chunk's mesh octree to collect its vertices and indices.
    pub mesh_walk: Duration,
    /// Lifting vertices into world space and appending them.
    pub transform: Duration,
    /// Rebasing chunk-local indices onto the concatenated vertex buffer.
    pub rebase: Duration,
}

impl ConcatTimings {
    pub fn add(&mut self, other: &Self) {
        self.mesh_walk += other.mesh_walk;
        self.transform += other.transform;
        self.rebase += other.rebase;
    }
}

/// Time spent rebuilding one chunk, split by phase.
#[derive(Debug, Clone, Copy, Default)]
struct ChunkRemeshTimings {
    build: MeshBuildTimings,
    collect: Duration,
    adjacency: Duration,
}

/// An independently placed chunk grid with a name, a frame and named anchors.
pub struct Segment {
    /// Unique within the level. Anchors are referred to as `segment.anchor`.
    name: String,

    /// Where this segment sits in the world.
    frame: SegmentFrame,

    /// Voxel chunks and their meshes, in segment-local space.
    grid: ChunkGrid,

    /// Edge-based triangle adjacency across this segment's chunks.
    ///
    /// Deliberately per-segment rather than level-wide: segments are the unit of
    /// load and unload, and a level-wide map keyed by position would have to be
    /// rebuilt whenever one came or went. Welded joins are not implemented, so
    /// no two segments have coincident geometry to link across anyway.
    adjacency: AdjacencyMap<ChunkTriangleRef>,

    /// Each chunk's render geometry in world space, so a terrain edit only
    /// rebuilds what it dirtied. Kept current by [`Self::remesh_chunk`].
    render_cache: ChunkRenderCache,

    /// Named local frames within this segment.
    anchors: Vec<Anchor>,

    /// Lifecycle state. Always `Active` today.
    state: SegmentState,

    /// World bounds of the allocated chunks, refreshed on each update.
    bounds: AABB,

    /// Cached mesh statistics, refreshed on each update.
    triangle_count: usize,
    leaf_count: usize,
    vertex_count: usize,
}

impl Segment {
    /// Create a segment from a populated grid. The caller must call
    /// [`Self::update`] before querying geometry.
    pub fn new(
        name: impl Into<String>,
        frame: SegmentFrame,
        mut grid: ChunkGrid,
        anchors: Vec<Anchor>,
    ) -> Self {
        // Cells straddling a chunk's minimum faces belong to the neighbour on
        // that side, so those neighbours have to exist before meshing.
        grid.allocate_seam_neighbours();

        let bounds = grid
            .allocated_bounds()
            .map(|b| frame.aabb_to_world(&b))
            .unwrap_or_else(|| AABB::new(frame.origin(), frame.origin()));

        Self {
            name: name.into(),
            frame,
            grid,
            adjacency: AdjacencyMap::new(),
            render_cache: ChunkRenderCache::new(),
            anchors,
            state: SegmentState::Active,
            bounds,
            triangle_count: 0,
            leaf_count: 0,
            vertex_count: 0,
        }
    }

    // === Identity and placement ===

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn frame(&self) -> &SegmentFrame {
        &self.frame
    }

    pub fn state(&self) -> SegmentState {
        self.state
    }

    pub fn anchors(&self) -> &[Anchor] {
        &self.anchors
    }

    /// Find an anchor by name.
    pub fn anchor(&self, name: &str) -> Option<&Anchor> {
        self.anchors.iter().find(|a| a.name() == name)
    }

    /// World bounds of the allocated chunks.
    pub fn bounds(&self) -> &AABB {
        &self.bounds
    }

    /// Edge length of one voxel, in world units.
    pub fn voxel_size(&self) -> f32 {
        self.grid.voxel_size()
    }

    /// Edge length of one chunk, in world units.
    pub fn chunk_extent(&self) -> f32 {
        self.grid.chunk_extent()
    }

    pub fn grid(&self) -> &ChunkGrid {
        &self.grid
    }

    /// World AABBs of this segment's chunks that hold at least one solid voxel.
    ///
    /// The unit Rule 4's contention check compares between segments.
    pub fn solid_chunk_bounds(&self) -> Vec<AABB> {
        self.grid
            .solid_coords()
            .into_iter()
            .map(|coord| self.frame.aabb_to_world(&self.grid.chunk_bounds(coord)))
            .collect()
    }

    /// World bounds of just the solid content, ignoring the seam shell.
    pub fn solid_bounds(&self) -> Option<AABB> {
        self.grid
            .bounds_of(self.grid.solid_coords())
            .map(|b| self.frame.aabb_to_world(&b))
    }

    // === Modification ===

    /// Damage voxels within a world-space sphere. Returns true if any chunk was
    /// marked dirty.
    pub fn damage_sphere(&mut self, center: Point3<f32>, radius: f32, damage: u8) -> bool {
        let local_center = self.frame.to_local(center);
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
        any_destroyed
    }

    /// Remesh the chunks marked dirty, appending their world bounds to
    /// `rebuilt`. Returns `None` if there was nothing to do.
    pub fn update(&mut self, rebuilt: &mut Vec<AABB>) -> Option<SegmentTimings> {
        let dirty: Vec<ChunkCoord> = self
            .grid
            .coords()
            .into_iter()
            .filter(|c| self.grid.chunk(*c).is_some_and(|chunk| chunk.is_dirty()))
            .collect();

        if dirty.is_empty() {
            return None;
        }

        let t0 = Instant::now();
        let mut adjacency = Duration::ZERO;
        let mut build = MeshBuildTimings::default();
        let mut collect = Duration::ZERO;

        for coord in &dirty {
            let chunk = self.remesh_chunk(*coord);
            adjacency += chunk.adjacency;
            collect += chunk.collect;
            build.add(&chunk.build);
            rebuilt.push(self.frame.aabb_to_world(&self.grid.chunk_bounds(*coord)));
        }

        // A chunk allocated purely to own a seam cell may have produced nothing.
        self.grid.prune_vacant();
        let grid = &self.grid;
        self.render_cache
            .retain_coords(|coord| grid.chunk(coord).is_some());
        self.refresh_stats();

        Some(SegmentTimings {
            chunks_dirtied: dirty.len(),
            remesh: t0.elapsed() - adjacency,
            build,
            collect,
            adjacency,
        })
    }

    /// Rebuild one chunk's mesh and patch adjacency.
    fn remesh_chunk(&mut self, coord: ChunkCoord) -> ChunkRemeshTimings {
        let local_bounds = self.grid.chunk_bounds(coord);
        let first_sample = self.grid.first_sample(coord);
        let voxel_size = self.grid.voxel_size();

        let mut mesh = MeshOctree::new(local_bounds);
        let build = mesh.generate_block(
            Point3::origin(),
            first_sample,
            CHUNK_VOXELS as usize,
            voxel_size,
            &self.grid,
        );

        let t_collect = Instant::now();
        let mut old_tris = Vec::new();
        if let Some(chunk) = self.grid.chunk(coord) {
            chunk.mesh().collect_all_triangles_into(&mut old_tris);
        }
        let mut new_tris = Vec::new();
        mesh.collect_all_triangles_into(&mut new_tris);
        let collect = t_collect.elapsed();

        let t = Instant::now();
        let old_refs = qualify(coord, &old_tris);
        let new_refs = qualify(coord, &new_tris);
        self.adjacency.update_region(
            &old_refs,
            &new_refs,
            voxel_size * ADJACENCY_TOLERANCE_FACTOR,
        );
        let adjacency = t.elapsed();

        let chunk = self.grid.chunk_or_insert(coord);
        chunk.replace_mesh(mesh);
        let data = build_chunk_render_data(chunk, &self.frame);
        self.render_cache.insert(coord, data);

        ChunkRemeshTimings {
            build,
            collect,
            adjacency,
        }
    }

    /// Recompute cached bounds and mesh statistics.
    fn refresh_stats(&mut self) {
        self.triangle_count = 0;
        self.leaf_count = 0;
        self.vertex_count = 0;
        for chunk in self.grid.chunks() {
            let mesh = chunk.mesh();
            self.triangle_count += mesh.triangle_count();
            self.leaf_count += mesh.leaf_count();
            self.vertex_count += mesh.vertex_count();
        }
        if let Some(bounds) = self.grid.allocated_bounds() {
            self.bounds = self.frame.aabb_to_world(&bounds);
        }
    }

    // === Queries, all in world space ===

    /// Voxel at a world position. Outside this segment reads as air.
    pub fn voxel_at(&self, world: Point3<f32>) -> Voxel {
        self.grid.get(self.frame.to_local(world))
    }

    /// All solid surface heights in a world-space column, top to bottom.
    ///
    /// Detected from voxel density transitions, at this segment's resolution.
    pub fn surface_heights_at(&self, x: f32, z: f32) -> Vec<f32> {
        let bounds = &self.bounds;
        if x < bounds.min.x || x > bounds.max.x || z < bounds.min.z || z > bounds.max.z {
            return Vec::new();
        }

        let step = self.grid.voxel_size();
        let mut surfaces = Vec::new();

        let mut y = bounds.max.y;
        let mut prev_density = self.voxel_at(Point3::new(x, y, z)).density;

        y -= step;
        while y >= bounds.min.y {
            let density = self.voxel_at(Point3::new(x, y, z)).density;
            if density > 0.0 && prev_density <= 0.0 {
                let t = density / (density - prev_density);
                surfaces.push(y + t * step);
            }
            prev_density = density;
            y -= step;
        }

        surfaces
    }

    /// Cast a ray against every chunk it passes through, returning world-space
    /// hits.
    pub fn ray_cast_all(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Vec<RayHit> {
        let local_origin = self.frame.to_local(origin);
        let local_dir = self.frame.rotate_to_local(direction);
        let end = local_origin + local_dir * length;
        let sweep = span(local_origin, end);

        let mut hits = Vec::new();
        for coord in self.grid.coords_in(&sweep) {
            let Some(chunk) = self.grid.chunk(coord) else {
                continue;
            };
            hits.extend(
                chunk
                    .mesh()
                    .ray_cast_all(local_origin, local_dir, length)
                    .into_iter()
                    .map(|hit| RayHit {
                        t: hit.t,
                        point: self.frame.to_world(hit.point),
                        normal: self.frame.rotate_to_world(hit.normal),
                    }),
            );
        }
        hits
    }

    /// Triangles overlapping a world-space AABB, in world space, each with the
    /// reference that identifies it within this segment.
    ///
    /// The AABB conversion is exact rather than conservative: a 90° yaw permutes
    /// the axes, so an axis-aligned box stays axis-aligned.
    pub fn query_region(&self, world: &AABB) -> Vec<(ChunkTriangleRef, Triangle)> {
        let local = self.frame.aabb_to_local(world);
        let mut results = Vec::new();
        for coord in self.grid.coords_in(&local) {
            let Some(chunk) = self.grid.chunk(coord) else {
                continue;
            };
            for (tri_ref, triangle) in chunk.mesh().query_aabb(&local) {
                results.push((
                    ChunkTriangleRef {
                        chunk: coord,
                        triangle: tri_ref,
                    },
                    Triangle::new(
                        self.frame.to_world(triangle.v0),
                        self.frame.to_world(triangle.v1),
                        self.frame.to_world(triangle.v2),
                    ),
                ));
            }
        }
        results
    }

    /// Adjacency neighbours of a triangle within this segment.
    pub fn neighbours(&self, tri_ref: &ChunkTriangleRef) -> Option<[Option<ChunkTriangleRef>; 3]> {
        self.adjacency.neighbors(tri_ref).map(|n| n.neighbors)
    }

    // === Render data ===

    /// Append this segment's geometry to level-wide buffers, transformed into
    /// world space.
    ///
    /// The frame is baked in here, at mesh-collection time, so everything
    /// downstream of the terrain stays world-space and untouched.
    pub fn append_render_data(
        &self,
        vertices: &mut Vec<Vertex>,
        indices: &mut Vec<u32>,
    ) -> ConcatTimings {
        let mut timings = ConcatTimings::default();
        for coord in self.grid.coords() {
            let Some(chunk) = self.grid.chunk(coord) else {
                continue;
            };

            // A miss means the chunk has never been remeshed, so build it here
            // rather than silently omit its geometry.
            let t_walk = Instant::now();
            let built;
            let data = match self.render_cache.get(coord) {
                Some(cached) => cached,
                None => {
                    built = build_chunk_render_data(chunk, &self.frame);
                    &built
                }
            };
            timings.mesh_walk += t_walk.elapsed();

            let t_copy = Instant::now();
            let base = vertices.len() as u32;
            vertices.extend_from_slice(&data.vertices);
            timings.transform += t_copy.elapsed();

            let t_indices = Instant::now();
            indices.extend(data.indices.iter().map(|idx| idx + base));
            timings.rebase += t_indices.elapsed();
        }
        timings
    }

    /// As [`Self::append_render_data`], but only the chunks' geometry that
    /// survives a frustum test.
    pub fn append_render_data_culled(
        &self,
        frustum_planes: &[(Vector3<f32>, f32); 6],
        vertices: &mut Vec<Vertex>,
        indices: &mut Vec<u32>,
    ) {
        for coord in self.grid.coords() {
            let Some(chunk) = self.grid.chunk(coord) else {
                continue;
            };
            let (v, i) = chunk.mesh().get_render_data_frustum_culled(frustum_planes);
            let base = vertices.len() as u32;
            vertices.extend(v.into_iter().map(|vert| self.to_world_vertex(vert)));
            indices.extend(i.into_iter().map(|idx| idx + base));
        }
    }

    /// Lift a mesh vertex into world space. Position and normal both rotate;
    /// the homogeneous `w` and the shading attributes do not.
    fn to_world_vertex(&self, mut vertex: Vertex) -> Vertex {
        let pos = self
            .frame
            .to_world(Point3::new(vertex.pos.x, vertex.pos.y, vertex.pos.z));
        vertex.pos.x = pos.x;
        vertex.pos.y = pos.y;
        vertex.pos.z = pos.z;
        vertex.normal = self.frame.rotate_to_world(vertex.normal);
        vertex
    }

    // === Statistics ===

    pub fn triangle_count(&self) -> usize {
        self.triangle_count
    }

    pub fn leaf_count(&self) -> usize {
        self.leaf_count
    }

    pub fn vertex_count(&self) -> usize {
        self.vertex_count
    }

    pub fn chunk_count(&self) -> usize {
        self.grid.chunk_count()
    }

    pub fn solid_chunk_count(&self) -> usize {
        self.grid.chunks().filter(|c| c.has_solid()).count()
    }

    /// Number of mesh edges belonging to exactly one triangle within this
    /// segment. A closed surface has none.
    pub fn open_edge_count(&self) -> usize {
        self.adjacency.boundary_edge_count(self.triangle_count)
    }

    #[cfg(test)]
    pub fn adjacency(&self) -> &AdjacencyMap<ChunkTriangleRef> {
        &self.adjacency
    }

    #[cfg(test)]
    pub fn set_voxel(&mut self, world: Point3<f32>, voxel: Voxel) {
        self.grid.set(self.frame.to_local(world), voxel);
        self.grid.allocate_seam_neighbours();
    }
}

/// Qualify a chunk's triangle refs with its coordinate, keeping the vertex
/// positions segment-local.
///
/// Adjacency matches by quantised position, and a segment's triangles are all
/// in one frame, so the local positions are already directly comparable.
fn qualify(
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
                *positions,
            )
        })
        .collect()
}

/// An AABB spanning two points in any order.
fn span(a: Point3<f32>, b: Point3<f32>) -> AABB {
    AABB::new(
        Point3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)),
        Point3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)),
    )
}
