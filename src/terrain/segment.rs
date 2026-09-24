//! One independently placed area of voxel terrain.
//!
//! ```text
//!   ┌──────────────────────────────────────────────────────┐
//!   │  Segment  "start_plaza"                              │
//!   │    SegmentFrame   origin + 90°-multiple yaw          │
//!   │    ChunkGrid      voxels, entirely segment-local     │
//!   │    SegmentAdjacency  triangle links, chunk by chunk  │
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
use rayon::prelude::*;
use std::borrow::Cow;
use std::time::{Duration, Instant};

use super::adjacency::{AdjacencyTimings, DefectiveEdge};
use super::anchor::Anchor;
use super::blast::{self, BlastConfig};
use super::chunk::{ChunkCoord, ChunkTriangleRef};
use super::chunk_grid::ChunkGrid;
use super::chunk_rebuild::{ChunkBuildTimings, ChunkRebuild};
use super::frame::SegmentFrame;
use super::render_cache::{build_chunk_render_data, ChunkRenderCache, ChunkRenderData};
use super::segment_adjacency::{adjacency_tolerance, SegmentAdjacency};
use super::voxel::Voxel;
use crate::collision::ray_triangle::RayHit;
use crate::collision::{Triangle, AABB};
use crate::rendering::vertex::Vertex;

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
    /// Wall clock of building every dirty chunk's replacement, in parallel.
    pub build: Duration,
    /// CPU time of that build summed over chunks, split by phase. More than
    /// `build` whenever chunks ran on several threads.
    pub build_cpu: ChunkBuildTimings,
    /// Wall clock of installing the rebuilt chunks, excluding `adjacency`:
    /// mesh swaps, render cache entries, the vacant-chunk prune and stats.
    pub commit: Duration,
    /// Patching the segment's adjacency map, split by phase.
    pub adjacency: AdjacencyTimings,
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
    adjacency: SegmentAdjacency,

    /// Each chunk's render geometry in world space, so a terrain edit only
    /// rebuilds what it dirtied. Kept current by [`Self::remesh_chunk`].
    render_cache: ChunkRenderCache,

    /// Named local frames within this segment.
    anchors: Vec<Anchor>,

    /// Chunks remeshed by the most recent [`Self::update`]; empty after one
    /// with nothing to do.
    last_rebuilt: Vec<ChunkCoord>,

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

        let adjacency = SegmentAdjacency::new(grid.voxel_size());
        Self {
            name: name.into(),
            frame,
            grid,
            adjacency,
            render_cache: ChunkRenderCache::new(),
            anchors,
            last_rebuilt: Vec::new(),
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

    /// Detonate a charge at a world-space point. Returns the world-space box
    /// the surface can have moved within, or `None` if nothing was carved.
    ///
    /// How far the cut reaches is the charge's budget against what it is digging
    /// through — see [`blast::effective_radius`]. The radius is resolved across
    /// the whole grid before anything is carved, so a blast on a chunk boundary
    /// spends one budget rather than one per chunk.
    pub fn detonate(&mut self, center: Point3<f32>, config: &BlastConfig) -> Option<AABB> {
        let local_center = self.frame.to_local(center);
        let voxel_size = self.grid.voxel_size();

        let radius = blast::effective_radius(&self.grid, local_center, config)?;

        // A changed voxel affects the marching-cubes cells on both sides of its
        // sample, so the neighbouring chunk across a seam has to remesh too.
        let reach = radius + voxel_size;
        let affected = AABB::from_center_half_extents(local_center, Vector3::repeat(reach));

        let coords: Vec<ChunkCoord> = self.grid.coords_in(&affected).collect();
        let mut any_destroyed = false;
        for coord in &coords {
            if let Some(chunk) = self.grid.chunk_mut(*coord) {
                any_destroyed |= chunk.carve_sphere(local_center, radius);
            }
        }

        if !any_destroyed {
            return None;
        }
        for coord in &coords {
            if let Some(chunk) = self.grid.chunk_mut(*coord) {
                chunk.mark_dirty();
            }
        }
        // The carve moves samples out to a voxel past the radius (a density is
        // a clamped distance), and the surface moves in every cell such a
        // sample is a corner of: one voxel further again.
        let surface_reach = radius + 2.0 * voxel_size;
        let surface = AABB::from_center_half_extents(local_center, Vector3::repeat(surface_reach));
        Some(self.frame.aabb_to_world(&surface))
    }

    /// Remesh the chunks marked dirty, appending their world bounds to
    /// `rebuilt`. Returns `None` if there was nothing to do.
    pub fn update(&mut self, rebuilt: &mut Vec<AABB>) -> Option<SegmentTimings> {
        self.last_rebuilt.clear();
        let dirty: Vec<ChunkCoord> = self
            .grid
            .coords()
            .into_iter()
            .filter(|c| self.grid.chunk(*c).is_some_and(|chunk| chunk.is_dirty()))
            .collect();

        if dirty.is_empty() {
            return None;
        }

        let t_build = Instant::now();
        let (grid, frame) = (&self.grid, &self.frame);
        let tolerance = adjacency_tolerance(grid.voxel_size());
        let rebuilds: Vec<ChunkRebuild> = dirty
            .par_iter()
            .map(|&coord| ChunkRebuild::build(grid, frame, coord, tolerance))
            .collect();
        let build = t_build.elapsed();

        let t_commit = Instant::now();
        let mut adjacency = AdjacencyTimings::default();
        let mut build_cpu = ChunkBuildTimings::default();
        for rebuild in rebuilds {
            build_cpu.add(&rebuild.timings);
            rebuilt.push(
                self.frame
                    .aabb_to_world(&self.grid.chunk_bounds(rebuild.coord)),
            );
            self.last_rebuilt.push(rebuild.coord);
            adjacency.add(&self.commit_chunk(rebuild));
        }

        // A chunk allocated purely to own a seam cell may have produced nothing.
        self.grid.prune_vacant();
        let grid = &self.grid;
        self.render_cache
            .retain_coords(|coord| grid.chunk(coord).is_some());
        self.refresh_stats();

        Some(SegmentTimings {
            chunks_dirtied: dirty.len(),
            build,
            build_cpu,
            commit: t_commit.elapsed().saturating_sub(adjacency.total()),
            adjacency,
        })
    }

    /// Install one rebuilt chunk: swap in its adjacency and relink its seams,
    /// swap in its mesh and cache its render data. Chunks must be committed one
    /// at a time, since they share seam links and the grid's chunk table.
    fn commit_chunk(&mut self, rebuild: ChunkRebuild) -> AdjacencyTimings {
        let adjacency = self
            .adjacency
            .replace_chunk(rebuild.coord, rebuild.adjacency);
        self.grid
            .chunk_or_insert(rebuild.coord)
            .replace_mesh(rebuild.mesh);
        self.render_cache.insert(rebuild.coord, rebuild.render_data);
        adjacency
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

    // === Per-chunk geometry ===

    /// Every allocated chunk, in lattice order.
    pub fn chunk_coords(&self) -> Vec<ChunkCoord> {
        self.grid.coords()
    }

    /// Chunks remeshed by the most recent [`Self::update`].
    pub fn last_rebuilt(&self) -> &[ChunkCoord] {
        &self.last_rebuilt
    }

    /// A chunk's world-space bounds.
    pub fn chunk_world_bounds(&self, coord: ChunkCoord) -> AABB {
        self.frame.aabb_to_world(&self.grid.chunk_bounds(coord))
    }

    /// A chunk's triangles in world space, or `None` if it is not allocated.
    ///
    /// Borrowed from the render cache when the chunk has been meshed, which is
    /// every chunk after the first update.
    pub fn chunk_geometry(&self, coord: ChunkCoord) -> Option<Cow<'_, ChunkRenderData>> {
        let chunk = self.grid.chunk(coord)?;
        Some(match self.render_cache.get(coord) {
            Some(cached) => Cow::Borrowed(cached),
            None => Cow::Owned(build_chunk_render_data(chunk.mesh(), &self.frame)),
        })
    }

    /// A chunk's triangles overlapping a world-space box, in world space.
    pub fn chunk_triangles_in(&self, coord: ChunkCoord, world: &AABB) -> Vec<Triangle> {
        let Some(chunk) = self.grid.chunk(coord) else {
            return Vec::new();
        };
        let local = self.frame.aabb_to_local(world);
        chunk
            .mesh()
            .query_aabb(&local)
            .into_iter()
            .map(|(_, t)| {
                Triangle::new(
                    self.frame.to_world(t.v0),
                    self.frame.to_world(t.v1),
                    self.frame.to_world(t.v2),
                )
            })
            .collect()
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
                    built = build_chunk_render_data(chunk.mesh(), &self.frame);
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

    /// Edges not shared by exactly two triangles, lifted into world space.
    ///
    /// The count these explain is `open_edge_count`, but each one carries a
    /// position and a triangle count, so a hole can be told from a
    /// self-intersecting seam and either can be looked at.
    pub fn defective_edges(&self) -> Vec<DefectiveEdge> {
        self.adjacency
            .defective_edges()
            .into_iter()
            .map(|mut edge| {
                edge.from = self.frame.to_world(edge.from);
                edge.to = self.frame.to_world(edge.to);
                edge
            })
            .collect()
    }

    /// Triangle links within this segment.
    pub fn adjacency(&self) -> &SegmentAdjacency {
        &self.adjacency
    }

    #[cfg(test)]
    pub fn set_voxel(&mut self, world: Point3<f32>, voxel: Voxel) {
        self.grid.set(self.frame.to_local(world), voxel);
        self.grid.allocate_seam_neighbours();
    }
}

/// An AABB spanning two points in any order.
fn span(a: Point3<f32>, b: Point3<f32>) -> AABB {
    AABB::new(
        Point3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)),
        Point3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)),
    )
}
