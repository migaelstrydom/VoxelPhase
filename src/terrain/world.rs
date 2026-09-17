//! The engine-facing view of a level's terrain: N independently placed segments.
//!
//! ```text
//!              ┌─────────────────────────────────────┐
//!              │            TerrainWorld             │
//!              │  StaticGeometry + ProbeTarget       │
//!              │  segment broadphase + dispatch      │
//!              │  concatenated render buffers        │
//!              └──────────────┬──────────────────────┘
//!                             │  world query → per-segment local query
//!              ┌──────────────┼──────────────┐
//!              ▼              ▼              ▼
//!        ┌──────────┐   ┌──────────┐   ┌──────────┐
//!        │ Segment  │   │ Segment  │   │ Segment  │
//!        │ frame    │   │ frame    │   │ frame    │
//!        │ ChunkGrid│   │ ChunkGrid│   │ ChunkGrid│
//!        └──────────┘   └──────────┘   └──────────┘
//! ```
//!
//! **Every method here takes and returns world coordinates**, exactly as the
//! single-grid `TerrainManager` it replaced did. Nothing outside `src/terrain/`
//! needs to know a segment exists: the render system, the water system, the
//! physics bridge and the terrain-anchored spawnables all ask the same
//! world-space questions they always asked, and the fan-out happens here.
//!
//! The broadphase is a linear scan. Segment counts are in the dozens and a
//! spatial index would be premature.

use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;
use std::time::{Duration, Instant};

use super::adjacency::DefectiveEdge;
use super::blast::BlastConfig;
use super::chunk::ChunkTriangleRef;
use super::mesh_octree::MeshBuildTimings;
use super::segment::{ConcatTimings, Segment};
use super::surface;
use crate::collision::ray_triangle::{ray_triangle, RayHit};
use crate::collision::{MeshPatch, PatchTriangle, AABB};
use crate::core::error::EngineResult;
use crate::physics::StaticGeometry;
use crate::rendering::vertex::Vertex;
use crate::resources::textures::{TextureHandle, TextureManager};
use crate::sensing::{ProbeHit, ProbeTarget};

/// Headroom left in the concatenated render buffers, as a fraction (1/N) of the
/// level's current size, so that terrain edits do not reallocate them.
const RENDER_BUFFER_SLACK_DIVISOR: usize = 8;

/// Make room for `needed` items, overshooting only when the buffer is actually
/// too small.
///
/// The guard matters as much as the slack: asking for `needed + slack` every
/// time would reallocate on every call, since the request keeps creeping past
/// whatever capacity the previous one settled on.
fn reserve_with_slack<T>(buffer: &mut Vec<T>, needed: usize) {
    debug_assert!(buffer.is_empty(), "capacity math assumes a cleared buffer");
    if buffer.capacity() < needed {
        buffer.reserve(needed + needed / RENDER_BUFFER_SLACK_DIVISOR);
    }
}

/// Voxel size reported for a level with no segments, so that callers using it
/// as a step size still get a usable number.
const DEFAULT_VOXEL_SIZE: f32 = 1.0;

/// Axis-aligned neighbour offsets for the 6-neighbour voxel check.
const NEIGHBOR_OFFSETS: [Vector3<f32>; 6] = [
    Vector3::new(1.0, 0.0, 0.0),
    Vector3::new(-1.0, 0.0, 0.0),
    Vector3::new(0.0, 1.0, 0.0),
    Vector3::new(0.0, -1.0, 0.0),
    Vector3::new(0.0, 0.0, 1.0),
    Vector3::new(0.0, 0.0, -1.0),
];

/// Wall-clock breakdown of one `TerrainWorld::update()` that did work.
///
/// The three phases scale differently, which is the whole reason they are timed
/// apart: `remesh` and `adjacency` are O(chunks dirtied) and roughly constant as
/// a level grows, whereas `concat` rebuilds every vertex in the level and is
/// therefore O(total triangles).
#[derive(Debug, Clone, Copy, Default)]
pub struct UpdateTimings {
    /// How many chunks were remeshed, across all segments.
    pub chunks_dirtied: usize,
    /// Marching cubes over the dirty chunks, plus the vacant-chunk prune.
    /// Excludes the adjacency patching measured separately.
    pub remesh: Duration,
    /// Mesh construction within `remesh`, split by phase. A sub-breakdown of
    /// `remesh`, so these do not add to the total separately.
    pub build: MeshBuildTimings,
    /// Listing old and new triangles for the adjacency diff, within `remesh`.
    pub collect: Duration,
    /// Incremental adjacency patching for the remeshed triangles.
    pub adjacency: Duration,
    /// Rebuilding the concatenated render vertex/index buffers.
    pub concat: Duration,
    /// Sub-breakdown of `concat`, which these do not add to separately.
    pub concat_split: ConcatTimings,
    /// Total triangles in the level after the update.
    pub triangles: usize,
}

impl UpdateTimings {
    /// Sum of the measured phases.
    pub fn total(&self) -> Duration {
        self.remesh + self.adjacency + self.concat
    }
}

/// A level's terrain: every placed segment, and the queries that span them.
pub struct TerrainWorld {
    /// Placed segments, in declaration order. The first is the placement root.
    segments: Vec<Segment>,

    /// World AABBs of the chunks rebuilt in the most recent `update()` call.
    /// Downstream systems (e.g. WaterSystem) read these to detect terrain changes.
    rebuilt_regions: Vec<AABB>,

    /// Render buffers, concatenated across every segment and already in world
    /// space. Per-segment draw calls are the eventual shape; see `PROGRESS.md`
    /// for the measurement that decides when.
    render_vertices: Vec<Vertex>,
    render_indices: Vec<u32>,

    /// Optional noise texture for terrain surface variation.
    texture: Option<TextureHandle>,

    /// World bounds spanning every segment, refreshed on each update.
    bounds: AABB,

    /// Timing breakdown of the most recent `update()` that had work to do.
    /// Retained across idle frames so it can still be read after the event.
    last_update: Option<UpdateTimings>,
}

impl TerrainWorld {
    /// Build a terrain world from already-placed segments, meshing them and
    /// generating the procedural noise texture used for surface variation.
    pub fn from_segments(
        segments: Vec<Segment>,
        texture_manager: &TextureManager,
    ) -> EngineResult<Self> {
        let texture = surface::create_surface_texture(texture_manager)?;
        log::info!("Generated terrain surface texture");

        let mut world = Self::unmeshed(segments);
        world.texture = Some(texture);

        let t0 = Instant::now();
        world.update();
        log::info!(
            "initial mesh build: {:?}, {} segments, {} chunks, {} triangles",
            t0.elapsed(),
            world.segments.len(),
            world.chunk_count(),
            world.triangle_count(),
        );

        Ok(world)
    }

    /// Build and mesh a terrain world with no texture, for tools that have no
    /// Vulkan device — `level_check` and offline analysis.
    pub fn from_segments_headless(segments: Vec<Segment>) -> Self {
        let mut world = Self::unmeshed(segments);
        world.update();
        world
    }

    fn unmeshed(segments: Vec<Segment>) -> Self {
        let bounds = union_bounds(&segments);
        Self {
            segments,
            rebuilt_regions: Vec::new(),
            render_vertices: Vec::new(),
            render_indices: Vec::new(),
            texture: None,
            bounds,
            last_update: None,
        }
    }

    // === Segments ===

    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    /// Find a segment by name.
    pub fn segment(&self, name: &str) -> Option<&Segment> {
        self.segments.iter().find(|s| s.name() == name)
    }

    /// Segments whose world bounds intersect a query box.
    fn segments_in<'a>(&'a self, world: &'a AABB) -> impl Iterator<Item = &'a Segment> + 'a {
        self.segments
            .iter()
            .filter(move |s| s.bounds().intersects(world))
    }

    // === Modification ===

    /// Detonate a charge at a world-space point.
    ///
    /// Routed to every segment within the charge's maximum reach, so an
    /// explosion straddling a join affects both sides. Each segment resolves the
    /// budget against its own material, which is what lets a charge cut deep
    /// into one segment's sand and barely mark the granite next to it.
    pub fn detonate(&mut self, center: Point3<f32>, config: &BlastConfig) {
        for segment in &mut self.segments {
            if segment
                .bounds()
                .intersects_sphere(center, config.max_radius)
            {
                segment.detonate(center, config);
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

        let mut chunks_dirtied = 0;
        let mut remesh = Duration::ZERO;
        let mut adjacency = Duration::ZERO;
        let mut build = MeshBuildTimings::default();
        let mut collect = Duration::ZERO;

        let mut rebuilt = Vec::new();
        for segment in &mut self.segments {
            if let Some(t) = segment.update(&mut rebuilt) {
                chunks_dirtied += t.chunks_dirtied;
                remesh += t.remesh;
                adjacency += t.adjacency;
                collect += t.collect;
                build.add(&t.build);
            }
        }
        self.rebuilt_regions = rebuilt;

        if chunks_dirtied == 0 {
            return;
        }

        let t1 = Instant::now();
        let concat_split = self.refresh_caches();
        let concat = t1.elapsed();

        let timings = UpdateTimings {
            chunks_dirtied,
            remesh,
            build,
            collect,
            adjacency,
            concat_split,
            concat,
            triangles: self.triangle_count(),
        };
        self.last_update = Some(timings);

        log::debug!(
            "Terrain updated: {} chunks remeshed across {} segments, {} triangles, {:?} remesh, {:?} adjacency, {:?} render data, {:?} total",
            timings.chunks_dirtied,
            self.segments.len(),
            timings.triangles,
            remesh,
            adjacency,
            concat,
            timings.total(),
        );
    }

    /// Timing breakdown of the most recent `update()` that had work to do.
    ///
    /// `None` until terrain has been meshed at least once. Retained rather than
    /// cleared on idle frames, so the numbers survive long enough to be read.
    pub fn last_update_timings(&self) -> Option<UpdateTimings> {
        self.last_update
    }

    /// Rebuild the concatenated render buffers and the level bounds.
    fn refresh_caches(&mut self) -> ConcatTimings {
        let mut timings = ConcatTimings::default();
        self.render_vertices.clear();
        self.render_indices.clear();

        // Sized up front, with slack, from the stats each segment refreshed
        // while remeshing.
        //
        // The slack is the point. Reserving exactly leaves capacity equal to the
        // level's vertex count, and destruction *adds* geometry — so the first
        // edit after load overflows by a hair and pays a grow-and-copy of a
        // tens-of-megabytes buffer into freshly mapped pages. That lands on the
        // first grenade the player throws, which is exactly when it is most
        // visible. Slack moves it to load time, where nobody is watching.
        let vertices: usize = self.segments.iter().map(Segment::vertex_count).sum();
        let triangles: usize = self.segments.iter().map(Segment::triangle_count).sum();
        reserve_with_slack(&mut self.render_vertices, vertices);
        reserve_with_slack(&mut self.render_indices, triangles * 3);

        for segment in &self.segments {
            let t = segment.append_render_data(&mut self.render_vertices, &mut self.render_indices);
            timings.add(&t);
        }
        self.bounds = union_bounds(&self.segments);
        timings
    }

    // === Bounds and resolution ===

    /// World bounds spanning every segment.
    ///
    /// Derived from allocated chunks, so an unfilled region of a segment's
    /// declared extent does not inflate them.
    pub fn bounds(&self) -> &AABB {
        &self.bounds
    }

    /// The finest voxel size in the level.
    ///
    /// Segments each carry their own resolution, so there is no single answer;
    /// the finest is the conservative one for anything that uses it as a step or
    /// a tolerance, which is every caller.
    pub fn voxel_size(&self) -> f32 {
        self.segments
            .iter()
            .map(Segment::voxel_size)
            .reduce(f32::min)
            .unwrap_or(DEFAULT_VOXEL_SIZE)
    }

    /// Voxel size of the segment containing a world position.
    ///
    /// Falls back to the world's finest resolution where no segment claims the
    /// point, which is the conservative answer: a smaller step never merges two
    /// cells into one.
    ///
    /// Anything reasoning about where a *surface* sits relative to its authored
    /// height wants this rather than [`Self::voxel_size`]: both the meshing
    /// tolerances and the surface band are fractions of the local resolution,
    /// and the world's finest is the wrong scale everywhere but the finest
    /// segment.
    pub fn voxel_size_at(&self, world: Point3<f32>) -> f32 {
        self.segments
            .iter()
            .filter(|s| s.bounds().contains_point(world))
            .map(Segment::voxel_size)
            .reduce(f32::max)
            .unwrap_or_else(|| self.voxel_size())
    }

    // === Voxel queries ===

    /// Whether the voxel at a world-space position is solid (density > 0).
    pub fn is_solid_at(&self, x: f32, y: f32, z: f32) -> bool {
        self.density_at(Point3::new(x, y, z)) > 0.0
    }

    /// Highest voxel density any segment reports at a world position.
    ///
    /// Rule 4 forbids segments contending for space, so at most one can answer
    /// solid — but the *search* does not rely on that, only the uniqueness of
    /// the answer does.
    fn density_at(&self, world: Point3<f32>) -> f32 {
        self.segments
            .iter()
            .filter(|s| s.bounds().contains_point(world))
            .map(|s| s.voxel_at(world).density)
            .fold(f32::NEG_INFINITY, f32::max)
    }

    /// Highest solid terrain surface height at a given (x, z) position.
    pub fn approx_surface_height_at(&self, x: f32, z: f32) -> Option<f32> {
        self.surface_heights_at(x, z).into_iter().next()
    }

    /// All solid surface heights in a column, sorted top-to-bottom.
    ///
    /// A surface is detected where density transitions from positive (solid,
    /// below) to negative (air, above). Segments are walked at their own voxel
    /// resolution and the results merged.
    pub fn surface_heights_at(&self, x: f32, z: f32) -> Vec<f32> {
        let mut heights: Vec<f32> = self
            .segments
            .iter()
            .flat_map(|s| s.surface_heights_at(x, z))
            .collect();
        heights.sort_by(|a, b| b.total_cmp(a));
        heights
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
    /// Uses a fast-path voxel neighbourhood check: if the point's voxel and all
    /// 6 axis-aligned neighbours agree (all solid or all air), the answer is
    /// immediate. Only near the surface — where marching cubes interpolation
    /// differs from the voxel grid — does this fall back to a ray parity test
    /// against the actual triangle mesh.
    ///
    /// The neighbourhood is stepped at the resolution of the segment the point
    /// is *in*, not the world's finest. `density_at` is a voxel lookup and so is
    /// constant across a cell: stepping a 1 m segment by a 0.5 m neighbour from
    /// another segment can land back in the same cell, agree with itself, and
    /// take the fast path within half a voxel of the surface — reporting solid
    /// up to half a voxel above ground that the mesh puts exactly where it was
    /// authored.
    pub fn is_mesh_solid_at(&self, x: f32, y: f32, z: f32) -> bool {
        let pos = Point3::new(x, y, z);
        let center_solid = self.density_at(pos) > 0.0;

        let vs = self.voxel_size_at(pos);
        let all_agree = NEIGHBOR_OFFSETS.iter().all(|offset| {
            let neighbor = pos + offset * vs;
            (self.density_at(neighbor) > 0.0) == center_solid
        });

        if all_agree {
            return center_solid;
        }

        // Near the surface — use ray parity test (cast +Y, count crossings).
        // Segment surfaces are closed and disjoint, so the parity of the total
        // crossing count is still the right answer across several of them.
        let ray_length = self.bounds.max.y - y + vs;
        if ray_length <= 0.0 {
            return false;
        }

        let hits = self.ray_cast_all(pos, Vector3::new(0.0, 1.0, 0.0), ray_length);
        hits.len() % 2 == 1
    }

    /// Highest mesh surface height at a given (x, z) position.
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

    /// All mesh surface heights in a column, sorted top-to-bottom.
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
        let vs = self.voxel_size();
        let origin = Point3::new(x, self.bounds.max.y + vs, z);
        let length = (self.bounds.max.y - self.bounds.min.y) + 2.0 * vs;
        (origin, length)
    }

    /// Cast a ray against every segment it passes through, in world space.
    fn ray_cast_all(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Vec<RayHit> {
        let end = origin + direction * length;
        let sweep = AABB::new(
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

        self.segments_in(&sweep)
            .flat_map(|s| s.ray_cast_all(origin, direction, length))
            .collect()
    }

    // === Rendering data ===

    /// Check if there's any geometry to render.
    pub fn has_geometry(&self) -> bool {
        !self.render_vertices.is_empty()
    }

    /// Get render vertices, in world space.
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
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        for segment in &self.segments {
            segment.append_render_data_culled(frustum_planes, &mut vertices, &mut indices);
        }
        (vertices, indices)
    }

    // === Statistics ===

    /// Number of mesh edges belonging to exactly one triangle.
    ///
    /// A closed surface has none. Terrain is closed wherever it is meshed, so a
    /// non-zero count means either a crack — including one at a chunk seam — or
    /// an ambiguous marching-cubes configuration that failed to close.
    pub fn open_edge_count(&self) -> usize {
        self.segments.iter().map(Segment::open_edge_count).sum()
    }

    /// Every defective edge in the world, tagged with the segment it came from.
    ///
    /// The counterpart to `open_edge_count`: the same defects, located.
    pub fn defective_edges(&self) -> Vec<(&str, DefectiveEdge)> {
        self.segments
            .iter()
            .flat_map(|segment| {
                segment
                    .defective_edges()
                    .into_iter()
                    .map(move |edge| (segment.name(), edge))
            })
            .collect()
    }

    pub fn triangle_count(&self) -> usize {
        self.segments.iter().map(Segment::triangle_count).sum()
    }

    pub fn leaf_count(&self) -> usize {
        self.segments.iter().map(Segment::leaf_count).sum()
    }

    pub fn chunk_count(&self) -> usize {
        self.segments.iter().map(Segment::chunk_count).sum()
    }

    /// Number of allocated chunks holding at least one solid voxel.
    ///
    /// The rest exist only to own the marching-cubes cells that close their
    /// neighbours' minimum faces, so `chunk_count()` on its own overstates how
    /// much content a level has — see `ChunkGrid::allocate_seam_neighbours`.
    pub fn solid_chunk_count(&self) -> usize {
        self.segments.iter().map(Segment::solid_chunk_count).sum()
    }
}

/// World bounds spanning every segment, or a degenerate box at the origin.
fn union_bounds(segments: &[Segment]) -> AABB {
    let mut iter = segments.iter().filter(|s| s.chunk_count() > 0);
    let Some(first) = iter.next() else {
        return AABB::new(Point3::origin(), Point3::origin());
    };
    iter.fold(*first.bounds(), |acc, s| {
        let b = s.bounds();
        AABB::new(
            Point3::new(
                acc.min.x.min(b.min.x),
                acc.min.y.min(b.min.y),
                acc.min.z.min(b.min.z),
            ),
            Point3::new(
                acc.max.x.max(b.max.x),
                acc.max.y.max(b.max.y),
                acc.max.z.max(b.max.z),
            ),
        )
    })
}

/// Largest span, in metres, a collision query may ask for along any axis.
///
/// Every caller of [`TerrainWorld::query_region`] asks for the region around
/// one collider, which is metres across at most even when swept. A query far
/// larger than that does not mean a large object, it means a body whose state
/// has gone wrong — and answering it honestly means gathering every triangle
/// in the level, with its adjacency, every frame. That is how a single runaway
/// piece of debris took a machine down rather than merely looking silly: the
/// physics bug was upstream, but this is where the cost landed.
const MAX_QUERY_SPAN: f32 = 256.0;

/// Whether a query region is one a collider could plausibly have asked for.
fn is_sane_query(aabb: &AABB) -> bool {
    let finite = aabb
        .min
        .coords
        .iter()
        .chain(aabb.max.coords.iter())
        .all(|v| v.is_finite());
    finite && (aabb.max - aabb.min).amax() <= MAX_QUERY_SPAN
}

impl StaticGeometry for TerrainWorld {
    fn query_region(&self, aabb: &AABB) -> MeshPatch {
        // A body that has run away asks for the whole level. Refusing costs it
        // its contacts, which for a body already in that state is no loss, and
        // keeps one broken body from being able to exhaust memory.
        if !is_sane_query(aabb) {
            log::error!(
                "Terrain: refusing a collision query spanning {:?} — from {:?} to {:?}",
                aabb.max - aabb.min,
                aabb.min,
                aabb.max,
            );
            return MeshPatch {
                triangles: Vec::new(),
            };
        }

        // A triangle is identified across the level by its segment plus its
        // in-segment reference; adjacency is per-segment, so both are needed to
        // resolve a neighbour to an index within this patch.
        let mut results: Vec<(usize, ChunkTriangleRef, crate::collision::Triangle)> = Vec::new();
        for (index, segment) in self.segments.iter().enumerate() {
            if !segment.bounds().intersects(aabb) {
                continue;
            }
            results.extend(
                segment
                    .query_region(aabb)
                    .into_iter()
                    .map(|(tri_ref, triangle)| (index, tri_ref, triangle)),
            );
        }

        let ref_to_index: FxHashMap<(usize, ChunkTriangleRef), u32> = results
            .iter()
            .enumerate()
            .map(|(i, (seg, tri_ref, _))| ((*seg, *tri_ref), i as u32))
            .collect();

        let triangles = results
            .iter()
            .map(|(seg, tri_ref, triangle)| {
                let neighbors = match self.segments[*seg].neighbours(tri_ref) {
                    Some(nbrs) => std::array::from_fn(|edge| {
                        nbrs[edge].and_then(|nbr| ref_to_index.get(&(*seg, nbr)).copied())
                    }),
                    None => [None; 3],
                };
                PatchTriangle {
                    triangle: *triangle,
                    neighbors,
                }
            })
            .collect();

        MeshPatch { triangles }
    }

    /// Adjacency-free variant: collects triangles straight from the segments,
    /// skipping the `ref_to_index` map and per-triangle neighbour resolution
    /// that `query_region` needs.
    fn query_region_triangles(&self, aabb: &AABB) -> Vec<crate::collision::Triangle> {
        let mut triangles = Vec::new();
        for segment in self.segments.iter() {
            if !segment.bounds().intersects(aabb) {
                continue;
            }
            triangles.extend(
                segment
                    .query_region(aabb)
                    .into_iter()
                    .map(|(_, triangle)| triangle),
            );
        }
        triangles
    }
}

impl ProbeTarget for TerrainWorld {
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
                if earliest.as_ref().is_none_or(|e: &RayHit| hit.t < e.t) {
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
    use crate::level::{TerrainFeature, VolumeFeature};
    use crate::terrain::chunk::CHUNK_VOXELS;
    use crate::terrain::chunk_grid::ChunkGrid;
    use crate::terrain::csg::union_solid;
    use crate::terrain::frame::SegmentFrame;
    use crate::terrain::voxel::{Voxel, VoxelMaterial};
    use crate::terrain::Anchor;
    use crate::terrain::BlastConfig;

    /// A solid slab from `min` to `max` inclusive, in **segment-local**
    /// coordinates. Authoring locally is the point: the same grid contents get
    /// placed at several frames, which is what relocatability means.
    /// Signed distance to an axis-aligned box, negative inside it.
    fn box_sdf(p: Point3<f32>, min: Point3<f32>, max: Point3<f32>) -> f32 {
        let centre = nalgebra::center(&min, &max);
        let half = (max - min) * 0.5;
        let q = (p - centre).abs() - half;
        q.map(|c| c.max(0.0)).magnitude() + q.max().min(0.0)
    }

    fn slab_grid(voxel_size: f32, min: Point3<f32>, max: Point3<f32>) -> ChunkGrid {
        let mut grid = ChunkGrid::new(voxel_size);

        // Written through `union_solid`, at its signed distance, and sampled one
        // voxel beyond every face so the samples just outside the slab carry
        // their distance to it too.
        //
        // Filling whole solid voxels instead is the mistake the destruction path
        // used to make in the other direction: a field of saturated densities
        // holds no sub-voxel surface position, and both marching cubes and the
        // occlusion bake read that position. A slab authored that way meshes
        // half a voxel out and bakes as though its flat top were partly
        // enclosed, which is why the occlusion assertions against it used to
        // need such loose floors.
        let mut x = min.x - voxel_size;
        while x <= max.x + voxel_size {
            let mut y = min.y - voxel_size;
            while y <= max.y + voxel_size {
                let mut z = min.z - voxel_size;
                while z <= max.z + voxel_size {
                    let p = Point3::new(x, y, z);
                    union_solid(
                        &mut grid,
                        p,
                        box_sdf(p, min, max),
                        voxel_size,
                        VoxelMaterial::Rock,
                    );
                    z += voxel_size;
                }
                y += voxel_size;
            }
            x += voxel_size;
        }
        grid
    }

    fn world_of(frame: SegmentFrame, grid: ChunkGrid) -> TerrainWorld {
        TerrainWorld::from_segments_headless(vec![Segment::new("test", frame, grid, Vec::new())])
    }

    /// A slab spanning the chunk seam at local x = 32, placed at `frame`.
    fn seam_slab(frame: SegmentFrame) -> TerrainWorld {
        world_of(
            frame,
            slab_grid(
                1.0,
                Point3::new(24.0, 0.0, 0.0),
                Point3::new(40.0, 2.0, 8.0),
            ),
        )
    }

    /// The frames every frame-sensitive test is run against: identity, a pure
    /// translation, and each quarter turn about a non-zero origin.
    fn frames() -> Vec<(&'static str, SegmentFrame)> {
        let o = Point3::new(-137.0, -8.5, 211.0);
        vec![
            ("identity", SegmentFrame::identity()),
            ("translated", SegmentFrame::new(o, 0)),
            ("yaw 90", SegmentFrame::new(o, 1)),
            ("yaw 180", SegmentFrame::new(o, 2)),
            ("yaw 270", SegmentFrame::new(o, 3)),
        ]
    }

    /// A coarse segment's solidity must not be reported at the resolution of a
    /// finer segment somewhere else in the level.
    ///
    /// `density_at` is a voxel lookup and so is constant across a cell. Stepping
    /// the 6-neighbour probe by the world's *finest* voxel size lands back in
    /// the same cell of a coarser segment, the neighbourhood agrees with itself,
    /// and the fast path answers at voxel granularity — reporting solid up to
    /// half a voxel above ground the mesh puts exactly where it was authored.
    /// Everything that asks "is this point in rock" then reads high, which shows
    /// up as objects authored to rest on a bench being rejected as buried.
    #[test]
    fn solidity_is_probed_at_the_local_segments_resolution() {
        let coarse = Segment::new(
            "coarse",
            SegmentFrame::identity(),
            slab_grid(1.0, Point3::new(0.0, 0.0, 0.0), Point3::new(8.0, 4.0, 8.0)),
            Vec::new(),
        );
        // Only here to drag the world's finest resolution below the coarse
        // segment's. Placed well clear of it.
        let fine = Segment::new(
            "fine",
            SegmentFrame::new(Point3::new(100.0, 0.0, 0.0), 0),
            slab_grid(0.5, Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 2.0, 4.0)),
            Vec::new(),
        );
        let world = TerrainWorld::from_segments_headless(vec![coarse, fine]);
        assert_eq!(world.voxel_size(), 0.5, "the fine segment sets the floor");

        let top = world
            .mesh_surface_heights_at(4.0, 4.0)
            .first()
            .copied()
            .expect("the coarse slab has a top surface");
        assert!(
            (top - 4.0).abs() < 0.05,
            "the mesh puts the slab top at its authored height, not {top}"
        );

        assert!(
            world.is_mesh_solid_at(4.0, top - 0.5, 4.0),
            "inside the slab"
        );
        assert!(
            !world.is_mesh_solid_at(4.0, top + 0.2, 4.0),
            "0.2 m above a surface at {top} is air, not rock"
        );
    }

    #[test]
    fn generates_geometry_from_voxels() {
        let world = world_of(
            SegmentFrame::identity(),
            slab_grid(
                1.0,
                Point3::new(-5.0, 0.0, -5.0),
                Point3::new(5.0, 0.0, 5.0),
            ),
        );
        assert!(world.has_geometry());
        assert!(world.triangle_count() > 0);
    }

    /// A slab far smaller than any world extent must allocate only the chunks it
    /// touches, plus the seam neighbours that own its minimum-face cells.
    #[test]
    fn chunk_allocation_is_proportional_to_content() {
        let world = world_of(
            SegmentFrame::identity(),
            slab_grid(1.0, Point3::new(1.0, 1.0, 1.0), Point3::new(4.0, 2.0, 4.0)),
        );
        // One chunk holds the slab; the seven negative-octant neighbours own the
        // cells that close its underside and minimum faces. Four of those turn
        // out empty and are pruned.
        assert!(
            world.chunk_count() <= 8,
            "a 4x2x4 voxel feature allocated {} chunks",
            world.chunk_count()
        );
        assert!(world.chunk_count() >= 1);
    }

    /// The declared voxel size is the voxel size that gets used — a regression
    /// test for the old `log2(world_size / voxel_size)` derivation, which
    /// produced voxels twice the declared size.
    #[test]
    fn voxel_size_is_as_declared() {
        for voxel_size in [0.25_f32, 0.5, 1.0, 2.0] {
            let half = 0.5 * voxel_size;
            let mut grid = ChunkGrid::new(voxel_size);
            grid.set(
                Point3::new(half, half, half),
                Voxel::solid(VoxelMaterial::Rock),
            );
            let world = world_of(SegmentFrame::identity(), grid);

            assert_eq!(world.voxel_size(), voxel_size);
            assert!(world.is_solid_at(half, half, half));
            assert!(
                !world.is_solid_at(half + voxel_size, half, half),
                "voxel at size {voxel_size} bled into its neighbour"
            );
        }
    }

    /// Terrain must not step or crack where it crosses a chunk seam. A flat
    /// slab spanning the boundary at local x = 32 should read the same height on
    /// both sides — at any frame, since a rigid transform cannot introduce a
    /// step.
    #[test]
    fn surface_height_is_continuous_across_a_chunk_seam() {
        for (label, frame) in frames() {
            let world = seam_slab(frame);
            let mut heights = Vec::new();
            for x in [28.5_f32, 30.5, 31.5, 32.5, 33.5, 35.5] {
                let p = frame.to_world(Point3::new(x, 0.0, 4.5));
                let h = world
                    .mesh_surface_height_at(p.x, p.z)
                    .unwrap_or_else(|| panic!("{label}: no surface at local x={x}"));
                heights.push(h);
            }

            let min = heights.iter().copied().fold(f32::INFINITY, f32::min);
            let max = heights.iter().copied().fold(f32::NEG_INFINITY, f32::max);
            assert!(
                max - min <= 1.0,
                "{label}: surface height jumped across the chunk seam: {heights:?}"
            );
        }
    }

    /// A crack at a chunk seam is an unmatched mesh edge. A solid slab spanning
    /// two chunks is a closed surface, so every edge must be shared by exactly
    /// two triangles — no exceptions anywhere, seam included, at any frame.
    #[test]
    fn mesh_has_no_open_edges_across_a_chunk_seam() {
        for (label, frame) in frames() {
            let world = seam_slab(frame);
            let open = world.open_edge_count();
            assert_eq!(
                open,
                0,
                "{label}: {open} open edges in a {}-triangle closed slab spanning a chunk seam",
                world.triangle_count()
            );
        }
    }

    /// Physics reads terrain through `query_region`. A triangle straddling a
    /// chunk seam must be reported exactly once — duplicates become duplicate
    /// contacts and phantom forces in the solver.
    #[test]
    fn seam_triangles_are_not_duplicated() {
        for (label, frame) in frames() {
            let world = seam_slab(frame);
            let query = frame.aabb_to_world(&AABB::new(
                Point3::new(30.0, -2.0, 2.0),
                Point3::new(34.0, 4.0, 6.0),
            ));
            let patch = world.query_region(&query);
            assert!(
                !patch.triangles.is_empty(),
                "{label}: seam query returned nothing"
            );

            let mut seen = std::collections::HashSet::new();
            for pt in &patch.triangles {
                let key = |p: Point3<f32>| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits());
                let id = (
                    key(pt.triangle.v0),
                    key(pt.triangle.v1),
                    key(pt.triangle.v2),
                );
                assert!(
                    seen.insert(id),
                    "{label}: duplicate triangle across chunk seam"
                );
            }
        }
    }

    /// Adjacency must link triangles that meet across a chunk boundary, or the
    /// physics feature-aware pipeline sees false boundary edges at every seam.
    #[test]
    fn adjacency_links_across_a_chunk_seam() {
        for (label, frame) in frames() {
            let world = seam_slab(frame);
            let query = frame.aabb_to_world(&AABB::new(
                Point3::new(31.0, 1.5, 2.0),
                Point3::new(33.0, 3.0, 6.0),
            ));
            let patch = world.query_region(&query);
            assert!(!patch.triangles.is_empty(), "{label}");

            let linked = patch
                .triangles
                .iter()
                .filter(|t| t.neighbors.iter().any(Option::is_some))
                .count();
            assert!(
                linked > 0,
                "{label}: no triangle in the seam patch has a neighbour ({} triangles)",
                patch.triangles.len()
            );
        }
    }

    /// **The relocatability guarantee.** The same segment definition placed at a
    /// non-zero frame must produce the same terrain, transformed — not merely
    /// similar terrain. This is the property segments exist to provide, so it is
    /// a test rather than an assumption.
    #[test]
    fn a_relocated_segment_matches_the_original_transformed() {
        let at_origin = seam_slab(SegmentFrame::identity());

        for (label, frame) in frames() {
            let placed = seam_slab(frame);

            assert_eq!(
                placed.triangle_count(),
                at_origin.triangle_count(),
                "{label}: triangle count changed"
            );
            assert_eq!(
                placed.chunk_count(),
                at_origin.chunk_count(),
                "{label}: chunk count changed"
            );

            // Surface heights must agree at every corresponding point.
            for x in [25.5_f32, 28.0, 32.0, 36.5, 39.0] {
                for z in [0.5_f32, 3.0, 7.5] {
                    let local = Point3::new(x, 0.0, z);
                    let here = at_origin.mesh_surface_height_at(local.x, local.z);
                    let there_p = frame.to_world(local);
                    let there = placed.mesh_surface_height_at(there_p.x, there_p.z);

                    match (here, there) {
                        (Some(a), Some(b)) => assert!(
                            (a + frame.origin().y - b).abs() < 1e-3,
                            "{label}: surface at local ({x}, {z}) was {a}, is {b}"
                        ),
                        (None, None) => {}
                        _ => panic!("{label}: surface presence differs at local ({x}, {z})"),
                    }
                }
            }

            // And the world bounds must be exactly the transformed bounds.
            let expected = frame.aabb_to_world(at_origin.bounds());
            let actual = placed.bounds();
            assert!(
                (expected.min - actual.min).norm() < 1e-3
                    && (expected.max - actual.max).norm() < 1e-3,
                "{label}: bounds {:?}..{:?}, expected {:?}..{:?}",
                actual.min,
                actual.max,
                expected.min,
                expected.max
            );
        }
    }

    /// `query_region` is the physics interface, so its triangles have to arrive
    /// in world space — not in the segment's local frame.
    #[test]
    fn query_region_returns_world_space_triangles_for_a_placed_segment() {
        let frame = SegmentFrame::new(Point3::new(500.0, 12.0, -300.0), 1);
        let world = seam_slab(frame);

        let query = frame.aabb_to_world(&AABB::new(
            Point3::new(24.0, -1.0, 0.0),
            Point3::new(40.0, 4.0, 8.0),
        ));
        let patch = world.query_region(&query);
        assert!(!patch.triangles.is_empty());

        // Every returned vertex must lie inside the query box, which is far from
        // the origin — local-frame triangles would land near it instead.
        for pt in &patch.triangles {
            for v in [pt.triangle.v0, pt.triangle.v1, pt.triangle.v2] {
                assert!(
                    v.x >= query.min.x - 1.0
                        && v.x <= query.max.x + 1.0
                        && v.z >= query.min.z - 1.0
                        && v.z <= query.max.z + 1.0,
                    "triangle vertex {v:?} is outside the world-space query {:?}..{:?}",
                    query.min,
                    query.max
                );
            }
        }
    }

    /// Render vertices carry the frame too, and so do their normals — a rotated
    /// segment shaded with unrotated normals is lit from the wrong direction.
    #[test]
    fn render_vertices_and_normals_are_in_world_space() {
        let frame = SegmentFrame::new(Point3::new(64.0, 0.0, 0.0), 1);
        let flat = |f| {
            world_of(
                f,
                slab_grid(
                    1.0,
                    Point3::new(2.0, 0.0, 2.0),
                    Point3::new(10.0, 1.0, 10.0),
                ),
            )
        };
        let at_origin = flat(SegmentFrame::identity());
        let placed = flat(frame);

        assert_eq!(
            placed.render_vertices().len(),
            at_origin.render_vertices().len()
        );
        for (a, b) in at_origin
            .render_vertices()
            .iter()
            .zip(placed.render_vertices())
        {
            let expected = frame.to_world(Point3::new(a.pos.x, a.pos.y, a.pos.z));
            assert!((expected.x - b.pos.x).abs() < 1e-3 && (expected.z - b.pos.z).abs() < 1e-3);
            let n = frame.rotate_to_world(a.normal);
            assert!(
                (n - b.normal).norm() < 1e-3,
                "normal {n:?} vs {:?}",
                b.normal
            );
        }
    }

    /// Damage is authored in world space and must reach the right segment even
    /// when that segment is rotated and far from the origin.
    #[test]
    fn damage_removes_geometry_and_reports_dirty_regions() {
        for (label, frame) in frames() {
            let mut world = world_of(
                frame,
                slab_grid(
                    1.0,
                    Point3::new(-5.0, 0.0, -5.0),
                    Point3::new(5.0, 3.0, 5.0),
                ),
            );
            let before = world.triangle_count();

            let centre = frame.to_world(Point3::new(0.0, 2.0, 0.0));
            world.detonate(centre, &BlastConfig::fixed_radius(3.0));
            world.update();

            assert!(!world.dirty_regions().is_empty(), "{label}");
            assert_ne!(world.triangle_count(), before, "{label}");
            assert!(
                !world.is_solid_at(centre.x, centre.y, centre.z),
                "{label}: voxel survived the blast"
            );
        }
    }

    /// Ambient occlusion is baked into the mesh, so destruction has to rebake
    /// it. The failure this guards is a fresh crater lit exactly like the flat
    /// ground it was cut from, which reads as a decal rather than a hollow.
    #[test]
    fn a_crater_darkens_after_damage() {
        let mut world = world_of(
            SegmentFrame::identity(),
            slab_grid(
                0.5,
                Point3::new(-8.0, 0.0, -8.0),
                Point3::new(8.0, 4.0, 8.0),
            ),
        );

        // Only the column over the crater: the slab's own rim is legitimately
        // occluded and would drown the signal being measured. The whole column
        // is solid before the blast, so the only surface in it is the flat top.
        let over_the_crater = |world: &TerrainWorld| {
            world
                .render_vertices()
                .iter()
                .filter(|v| v.pos.x * v.pos.x + v.pos.z * v.pos.z < 4.0 && v.pos.y > 0.5)
                .map(|v| v.ao)
                .fold(f32::MAX, f32::min)
        };

        // Tight, and it is allowed to be: `slab_grid` authors the slab through
        // `union_solid` at its signed distance, so the flat top carries the
        // sub-voxel surface position the occlusion bake reads. This floor was
        // 0.8 while the helper filled whole solid voxels instead.
        let before = over_the_crater(&world);
        assert!(
            before > 0.99,
            "flat ground over the crater site started at {before}, not open"
        );

        world.detonate(Point3::new(0.0, 4.0, 0.0), &BlastConfig::fixed_radius(3.0));
        world.update();

        let after = over_the_crater(&world);
        assert!(
            after < before - 0.05,
            "the crater bakes at {after}, barely darker than the {before} it replaced"
        );
    }

    /// Ground beside a crater is flat, and its normals have to say so.
    ///
    /// The artefact this guards: destruction used to write whole air voxels at
    /// a saturated -1.0, which put a step into the density field horizontally,
    /// beside the crater, on ground that is geometrically flat. Marching cubes
    /// derives its normals from that field's gradient, so it read the step as a
    /// slope: vertices whose heights agreed to within a centimetre carried
    /// normals tilted by up to 36 degrees, and the terrain shader duly shaded
    /// flat ground as though it fell away into the hole.
    ///
    /// Both voxel sizes, because the contaminated band was about two voxels
    /// wide *in voxels* and therefore four times wider in metres at the game's
    /// 1 m voxels than at the visual bench's 0.25 m. That scaling is why the
    /// bench sheets looked clean while the game did not, and why a single-scale
    /// test could have passed with the bug still in.
    #[test]
    fn flat_ground_beside_a_crater_keeps_its_normals() {
        use crate::terrain::generation::generate_terrain;
        use crate::terrain::{ChunkGrid, SegmentFrame};

        const BLAST_RADIUS: f32 = 4.0;

        for voxel_size in [1.0_f32, 0.5, 0.25] {
            let terrain = crate::level::Terrain {
                bedrock_thickness: 0.0,
                voxel_size,
                bounds: crate::level::Extent {
                    min: (-24.0, -12.0, -24.0),
                    max: (24.0, 12.0, 24.0),
                },
                base_height: 0.0,
                material_layers: Vec::new(),
                features: Vec::new(),
                volumes: Vec::new(),
            };
            let mut grid = ChunkGrid::new(voxel_size);
            generate_terrain(&mut grid, &terrain, &terrain.bounds.to_aabb());
            let mut world = world_of(SegmentFrame::identity(), grid);

            let ground_height = world
                .render_vertices()
                .iter()
                .map(|v| v.pos.y)
                .fold(f32::MIN, f32::max);

            world.detonate(
                Point3::new(0.0, 0.0, 0.0),
                &BlastConfig::fixed_radius(BLAST_RADIUS),
            );
            world.update();

            // Undisturbed ground: level with the original surface, and clear of
            // the rim, whose curve is a real slope that a smooth normal should
            // follow. The cut reaches about a voxel past the blast radius.
            let mut worst: Option<&Vertex> = None;
            for v in world.render_vertices() {
                let radius = (v.pos.x * v.pos.x + v.pos.z * v.pos.z).sqrt();
                if (v.pos.y - ground_height).abs() > 0.01
                    || radius < BLAST_RADIUS + 2.0 * voxel_size
                    || radius > 16.0
                {
                    continue;
                }
                if worst.is_none_or(|w| v.normal.y < w.normal.y) {
                    worst = Some(v);
                }
            }

            let worst = worst.expect("no flat ground was found around the crater");
            assert!(
                worst.normal.y > 0.99,
                "at {voxel_size} m voxels, flat ground {:.2} m from a {BLAST_RADIUS} m crater \
                 carries a normal tilted {:.1} degrees off vertical ({:?} at {:?})",
                (worst.pos.x * worst.pos.x + worst.pos.z * worst.pos.z).sqrt(),
                worst.normal.y.acos().to_degrees(),
                worst.normal,
                worst.pos,
            );
        }
    }

    /// Two segments placed side by side answer as one world: each query reaches
    /// whichever segment owns the point, and neither leaks into the other.
    #[test]
    fn queries_fan_out_across_segments() {
        let a = Segment::new(
            "a",
            SegmentFrame::identity(),
            slab_grid(
                1.0,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(16.0, 2.0, 16.0),
            ),
            vec![Anchor::new("east", SegmentFrame::identity())],
        );
        let b = Segment::new(
            "b",
            SegmentFrame::new(Point3::new(40.0, 8.0, 0.0), 1),
            slab_grid(
                1.0,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(16.0, 2.0, 16.0),
            ),
            Vec::new(),
        );
        let world = TerrainWorld::from_segments_headless(vec![a, b]);

        assert_eq!(world.segments().len(), 2);
        assert!(world.segment("b").is_some());
        assert_eq!(world.segment("a").unwrap().anchors().len(), 1);

        // A point in each segment, and one in the gap between them.
        assert!(world.is_solid_at(8.0, 1.0, 8.0), "segment a not reachable");
        let in_b = Point3::new(40.0, 8.0, 0.0) + Vector3::new(8.0, 1.0, -8.0);
        assert!(
            world.is_solid_at(in_b.x, in_b.y, in_b.z),
            "segment b not reachable"
        );
        assert!(!world.is_solid_at(28.0, 1.0, 8.0), "the gap reads as solid");

        // Totals are the sum, and the bounds span both.
        assert_eq!(
            world.triangle_count(),
            world
                .segments()
                .iter()
                .map(|s| s.triangle_count())
                .sum::<usize>()
        );
        assert!(world.bounds().max.x >= 40.0);
    }

    // -----------------------------------------------------------------------
    // Authored surfaces on the voxel lattice
    // -----------------------------------------------------------------------

    /// A terrain description with nothing in it but a flat surface, generated
    /// within a box big enough that its own walls close the mesh.
    fn flat_terrain(voxel_size: f32, base_height: f32) -> crate::level::Terrain {
        crate::level::Terrain {
            bedrock_thickness: 0.0,
            voxel_size,
            bounds: crate::level::Extent {
                min: (0.0, 0.0, 0.0),
                max: (16.0, 16.0, 16.0),
            },
            base_height,
            material_layers: Vec::new(),
            features: Vec::new(),
            volumes: Vec::new(),
        }
    }

    /// Generate and mesh a terrain description at the origin.
    fn meshed(terrain: &crate::level::Terrain) -> TerrainWorld {
        use crate::terrain::generation::generate_terrain;
        use crate::terrain::{ChunkGrid, SegmentFrame};

        let mut grid = ChunkGrid::new(terrain.voxel_size);
        generate_terrain(&mut grid, terrain, &terrain.bounds.to_aabb());
        world_of(SegmentFrame::identity(), grid)
    }

    fn assert_watertight(label: &str, world: &TerrainWorld) {
        assert!(world.triangle_count() > 0, "{label}: meshed to nothing");
        assert_eq!(
            world.open_edge_count(),
            0,
            "{label}: {} open edges in {} triangles",
            world.open_edge_count(),
            world.triangle_count()
        );
    }

    /// A flat surface authored at a height the voxel lattice hits exactly is the
    /// worst case for marching cubes: the iso-surface passes through a whole
    /// plane of lattice samples at once, and every cell along it degenerates.
    /// Noisy terrain never lines up like that; an authored one does nothing else.
    #[test]
    fn flat_terrain_on_the_lattice_is_watertight() {
        for (voxel_size, base_height) in [(1.0_f32, 8.0_f32), (0.5, 8.0), (0.5, 8.5), (2.0, 8.0)] {
            let world = meshed(&flat_terrain(voxel_size, base_height));
            assert_watertight(
                &format!("base_height {base_height} at {voxel_size} m voxels"),
                &world,
            );
        }
    }

    /// The same exposure reached through a feature rather than `base_height`:
    /// `Plateau` sets an absolute height, so an authored round number lands on
    /// the lattice just as squarely.
    #[test]
    fn plateau_on_the_lattice_is_watertight() {
        for voxel_size in [1.0_f32, 0.5] {
            let mut terrain = flat_terrain(voxel_size, 4.0);
            terrain.features.push(TerrainFeature::Plateau {
                min: (4.0, 4.0),
                max: (12.0, 12.0),
                height: 8.0,
            });
            let world = meshed(&terrain);
            assert_watertight(&format!("plateau at {voxel_size} m voxels"), &world);
        }
    }

    /// A cliff is two flat shelves joined by a sigmoid. Both shelves are
    /// authored heights, so both are exposed wherever they fall on the lattice.
    #[test]
    fn cliff_shelves_on_the_lattice_are_watertight() {
        for voxel_size in [1.0_f32, 0.5] {
            let mut terrain = flat_terrain(voxel_size, 4.0);
            terrain.features.push(TerrainFeature::Cliff {
                from: (0.0, 8.0),
                to: (16.0, 8.0),
                low_height: 4.0,
                high_height: 8.0,
                high_side: (0.0, 1.0),
                steepness: 2.0,
                end_falloff: 0.0,
                roughness: 0.0,
                roughness_seed: 0,
            });
            let world = meshed(&terrain);
            assert_watertight(&format!("cliff at {voxel_size} m voxels"), &world);
        }
    }

    /// An overhang is a volumetric slab with an authored top face and a
    /// thickness that puts its underside on a round number too.
    #[test]
    fn overhang_faces_on_the_lattice_are_watertight() {
        for voxel_size in [1.0_f32, 0.5] {
            let mut terrain = flat_terrain(voxel_size, 4.0);
            terrain.volumes.push(VolumeFeature::Overhang {
                from: (4.0, 8.0),
                to: (12.0, 8.0),
                height: 10.0,
                depth: 4.0,
                thickness: 2.0,
                direction: (0.0, 1.0),
                noise: 0.0,
                noise_seed: 0,
            });
            let world = meshed(&terrain);
            assert_watertight(&format!("overhang at {voxel_size} m voxels"), &world);
        }
    }

    /// End-to-end over the primary fixture: the level generates, meshes, and
    /// allocates far fewer chunks than its declared extent could hold.
    #[test]
    fn test_arena_generates_sparsely_at_its_declared_voxel_size() {
        use crate::terrain::generation::generate_terrain;
        use crate::terrain::{ChunkGrid, SegmentFrame};

        let level = crate::level::load_level(std::path::Path::new("levels/test_arena.level.ron"))
            .expect("test_arena should load");
        let terrain = &level.segments[0].terrain;
        let bounds = terrain.bounds.to_aabb();

        let mut grid = ChunkGrid::new(terrain.voxel_size);
        generate_terrain(&mut grid, terrain, &bounds);
        let world = world_of(SegmentFrame::identity(), grid);

        assert_eq!(world.voxel_size(), terrain.voxel_size);
        assert!(world.has_geometry());

        // Sparsity is the property that allocation follows the content, not the
        // declared extent: quadrupling the empty headroom above the terrain must
        // not allocate a single extra chunk.
        let allocated = |extra_height: f32| {
            let extent = AABB::new(
                bounds.min,
                Point3::new(bounds.max.x, bounds.max.y + extra_height, bounds.max.z),
            );
            let mut g = ChunkGrid::new(terrain.voxel_size);
            generate_terrain(&mut g, terrain, &extent);
            g.chunk_count()
        };
        let headroom = 4.0 * CHUNK_VOXELS as f32 * terrain.voxel_size;
        assert_eq!(
            allocated(headroom),
            allocated(0.0),
            "raising the ceiling by {headroom} m changed how many chunks were allocated"
        );

        // The terrain surface sits at base_height away from any feature.
        let h = world
            .mesh_surface_height_at(20.0, 30.0)
            .expect("no surface at an unfeatured column");
        assert!(
            (h - terrain.base_height).abs() < terrain.voxel_size,
            "flat terrain surfaced at {h}, expected {}",
            terrain.base_height
        );
    }

    /// Measure where the cost of a terrain rebuild actually goes.
    ///
    /// Ignored by default because it takes seconds and is a measurement rather
    /// than an assertion. Run it in release when the answer matters:
    ///
    /// ```bash
    /// cargo test --release --lib -- --ignored --nocapture grenade_update_cost
    /// ```
    ///
    /// The point is which term dominates: remesh is O(chunks dirtied) and stays
    /// put as levels grow, whereas the render buffer concatenation is O(total
    /// level triangles) and grows with them.
    #[test]
    #[ignore]
    fn grenade_update_cost_split() {
        let level = crate::level::load_level(std::path::Path::new("levels/test_arena.level.ron"))
            .expect("test_arena should load");
        let segments = crate::level::build_segments(&level).expect("placement resolves");
        let mut world = TerrainWorld::from_segments_headless(segments);
        println!(
            "test_arena: {} chunks, {} triangles, {} vertices",
            world.chunk_count(),
            world.triangle_count(),
            world.render_vertices().len()
        );

        // Explosion defaults: a 2.5 m crater at full voxel damage.
        for (i, centre) in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(16.0, 0.0, 16.0),
            Point3::new(-20.0, 0.0, 8.0),
            // Two more wide craters. Grenade 0 used to cost several times these
            // because the concat buffers reallocated on the first edit after
            // load; keeping later same-size craters here is what makes that kind
            // of regression visible as an outlier rather than the norm.
            Point3::new(32.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 32.0),
        ]
        .into_iter()
        .enumerate()
        {
            world.detonate(centre, &BlastConfig::fixed_radius(2.5));
            world.update();
            let t = world
                .last_update_timings()
                .expect("an update that destroyed voxels should be timed");
            let ms = |d: Duration| d.as_secs_f64() * 1000.0;
            println!(
                "grenade {i} at {centre:?}: {} chunks dirtied | remesh {:.2} ms | adjacency {:.2} ms | concat {:.2} ms | total {:.2} ms",
                t.chunks_dirtied,
                ms(t.remesh),
                ms(t.adjacency),
                ms(t.concat),
                ms(t.total())
            );
            println!(
                "    remesh split: grid alloc {:.2} | sample {:.2} | marching cubes {:.2} | ambient occlusion {:.2} | octree insert {:.2} | neighbour refs {:.2} | collect {:.2} ms",
                ms(t.build.grid_alloc),
                ms(t.build.sample),
                ms(t.build.marching_cubes),
                ms(t.build.ambient_occlusion),
                ms(t.build.insert),
                ms(t.build.neighbor_refs),
                ms(t.collect),
            );
            println!(
                "    concat split: mesh walk {:.2} | world transform {:.2} | index rebase {:.2} ms",
                ms(t.concat_split.mesh_walk),
                ms(t.concat_split.transform),
                ms(t.concat_split.rebase),
            );
        }
    }
}

#[cfg(test)]
mod query_guard_tests {
    use super::*;

    fn span(size: f32) -> AABB {
        AABB::new(
            Point3::new(-size * 0.5, -size * 0.5, -size * 0.5),
            Point3::new(size * 0.5, size * 0.5, size * 0.5),
        )
    }

    /// An ordinary collider-sized query is answered.
    #[test]
    fn a_collider_sized_query_is_allowed() {
        assert!(is_sane_query(&span(0.5)));
        assert!(is_sane_query(&span(64.0)));
    }

    /// A query the size of a level is a broken body, not a large one.
    #[test]
    fn a_level_sized_query_is_refused() {
        assert!(!is_sane_query(&span(1.0e6)));
        assert!(!is_sane_query(&span(f32::INFINITY)));
    }

    /// And so is one that has gone to NaN, which compares false against every
    /// bound and would otherwise sail through a size check.
    #[test]
    fn a_query_that_has_gone_to_nan_is_refused() {
        let mut aabb = span(1.0);
        aabb.max.x = f32::NAN;
        assert!(!is_sane_query(&aabb));
    }
}
