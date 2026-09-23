//! Building a dirty chunk's replacement mesh without touching the segment.
//!
//! ```text
//!   dirty coords ──par_iter──▶ ChunkRebuild::build(&grid, &frame, coord)
//!                                 sample · marching cubes · AO · octree
//!                                 old/new triangle lists · render data
//!                                          │  (reads only)
//!                                          ▼
//!                               Vec<ChunkRebuild>, in dirty order
//!                                          │
//!                         Segment::commit_chunk, one at a time
//!                           adjacency patch · mesh swap · render cache
//! ```
//!
//! Everything expensive about a remesh depends only on the voxel grid, which a
//! rebuild reads and never writes, so dirty chunks build independently. Only
//! installing the result touches state the chunks share — the segment's
//! adjacency map and the grid's chunk table — and that stays sequential, in
//! the order the chunks were listed, so a parallel build commits exactly what a
//! serial one would.

use std::time::{Duration, Instant};

use nalgebra::Point3;

use super::chunk::{ChunkCoord, ChunkTriangleRef, CHUNK_VOXELS};
use super::chunk_grid::ChunkGrid;
use super::frame::SegmentFrame;
use super::mesh_octree::{MeshBuildTimings, MeshOctree, TriangleRef};
use super::render_cache::{build_chunk_render_data, ChunkRenderData};

/// A triangle keyed segment-wide, with its segment-local corners.
pub type QualifiedTriangle = (ChunkTriangleRef, [Point3<f32>; 3]);

/// CPU time spent rebuilding chunks, summed over them.
///
/// Chunks build on several threads at once, so this is more than the wall
/// clock the build took; the ratio between the two is the speed-up.
#[derive(Debug, Clone, Copy, Default)]
pub struct ChunkBuildTimings {
    /// Sampling, marching cubes, occlusion and octree construction.
    pub mesh: MeshBuildTimings,
    /// Listing the old and new meshes' triangles for the adjacency diff.
    pub collect: Duration,
    /// Lifting the new mesh into world space for the render cache.
    pub render_data: Duration,
}

impl ChunkBuildTimings {
    pub fn total(&self) -> Duration {
        self.mesh.total() + self.collect + self.render_data
    }

    pub fn add(&mut self, other: &Self) {
        self.mesh.add(&other.mesh);
        self.collect += other.collect;
        self.render_data += other.render_data;
    }
}

/// One chunk's replacement mesh and everything needed to install it.
pub struct ChunkRebuild {
    pub coord: ChunkCoord,
    pub mesh: MeshOctree,
    /// The triangles the chunk's current mesh holds, to unlink.
    pub old_triangles: Vec<QualifiedTriangle>,
    /// The triangles the new mesh holds, to link.
    pub new_triangles: Vec<QualifiedTriangle>,
    /// The new mesh in world space, for the render cache.
    pub render_data: ChunkRenderData,
    pub timings: ChunkBuildTimings,
}

impl ChunkRebuild {
    /// Mesh chunk `coord` from the voxels in `grid`, as they are now.
    pub fn build(grid: &ChunkGrid, frame: &SegmentFrame, coord: ChunkCoord) -> Self {
        let mut mesh = MeshOctree::new(grid.chunk_bounds(coord));
        let mesh_timings = mesh.generate_block(
            Point3::origin(),
            grid.first_sample(coord),
            CHUNK_VOXELS as usize,
            grid.voxel_size(),
            grid,
        );

        let t_collect = Instant::now();
        let old_triangles = grid
            .chunk(coord)
            .map(|chunk| qualified_triangles(coord, chunk.mesh()))
            .unwrap_or_default();
        let new_triangles = qualified_triangles(coord, &mesh);
        let collect = t_collect.elapsed();

        let t_render = Instant::now();
        let render_data = build_chunk_render_data(&mesh, frame);
        let render_data_time = t_render.elapsed();

        Self {
            coord,
            mesh,
            old_triangles,
            new_triangles,
            render_data,
            timings: ChunkBuildTimings {
                mesh: mesh_timings,
                collect,
                render_data: render_data_time,
            },
        }
    }
}

/// Every triangle of a chunk's mesh, keyed by the chunk as well.
///
/// Adjacency matches by quantised position, and a segment's triangles are all
/// in one frame, so the local positions are already directly comparable.
fn qualified_triangles(coord: ChunkCoord, mesh: &MeshOctree) -> Vec<QualifiedTriangle> {
    let mut triangles: Vec<(TriangleRef, [Point3<f32>; 3])> = Vec::new();
    mesh.collect_all_triangles_into(&mut triangles);
    triangles
        .into_iter()
        .map(|(triangle, positions)| {
            (
                ChunkTriangleRef {
                    chunk: coord,
                    triangle,
                },
                positions,
            )
        })
        .collect()
}
