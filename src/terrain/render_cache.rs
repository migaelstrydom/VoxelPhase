//! Per-chunk render geometry, kept in world space so publishing a terrain edit
//! costs a copy rather than a rebuild.
//!
//! Concatenating the level's render buffers used to walk every chunk's mesh
//! octree and transform every one of its vertices, on every terrain update —
//! O(total level triangles) work to publish an edit that touched a handful of
//! chunks. A chunk that was not remeshed cannot have changed, so its
//! contribution is cached here and the concatenation reduces to appending
//! ready-made slices.
//!
//! ```text
//!   Segment::remesh_chunk ──insert──► ChunkRenderCache ──append──► level buffers
//!         (only dirty chunks)          (one entry per chunk)
//! ```

use rustc_hash::FxHashMap;

use super::chunk::ChunkCoord;
use super::frame::SegmentFrame;
use super::mesh_octree::MeshOctree;
use crate::rendering::vertex::Vertex;

/// One chunk's render geometry, already lifted into world space.
#[derive(Clone)]
pub struct ChunkRenderData {
    /// World-space vertices.
    pub vertices: Vec<Vertex>,
    /// Indices relative to this chunk's own first vertex, so the geometry can
    /// be appended at any offset in the concatenated buffer.
    pub indices: Vec<u32>,
}

/// Collect a chunk's mesh and lift it into world space.
///
/// The frame is baked in at collection time, which is what keeps everything
/// downstream of the terrain in world space.
pub fn build_chunk_render_data(mesh: &MeshOctree, frame: &SegmentFrame) -> ChunkRenderData {
    let (vertices, indices) = mesh.get_render_data();
    ChunkRenderData {
        vertices: vertices
            .into_iter()
            .map(|vertex| to_world_vertex(frame, vertex))
            .collect(),
        indices,
    }
}

/// Lift a mesh vertex into world space. Position and normal both rotate; the
/// homogeneous `w` and the shading attributes do not.
fn to_world_vertex(frame: &SegmentFrame, mut vertex: Vertex) -> Vertex {
    let pos = frame.to_world(nalgebra::Point3::new(
        vertex.pos.x,
        vertex.pos.y,
        vertex.pos.z,
    ));
    vertex.pos.x = pos.x;
    vertex.pos.y = pos.y;
    vertex.pos.z = pos.z;
    vertex.normal = frame.rotate_to_world(vertex.normal);
    vertex
}

/// One segment's cached chunk geometry.
#[derive(Default)]
pub struct ChunkRenderCache {
    entries: FxHashMap<ChunkCoord, ChunkRenderData>,
}

impl ChunkRenderCache {
    pub fn new() -> Self {
        Self::default()
    }

    /// Replace a chunk's cached geometry. Called for each remeshed chunk, and
    /// is the only way an entry becomes current again.
    pub fn insert(&mut self, coord: ChunkCoord, data: ChunkRenderData) {
        self.entries.insert(coord, data);
    }

    pub fn get(&self, coord: ChunkCoord) -> Option<&ChunkRenderData> {
        self.entries.get(&coord)
    }

    /// Drop entries for chunks that no longer exist, so pruning a vacant chunk
    /// does not leak its geometry.
    pub fn retain_coords(&mut self, keep: impl Fn(ChunkCoord) -> bool) {
        self.entries.retain(|coord, _| keep(*coord));
    }
}
