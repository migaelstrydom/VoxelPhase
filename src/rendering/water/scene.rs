//! What the water renderer draws, and the meshes it draws them from.
//!
//! ```text
//!   WaterScene ──▶ still surfaces (WaterMesh): level and swell pushed per draw
//!              ──▶ river surfaces (RiverMesh) by id: built, then read again
//!              ──▶ awake ripple tiles (RippleTileView), drawn fine
//! ```

use crate::water::geometry::{SpanChunkCoord, COLUMNS_PER_CHUNK};
use crate::water::ids::SurfaceId;
use crate::water::surface::{RippleTiles, Swell, TILE_CORNERS};

use super::river_mesh::RiverMesh;
use super::vertex::BasinVertex;

/// One draw: a still surface's quads within one 8 m tile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaterDraw {
    pub body: SurfaceId,
    pub tile: SpanChunkCoord,
    pub first_index: u32,
    pub index_count: u32,
}

/// Every still surface, in one vertex and index buffer.
#[derive(Debug, Clone, Default)]
pub struct WaterMesh {
    pub vertices: Vec<BasinVertex>,
    pub indices: Vec<u32>,
    pub draws: Vec<WaterDraw>,
}

/// What the mesh was built from: each body and the version of its shape. A
/// different key means a rebuild.
pub type MeshKey = Vec<(SurfaceId, u32)>;

/// One awake ripple tile, as the renderer uploads it.
#[derive(Clone, Copy)]
pub struct RippleTileView<'a> {
    pub tile: SpanChunkCoord,
    pub body: SurfaceId,
    /// Floor under each column; NaN where the body holds no water there.
    pub floors: &'a [f32; COLUMNS_PER_CHUNK],
    /// Floor at each column corner, as the coarse surface takes it.
    pub corners: &'a [f32; TILE_CORNERS * TILE_CORNERS],
    /// Edges beside coarse water, one bit each: +x, −x, +z, −z.
    pub sealed: u32,
    ripples: &'a RippleTiles,
}

impl<'a> RippleTileView<'a> {
    pub fn new(
        ripples: &'a RippleTiles,
        tile: SpanChunkCoord,
        body: SurfaceId,
        floors: &'a [f32; COLUMNS_PER_CHUNK],
        corners: &'a [f32; TILE_CORNERS * TILE_CORNERS],
        sealed: u32,
    ) -> Self {
        Self {
            tile,
            body,
            floors,
            corners,
            sealed,
            ripples,
        }
    }

    /// Write the tile's heights as drawn, over `PADDED_CELLS`².
    pub fn write_heights(&self, out: &mut [f32]) {
        self.ripples.write_padded(&(self.tile, self.body), out);
    }
}

/// Anything the water renderer can draw: still surfaces with static meshes
/// and a level each (a bench's pool, the sea's ring), swell and awake ripple
/// tiles, and river surfaces built and read again by id (the field's
/// chunks).
pub trait WaterScene {
    /// Changes whenever [`Self::build_mesh`] would build something different.
    fn mesh_key(&self) -> MeshKey;
    fn build_mesh(&self) -> WaterMesh;
    /// A body's surface level now.
    fn level(&self, body: SurfaceId) -> Option<f32>;

    /// A body's swell.
    fn swell(&self, _body: SurfaceId) -> Swell {
        Swell::default()
    }

    /// The clock the swell runs on, s.
    fn clock(&self) -> f32 {
        0.0
    }

    /// Every awake ripple tile.
    fn ripple_tiles(&self) -> Vec<RippleTileView<'_>> {
        Vec::new()
    }

    /// Changes whenever [`Self::build_ocean`] would build something
    /// different: only when the sea claims a lowland.
    fn ocean_key(&self) -> MeshKey {
        Vec::new()
    }

    /// The sea's surface, drawn like any still one.
    fn build_ocean(&self) -> WaterMesh {
        WaterMesh::default()
    }

    /// Every river surface by id, with a version that changes whenever
    /// [`Self::build_river`] would build that one differently, or its water
    /// has moved.
    fn river_key(&self) -> MeshKey {
        Vec::new()
    }

    /// One river surface: its own vertices, indices and draws.
    fn build_river(&self, _id: SurfaceId) -> RiverMesh {
        RiverMesh::default()
    }

    /// Read a built river surface's heights, depths and flow again from the
    /// water, in place. Returns whether it kept its shape; where it did not,
    /// it must be built again.
    fn refresh_river(&self, _id: SurfaceId, _mesh: &mut RiverMesh) -> bool {
        false
    }
}
