//! What the water renderer draws, and the meshes it draws them from.
//!
//! ```text
//!   WaterScene ──▶ still surfaces (WaterMesh): level and swell pushed per draw
//!              ──▶ river surfaces (RiverMesh) by id: built, then read again
//!              ──▶ awake ripple tiles (RippleTileView), drawn fine
//! ```

use crate::water::geometry::{SpanChunkCoord, COLUMNS_PER_CHUNK};
use crate::water::ids::SurfaceId;
use crate::water::surface::{RippleKey, RippleTiles, Swell, TILE_CORNERS};
use crate::water::waves::Sea;

use super::divide::WaterPatch;
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
    /// The still level the tile ripples about, and the swell on it.
    pub level: f32,
    pub swell: Swell,
    /// The river surface it ripples, which draws it; none for a still
    /// draw's tile, drawn on its own fine grid.
    pub surface: Option<SurfaceId>,
    ripples: &'a RippleTiles,
}

impl<'a> RippleTileView<'a> {
    /// The awake tile at `key`, rippling about `level` under `swell`, drawn
    /// on the river surface `surface` if it has one.
    pub fn new(
        ripples: &'a RippleTiles,
        key: &RippleKey,
        level: f32,
        swell: Swell,
        surface: Option<SurfaceId>,
    ) -> Option<Self> {
        let (_, tile) = ripples.active().find(|(k, _)| *k == key)?;
        Some(Self {
            tile: key.0,
            body: key.1,
            floors: &tile.mask.floors,
            corners: &tile.mask.corners,
            sealed: ripples.sealed_edges(key),
            level,
            swell,
            surface,
            ripples,
        })
    }

    /// Write the tile's heights as drawn, over `PADDED_CELLS`².
    pub fn write_heights(&self, out: &mut [f32]) {
        self.ripples.write_padded(&(self.tile, self.body), out);
    }
}

/// What reading a built river surface again found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refreshed {
    /// It kept its shape and is up to date.
    Held,
    /// Its shape changed; it was read as far as it could be, and must be
    /// built again, but can wait.
    Reshaped,
    /// It could not be read at all: the cells it reads were laid out again.
    /// It must be built again before it is drawn as it is now.
    Unreadable,
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

    /// The sea's level and swell, which river surfaces near sea level take
    /// up (`waves::swell_share`).
    fn sea(&self) -> Option<Sea> {
        None
    }

    /// Every awake ripple tile.
    fn ripple_tiles(&self) -> Vec<RippleTileView<'_>> {
        Vec::new()
    }

    /// The water the river surfaces hold, for the renderer to divide
    /// blended surfaces by: a patch per tile of still water, and per column
    /// of running water. Still draws are divided by their own tiles.
    fn surface_patches(&self) -> Vec<WaterPatch> {
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
    /// water, in place, as far as it can be.
    fn refresh_river(&self, _id: SurfaceId, _mesh: &mut RiverMesh) -> Refreshed {
        Refreshed::Unreadable
    }
}
