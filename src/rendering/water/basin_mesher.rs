//! Static water meshes for basins: built on topology changes, never per frame.
//!
//! ```text
//!   Basin region columns ──▶ one quad per column, plus a ring of columns
//!   (+ one column under        under the terrain at the waterline
//!    the terrain)           ──▶ grouped into 8 m tiles, one draw per (basin, tile)
//! ```
//!
//! The surface height is not in the mesh: each draw pushes its basin's level,
//! so a lake that drains keeps its mesh and only a number changes. The depth
//! test against the terrain makes the shorelines. Each vertex carries the
//! floor beneath it, for depth tint and swell attenuation.

use nalgebra::Vector2;
use rustc_hash::FxHashMap;

use crate::water::geometry::{Column, SpanChunkCoord, COLUMNS_PER_CHUNK, COLUMN_SIZE, ORTHOGONAL};
use crate::water::ids::{LinkId, StoreId};
use crate::water::network::Basin;
use crate::water::surface::{corner_floor, RippleTiles, Swell, TILE_CORNERS};
use crate::water::WaterWorld;

use super::fall_mesher::{FallKey, FallMesh, FallState};
use super::reach_mesher::{RiverMesh, RiverState};
use super::vertex::BasinVertex;

/// One draw: a basin's quads within one 8 m tile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaterDraw {
    pub body: StoreId,
    pub tile: SpanChunkCoord,
    pub first_index: u32,
    pub index_count: u32,
}

/// Every basin's surface, in one vertex and index buffer.
#[derive(Debug, Clone, Default)]
pub struct WaterMesh {
    pub vertices: Vec<BasinVertex>,
    pub indices: Vec<u32>,
    pub draws: Vec<WaterDraw>,
}

/// What the mesh was built from: each body and the version of its shape. A
/// different key means a rebuild.
pub type MeshKey = Vec<(StoreId, u32)>;

/// One awake ripple tile, as the renderer uploads it.
#[derive(Clone, Copy)]
pub struct RippleTileView<'a> {
    pub tile: SpanChunkCoord,
    pub body: StoreId,
    /// Floor under each column; NaN where the body holds no water there.
    pub floors: &'a [f32; COLUMNS_PER_CHUNK],
    /// Floor at each column corner, as the coarse surface takes it.
    pub corners: &'a [f32; TILE_CORNERS * TILE_CORNERS],
    /// Edges beside coarse water, one bit each: +x, −x, +z, −z.
    pub sealed: u32,
    ripples: &'a RippleTiles,
}

impl RippleTileView<'_> {
    /// Write the tile's heights as drawn, over `PADDED_CELLS`².
    pub fn write_heights(&self, out: &mut [f32]) {
        self.ripples.write_padded(&(self.tile, self.body), out);
    }
}

/// Anything the water renderer can draw: bodies with static meshes and a
/// level each, and optionally swell and awake ripple tiles.
pub trait WaterScene {
    /// Changes whenever [`Self::build_mesh`] would build something different.
    fn mesh_key(&self) -> MeshKey;
    fn build_mesh(&self) -> WaterMesh;
    /// A body's surface level now.
    fn level(&self, body: StoreId) -> Option<f32>;

    /// A body's swell.
    fn swell(&self, _body: StoreId) -> Swell {
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

    /// The sea's surface, drawn like a basin's.
    fn build_ocean(&self) -> WaterMesh {
        WaterMesh::default()
    }

    /// Changes whenever [`Self::build_rivers`] would build something
    /// different.
    fn river_key(&self) -> MeshKey {
        Vec::new()
    }

    fn build_rivers(&self) -> RiverMesh {
        RiverMesh::default()
    }

    /// A reach's state for its draw, if it is still there.
    fn river_state(&self, _reach: StoreId) -> Option<RiverState> {
        None
    }

    /// Changes whenever [`Self::build_falls`] would build something
    /// different.
    fn fall_key(&self) -> FallKey {
        Vec::new()
    }

    fn build_falls(&self) -> FallMesh {
        FallMesh::default()
    }

    /// A fall's state for its draw: `None` when nothing falls.
    fn fall_state(&self, _link: LinkId) -> Option<FallState> {
        None
    }
}

impl WaterScene for WaterWorld {
    fn mesh_key(&self) -> MeshKey {
        mesh_key(self.basins())
    }

    fn build_mesh(&self) -> WaterMesh {
        build(self.basins())
    }

    fn ocean_key(&self) -> MeshKey {
        self.ocean()
            .map(|(id, o)| (id, o.region_version))
            .into_iter()
            .collect()
    }

    fn build_ocean(&self) -> WaterMesh {
        let mut mesh = WaterMesh::default();
        if let Some((id, ocean)) = self.ocean() {
            let outlets = self.geometry().drainage().outlets();
            super::ocean_mesher::append_ocean(
                &mut mesh,
                id,
                ocean,
                self.geometry().graph(),
                outlets,
            );
        }
        mesh
    }

    fn level(&self, body: StoreId) -> Option<f32> {
        WaterWorld::level(self, body)
    }

    fn swell(&self, body: StoreId) -> Swell {
        WaterWorld::swell(self, body)
    }

    fn clock(&self) -> f32 {
        WaterWorld::clock(self)
    }

    fn river_key(&self) -> MeshKey {
        self.network()
            .stores()
            .filter_map(|(id, s)| s.as_reach().map(|r| (id, r.version)))
            .collect()
    }

    fn build_rivers(&self) -> RiverMesh {
        super::reach_mesher::build(
            self.network()
                .stores()
                .filter_map(|(id, s)| s.as_reach().map(|r| (id, r))),
        )
    }

    fn river_state(&self, reach: StoreId) -> Option<RiverState> {
        self.network()
            .store(reach)
            .and_then(|s| s.as_reach())
            .map(RiverState::of)
    }

    fn fall_key(&self) -> FallKey {
        super::fall_mesher::fall_key(self.falls())
    }

    fn build_falls(&self) -> FallMesh {
        super::fall_mesher::build(self.falls())
    }

    fn fall_state(&self, link: LinkId) -> Option<FallState> {
        FallState::carrying(self.link_discharge(link)?)
    }

    fn ripple_tiles(&self) -> Vec<RippleTileView<'_>> {
        let ripples = self.ripples();
        ripples
            .active()
            .map(|(key, tile)| RippleTileView {
                tile: key.0,
                body: key.1,
                floors: &tile.mask.floors,
                corners: &tile.mask.corners,
                sealed: ripples.sealed_edges(key),
                ripples,
            })
            .collect()
    }
}

/// The mesh key of a set of basins.
pub fn mesh_key<'a>(basins: impl Iterator<Item = (StoreId, &'a Basin)>) -> MeshKey {
    basins.map(|(id, b)| (id, b.region_version)).collect()
}

/// Build the surface of every basin.
pub fn build<'a>(basins: impl Iterator<Item = (StoreId, &'a Basin)>) -> WaterMesh {
    let mut mesh = WaterMesh::default();
    for (id, basin) in basins {
        append_basin(&mut mesh, id, basin);
    }
    mesh
}

/// A basin's columns with the floor under each, and the ring of columns just
/// outside it whose ground stands above its highest possible surface: those
/// are under the terrain at the waterline, and let the depth test cut the
/// shore cleanly. Columns beyond a crest, lower than the water, are left out.
/// Also returns the basin's own columns' floors.
fn columns(basin: &Basin) -> (Vec<(Column, f32)>, FxHashMap<Column, f32>) {
    let mut floors: FxHashMap<Column, f32> = FxHashMap::default();
    for r in &basin.region {
        let floor = floors.entry(r.span.column).or_insert(f32::INFINITY);
        *floor = floor.min(r.shape.floor_min);
    }
    let crest_columns: rustc_hash::FxHashSet<Column> =
        basin.crests.iter().map(|c| c.outside.column).collect();
    let mut ring: Vec<(Column, f32)> = Vec::new();
    for (&column, &floor) in &floors {
        for step in ORTHOGONAL {
            let next = column.offset(step.di, step.dk);
            if floors.contains_key(&next) || crest_columns.contains(&next) {
                continue;
            }
            ring.push((next, floor));
        }
    }
    let mut out: Vec<(Column, f32)> = floors.iter().map(|(&c, &f)| (c, f)).collect();
    ring.sort_by(|a, b| a.0.cmp(&b.0));
    ring.dedup_by(|a, b| a.0 == b.0);
    out.extend(ring);
    out.sort_by(|a, b| a.0.chunk().cmp(&b.0.chunk()).then(a.0.cmp(&b.0)));
    (out, floors)
}

fn append_basin(mesh: &mut WaterMesh, id: StoreId, basin: &Basin) {
    let (columns, wet) = columns(basin);
    append_columns(mesh, id, &columns, &wet);
}

/// One quad per column, drawn per 8 m tile. `columns` must be sorted by
/// tile, then column. Each corner takes the [`corner_floor`] of the `wet`
/// columns touching it, as a ripple tile's corners do, so the surfaces meet
/// at the same height; a corner with none takes its column's own floor.
pub(super) fn append_columns(
    mesh: &mut WaterMesh,
    id: StoreId,
    columns: &[(Column, f32)],
    wet: &FxHashMap<Column, f32>,
) {
    let wet_floor = |i: i32, k: i32| wet.get(&Column::new(i, k)).copied().unwrap_or(f32::NAN);
    let mut corners = [u32::MAX; TILE_CORNERS * TILE_CORNERS];
    let mut start = 0;
    while start < columns.len() {
        let tile = columns[start].0.chunk();
        let end = start
            + columns[start..]
                .iter()
                .take_while(|(c, _)| c.chunk() == tile)
                .count();
        let first_index = mesh.indices.len() as u32;
        let origin = tile.column(0);
        // Corners shared between the tile's quads, on the tile's own lattice.
        corners.fill(u32::MAX);
        for &(column, floor) in &columns[start..end] {
            let (li, lk) = (
                (column.i - origin.i) as usize,
                (column.k - origin.k) as usize,
            );
            let mut corner = |di: usize, dk: usize, mesh: &mut WaterMesh| {
                let slot = &mut corners[(lk + dk) * TILE_CORNERS + li + di];
                if *slot == u32::MAX {
                    let (ci, ck) = (column.i + di as i32, column.k + dk as i32);
                    let corner = corner_floor([
                        wet_floor(ci - 1, ck - 1),
                        wet_floor(ci, ck - 1),
                        wet_floor(ci - 1, ck),
                        wet_floor(ci, ck),
                    ]);
                    mesh.vertices.push(BasinVertex {
                        xz: Vector2::new(
                            (column.i + di as i32) as f32 * COLUMN_SIZE,
                            (column.k + dk as i32) as f32 * COLUMN_SIZE,
                        ),
                        floor: if corner.is_nan() { floor } else { corner },
                        swell_share: 1.0,
                    });
                    *slot = mesh.vertices.len() as u32 - 1;
                }
                *slot
            };
            let a = corner(0, 0, mesh);
            let b = corner(1, 0, mesh);
            let c = corner(1, 1, mesh);
            let d = corner(0, 1, mesh);
            // Counter-clockwise seen from above (+y), matching the terrain's
            // outward winding.
            mesh.indices.extend_from_slice(&[a, d, c, a, c, b]);
        }
        mesh.draws.push(WaterDraw {
            body: id,
            tile,
            first_index,
            index_count: mesh.indices.len() as u32 - first_index,
        });
        start = end;
    }
}
