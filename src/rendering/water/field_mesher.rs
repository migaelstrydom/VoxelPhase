//! The shallow-water field's surface, drawn through the river pipeline.
//!
//! ```text
//!   wet cell ──▶ one quad over its column, each corner the mean surface of
//!                its group of the wet cells meeting there: split where two
//!                surfaces stand more than JOIN apart
//!   two wet cells whose surfaces step more than JOIN
//!            ──▶ a sheet down their shared edge, from the upper surface's
//!                corners to the lower's: water going over the edge
//!   dry column beside one, or diagonal to it between two dry sides
//!            ──▶ a shore patch carrying the cell's surface half a column
//!                into it: drawn on into rock, which hides it; cut over a
//!                hollow below the water
//! ```
//!
//! Each vertex also carries the water's depth there, negative out over a
//! hollow; the draw cuts the surface at `river.vert`'s shore depth. Against
//! rock the terrain itself draws the shoreline, where it rises through the
//! surface; over a hollow the shoreline is a contour across the patch.
//!
//! Every vertex carries its own height and a velocity, the mean of the cells
//! its corner joins, so the river shader's flow mapping runs on the field's
//! currents and runs smoothly across cells. A surface is built from the graph
//! when its shape changes; while only its water moves, every vertex is read
//! again from the cells it was built from (`field_sources`). Past the map's
//! open edges, the sea runs out to the horizon as a ring.

use nalgebra::Vector2;
use smallvec::SmallVec;

use crate::water::field::{DrawnDepths, WaterField};
use crate::water::geometry::{Column, SpanIndex, SpanRef, Step, COLUMN_SIZE, ORTHOGONAL};
use crate::water::ids::SurfaceId;
use crate::water::surface::Swell;
use crate::water::waves::{still_level, Sea, SurfaceWaves};
use crate::water::{WaterWorld, FIELD_BODY};

use super::divide::WaterPatch;
use super::field_sources::{
    self, corner_flow, gather, Corner, DrawnCells, Meeting, VertexSource, JOIN,
};
use super::ocean_ring::{append_ring, occupied_bounds, MapRect};
use super::river_mesh::{RiverDraw, RiverMesh};
use super::scene::{MeshKey, Refreshed, RippleTileView, WaterMesh, WaterScene};
use super::vertex::RiverVertex;

/// The field, as the water renderer sees it.
pub struct FieldScene<'a> {
    pub field: &'a WaterField,
    /// Each chunk's surface version, by slot: changes whenever its surface
    /// has moved since it was last drawn.
    pub versions: &'a [u32],
    /// Each chunk's depths as its surface is drawn from them.
    pub depths: DrawnDepths<'a>,
    /// Seconds simulated: the clock the drawn currents run on.
    pub clock: f32,
    /// The ripples and swell over the field, if it has any.
    pub waves: Option<&'a SurfaceWaves>,
}

impl<'a> FieldScene<'a> {
    /// The game's water, as drawn.
    pub fn of(world: &'a WaterWorld) -> Self {
        Self {
            field: world.field(),
            versions: world.versions(),
            depths: world.drawn_depths(),
            clock: world.clock(),
            waves: Some(world.waves()),
        }
    }
}

impl<'a> FieldScene<'a> {
    /// Each awake tile, over still water, drawn on its chunk's surface.
    fn ripple_views(&self) -> Vec<RippleTileView<'a>> {
        let Some(waves) = self.waves else {
            return Vec::new();
        };
        let cells = self.field.cells();
        waves
            .ripples()
            .active()
            .filter_map(|(key, tile)| {
                let level = tile.mask.level;
                let surface = cells.slot_of(key.0).map(|slot| SurfaceId(slot as u32));
                RippleTileView::new(waves.ripples(), key, level, waves.swell_at(level), surface)
            })
            .collect()
    }

    fn drawn_cells(&self) -> DrawnCells<'a> {
        DrawnCells {
            cells: self.field.cells(),
            graph: self.field.graph(),
            depths: self.depths,
        }
    }

    fn cell(&self, span: SpanRef) -> Option<SpanIndex> {
        self.field
            .cells()
            .index(self.field.graph(), span.column, span.ordinal)
    }

    fn meeting(&self, span: SpanRef) -> Option<Meeting> {
        Some(Meeting::of(&self.drawn_cells(), self.cell(span)?))
    }

    /// The span joined to `span` across one orthogonal step whose water a
    /// surface at `level` would meet: the wet one whose surface stands
    /// nearest, else the one whose floor does.
    fn across(&self, span: SpanRef, di: i32, dk: i32, level: f64) -> Option<SpanRef> {
        let near = |m: &Meeting| (m.surface.unwrap_or(m.bed) - level).abs();
        let mut wet: Option<(f64, SpanRef)> = None;
        let mut any: Option<(f64, SpanRef)> = None;
        for n in self.field.graph().joined(span, Step { di, dk }) {
            let key = SpanRef::key(n.span.column, n.span.ordinal);
            let Some(m) = self.meeting(key) else {
                continue;
            };
            let d = near(&m);
            if m.surface.is_some() && wet.is_none_or(|(best, _)| d < best) {
                wet = Some((d, key));
            }
            if any.is_none_or(|(best, _)| d < best) {
                any = Some((d, key));
            }
        }
        wet.or(any).map(|(_, key)| key)
    }

    /// The corner of `span`'s column on the (sx, sz) side, for a cell whose
    /// surface stands at `level`, its flow, and where it reads from: the cell and the
    /// cells across from it on each side and diagonally, as `gather` joins
    /// them.
    fn corner(
        &self,
        span: SpanRef,
        level: f64,
        sx: i32,
        sz: i32,
    ) -> (Corner, Vector2<f32>, VertexSource) {
        let x = self.across(span, sx, 0, level);
        let z = self.across(span, 0, sz, level);
        let diagonal = x
            .and_then(|n| self.across(n, 0, sz, level))
            .or_else(|| z.and_then(|n| self.across(n, sx, 0, level)));
        let cells: SmallVec<[SpanIndex; 4]> = [Some(span), x, z, diagonal]
            .into_iter()
            .flatten()
            .filter_map(|s| self.cell(s))
            .collect();
        let drawn = self.drawn_cells();
        let meetings: SmallVec<[Meeting; 4]> =
            cells.iter().map(|&c| Meeting::of(&drawn, c)).collect();
        let (corner, shape) = gather(&meetings, level);
        let flow = corner_flow(self.field, &cells, shape);
        (corner, flow, VertexSource::Corner { cells, shape })
    }

    /// Every wet cell joined to `span` across one orthogonal step.
    fn wet_across(&self, span: SpanRef, di: i32, dk: i32) -> SmallVec<[(SpanRef, SpanIndex); 2]> {
        let drawn = self.drawn_cells();
        self.field
            .graph()
            .joined(span, Step { di, dk })
            .into_iter()
            .map(|n| SpanRef::key(n.span.column, n.span.ordinal))
            .filter_map(|key| Some((key, self.cell(key)?)))
            .filter(|&(_, c)| Meeting::of(&drawn, c).surface.is_some())
            .collect()
    }

    /// A cell's quad, and the falling sheets and shore patches around it.
    fn cell_surface(&self, mesh: &mut RiverMesh, span: SpanRef, at: SpanIndex, level: f64) {
        let cells = self.field.cells();
        let drawn = self.drawn_cells();
        let column = span.column;
        let built =
            [(-1, -1), (1, -1), (1, 1), (-1, 1)].map(|(sx, sz)| self.corner(span, level, sx, sz));
        let corners = built.clone().map(|(corner, _, _)| corner);
        let flows = built.clone().map(|(_, flow, _)| flow);
        let first = mesh.vertices.len() as u32;
        for (_, _, source) in &built {
            if let VertexSource::Corner { cells: read, .. } = source {
                for &c in read {
                    mesh.sources.reads(cells, c);
                }
            }
        }
        quad(
            mesh,
            column,
            corners,
            flows,
            built.map(|(_, _, source)| source),
        );

        for step in ORTHOGONAL {
            for (other_span, other) in self.wet_across(span, step.di, step.dk) {
                let other_level = drawn.surface(other);
                if level - other_level > JOIN {
                    self.sheet(mesh, column, step, first, at, (other_span, other_level));
                }
            }
        }

        self.shore(mesh, column, level, first);
    }

    /// What stands in `column` at `level`, beside a wet cell there: water,
    /// a dry cell (none where the column is rock at that height), or the
    /// map's edge.
    fn beside(&self, column: Column, level: f64) -> Beside {
        let graph = self.field.graph();
        let Some(slot) = graph.slot(column.chunk()) else {
            return Beside::Outside;
        };
        let cell = graph
            .span_at(column, level as f32)
            .and_then(|span| self.cell(span));
        match cell {
            Some(cell) if self.drawn_cells().drawn(cell) => Beside::Wet,
            bank => Beside::Dry(slot as u32, bank),
        }
    }

    /// The cell's surface carried out over the dry columns around it, half
    /// a column into each beside it and a quarter into each diagonal whose
    /// two sides are dry too, so a shore meets the terrain at outer and inner
    /// corners alike. Each patch's corners copy the cell's nearest corner
    /// (vertex `first + i`); `shore_depth` draws each over rock and cuts it
    /// over a hollow. Patches from cells on either side of a dry column meet
    /// at its centre, without overlapping.
    fn shore(&self, mesh: &mut RiverMesh, column: Column, level: f64, first: u32) {
        let cells = self.field.cells();
        let drawn = self.drawn_cells();
        let around: [[Beside; 3]; 3] =
            [-1, 0, 1].map(|dk| [-1, 0, 1].map(|di| self.beside(column.offset(di, dk), level)));
        let at = |di: i32, dk: i32| around[(dk + 1) as usize][(di + 1) as usize];
        let dry = |di: i32, dk: i32| matches!(at(di, dk), Beside::Dry(..));
        let mut patches: SmallVec<[(i32, i32); 8]> =
            ORTHOGONAL.iter().map(|s| (s.di, s.dk)).collect();
        for (sx, sz) in [(-1, -1), (1, -1), (1, 1), (-1, 1)] {
            if dry(sx, 0) && dry(0, sz) {
                patches.push((sx, sz));
            }
        }
        let (x0, z0) = column.min_corner();
        for (di, dk) in patches {
            if !dry(di, dk) {
                continue;
            }
            // The patch's span along one axis, in columns from the cell's
            // minimum corner.
            let range = |d: i32| match d {
                1 => (1.0, 1.5),
                -1 => (-0.5, 0.0),
                _ => (0.0, 1.0),
            };
            let ((xa, xb), (za, zb)) = (range(di), range(dk));
            let base = mesh.vertices.len() as u32;
            for (x, z) in [(xa, za), (xb, za), (xb, zb), (xa, zb)] {
                // The cell corner nearest, and the column beside the cell
                // the patch corner lies out over, if any.
                let (cx, cz) = (x >= 1.0, z >= 1.0);
                let i = match (cx, cz) {
                    (false, false) => 0,
                    (true, false) => 1,
                    (true, true) => 2,
                    (false, true) => 3,
                };
                let out = |c: f32| (c > 1.0) as i32 - (c < 0.0) as i32;
                let from = mesh.vertices[(first as usize) + i];
                let (depth, source) = match at(out(x), out(z)) {
                    Beside::Dry(slot, bank) if (out(x), out(z)) != (0, 0) => {
                        mesh.sources.reads_slot(cells, slot);
                        let depth = field_sources::shore_depth(
                            &drawn,
                            bank,
                            from.surface as f64,
                            from.depth,
                        );
                        let i = first + i as u32;
                        (depth, VertexSource::Shore { from: i, bank })
                    }
                    _ => (
                        from.depth,
                        VertexSource::Edge {
                            from: first + i as u32,
                        },
                    ),
                };
                mesh.vertices.push(RiverVertex {
                    xz: Vector2::new(x0 + x * COLUMN_SIZE, z0 + z * COLUMN_SIZE),
                    depth,
                    ..from
                });
                mesh.sources.vertices.push(source);
            }
            mesh.indices
                .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
        }
    }
}

/// What a wet cell's surface meets in a column beside it.
#[derive(Debug, Clone, Copy)]
enum Beside {
    /// Water drawn there: its own surface covers the column.
    Wet,
    /// No water: the column's slot, and its dry cell at the water's height,
    /// none where the column is rock there.
    Dry(u32, Option<SpanIndex>),
    /// Past the map's edge.
    Outside,
}

impl FieldScene<'_> {
    /// A sheet of water falling over the edge a cell shares with the lower
    /// cell across `step`: from the cell's own corners on that edge (vertex
    /// `first + i`) down to the lower cell's corners there, so it meets both
    /// surfaces. It is as deep as the water pours over the edge, no deeper
    /// than the water it falls from, so it is cut where little or nothing
    /// pours.
    ///
    /// Where the lower column's own floor carries the ledge on (`lip`), each
    /// end of the sheet hangs out at the lip instead, and the cell's surface
    /// runs on over the ledge to meet it. The ledge is told by the two beds
    /// alone, so the sheet keeps its place while the water moves.
    fn sheet(
        &self,
        mesh: &mut RiverMesh,
        column: Column,
        step: Step,
        first: u32,
        own: SpanIndex,
        (lower_span, lower_level): (SpanRef, f64),
    ) {
        // The edge's ends, as this cell's corner indices and as the sides of
        // the lower cell's column they lie on.
        let ends: [(usize, (i32, i32)); 2] = match (step.di, step.dk) {
            (1, _) => [(1, (-1, -1)), (2, (-1, 1))],
            (-1, _) => [(0, (1, -1)), (3, (1, 1))],
            (_, 1) => [(3, (-1, -1)), (2, (1, -1))],
            _ => [(0, (-1, 1)), (1, (1, 1))],
        };
        let offsets = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let (x0, z0) = column.min_corner();
        let face = Vector2::new(step.di as f32, step.dk as f32);
        let lower = self.field.graph().span(lower_span);
        let ledge = 0.5 * (self.field.cells().bed(own) + lower.bed);
        let quarters = lower.quarters;
        let shift = ends.map(|(_, sides)| face * lip(step, sides, &quarters, ledge) * COLUMN_SIZE);
        if shift.iter().any(|s| *s != Vector2::zeros()) {
            let base = mesh.vertices.len() as u32;
            for (&(i, _), shift) in ends.iter().zip(shift) {
                let top = mesh.vertices[(first as usize) + i];
                mesh.vertices.push(RiverVertex {
                    xz: top.xz + shift,
                    ..top
                });
                mesh.sources.vertices.push(VertexSource::Edge {
                    from: first + i as u32,
                });
            }
            let [a, b] = ends.map(|(i, _)| first + i as u32);
            mesh.indices
                .extend_from_slice(&[a, b, base + 1, a, base + 1, base]);
        }
        let base = mesh.vertices.len() as u32;
        let pour = field_sources::pour(self.field, &self.drawn_cells(), own, face);
        for (&(i, _), shift) in ends.iter().zip(shift) {
            let top = mesh.vertices[(first as usize) + i];
            mesh.vertices.push(RiverVertex {
                xz: top.xz + shift,
                face,
                depth: top.depth.min(pour),
                ..top
            });
            mesh.sources.vertices.push(VertexSource::Lip {
                from: first + i as u32,
                own,
            });
        }
        for (end, &(i, (sx, sz))) in ends.iter().enumerate().rev() {
            let (foot, _, source) = self.corner(lower_span, lower_level, sx, sz);
            let VertexSource::Corner { cells, shape } = source else {
                unreachable!("a corner reads its cells");
            };
            for &c in &cells {
                mesh.sources.reads(self.field.cells(), c);
            }
            let top = mesh.vertices[base as usize + end];
            let (cx, cz) = offsets[i];
            mesh.vertices.push(RiverVertex {
                xz: Vector2::new(x0 + cx * COLUMN_SIZE, z0 + cz * COLUMN_SIZE) + shift[end],
                surface: foot.height as f32,
                ..top
            });
            mesh.sources.vertices.push(VertexSource::Foot {
                from: base + end as u32,
                cells,
                shape,
                own,
            });
        }
        mesh.indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
}

/// How far past the edge into the lower column across `step` its floor
/// stands above `ledge`, in columns, along the row of quarters from the end
/// of the edge on the lower column's `sides`: where the ledge the water
/// falls from really ends.
fn lip(step: Step, (sx, sz): (i32, i32), quarters: &[f32; 4], ledge: f32) -> f32 {
    let near = (sx > 0) as usize + 2 * (sz > 0) as usize;
    let far = near ^ if step.di != 0 { 1 } else { 2 };
    match (quarters[near] > ledge, quarters[far] > ledge) {
        (false, _) => 0.0,
        (true, false) => 0.5,
        (true, true) => 1.0,
    }
}

fn quad(
    mesh: &mut RiverMesh,
    column: Column,
    corners: [Corner; 4],
    flows: [Vector2<f32>; 4],
    sources: [VertexSource; 4],
) {
    let (x0, z0) = column.min_corner();
    let offsets = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    let base = mesh.vertices.len() as u32;
    for (((cx, cz), corner), flow) in offsets.iter().zip(corners).zip(flows) {
        mesh.vertices.push(RiverVertex {
            xz: Vector2::new(x0 + cx * COLUMN_SIZE, z0 + cz * COLUMN_SIZE),
            surface: corner.height as f32,
            depth: corner.depth as f32,
            flow,
            face: Vector2::zeros(),
        });
    }
    mesh.sources.vertices.extend(sources);
    mesh.indices
        .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
}

impl WaterScene for FieldScene<'_> {
    fn mesh_key(&self) -> MeshKey {
        Vec::new()
    }

    fn build_mesh(&self) -> WaterMesh {
        WaterMesh::default()
    }

    /// The sea's level, for its ring past the map's edges.
    fn level(&self, body: SurfaceId) -> Option<f32> {
        self.field
            .sea()
            .filter(|_| body == FIELD_BODY)
            .map(|sea| sea.level)
    }

    fn clock(&self) -> f32 {
        self.clock
    }

    /// The sea's swell, for its ring past the map's edges.
    fn swell(&self, body: SurfaceId) -> Swell {
        self.sea()
            .filter(|_| body == FIELD_BODY)
            .map_or(Swell::default(), |sea| sea.swell)
    }

    fn sea(&self) -> Option<Sea> {
        self.waves?.sea()
    }

    fn ripple_tiles(&self) -> Vec<RippleTileView<'_>> {
        self.ripple_views()
    }

    /// Every chunk whose water stands still.
    fn still_water(&self) -> Vec<WaterPatch> {
        self.field
            .cells()
            .chunks
            .iter()
            .filter_map(|chunk| {
                still_level(self.field, chunk.coord)
                    .map(|level| WaterPatch::tile(chunk.coord.x, chunk.coord.z, level))
            })
            .collect()
    }

    fn ocean_key(&self) -> MeshKey {
        self.field
            .sea()
            .map(|_| (FIELD_BODY, 0))
            .into_iter()
            .collect()
    }

    /// The ring past the map's open edges only: the sea inside them is the
    /// field's own water.
    fn build_ocean(&self) -> WaterMesh {
        let mut mesh = WaterMesh::default();
        let graph = self.field.graph();
        if let (Some(sea), Some((min, max))) = (self.field.sea(), occupied_bounds(graph)) {
            let map = MapRect {
                i0: min.i,
                k0: min.k,
                i1: max.i + 1,
                k1: max.k + 1,
            };
            // The sea's edges run +x, +z, −x, −z; the ring's −x, +x, −z, +z.
            let open = [sea.open[2], sea.open[0], sea.open[3], sea.open[1]];
            append_ring(&mut mesh, FIELD_BODY, sea.level, map, open);
        }
        mesh
    }

    /// One surface per chunk, by slot.
    fn river_key(&self) -> MeshKey {
        self.versions
            .iter()
            .enumerate()
            .map(|(slot, &v)| (SurfaceId(slot as u32), v))
            .collect()
    }

    fn build_river(&self, id: SurfaceId) -> RiverMesh {
        let slot = id.0 as usize;
        let mut mesh = RiverMesh::default();
        let Some(chunk) = self.field.cells().chunks.get(slot) else {
            return mesh;
        };
        let drawn = self.drawn_cells();
        let at = |flat: usize| SpanIndex {
            slot: slot as u32,
            flat: flat as u32,
        };
        mesh.sources.slot = slot as u32;
        mesh.sources.read = self.depths.now;
        // Its own layout, which an empty surface reads nothing else to note.
        mesh.sources.reads(self.field.cells(), at(0));
        mesh.sources.drawn = (0..chunk.depth.len())
            .map(|flat| drawn.drawn(at(flat)))
            .collect();
        for (flat, (local, ordinal)) in chunk.cell_columns().enumerate() {
            if !mesh.sources.drawn[flat] {
                continue;
            }
            let span = SpanRef::key(chunk.coord.column(local), ordinal);
            self.cell_surface(&mut mesh, span, at(flat), drawn.surface(at(flat)));
        }
        if !mesh.indices.is_empty() {
            mesh.draws.push(RiverDraw {
                id,
                first_index: 0,
                index_count: mesh.indices.len() as u32,
            });
        }
        mesh
    }

    fn refresh_river(&self, _id: SurfaceId, mesh: &mut RiverMesh) -> Refreshed {
        field_sources::refresh(self.field, &self.drawn_cells(), mesh)
    }
}

impl WaterScene for WaterWorld {
    fn mesh_key(&self) -> MeshKey {
        FieldScene::of(self).mesh_key()
    }

    fn build_mesh(&self) -> WaterMesh {
        FieldScene::of(self).build_mesh()
    }

    fn level(&self, body: SurfaceId) -> Option<f32> {
        FieldScene::of(self).level(body)
    }

    fn clock(&self) -> f32 {
        FieldScene::of(self).clock()
    }

    fn swell(&self, body: SurfaceId) -> Swell {
        FieldScene::of(self).swell(body)
    }

    fn sea(&self) -> Option<Sea> {
        FieldScene::of(self).sea()
    }

    fn ripple_tiles(&self) -> Vec<RippleTileView<'_>> {
        FieldScene::of(self).ripple_views()
    }

    fn still_water(&self) -> Vec<WaterPatch> {
        FieldScene::of(self).still_water()
    }

    fn ocean_key(&self) -> MeshKey {
        FieldScene::of(self).ocean_key()
    }

    fn build_ocean(&self) -> WaterMesh {
        FieldScene::of(self).build_ocean()
    }

    fn river_key(&self) -> MeshKey {
        FieldScene::of(self).river_key()
    }

    fn build_river(&self, id: SurfaceId) -> RiverMesh {
        FieldScene::of(self).build_river(id)
    }

    fn refresh_river(&self, id: SurfaceId, mesh: &mut RiverMesh) -> Refreshed {
        FieldScene::of(self).refresh_river(id, mesh)
    }
}
