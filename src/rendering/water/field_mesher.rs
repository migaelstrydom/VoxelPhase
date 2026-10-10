//! The shallow-water field's surface, drawn through the river pipeline.
//!
//! ```text
//!   wet cell ──▶ one quad over its column, each corner the mean surface of
//!                its group of the wet cells meeting there: split where two
//!                surfaces stand more than JOIN apart
//!   two wet cells whose surfaces step more than JOIN
//!            ──▶ a curtain down their shared edge, upper surface to lower:
//!                water going over the edge
//!   dry cell beside one, its square dipping under that water
//!            ──▶ a skirt carrying the shared edge's heights across it
//! ```
//!
//! Each vertex also carries the water's depth there, negative over a bank,
//! in the river vertex's `along`; the draw's wet range cuts the surface at
//! `SHORE_DEPTH`, so a shoreline follows a contour across the quads instead
//! of the edges of columns.
//!
//! Every vertex carries its own height and a velocity, the mean of the cells
//! its corner joins, so the river shader's flow mapping runs on the field's
//! currents and runs smoothly across cells. A surface is built
//! from the graph when its shape changes; while only its water moves, every
//! vertex is read again from the cells it was built from (`field_sources`). Past the map's open edges, the sea runs out
//! to the horizon as the network's ring.

use nalgebra::Vector2;
use smallvec::SmallVec;

use crate::water::field::WaterField;
use crate::water::geometry::{Column, SpanIndex, SpanRef, Step, COLUMN_SIZE, ORTHOGONAL};
use crate::water::ids::StoreId;
use crate::water::network::ReachEnds;
use crate::water::{WaterWorld, FIELD_BODY};

use super::basin_mesher::{MeshKey, WaterMesh, WaterScene};
use super::field_sources::{
    self, corner_flow, gather, Corner, Meeting, VertexSource, DRAWN_DEPTH, JOIN,
};
use super::ocean_mesher::occupied_bounds;
use super::ocean_ring::{append_ring, MapRect};
use super::reach_mesher::{RiverDraw, RiverMesh, RiverState};
use super::vertex::RiverVertex;

/// Depth at which the drawn surface is cut, m: the shoreline's contour.
/// Thinner water would lie on its floor and fight it for depth.
const SHORE_DEPTH: f32 = 0.005;

/// The field, as the water renderer sees it.
pub struct FieldScene<'a> {
    pub field: &'a WaterField,
    /// Each chunk's surface version, by slot: changes whenever its surface
    /// has moved since it was last drawn.
    pub versions: &'a [u32],
    /// Seconds simulated: the clock the drawn currents run on.
    pub clock: f32,
}

impl<'a> FieldScene<'a> {
    /// The game's water, as drawn.
    pub fn of(world: &'a WaterWorld) -> Self {
        Self {
            field: world.field(),
            versions: world.versions(),
            clock: world.clock(),
        }
    }
}

impl FieldScene<'_> {
    fn cell(&self, span: SpanRef) -> Option<SpanIndex> {
        self.field
            .cells()
            .index(self.field.graph(), span.column, span.ordinal)
    }

    fn meeting(&self, span: SpanRef) -> Option<Meeting> {
        Some(Meeting::of(self.field.cells(), self.cell(span)?))
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
        let meetings: SmallVec<[Meeting; 4]> = cells
            .iter()
            .map(|&c| Meeting::of(self.field.cells(), c))
            .collect();
        let (corner, shape) = gather(&meetings, level);
        let flow = corner_flow(self.field, &cells, shape);
        (corner, flow, VertexSource::Corner { cells, shape })
    }

    /// Every wet cell joined to `span` across one orthogonal step.
    fn wet_across(&self, span: SpanRef, di: i32, dk: i32) -> SmallVec<[SpanIndex; 2]> {
        self.field
            .graph()
            .joined(span, Step { di, dk })
            .into_iter()
            .filter_map(|n| self.cell(SpanRef::key(n.span.column, n.span.ordinal)))
            .filter(|&c| Meeting::of(self.field.cells(), c).surface.is_some())
            .collect()
    }

    /// A cell's quad, and the skirts and curtains around it.
    fn cell_surface(&self, mesh: &mut RiverMesh, span: SpanRef, at: SpanIndex, level: f64) {
        let graph = self.field.graph();
        let cells = self.field.cells();
        let column = span.column;
        let built =
            [(-1, -1), (1, -1), (1, 1), (-1, 1)].map(|(sx, sz)| self.corner(span, level, sx, sz));
        let corners = built.clone().map(|(corner, _, _)| corner);
        let flows = built.clone().map(|(_, flow, _)| flow);
        let flow = self.field.velocity(at);
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

        for (di, dk) in [(1, 0), (0, 1)] {
            for other in self.wet_across(span, di, dk) {
                let other_level = cells.surface(other);
                if (other_level - level).abs() > JOIN {
                    mesh.sources.reads(cells, other);
                    curtain(mesh, column, di, (at, level), (other, other_level), flow);
                }
            }
        }

        for step in ORTHOGONAL {
            let next = column.offset(step.di, step.dk);
            if !graph.contains_column(next) {
                continue;
            }
            let dry: SmallVec<[(SpanIndex, Meeting, f32); 2]> = graph
                .joined(span, step)
                .into_iter()
                .map(|n| SpanRef::key(n.span.column, n.span.ordinal))
                .filter_map(|key| {
                    let cell = self.cell(key)?;
                    let m = Meeting::of(cells, cell);
                    m.surface
                        .is_none()
                        .then_some((cell, m, graph.span(key).floor_min))
                })
                .collect();
            let bank = dry.iter().find(|d| (d.2 as f64) < level);
            if bank.is_none() {
                if let Some(lowest) = dry.iter().map(|d| d.2).min_by(f32::total_cmp) {
                    mesh.sources.high_banks.push((at, lowest));
                }
            }
            if let Some(&(bank_cell, bank, floor)) = bank {
                mesh.sources.reads(cells, bank_cell);
                let far = (level - bank.bed).min(0.0);
                let ends = skirt(step.di, step.dk);
                let skirt_corners = ends.map(|(i, away)| Corner {
                    height: corners[i].height,
                    depth: if away { far } else { corners[i].depth },
                });
                let sources = ends.map(|(i, away)| {
                    let from = first + i as u32;
                    if away {
                        VertexSource::Bank {
                            from,
                            own: at,
                            bank: bank_cell,
                            floor,
                        }
                    } else {
                        VertexSource::Edge { from }
                    }
                });
                quad(
                    mesh,
                    next,
                    skirt_corners,
                    ends.map(|(i, _)| flows[i]),
                    sources,
                );
            }
        }
    }
}

/// A skirt's corners over the column beside a wet cell, as the cell's
/// corners it takes and whether each lies on the far edge: the edge they
/// share keeps the cell's corners, their heights carried straight across so
/// the skirt follows the surface's slope along the bank, and the far edge is
/// as deep as the bank's own floor makes it. Corners run (−x, −z), (+x, −z),
/// (+x, +z), (−x, +z).
fn skirt(di: i32, dk: i32) -> [(usize, bool); 4] {
    match (di, dk) {
        (1, _) => [(1, false), (1, true), (2, true), (2, false)],
        (-1, _) => [(0, true), (0, false), (3, false), (3, true)],
        (_, 1) => [(3, false), (2, false), (2, true), (3, true)],
        _ => [(0, true), (1, true), (1, false), (0, false)],
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
            bed: corner.height as f32,
            depth: 0.0,
            along: corner.depth as f32,
            flow,
        });
    }
    mesh.sources.vertices.extend(sources);
    mesh.indices
        .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
}

/// A vertical quad, both sides, down the edge a column shares with its +x
/// neighbour (`di` = 1) or its +z neighbour, between the surfaces of the
/// column's cell and the cell across that edge.
fn curtain(
    mesh: &mut RiverMesh,
    column: Column,
    di: i32,
    (own, a): (SpanIndex, f64),
    (other, b): (SpanIndex, f64),
    flow: Vector2<f32>,
) {
    let (x0, z0) = column.min_corner();
    let (from, to) = if di != 0 {
        let x = x0 + COLUMN_SIZE;
        ((x, z0), (x, z0 + COLUMN_SIZE))
    } else {
        let z = z0 + COLUMN_SIZE;
        ((x0, z), (x0 + COLUMN_SIZE, z))
    };
    let (top, bottom) = (a.max(b) as f32, a.min(b) as f32);
    let base = mesh.vertices.len() as u32;
    for ((x, z), y, high) in [
        (from, top, true),
        (to, top, true),
        (to, bottom, false),
        (from, bottom, false),
    ] {
        mesh.vertices.push(RiverVertex {
            xz: Vector2::new(x, z),
            bed: y,
            depth: 0.0,
            along: 1.0,
            flow,
        });
        mesh.sources.vertices.push(VertexSource::Curtain {
            own,
            other,
            top: high,
        });
    }
    mesh.indices.extend_from_slice(&[
        base,
        base + 1,
        base + 2,
        base,
        base + 2,
        base + 3,
        base,
        base + 2,
        base + 1,
        base,
        base + 3,
        base + 2,
    ]);
}

impl WaterScene for FieldScene<'_> {
    fn mesh_key(&self) -> MeshKey {
        Vec::new()
    }

    fn build_mesh(&self) -> WaterMesh {
        WaterMesh::default()
    }

    /// The sea's level, for its ring past the map's edges.
    fn level(&self, body: StoreId) -> Option<f32> {
        self.field
            .sea()
            .filter(|_| body == FIELD_BODY)
            .map(|sea| sea.level)
    }

    fn clock(&self) -> f32 {
        self.clock
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
            .map(|(slot, &v)| (StoreId(slot as u32), v))
            .collect()
    }

    fn build_river(&self, id: StoreId) -> RiverMesh {
        let slot = id.0 as usize;
        let mut mesh = RiverMesh::default();
        let Some(chunk) = self.field.cells().chunks.get(slot) else {
            return mesh;
        };
        mesh.sources.slot = slot as u32;
        mesh.sources.drawn = chunk.depth.iter().map(|&d| d > DRAWN_DEPTH).collect();
        for (flat, (local, ordinal)) in chunk.cell_columns().enumerate() {
            if chunk.depth[flat] <= DRAWN_DEPTH {
                continue;
            }
            let at = SpanIndex {
                slot: slot as u32,
                flat: flat as u32,
            };
            let span = SpanRef::key(chunk.coord.column(local), ordinal);
            self.cell_surface(&mut mesh, span, at, chunk.surface(flat));
        }
        if !mesh.indices.is_empty() {
            mesh.draws.push(RiverDraw {
                reach: id,
                first_index: 0,
                index_count: mesh.indices.len() as u32,
            });
        }
        mesh
    }

    fn refresh_river(&self, _id: StoreId, mesh: &mut RiverMesh) -> bool {
        field_sources::refresh(self.field, mesh)
    }

    fn river_state(&self, _reach: StoreId) -> Option<RiverState> {
        Some(RiverState {
            depth_scale: 1.0,
            speed_scale: 1.0,
            tail: SHORE_DEPTH,
            front: f32::INFINITY,
            ends: ReachEnds {
                upstream: 0.0,
                downstream: 0.0,
            },
            length: 1.0e6,
        })
    }
}

impl WaterScene for WaterWorld {
    fn mesh_key(&self) -> MeshKey {
        FieldScene::of(self).mesh_key()
    }

    fn build_mesh(&self) -> WaterMesh {
        FieldScene::of(self).build_mesh()
    }

    fn level(&self, body: StoreId) -> Option<f32> {
        FieldScene::of(self).level(body)
    }

    fn clock(&self) -> f32 {
        FieldScene::of(self).clock()
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

    fn build_river(&self, id: StoreId) -> RiverMesh {
        FieldScene::of(self).build_river(id)
    }

    fn refresh_river(&self, id: StoreId, mesh: &mut RiverMesh) -> bool {
        FieldScene::of(self).refresh_river(id, mesh)
    }

    fn river_state(&self, reach: StoreId) -> Option<RiverState> {
        FieldScene::of(self).river_state(reach)
    }
}
