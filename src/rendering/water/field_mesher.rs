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
//! Every vertex carries its own height and the cell's velocity, so the river
//! shader's flow mapping runs on the field's currents. The mesh is built from
//! the field each time it moves. Past the map's open edges, the sea runs out
//! to the horizon as the network's ring.

use nalgebra::Vector2;
use smallvec::SmallVec;

use crate::water::field::WaterField;
use crate::water::geometry::{Column, SpanIndex, SpanRef, Step, COLUMN_SIZE, ORTHOGONAL};
use crate::water::ids::StoreId;
use crate::water::network::ReachEnds;
use crate::water::{WaterWorld, FIELD_BODY};

use super::basin_mesher::{MeshKey, WaterMesh, WaterScene};
use super::ocean_mesher::occupied_bounds;
use super::ocean_ring::{append_ring, MapRect};
use super::reach_mesher::{RiverDraw, RiverMesh, RiverState};
use super::vertex::RiverVertex;

/// Depth a cell must hold to be drawn, m.
const DRAWN_DEPTH: f64 = 0.002;

/// Depth at which the drawn surface is cut, m: the shoreline's contour.
/// Thinner water would lie on its floor and fight it for depth.
const SHORE_DEPTH: f32 = 0.005;

/// Farthest below the water a dry cell beside it counts, m, in a corner's
/// depth: a wall is a bank whose foot the water meets, so a channel between
/// high walls still has corners deep enough to draw.
const BANK_DEPTH: f64 = 0.1;

/// Surfaces further apart than this do not share a corner, m: a step this
/// high between two wet cells is water going over an edge, drawn as a
/// curtain. Lower steps are a sheet running down a slope, up to 200 %.
const JOIN: f64 = 1.0;

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

/// One cell meeting at a corner, as the corner sees it.
#[derive(Debug, Clone, Copy)]
struct Meeting {
    /// Its surface, if it is wet enough to draw.
    surface: Option<f64>,
    /// Its floor.
    bed: f64,
    /// Its depth.
    depth: f64,
}

/// A corner of a cell's quad.
#[derive(Debug, Clone, Copy)]
struct Corner {
    height: f64,
    /// Depth of water there: positive under water, negative over a bank
    /// standing above it. The shader cuts the surface where it falls below
    /// `SHORE_DEPTH`, so a shore is a contour, not the edge of a column.
    depth: f64,
}

impl FieldScene<'_> {
    fn cell(&self, span: SpanRef) -> Option<SpanIndex> {
        self.field
            .cells()
            .index(self.field.graph(), span.column, span.ordinal)
    }

    fn meeting(&self, span: SpanRef) -> Option<Meeting> {
        let cell = self.cell(span)?;
        let cells = self.field.cells();
        let depth = cells.depth(cell);
        Some(Meeting {
            surface: (depth > DRAWN_DEPTH).then(|| cells.surface(cell)),
            bed: cells.bed(cell) as f64,
            depth,
        })
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
    /// surface stands at `level`.
    ///
    /// Its height is the mean surface of the cell's group among the wet
    /// cells meeting there: sorted by surface, split wherever two stand more
    /// than `JOIN` apart, so every cell at a corner sees the same groups.
    /// Its depth is the mean over the cells meeting there of each one's
    /// depth, a dry one's taken as no more than zero and as far below as its
    /// floor stands above the water, down to `BANK_DEPTH`.
    fn corner(&self, span: SpanRef, level: f64, sx: i32, sz: i32) -> Corner {
        let x = self.across(span, sx, 0, level);
        let z = self.across(span, 0, sz, level);
        let diagonal = x
            .and_then(|n| self.across(n, 0, sz, level))
            .or_else(|| z.and_then(|n| self.across(n, sx, 0, level)));
        let meetings: SmallVec<[Meeting; 4]> = [Some(span), x, z, diagonal]
            .into_iter()
            .flatten()
            .filter_map(|s| self.meeting(s))
            .collect();

        let mut surfaces: SmallVec<[f64; 4]> = meetings.iter().filter_map(|m| m.surface).collect();
        surfaces.sort_by(f64::total_cmp);
        let mut group: SmallVec<[f64; 4]> = SmallVec::new();
        for s in surfaces {
            if group.last().is_some_and(|&last| s - last > JOIN) {
                if group.contains(&level) {
                    break;
                }
                group.clear();
            }
            group.push(s);
        }
        let height = group.iter().sum::<f64>() / group.len() as f64;
        let (low, high) = (group[0], group[group.len() - 1]);

        let (mut sum, mut count) = (0.0, 0.0);
        for m in &meetings {
            let depth = match m.surface {
                Some(s) if s >= low && s <= high => m.depth,
                Some(_) => continue,
                None => (height - m.bed).clamp(-BANK_DEPTH, 0.0),
            };
            sum += depth;
            count += 1.0;
        }
        Corner {
            height,
            depth: sum / count,
        }
    }

    /// Every wet cell joined to `span` across one orthogonal step, with its
    /// surface.
    fn wet_across(&self, span: SpanRef, di: i32, dk: i32) -> SmallVec<[f64; 2]> {
        self.field
            .graph()
            .joined(span, Step { di, dk })
            .into_iter()
            .filter_map(|n| {
                self.meeting(SpanRef::key(n.span.column, n.span.ordinal))?
                    .surface
            })
            .collect()
    }

    /// A cell's quad, and the skirts and curtains around it.
    fn cell_surface(&self, mesh: &mut RiverMesh, span: SpanRef, at: SpanIndex, level: f64) {
        let graph = self.field.graph();
        let column = span.column;
        let corners = [
            self.corner(span, level, -1, -1),
            self.corner(span, level, 1, -1),
            self.corner(span, level, 1, 1),
            self.corner(span, level, -1, 1),
        ];
        let flow = self.field.velocity(at);
        quad(mesh, column, corners, flow);

        for (di, dk) in [(1, 0), (0, 1)] {
            for other in self.wet_across(span, di, dk) {
                if (other - level).abs() > JOIN {
                    curtain(mesh, column, di, level, other, flow);
                }
            }
        }

        for step in ORTHOGONAL {
            let next = column.offset(step.di, step.dk);
            if !graph.contains_column(next) {
                continue;
            }
            let bank = graph
                .joined(span, step)
                .into_iter()
                .map(|n| SpanRef::key(n.span.column, n.span.ordinal))
                .find_map(|key| {
                    let m = self.meeting(key)?;
                    (m.surface.is_none() && (graph.span(key).floor_min as f64) < level).then_some(m)
                });
            if let Some(bank) = bank {
                let far = (level - bank.bed).min(0.0);
                quad(mesh, next, skirt(corners, step.di, step.dk, far), flow);
            }
        }
    }
}

/// A skirt's corners over the column beside a wet cell: the cell's corners
/// along the edge they share, their heights carried straight across so the
/// skirt follows the surface's slope along the bank, and the far edge as
/// deep as the bank's own floor makes it. Corners run (−x, −z), (+x, −z),
/// (+x, +z), (−x, +z).
fn skirt(cell: [Corner; 4], di: i32, dk: i32, far: f64) -> [Corner; 4] {
    let [a, b, c, d] = cell;
    let away = |corner: Corner| Corner {
        height: corner.height,
        depth: far,
    };
    match (di, dk) {
        (1, _) => [b, away(b), away(c), c],
        (-1, _) => [away(a), a, d, away(d)],
        (_, 1) => [d, c, away(c), away(d)],
        _ => [away(a), away(b), b, a],
    }
}

fn quad(mesh: &mut RiverMesh, column: Column, corners: [Corner; 4], flow: Vector2<f32>) {
    let (x0, z0) = column.min_corner();
    let offsets = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    let base = mesh.vertices.len() as u32;
    for ((cx, cz), corner) in offsets.iter().zip(corners) {
        mesh.vertices.push(RiverVertex {
            xz: Vector2::new(x0 + cx * COLUMN_SIZE, z0 + cz * COLUMN_SIZE),
            bed: corner.height as f32,
            depth: 0.0,
            along: corner.depth as f32,
            flow,
        });
    }
    mesh.indices
        .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
}

/// A vertical quad, both sides, down the edge a column shares with its +x
/// neighbour (`di` = 1) or its +z neighbour, between two surface heights.
fn curtain(mesh: &mut RiverMesh, column: Column, di: i32, a: f64, b: f64, flow: Vector2<f32>) {
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
    for ((x, z), y) in [(from, top), (to, top), (to, bottom), (from, bottom)] {
        mesh.vertices.push(RiverVertex {
            xz: Vector2::new(x, z),
            bed: y,
            depth: 0.0,
            along: 1.0,
            flow,
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

    fn river_state(&self, reach: StoreId) -> Option<RiverState> {
        FieldScene::of(self).river_state(reach)
    }
}
