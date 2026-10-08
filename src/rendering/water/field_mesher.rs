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
//! the field each time it is drawn.

use nalgebra::Vector2;

use crate::water::field::WaterField;
use crate::water::geometry::{Column, SpanIndex, SpanRef, COLUMN_SIZE, ORTHOGONAL};
use crate::water::ids::StoreId;
use crate::water::network::ReachEnds;

use super::basin_mesher::{MeshKey, WaterMesh, WaterScene};
use super::reach_mesher::{RiverDraw, RiverMesh, RiverState};
use super::vertex::RiverVertex;

/// Depth a cell must hold to be drawn, m.
const DRAWN_DEPTH: f64 = 0.002;

/// Depth at which the drawn surface is cut, m: the shoreline's contour.
/// Thinner water would lie on its floor and fight it for depth.
const SHORE_DEPTH: f32 = 0.005;

/// Surfaces further apart than this do not share a corner, m: a step this
/// high between two wet cells is water going over an edge, drawn as a
/// curtain. Lower steps are a sheet running down a slope, up to 200 %.
const JOIN: f64 = 1.0;

/// The field, as the water renderer sees it.
pub struct FieldScene<'a> {
    pub field: &'a WaterField,
    /// Changes whenever the field has moved since the last draw.
    pub version: u32,
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
        let candidates: Vec<(SpanRef, Meeting)> = self
            .field
            .graph()
            .orthogonal_neighbours(span)
            .into_iter()
            .filter(|n| n.step.di == di && n.step.dk == dk)
            .filter_map(|n| {
                let key = SpanRef::key(n.span.column, n.span.ordinal);
                Some((key, self.meeting(key)?))
            })
            .collect();
        let near = |m: &Meeting| (m.surface.unwrap_or(m.bed) - level).abs();
        candidates
            .iter()
            .filter(|(_, m)| m.surface.is_some())
            .min_by(|a, b| near(&a.1).total_cmp(&near(&b.1)))
            .or_else(|| {
                candidates
                    .iter()
                    .min_by(|a, b| near(&a.1).total_cmp(&near(&b.1)))
            })
            .map(|(key, _)| *key)
    }

    /// The corner of `span`'s column on the (sx, sz) side, for a cell whose
    /// surface stands at `level`.
    ///
    /// Its height is the mean surface of the cell's group among the wet
    /// cells meeting there: sorted by surface, split wherever two stand more
    /// than `JOIN` apart, so every cell at a corner sees the same groups.
    /// Its depth is the mean over the cells meeting there of each one's
    /// depth, a dry one's taken as no more than zero and as far below as its
    /// floor stands above the water.
    fn corner(&self, span: SpanRef, level: f64, sx: i32, sz: i32) -> Corner {
        let x = self.across(span, sx, 0, level);
        let z = self.across(span, 0, sz, level);
        let diagonal = x
            .and_then(|n| self.across(n, 0, sz, level))
            .or_else(|| z.and_then(|n| self.across(n, sx, 0, level)));
        let meetings: Vec<Meeting> = [Some(span), x, z, diagonal]
            .into_iter()
            .flatten()
            .filter_map(|s| self.meeting(s))
            .collect();

        let mut surfaces: Vec<f64> = meetings.iter().filter_map(|m| m.surface).collect();
        surfaces.sort_by(f64::total_cmp);
        let mut group: Vec<f64> = Vec::new();
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

        let depths: Vec<f64> = meetings
            .iter()
            .filter_map(|m| match m.surface {
                Some(s) if s >= low && s <= high => Some(m.depth),
                Some(_) => None,
                None => Some((height - m.bed).min(0.0)),
            })
            .collect();
        let depth = depths.iter().sum::<f64>() / depths.len() as f64;
        Corner { height, depth }
    }

    /// Every wet cell joined to `span` across one orthogonal step, with its
    /// surface.
    fn wet_across(&self, span: SpanRef, di: i32, dk: i32) -> Vec<f64> {
        self.field
            .graph()
            .orthogonal_neighbours(span)
            .into_iter()
            .filter(|n| n.step.di == di && n.step.dk == dk)
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
                .orthogonal_neighbours(span)
                .into_iter()
                .filter(|n| n.step == step)
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

    fn level(&self, _body: StoreId) -> Option<f32> {
        None
    }

    fn river_key(&self) -> MeshKey {
        vec![(StoreId(0), self.version)]
    }

    fn build_rivers(&self) -> RiverMesh {
        let cells = self.field.cells();
        let mut mesh = RiverMesh::default();
        for (slot, chunk) in cells.chunks.iter().enumerate() {
            let first = mesh.indices.len() as u32;
            for (flat, (local, ordinal)) in chunk.cell_columns().enumerate() {
                let at = SpanIndex {
                    slot: slot as u32,
                    flat: flat as u32,
                };
                if chunk.depth[flat] <= DRAWN_DEPTH {
                    continue;
                }
                let span = SpanRef::key(chunk.coord.column(local), ordinal);
                self.cell_surface(&mut mesh, span, at, chunk.surface(flat));
            }
            let count = mesh.indices.len() as u32 - first;
            if count > 0 {
                mesh.draws.push(RiverDraw {
                    reach: StoreId(slot as u32),
                    first_index: first,
                    index_count: count,
                });
            }
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
