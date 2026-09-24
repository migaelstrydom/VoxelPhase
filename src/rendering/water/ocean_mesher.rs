//! The sea's surface: every column it owns inside the map, and a ring out to
//! the horizon beyond each open edge (§12).
//!
//! ```text
//!   span ownership ──▶ one quad per sea column, plus the shore ring: the
//!                      columns beside it whose ground stands above the sea,
//!                      under the terrain, so the depth test cuts the shore
//!   open edges ──────▶ strips of quads from the map's edge outwards, finer
//!                      near it and coarser towards the horizon
//! ```
//!
//! Built at load and again only when the sea claims a lowland. Like a
//! basin, the level and swell are pushed per draw.

use nalgebra::Vector2;
use rustc_hash::FxHashMap;

use crate::water::geometry::{Column, Outlets, SpanChunkCoord, SpanGraph, COLUMN_SIZE, ORTHOGONAL};
use crate::water::ids::StoreId;
use crate::water::network::Ocean;

use super::basin_mesher::{append_columns, WaterDraw, WaterMesh};
use super::vertex::BasinVertex;

/// How far the ring reaches beyond the map, m.
const HORIZON: f32 = 1024.0;

/// Distances from the map's edge at which the ring's rows start, m: fine near
/// the edge, where the swell shows, and coarse beyond.
const RING_ROWS: [f32; 9] = [0.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, HORIZON];

/// Width of the ring's quads along the edge, m. The map's edge is a whole
/// number of these, so the ring meets the sea inside it.
const RING_STEP: f32 = 8.0;

/// Depth given to the ring's floor: open water, deep enough that the swell
/// runs at full height and the tint is the sea's.
const RING_DEPTH: f32 = 100.0;

/// The ring's draws are keyed to a tile no ripple tile can have.
const RING_TILE: SpanChunkCoord = SpanChunkCoord {
    x: i32::MIN,
    z: i32::MIN,
};

/// Append the sea's surface to `mesh`.
pub fn append_ocean(
    mesh: &mut WaterMesh,
    id: StoreId,
    ocean: &Ocean,
    graph: &SpanGraph,
    outlets: &Outlets,
) {
    append_columns(mesh, id, &sea_columns(id, ocean, graph));
    if let Some(sea) = outlets.sea() {
        append_ring(mesh, id, ocean, graph, sea.open);
    }
}

/// Columns the sea owns, with the floor under each, and the shore ring.
fn sea_columns(id: StoreId, ocean: &Ocean, graph: &SpanGraph) -> Vec<(Column, f32)> {
    let mut floors: FxHashMap<Column, f32> = FxHashMap::default();
    for span in graph.all_refs() {
        if graph.owner(span).body == Some(id) {
            let floor = floors.entry(span.column).or_insert(f32::INFINITY);
            *floor = floor.min(graph.span(span).floor_min);
        }
    }
    // Beside the sea, a column whose ground holds the sea back: under the
    // terrain at the waterline. A column the sea could stand in but does not
    // own (a lowland behind a wall) is left out.
    let mut ring: Vec<(Column, f32)> = Vec::new();
    for (&column, &floor) in &floors {
        for step in ORTHOGONAL {
            let next = column.offset(step.di, step.dk);
            if floors.contains_key(&next) || !graph.contains_column(next) {
                continue;
            }
            let dry = graph
                .span_at(next, ocean.level)
                .is_none_or(|s| graph.span(s).floor_min >= ocean.level);
            if dry {
                ring.push((next, floor));
            }
        }
    }
    let mut out: Vec<(Column, f32)> = floors.into_iter().collect();
    ring.sort_by(|a, b| a.0.cmp(&b.0));
    ring.dedup_by(|a, b| a.0 == b.0);
    out.extend(ring);
    out.sort_by(|a, b| a.0.chunk().cmp(&b.0.chunk()).then(a.0.cmp(&b.0)));
    out
}

/// The first and last columns with any spans: the terrain's own edge, which
/// the graph's chunk-aligned bounds can overhang.
fn occupied_bounds(graph: &SpanGraph) -> Option<(Column, Column)> {
    let (min, max) = graph.column_bounds();
    let mut out: Option<(Column, Column)> = None;
    for k in min.k..=max.k {
        for i in min.i..=max.i {
            let column = Column::new(i, k);
            if graph.spans(column).is_empty() {
                continue;
            }
            out = Some(match out {
                None => (column, column),
                Some((lo, hi)) => (
                    Column::new(lo.i.min(i), lo.k.min(k)),
                    Column::new(hi.i.max(i), hi.k.max(k)),
                ),
            });
        }
    }
    out
}

/// Strips beyond each open edge: −x and +x span the map's depth, −z and +z
/// its width plus the corners.
fn append_ring(
    mesh: &mut WaterMesh,
    id: StoreId,
    ocean: &Ocean,
    graph: &SpanGraph,
    open: [bool; 4],
) {
    let Some((min, max)) = occupied_bounds(graph) else {
        return;
    };
    let (x0, z0) = (min.i as f32 * COLUMN_SIZE, min.k as f32 * COLUMN_SIZE);
    let (x1, z1) = (
        (max.i + 1) as f32 * COLUMN_SIZE,
        (max.k + 1) as f32 * COLUMN_SIZE,
    );
    let floor = ocean.level - RING_DEPTH;
    let first_index = mesh.indices.len() as u32;
    // (outward normal, span along the edge)
    let sides = [
        (open[0], (-1.0, 0.0), (z0, z1), x0),
        (open[1], (1.0, 0.0), (z0, z1), x1),
        (open[2], (0.0, -1.0), (x0 - HORIZON, x1 + HORIZON), z0),
        (open[3], (0.0, 1.0), (x0 - HORIZON, x1 + HORIZON), z1),
    ];
    for (is_open, (nx, nz), (a, b), edge) in sides {
        if !is_open {
            continue;
        }
        let along = ((b - a) / RING_STEP).round() as usize;
        for row in RING_ROWS.windows(2) {
            // A coarse row takes coarse steps along the edge too.
            let stride = ((row[1] - row[0]) / RING_STEP).max(1.0) as usize;
            let mut i = 0;
            while i < along {
                let (s0, s1) = (
                    a + i as f32 * RING_STEP,
                    a + (i + stride).min(along) as f32 * RING_STEP,
                );
                let corner = |s: f32, d: f32| {
                    if nx != 0.0 {
                        Vector2::new(edge + nx * d, s)
                    } else {
                        Vector2::new(s, edge + nz * d)
                    }
                };
                let base = mesh.vertices.len() as u32;
                for p in [
                    corner(s0, row[0]),
                    corner(s1, row[0]),
                    corner(s1, row[1]),
                    corner(s0, row[1]),
                ] {
                    mesh.vertices.push(BasinVertex { xz: p, floor });
                }
                mesh.indices.extend_from_slice(&[
                    base,
                    base + 1,
                    base + 2,
                    base,
                    base + 2,
                    base + 3,
                ]);
                i += stride;
            }
        }
    }
    if mesh.indices.len() as u32 > first_index {
        mesh.draws.push(WaterDraw {
            body: id,
            tile: RING_TILE,
            first_index,
            index_count: mesh.indices.len() as u32 - first_index,
        });
    }
}
