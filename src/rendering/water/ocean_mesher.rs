//! The sea's surface: every column it owns inside the map, and a ring out to
//! the horizon beyond each open edge (§12).
//!
//! ```text
//!   span ownership ──▶ one quad per sea column, plus the shore ring: the
//!                      columns beside it whose ground stands above the sea,
//!                      under the terrain, so the depth test cuts the shore
//!   open edges ──────▶ the ring out to the horizon (`ocean_ring`)
//! ```
//!
//! Built at load and again only when the sea claims a lowland. Like a
//! basin, the level and swell are pushed per draw.

use rustc_hash::FxHashMap;

use crate::water::geometry::{Column, Outlets, SpanGraph, ORTHOGONAL};
use crate::water::ids::StoreId;
use crate::water::network::Ocean;

use super::basin_mesher::{append_columns, WaterMesh};
use super::ocean_ring::{append_ring, MapRect};

/// Append the sea's surface to `mesh`.
pub fn append_ocean(
    mesh: &mut WaterMesh,
    id: StoreId,
    ocean: &Ocean,
    graph: &SpanGraph,
    outlets: &Outlets,
) {
    let (columns, wet) = sea_columns(id, ocean, graph);
    append_columns(mesh, id, &columns, &wet);
    if let (Some(sea), Some((min, max))) = (outlets.sea(), occupied_bounds(graph)) {
        let map = MapRect {
            i0: min.i,
            k0: min.k,
            i1: max.i + 1,
            k1: max.k + 1,
        };
        append_ring(mesh, id, ocean.level, map, sea.open);
    }
}

/// Columns the sea owns, with the floor under each, and the shore ring;
/// and the owned columns' floors alone.
fn sea_columns(
    id: StoreId,
    ocean: &Ocean,
    graph: &SpanGraph,
) -> (Vec<(Column, f32)>, FxHashMap<Column, f32>) {
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
    let mut out: Vec<(Column, f32)> = floors.iter().map(|(&c, &f)| (c, f)).collect();
    ring.sort_by(|a, b| a.0.cmp(&b.0));
    ring.dedup_by(|a, b| a.0 == b.0);
    out.extend(ring);
    out.sort_by(|a, b| a.0.chunk().cmp(&b.0.chunk()).then(a.0.cmp(&b.0)));
    (out, floors)
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
