//! Filling cells to a level at load: authored pools, and the sea from the
//! map's open edges.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use rustc_hash::FxHashSet;

use crate::water::geometry::{Column, SpanGraph, SpanRef, ORTHOGONAL};

use super::cells::FieldCells;
use super::faces::SeaBoundary;

/// Fill the depression holding `seeds` up to `level`. Water already standing
/// higher is left as it is.
///
/// The depression grows lowest saddle first (Priority-Flood), so it takes
/// the ground in the order water rising from the seeds would reach it. A
/// span lower than the highest saddle crossed to reach it lies beyond a
/// crest: water above that crest runs out over it, and the flow law takes
/// it from there, so the fill stops at the crest instead of standing at
/// `level` down the far side.
pub fn flood(graph: &SpanGraph, cells: &mut FieldCells, seeds: &[SpanRef], level: f32) {
    let mut seen: FxHashSet<SpanRef> = FxHashSet::default();
    let mut queue: BinaryHeap<Reverse<(Height, SpanRef)>> = BinaryHeap::new();
    for &seed in seeds {
        let seed = bottom(graph, seed);
        let floor = graph.span(seed).floor_c;
        if floor < level && seen.insert(seed) {
            queue.push(Reverse((Height(floor), seed)));
        }
    }
    let mut crest = f32::NEG_INFINITY;
    while let Some(Reverse((Height(entry), span))) = queue.pop() {
        if entry >= level {
            break;
        }
        let here = *graph.span(span);
        if here.floor_c < crest - CREST_DROP {
            continue;
        }
        crest = crest.max(entry);
        if let Some(at) = cells.index(graph, span.column, span.ordinal) {
            let depth = (level.min(here.ceiling) - here.floor_c).max(0.0) as f64;
            let cell = cells.depth_mut(at);
            *cell = cell.max(depth);
        }
        for next in graph.orthogonal_neighbours(span) {
            let key = SpanRef::key(next.span.column, next.span.ordinal);
            let floor = graph.span(key).floor_c;
            if floor < level && seen.insert(key) {
                queue.push(Reverse((Height(next.saddle.max(floor)), key)));
            }
        }
    }
}

/// The bottom of the hollow a span lies in: downhill, neighbour by
/// neighbour, until no neighbour lies lower. A seed is any point of a pool,
/// not its deepest, and the flood has to start from there.
fn bottom(graph: &SpanGraph, mut span: SpanRef) -> SpanRef {
    loop {
        let floor = graph.span(span).floor_c;
        let lowest = graph
            .orthogonal_neighbours(span)
            .into_iter()
            .map(|n| SpanRef::key(n.span.column, n.span.ordinal))
            .min_by(|a, b| graph.span(*a).floor_c.total_cmp(&graph.span(*b).floor_c));
        match lowest {
            Some(next) if graph.span(next).floor_c < floor => span = next,
            _ => return span,
        }
    }
}

/// How far below the crest a span may lie and still be in the depression,
/// m: the floor's own unevenness within a column, not a way out.
const CREST_DROP: f32 = 0.05;

/// A height that orders totally, for the flood's queue.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Height(f32);

impl Eq for Height {}

impl PartialOrd for Height {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Height {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}

/// The span a pool seeded at (x, z) with its surface at `level` stands in:
/// the one whose band holds the level.
pub fn pool_seed(graph: &SpanGraph, x: f32, z: f32, level: f32) -> Option<SpanRef> {
    let column = Column::containing(x, z);
    if !graph.contains_column(column) {
        return None;
    }
    let span = graph.span_at(column, level)?;
    Some(SpanRef::key(column, span.ordinal))
}

/// Every span on an edge of the map that opens onto the sea: beside a
/// column outside the graph, or one with no spans at all, on an open side.
pub fn sea_seeds(graph: &SpanGraph, sea: &SeaBoundary) -> Vec<SpanRef> {
    let (min, max) = graph.column_bounds();
    let mut seeds = Vec::new();
    for k in min.k..=max.k {
        for i in min.i..=max.i {
            let column = Column::new(i, k);
            let on_open_edge = ORTHOGONAL.iter().enumerate().any(|(d, step)| {
                let next = column.offset(step.di, step.dk);
                sea.open[d] && (!graph.contains_column(next) || graph.spans(next).is_empty())
            });
            if on_open_edge {
                seeds.extend((0..graph.spans(column).len()).map(|o| SpanRef::key(column, o as u8)));
            }
        }
    }
    seeds
}
