//! Filling cells to a level at load: authored pools, each no higher than it
//! can keep; the sea from the map's open edges; and the hollows a source's
//! water fills on its way down.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use rustc_hash::FxHashSet;

use crate::water::geometry::{Column, SpanGraph, SpanRef, COLUMN_SIZE, ORTHOGONAL};

use super::cells::FieldCells;
use super::faces::SeaBoundary;

/// The spill level of every span: the lowest surface water standing on it
/// could keep, since higher water finds a way off the map. Found lowest
/// first inward from every way off (Priority-Flood): a span's level is the
/// highest floor or saddle on the lowest path from it to the map's edge.
/// A span with no way off has none, and holds any level.
pub struct SpillLevels {
    /// By slot, then flat index, as the cells are laid out.
    levels: Vec<Vec<f32>>,
}

impl SpillLevels {
    /// Water leaves over the map's edges and into holes through it; with a
    /// sea, it can fall no lower than sea level there.
    pub fn find(graph: &SpanGraph, cells: &FieldCells, sea: Option<&SeaBoundary>) -> Self {
        let mut levels: Vec<Vec<f32>> = cells
            .chunks
            .iter()
            .map(|c| vec![f32::INFINITY; c.depth.len()])
            .collect();
        let floor_at_edge = sea.map_or(f32::NEG_INFINITY, |sea| sea.level);
        let mut queue: BinaryHeap<Reverse<(Height, SpanRef)>> = BinaryHeap::new();
        for span in graph.all_refs() {
            if on_map_edge(graph, span.column) {
                let level = graph.span(span).floor_c.max(floor_at_edge);
                queue.push(Reverse((Height(level), span)));
            }
        }
        while let Some(Reverse((Height(level), span))) = queue.pop() {
            let Some(at) = cells.index(graph, span.column, span.ordinal) else {
                continue;
            };
            let slot = &mut levels[at.slot as usize][at.flat as usize];
            if *slot <= level {
                continue;
            }
            *slot = level;
            for next in graph.orthogonal_neighbours(span) {
                let key = SpanRef::key(next.span.column, next.span.ordinal);
                let through = level.max(next.saddle).max(graph.span(key).floor_c);
                queue.push(Reverse((Height(through), key)));
            }
        }
        Self { levels }
    }

    /// The level water in a span can stand at before it runs off: infinite
    /// in a pocket with no way out.
    pub fn at(&self, cells: &FieldCells, graph: &SpanGraph, span: SpanRef) -> f32 {
        cells
            .index(graph, span.column, span.ordinal)
            .map_or(f32::INFINITY, |at| {
                self.levels[at.slot as usize][at.flat as usize]
            })
    }
}

/// Fill the pool seeded at `seed` to `level`, or to the spill level where
/// that is lower: water above it would run off. Water already standing
/// higher is left as it is.
pub fn fill_pool(
    graph: &SpanGraph,
    cells: &mut FieldCells,
    spill: &SpillLevels,
    seed: SpanRef,
    level: f32,
) {
    let surface = |cells: &FieldCells, span: SpanRef| level.min(spill.at(cells, graph, span));
    fill_under(graph, cells, &[bottom(graph, seed)], surface);
}

/// Fill everything joined to the map's open edges below sea level.
pub fn fill_sea(graph: &SpanGraph, cells: &mut FieldCells, sea: &SeaBoundary) {
    fill_under(graph, cells, &sea_seeds(graph, sea), |_, _| sea.level);
}

/// Fill every span joined to `seeds` under water: each to its `surface`,
/// across saddles below the surface on the side water comes from.
fn fill_under(
    graph: &SpanGraph,
    cells: &mut FieldCells,
    seeds: &[SpanRef],
    surface: impl Fn(&FieldCells, SpanRef) -> f32,
) {
    let mut seen: FxHashSet<SpanRef> = FxHashSet::default();
    let mut stack: Vec<SpanRef> = Vec::new();
    for &seed in seeds {
        if graph.span(seed).floor_c < surface(cells, seed) && seen.insert(seed) {
            stack.push(seed);
        }
    }
    while let Some(span) = stack.pop() {
        let here = *graph.span(span);
        let level = surface(cells, span);
        if let Some(at) = cells.index(graph, span.column, span.ordinal) {
            let depth = (level.min(here.ceiling) - here.floor_c).max(0.0) as f64;
            let cell = cells.depth_mut(at);
            *cell = cell.max(depth);
        }
        for next in graph.orthogonal_neighbours(span) {
            let key = SpanRef::key(next.span.column, next.span.ordinal);
            if next.saddle < level
                && graph.span(key).floor_c < surface(cells, key)
                && seen.insert(key)
            {
                stack.push(key);
            }
        }
    }
}

/// Fill every hollow water running from `start` at `discharge` m³/s fills
/// on its way down, each to its crest and the head over it that carries
/// the water away, and wet the channels between them: what a source does to
/// the ground below it, given time.
///
/// Each hollow grows lowest saddle first from its bottom (Priority-Flood),
/// until a span lies beyond the crest; the water runs on down from that
/// span to the next hollow. It ends where the water leaves the map, at a
/// hollow with no way out (left as it is: it would fill to its roof), or
/// after `MAX_HOLLOWS`. The head is a guess, made as for a weir over the
/// crest's lip: the run that follows settles it.
pub fn fill_fed(
    graph: &SpanGraph,
    cells: &mut FieldCells,
    start: SpanRef,
    discharge: f32,
    manning_n: f32,
) {
    const MAX_HOLLOWS: usize = 64;
    let mut start = start;
    // The channel a hollow's water runs on down is taken as wide as the lip
    // it left over; from the source to the first, as one column.
    let mut width = COLUMN_SIZE;
    for _ in 0..MAX_HOLLOWS {
        let path = descent(graph, start);
        wet_channel(graph, cells, &path, discharge, width, manning_n);
        let seed = *path.last().expect("a descent starts where it is");
        let mut seen: FxHashSet<SpanRef> = FxHashSet::default();
        let mut queue: BinaryHeap<Reverse<(Height, SpanRef)>> = BinaryHeap::new();
        let mut members = Vec::new();
        let mut crest = f32::NEG_INFINITY;
        seen.insert(seed);
        queue.push(Reverse((Height(graph.span(seed).floor_c), seed)));
        let mut spill = None;
        let mut beyond: FxHashSet<SpanRef> = FxHashSet::default();
        let mut off_map = false;
        while let Some(Reverse((Height(entry), span))) = queue.pop() {
            // Once the crest is found, carry on just over it, to find the
            // whole lip: shore spans at the crest, and the ground past it.
            if spill.is_some() && entry > crest + LIP_BAND {
                break;
            }
            if graph.span(span).floor_c < crest - CREST_DROP {
                spill.get_or_insert(span);
                beyond.insert(span);
                continue;
            }
            if spill.is_none() {
                crest = crest.max(entry);
            }
            members.push(span);
            if on_map_edge(graph, span.column) {
                off_map = true;
                break;
            }
            for next in graph.orthogonal_neighbours(span) {
                let key = SpanRef::key(next.span.column, next.span.ordinal);
                if seen.insert(key) {
                    let floor = graph.span(key).floor_c;
                    queue.push(Reverse((Height(next.saddle.max(floor)), key)));
                }
            }
        }
        if spill.is_none() && !off_map {
            return;
        }
        // Off the map's edge the water just leaves: there is no lip to stand
        // a head over.
        width = lip_width(graph, &members, &beyond, crest);
        let head = if off_map {
            0.0
        } else {
            weir_head(discharge, width)
        };
        let level = crest + head;
        for &span in &members {
            let here = graph.span(span);
            if let Some(at) = cells.index(graph, span.column, span.ordinal) {
                let depth = (level.min(here.ceiling) - here.floor_c).max(0.0) as f64;
                let cell = cells.depth_mut(at);
                *cell = cell.max(depth);
            }
        }
        match spill {
            Some(next) => start = next,
            None => return,
        }
    }
}

/// How wide a hollow's lip is, m: its spans at the crest beside ground
/// beyond it. At least one column.
fn lip_width(
    graph: &SpanGraph,
    members: &[SpanRef],
    beyond: &FxHashSet<SpanRef>,
    crest: f32,
) -> f32 {
    let lips = members
        .iter()
        .filter(|&&span| (graph.span(span).floor_c - crest).abs() <= LIP_BAND)
        .filter(|&&span| {
            graph
                .orthogonal_neighbours(span)
                .into_iter()
                .any(|n| beyond.contains(&SpanRef::key(n.span.column, n.span.ordinal)))
        })
        .count();
    lips.max(1) as f32 * COLUMN_SIZE
}

/// How far from the crest a span may stand and still be part of the lip, m.
const LIP_BAND: f32 = 0.1;

/// Head over a lip `width` m wide carrying `discharge` m³/s, m: the water
/// crosses at critical depth, two thirds of the head.
fn weir_head(discharge: f32, width: f32) -> f32 {
    const G: f32 = 9.81;
    let q = discharge / width;
    1.5 * (q * q / G).cbrt()
}

/// Whether a column lies on the map's edge: beside one outside the graph or
/// with no spans, where water leaves.
fn on_map_edge(graph: &SpanGraph, column: Column) -> bool {
    ORTHOGONAL.iter().any(|step| {
        let next = column.offset(step.di, step.dk);
        !graph.contains_column(next) || graph.spans(next).is_empty()
    })
}

/// The bottom of the hollow a span lies in: downhill, neighbour by
/// neighbour, until no neighbour lies lower. A seed is any point of a pool,
/// not its deepest, and the flood has to start from there.
fn bottom(graph: &SpanGraph, span: SpanRef) -> SpanRef {
    *descent(graph, span)
        .last()
        .expect("a descent starts where it is")
}

/// The way down from a span to the bottom of its hollow, both ends
/// included: steepest neighbour first, as water runs.
fn descent(graph: &SpanGraph, mut span: SpanRef) -> Vec<SpanRef> {
    let mut path = vec![span];
    loop {
        let floor = graph.span(span).floor_c;
        let lowest = graph
            .orthogonal_neighbours(span)
            .into_iter()
            .map(|n| SpanRef::key(n.span.column, n.span.ordinal))
            .min_by(|a, b| graph.span(*a).floor_c.total_cmp(&graph.span(*b).floor_c));
        match lowest {
            Some(next) if graph.span(next).floor_c < floor => {
                span = next;
                path.push(span);
            }
            _ => return path,
        }
    }
}

/// Wet a channel running down `path` with `discharge` m³/s across `width`
/// m at its normal depth (Manning, roughness `manning_n`): what the water
/// running down it stands at once it runs steady. Each span of the path
/// takes the depth for the slope it is reached down, and the spans beside it
/// within half the width, as far as that water covers them.
fn wet_channel(
    graph: &SpanGraph,
    cells: &mut FieldCells,
    path: &[SpanRef],
    discharge: f32,
    width: f32,
    manning_n: f32,
) {
    const LEAST_SLOPE: f32 = 1.0e-3;
    let q = discharge / width;
    let reach = (0.5 * width / COLUMN_SIZE).round() as i32;
    for pair in path.windows(2) {
        let (from, to) = (graph.span(pair[0]).floor_c, graph.span(pair[1]).floor_c);
        let slope = ((from - to) / COLUMN_SIZE).max(LEAST_SLOPE);
        let level = to + (q * manning_n / slope.sqrt()).powf(0.6);
        let centre = pair[1].column;
        let mut seen: FxHashSet<SpanRef> = FxHashSet::default();
        let mut stack = vec![pair[1]];
        seen.insert(pair[1]);
        while let Some(span) = stack.pop() {
            let here = graph.span(span);
            if let Some(at) = cells.index(graph, span.column, span.ordinal) {
                let depth = (level.min(here.ceiling) - here.floor_c).max(0.0) as f64;
                let cell = cells.depth_mut(at);
                *cell = cell.max(depth);
            }
            for n in graph.orthogonal_neighbours(span) {
                let key = SpanRef::key(n.span.column, n.span.ordinal);
                let near = (key.column.i - centre.i)
                    .abs()
                    .max((key.column.k - centre.k).abs());
                if near <= reach && graph.span(key).floor_c < level && seen.insert(key) {
                    stack.push(key);
                }
            }
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
fn sea_seeds(graph: &SpanGraph, sea: &SeaBoundary) -> Vec<SpanRef> {
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
