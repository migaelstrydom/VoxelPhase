//! The span graph: every span over the level, and which ones water can pass
//! between.
//!
//! Neighbours are implicit. Two spans in orthogonally adjacent columns are
//! joined if their intervals overlap above the higher floor; the lowest level
//! water passes at is their *saddle*. Diagonal steps exist only for routing,
//! and only where one of the two orthogonal columns between them joins both:
//! water never leaks through a corner that 4-connectivity would close.

use smallvec::SmallVec;

use super::span::{Column, Span, SpanChunk, SpanChunkCoord, SpanOwner, SpanRef, CHUNK_COLUMNS};

/// The least clearance between the higher floor and the lower ceiling for two
/// spans to be joined, in metres. A hairline crack passes no water.
pub const MIN_OPENING: f32 = 0.02;

/// A lattice step to a neighbouring column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Step {
    pub di: i32,
    pub dk: i32,
}

impl Step {
    pub const fn is_diagonal(self) -> bool {
        self.di != 0 && self.dk != 0
    }

    /// Horizontal length of the step, in columns.
    pub fn length(self) -> f32 {
        if self.is_diagonal() {
            std::f32::consts::SQRT_2
        } else {
            1.0
        }
    }
}

/// The four orthogonal steps, in a fixed order.
pub const ORTHOGONAL: [Step; 4] = [
    Step { di: 1, dk: 0 },
    Step { di: 0, dk: 1 },
    Step { di: -1, dk: 0 },
    Step { di: 0, dk: -1 },
];

/// All eight steps, orthogonal first.
pub const EIGHT: [Step; 8] = [
    Step { di: 1, dk: 0 },
    Step { di: 0, dk: 1 },
    Step { di: -1, dk: 0 },
    Step { di: 0, dk: -1 },
    Step { di: 1, dk: 1 },
    Step { di: -1, dk: 1 },
    Step { di: -1, dk: -1 },
    Step { di: 1, dk: -1 },
];

/// A span water can reach in one step, and the lowest level it passes at.
#[derive(Debug, Clone, Copy)]
pub struct Neighbour {
    pub span: SpanRef,
    pub saddle: f32,
    pub step: Step,
}

/// Where a span lives in the graph's storage: its chunk's slot and its index
/// in that chunk's flat span array. Stable until the chunk is rebuilt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SpanIndex {
    pub slot: u32,
    pub flat: u32,
}

/// Saddle between two spans in adjacent columns, if they are joined.
pub fn saddle(a: &Span, b: &Span) -> Option<f32> {
    let saddle = a.floor_min.max(b.floor_min);
    (a.ceiling.min(b.ceiling) > saddle + MIN_OPENING).then_some(saddle)
}

/// Every span over the level, in a dense grid of span chunks.
#[derive(Debug, Clone)]
pub struct SpanGraph {
    /// Lattice coordinate of `chunks[0]`.
    origin: SpanChunkCoord,
    /// Chunks along x and z.
    width: i32,
    depth: i32,
    /// Row-major, `width × depth`.
    chunks: Vec<SpanChunk>,
}

impl SpanGraph {
    /// An empty graph over the chunks from `min` to `max` inclusive.
    pub fn new(min: SpanChunkCoord, max: SpanChunkCoord) -> Self {
        let width = (max.x - min.x + 1).max(0);
        let depth = (max.z - min.z + 1).max(0);
        Self {
            origin: min,
            width,
            depth,
            chunks: vec![SpanChunk::default(); (width * depth) as usize],
        }
    }

    /// Every chunk coordinate in the graph, row-major.
    pub fn chunk_coords(&self) -> impl Iterator<Item = SpanChunkCoord> + '_ {
        (0..self.depth).flat_map(move |z| {
            (0..self.width).map(move |x| SpanChunkCoord {
                x: self.origin.x + x,
                z: self.origin.z + z,
            })
        })
    }

    /// The inclusive column range the graph covers, as (min, max).
    pub fn column_bounds(&self) -> (Column, Column) {
        (
            Column::new(self.origin.x * CHUNK_COLUMNS, self.origin.z * CHUNK_COLUMNS),
            Column::new(
                (self.origin.x + self.width) * CHUNK_COLUMNS - 1,
                (self.origin.z + self.depth) * CHUNK_COLUMNS - 1,
            ),
        )
    }

    pub fn contains_column(&self, column: Column) -> bool {
        self.chunk_index(column.chunk()).is_some()
    }

    /// Number of chunk slots: `slot` in a [`SpanIndex`] is below this.
    pub fn slot_count(&self) -> usize {
        self.chunks.len()
    }

    /// The slot of a chunk coordinate, if it is in the graph.
    pub fn slot(&self, coord: SpanChunkCoord) -> Option<usize> {
        self.chunk_index(coord)
    }

    /// The chunk in a slot.
    pub fn chunk_at_slot(&self, slot: usize) -> &SpanChunk {
        &self.chunks[slot]
    }

    /// Where a span lives in storage, if it exists.
    pub fn index_of(&self, span: SpanRef) -> Option<SpanIndex> {
        let slot = self.chunk_index(span.column.chunk())?;
        let chunk = &self.chunks[slot];
        self.check_generation(chunk, span);
        let flat = chunk.flat_index(span.column.local_index(), span.ordinal)?;
        Some(SpanIndex {
            slot: slot as u32,
            flat: flat as u32,
        })
    }

    fn chunk_index(&self, coord: SpanChunkCoord) -> Option<usize> {
        let x = coord.x - self.origin.x;
        let z = coord.z - self.origin.z;
        (x >= 0 && z >= 0 && x < self.width && z < self.depth)
            .then(|| (z * self.width + x) as usize)
    }

    pub fn chunk(&self, coord: SpanChunkCoord) -> Option<&SpanChunk> {
        self.chunk_index(coord).map(|i| &self.chunks[i])
    }

    /// Replace a chunk wholesale. The rasteriser's only way in.
    pub fn replace_chunk(&mut self, coord: SpanChunkCoord, chunk: SpanChunk) {
        let index = self
            .chunk_index(coord)
            .expect("span chunk outside the graph");
        self.chunks[index] = chunk;
    }

    /// A column's spans, bottom to top. Empty outside the graph and in columns
    /// with no floor at all.
    pub fn spans(&self, column: Column) -> &[Span] {
        match self.chunk(column.chunk()) {
            Some(chunk) => chunk.column_spans(column.local_index()),
            None => &[],
        }
    }

    /// Refs to a column's spans, bottom to top.
    pub fn refs(&self, column: Column) -> impl Iterator<Item = SpanRef> + '_ {
        let count = self.spans(column).len();
        (0..count).map(move |ordinal| self.make_ref(column, ordinal as u8))
    }

    /// A ref to a span, stamped with its chunk's current generation.
    pub fn make_ref(&self, column: Column, ordinal: u8) -> SpanRef {
        SpanRef {
            column,
            ordinal,
            #[cfg(debug_assertions)]
            generation: self.chunk(column.chunk()).map_or(0, SpanChunk::generation),
        }
    }

    /// The span a ref names.
    ///
    /// # Panics
    ///
    /// If the span does not exist, and in debug builds if its chunk has been
    /// rebuilt since the ref was taken.
    pub fn span(&self, span: SpanRef) -> &Span {
        self.try_span(span).expect("dangling SpanRef")
    }

    /// The span a ref names, if it exists.
    pub fn try_span(&self, span: SpanRef) -> Option<&Span> {
        let chunk = self.chunk(span.column.chunk())?;
        self.check_generation(chunk, span);
        chunk
            .column_spans(span.column.local_index())
            .get(span.ordinal as usize)
    }

    pub fn owner(&self, span: SpanRef) -> SpanOwner {
        let chunk = self
            .chunk(span.column.chunk())
            .expect("SpanRef outside the graph");
        self.check_generation(chunk, span);
        chunk.column_owners(span.column.local_index())[span.ordinal as usize]
    }

    pub fn owner_mut(&mut self, span: SpanRef) -> &mut SpanOwner {
        let index = self
            .chunk_index(span.column.chunk())
            .expect("SpanRef outside the graph");
        #[cfg(debug_assertions)]
        {
            let generation = self.chunks[index].generation();
            assert!(
                span.generation == u32::MAX || span.generation == generation,
                "stale SpanRef {:?}: chunk is at generation {generation}",
                span
            );
        }
        &mut self.chunks[index].column_owners_mut(span.column.local_index())[span.ordinal as usize]
    }

    #[allow(unused_variables)]
    fn check_generation(&self, chunk: &SpanChunk, span: SpanRef) {
        #[cfg(debug_assertions)]
        assert!(
            span.generation == u32::MAX || span.generation == chunk.generation(),
            "stale SpanRef {:?}: chunk is at generation {}",
            span,
            chunk.generation()
        );
    }

    /// The span whose band holds height `y` in a column: the lowest span whose
    /// ceiling is above `y`. `None` in a column with no spans.
    ///
    /// For a point in air this is the span containing it. For a point in solid
    /// ground it is the span resting on that ground.
    pub fn span_at(&self, column: Column, y: f32) -> Option<SpanRef> {
        let spans = self.spans(column);
        let ordinal = spans.iter().position(|s| s.ceiling > y)?;
        Some(self.make_ref(column, ordinal as u8))
    }

    /// The spans joined to `span` across one orthogonal step.
    pub fn orthogonal_neighbours(&self, span: SpanRef) -> SmallVec<[Neighbour; 6]> {
        let here = *self.span(span);
        let mut out = SmallVec::new();
        for step in ORTHOGONAL {
            self.push_joined(&here, span.column.offset(step.di, step.dk), step, &mut out);
        }
        out
    }

    /// The spans reachable in one step, orthogonal or open diagonal.
    ///
    /// A diagonal's saddle is the higher of the two orthogonal saddles through
    /// the lower of the two corners it can pass.
    pub fn neighbours(&self, span: SpanRef) -> SmallVec<[Neighbour; 12]> {
        let here = *self.span(span);
        let mut out: SmallVec<[Neighbour; 12]> = SmallVec::new();
        for step in ORTHOGONAL {
            let mut joined: SmallVec<[Neighbour; 6]> = SmallVec::new();
            self.push_joined(
                &here,
                span.column.offset(step.di, step.dk),
                step,
                &mut joined,
            );
            out.extend(joined);
        }
        for step in &EIGHT[4..] {
            let target_column = span.column.offset(step.di, step.dk);
            let targets = self.spans(target_column);
            for (ordinal, target) in targets.iter().enumerate() {
                let mut best: Option<f32> = None;
                for corner in [
                    span.column.offset(step.di, 0),
                    span.column.offset(0, step.dk),
                ] {
                    for mid in self.spans(corner) {
                        let (Some(a), Some(b)) = (saddle(&here, mid), saddle(mid, target)) else {
                            continue;
                        };
                        let through = a.max(b);
                        best = Some(best.map_or(through, |s: f32| s.min(through)));
                    }
                }
                if let Some(saddle) = best {
                    out.push(Neighbour {
                        span: self.make_ref(target_column, ordinal as u8),
                        saddle,
                        step: *step,
                    });
                }
            }
        }
        out
    }

    fn push_joined<A: smallvec::Array<Item = Neighbour>>(
        &self,
        here: &Span,
        column: Column,
        step: Step,
        out: &mut SmallVec<A>,
    ) {
        for (ordinal, there) in self.spans(column).iter().enumerate() {
            if let Some(saddle) = saddle(here, there) {
                out.push(Neighbour {
                    span: self.make_ref(column, ordinal as u8),
                    saddle,
                    step,
                });
            }
        }
    }

    /// Whether any orthogonal neighbour column of `column` is outside the
    /// graph or holds no span at all: water leaving that way leaves the world.
    pub fn borders_void(&self, column: Column) -> bool {
        ORTHOGONAL.iter().any(|step| {
            let next = column.offset(step.di, step.dk);
            !self.contains_column(next) || self.spans(next).is_empty()
        })
    }

    /// Total spans held.
    pub fn span_count(&self) -> usize {
        self.chunks.iter().map(SpanChunk::span_count).sum()
    }

    /// Every span ref in the graph, chunk by chunk.
    pub fn all_refs(&self) -> impl Iterator<Item = SpanRef> + '_ {
        self.chunk_coords().flat_map(move |coord| {
            let chunk = self.chunk(coord).expect("coord from this graph");
            chunk
                .flat_columns()
                .map(move |(local, ordinal)| self.make_ref(coord.column(local), ordinal))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::geometry::span::COLUMNS_PER_CHUNK;

    fn span(floor: f32, ceiling: f32) -> Span {
        Span {
            floor_c: floor,
            floor_min: floor,
            floor_max: floor,
            ceiling,
        }
    }

    /// One chunk whose columns are given by `f(i, k)`.
    fn graph(f: impl Fn(i32, i32) -> Vec<Span>) -> SpanGraph {
        let coord = SpanChunkCoord { x: 0, z: 0 };
        let mut graph = SpanGraph::new(coord, coord);
        let columns: Vec<Vec<Span>> = (0..COLUMNS_PER_CHUNK)
            .map(|local| {
                let c = coord.column(local);
                f(c.i, c.k)
            })
            .collect();
        graph.replace_chunk(coord, SpanChunk::from_columns(&columns, 1));
        graph
    }

    #[test]
    fn saddle_is_the_higher_floor_below_the_lower_ceiling() {
        assert_eq!(saddle(&span(1.0, 5.0), &span(2.0, 9.0)), Some(2.0));
        // A ceiling below the other's floor closes the join.
        assert_eq!(saddle(&span(1.0, 1.5), &span(2.0, 9.0)), None);
    }

    #[test]
    fn a_corner_closed_on_both_sides_blocks_the_diagonal() {
        // A checkerboard of low and walled columns: (0,0) and (1,1) are low,
        // (1,0) and (0,1) are capped at 0.5 m, below the low floors.
        let g = graph(|i, k| {
            if (i + k) % 2 == 0 {
                vec![span(0.0, f32::INFINITY)]
            } else {
                vec![span(-2.0, -1.0)]
            }
        });
        let from = g.make_ref(Column::new(4, 4), 0);
        let diagonals = g
            .neighbours(from)
            .into_iter()
            .filter(|n| n.step.is_diagonal())
            .count();
        assert_eq!(diagonals, 0);
    }

    #[test]
    fn an_open_corner_passes_the_diagonal_at_its_saddle() {
        let g = graph(|i, _| vec![span(if i == 5 { 3.0 } else { 0.0 }, f32::INFINITY)]);
        let from = g.make_ref(Column::new(4, 4), 0);
        let diagonal = g
            .neighbours(from)
            .into_iter()
            .find(|n| n.step == Step { di: 1, dk: 1 })
            .expect("open diagonal");
        // Both corners route through a column of floor 3 or 0; the (4,5)
        // corner has floor 0, but the target itself is at 3.
        assert_eq!(diagonal.saddle, 3.0);
    }

    #[test]
    fn span_at_picks_the_band() {
        let g = graph(|_, _| vec![span(0.0, 4.0), span(6.0, f32::INFINITY)]);
        let column = Column::new(1, 1);
        assert_eq!(g.span_at(column, 2.0).unwrap().ordinal, 0);
        // Inside the solid between the two, the upper span rests on it.
        assert_eq!(g.span_at(column, 5.0).unwrap().ordinal, 1);
        assert_eq!(g.span_at(column, 100.0).unwrap().ordinal, 1);
    }
}
