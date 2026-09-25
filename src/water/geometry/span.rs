//! Spans: the vertical intervals of air that water can rest in.
//!
//! The world is cut into a global lattice of 0.5 m columns. Each column holds
//! the intervals of air stacked in it, bottom to top, each from a floor to a
//! ceiling. A column over a floating island above the sea holds two.
//!
//! ```text
//!   Column (i, k)  covers x ∈ [0.5 i, 0.5 i + 0.5), z likewise;
//!                  its structure is sampled at the centre (0.25 + 0.5 i, …)
//!   SpanChunk      16 × 16 columns = 8 m × 8 m, spans in CSR order
//!   SpanRef        (column, ordinal): the ordinal-th span up the column
//! ```

use crate::water::ids::StoreId;

/// Width of a column, in metres.
pub const COLUMN_SIZE: f32 = 0.5;

/// Columns along each side of a span chunk.
pub const CHUNK_COLUMNS: i32 = 16;

/// Columns in a span chunk.
pub const COLUMNS_PER_CHUNK: usize = (CHUNK_COLUMNS * CHUNK_COLUMNS) as usize;

/// A column of the global water lattice.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Column {
    /// Row, along z. First so that the derived order is row-major.
    pub k: i32,
    /// Position along x.
    pub i: i32,
}

impl Column {
    pub fn new(i: i32, k: i32) -> Self {
        Self { k, i }
    }

    /// The column containing a world (x, z).
    pub fn containing(x: f32, z: f32) -> Self {
        Self::new(
            (x / COLUMN_SIZE).floor() as i32,
            (z / COLUMN_SIZE).floor() as i32,
        )
    }

    /// World (x, z) of the column's centre, where its structure is sampled.
    pub fn centre(self) -> (f32, f32) {
        (
            (self.i as f32 + 0.5) * COLUMN_SIZE,
            (self.k as f32 + 0.5) * COLUMN_SIZE,
        )
    }

    /// World (x, z) of the column's minimum corner.
    pub fn min_corner(self) -> (f32, f32) {
        (self.i as f32 * COLUMN_SIZE, self.k as f32 * COLUMN_SIZE)
    }

    /// The column offset by a lattice step.
    pub fn offset(self, di: i32, dk: i32) -> Self {
        Self::new(self.i + di, self.k + dk)
    }

    /// The span chunk holding this column.
    pub fn chunk(self) -> SpanChunkCoord {
        SpanChunkCoord {
            x: self.i.div_euclid(CHUNK_COLUMNS),
            z: self.k.div_euclid(CHUNK_COLUMNS),
        }
    }

    /// This column's index within its chunk, row-major.
    pub fn local_index(self) -> usize {
        let li = self.i.rem_euclid(CHUNK_COLUMNS);
        let lk = self.k.rem_euclid(CHUNK_COLUMNS);
        (lk * CHUNK_COLUMNS + li) as usize
    }
}

/// Lattice coordinate of a span chunk.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SpanChunkCoord {
    pub x: i32,
    pub z: i32,
}

impl SpanChunkCoord {
    /// The column at a local index of this chunk.
    pub fn column(self, local: usize) -> Column {
        let local = local as i32;
        Column::new(
            self.x * CHUNK_COLUMNS + local % CHUNK_COLUMNS,
            self.z * CHUNK_COLUMNS + local / CHUNK_COLUMNS,
        )
    }

    /// Every column of this chunk, in local-index order.
    pub fn columns(self) -> impl Iterator<Item = Column> {
        (0..COLUMNS_PER_CHUNK).map(move |local| self.column(local))
    }
}

/// A vertical interval of air in a column, from a floor up to a ceiling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Span {
    /// The floor at the column centre. The structure, the remap and the
    /// removal-only invariant are all defined here.
    pub floor_c: f32,
    /// Lowest point of this span's floor anywhere in the column's square.
    /// Saddles use it, so water errs low and never stands in the air.
    pub floor_min: f32,
    /// Highest point of this span's floor anywhere in the column's square.
    pub floor_max: f32,
    /// The ceiling at the column centre; `f32::INFINITY` under open sky.
    pub ceiling: f32,
}

/// One span, named across the world.
///
/// Debug builds carry the generation of the chunk it was taken from, and
/// dereferencing a ref into a column rebuilt since panics: a holder must be
/// re-derived after a rebuild, never left pointing at a different span. A
/// ref into a column the rebuild left alone still names the same span.
#[derive(Debug, Clone, Copy)]
pub struct SpanRef {
    pub column: Column,
    /// Position up the column, 0 for the lowest span.
    pub ordinal: u8,
    #[cfg(debug_assertions)]
    pub generation: u32,
}

impl SpanRef {
    /// A ref with no generation check, for comparisons and lookups by key.
    pub fn key(column: Column, ordinal: u8) -> Self {
        Self {
            column,
            ordinal,
            #[cfg(debug_assertions)]
            generation: u32::MAX,
        }
    }
}

impl PartialEq for SpanRef {
    fn eq(&self, other: &Self) -> bool {
        self.column == other.column && self.ordinal == other.ordinal
    }
}

impl Eq for SpanRef {}

impl std::hash::Hash for SpanRef {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.column.hash(state);
        self.ordinal.hash(state);
    }
}

impl PartialOrd for SpanRef {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// Ties in every priority queue break on this order, never on hash order.
impl Ord for SpanRef {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.column, self.ordinal).cmp(&(other.column, other.ordinal))
    }
}

/// Which network objects claim a span. Parallel to the spans of a chunk.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SpanOwner {
    /// Water body whose region contains this span, wet or not.
    pub body: Option<StoreId>,
    /// Reach, and cell along it, whose channel runs over this span.
    pub reach: Option<(StoreId, u16)>,
}

/// 16 × 16 columns of spans, in CSR order.
#[derive(Debug, Clone)]
pub struct SpanChunk {
    /// `spans[column_offsets[c]..column_offsets[c + 1]]` are column `c`'s
    /// spans, bottom to top.
    column_offsets: [u16; COLUMNS_PER_CHUNK + 1],
    spans: Vec<Span>,
    /// Parallel to `spans`.
    owner: Vec<SpanOwner>,
    /// Bumped on every rebuild.
    generation: u32,
    /// The generation at which each column's spans last changed: a ref taken
    /// since is current. Checked against `SpanRef::generation` in debug
    /// builds.
    changed: [u32; COLUMNS_PER_CHUNK],
}

impl Default for SpanChunk {
    fn default() -> Self {
        Self {
            column_offsets: [0; COLUMNS_PER_CHUNK + 1],
            spans: Vec::new(),
            owner: Vec::new(),
            generation: 0,
            changed: [0; COLUMNS_PER_CHUNK],
        }
    }
}

impl SpanChunk {
    /// A chunk from per-column span lists, in local-index order, with no
    /// owners.
    pub fn from_columns(columns: &[Vec<Span>], generation: u32) -> Self {
        debug_assert_eq!(columns.len(), COLUMNS_PER_CHUNK);
        let mut chunk = Self {
            generation,
            changed: [generation; COLUMNS_PER_CHUNK],
            ..Self::default()
        };
        for (local, spans) in columns.iter().enumerate() {
            chunk.spans.extend_from_slice(spans);
            chunk.column_offsets[local + 1] = chunk.spans.len() as u16;
        }
        chunk.owner = vec![SpanOwner::default(); chunk.spans.len()];
        chunk
    }

    pub fn generation(&self) -> u32 {
        self.generation
    }

    /// The generation at which the column at a local index last changed.
    pub fn changed(&self, local: usize) -> u32 {
        self.changed[local]
    }

    /// Mark the column at a local index as last changed at `generation`: a
    /// rebuild that carried its spans over unchanged.
    pub fn keep_changed(&mut self, local: usize, generation: u32) {
        self.changed[local] = generation;
    }

    /// Spans of the column at a local index.
    pub fn column_spans(&self, local: usize) -> &[Span] {
        &self.spans[self.range(local)]
    }

    /// Owners of the column at a local index, parallel to its spans.
    pub fn column_owners(&self, local: usize) -> &[SpanOwner] {
        &self.owner[self.range(local)]
    }

    pub fn column_owners_mut(&mut self, local: usize) -> &mut [SpanOwner] {
        let range = self.range(local);
        &mut self.owner[range]
    }

    /// Index into the flat span array of a column's `ordinal`-th span.
    pub fn flat_index(&self, local: usize, ordinal: u8) -> Option<usize> {
        let range = self.range(local);
        let index = range.start + ordinal as usize;
        (index < range.end).then_some(index)
    }

    /// Total spans held.
    pub fn span_count(&self) -> usize {
        self.spans.len()
    }

    /// Every span's column-local index, in flat order.
    pub fn flat_columns(&self) -> impl Iterator<Item = (usize, u8)> + '_ {
        (0..COLUMNS_PER_CHUNK).flat_map(move |local| {
            (0..self.range(local).len()).map(move |ordinal| (local, ordinal as u8))
        })
    }

    fn range(&self, local: usize) -> std::ops::Range<usize> {
        self.column_offsets[local] as usize..self.column_offsets[local + 1] as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_column_contains_its_own_centre() {
        for (i, k) in [(0, 0), (-1, 3), (17, -40), (-16, -17)] {
            let column = Column::new(i, k);
            let (x, z) = column.centre();
            assert_eq!(Column::containing(x, z), column);
        }
    }

    #[test]
    fn negative_columns_land_in_negative_chunks() {
        let column = Column::new(-1, -17);
        let chunk = column.chunk();
        assert_eq!(chunk, SpanChunkCoord { x: -1, z: -2 });
        assert_eq!(chunk.column(column.local_index()), column);
    }

    #[test]
    fn csr_returns_each_column_its_own_spans() {
        let span = |y: f32| Span {
            floor_c: y,
            floor_min: y,
            floor_max: y,
            ceiling: f32::INFINITY,
        };
        let mut columns = vec![Vec::new(); COLUMNS_PER_CHUNK];
        columns[0] = vec![span(1.0)];
        columns[5] = vec![span(2.0), span(3.0)];
        let chunk = SpanChunk::from_columns(&columns, 7);
        assert_eq!(chunk.column_spans(0), &[span(1.0)]);
        assert!(chunk.column_spans(1).is_empty());
        assert_eq!(chunk.column_spans(5), &[span(2.0), span(3.0)]);
        assert_eq!(chunk.flat_index(5, 1), Some(2));
        assert_eq!(chunk.flat_index(5, 2), None);
        assert_eq!(chunk.generation(), 7);
    }
}
