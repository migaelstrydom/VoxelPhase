//! The field's cells: one per span, holding the water standing in it.
//!
//! Cells are stored per span chunk, parallel to the chunk's spans, so an
//! edit that rebuilds a chunk rebuilds only that chunk's cells. Each chunk
//! keeps its own copy of the geometry the solver reads (bed and ceiling) and
//! of the column layout, so the cells can be carried across a rebuild after
//! the graph has already replaced the chunk.

use nalgebra::Vector2;

use crate::water::geometry::{
    Column, SpanChunk, SpanChunkCoord, SpanGraph, SpanIndex, CHUNK_COLUMNS, COLUMNS_PER_CHUNK,
    COLUMN_SIZE,
};

use super::faces::{BoundaryFace, Face};

/// Plan area of one cell, m².
pub const CELL_AREA: f64 = (COLUMN_SIZE * COLUMN_SIZE) as f64;

/// One span chunk's cells and the faces they own.
#[derive(Debug, Clone)]
pub struct FieldChunk {
    pub coord: SpanChunkCoord,
    /// `cells[offsets[c]..offsets[c + 1]]` are column `c`'s cells, bottom to
    /// top, as the chunk's spans were when the cells were built.
    pub offsets: Vec<u32>,
    /// Floor at the column centre, per cell.
    pub bed: Vec<f32>,
    /// Ceiling at the column centre, per cell; infinite under open sky.
    pub ceiling: Vec<f32>,
    /// Water depth over the bed, m, per cell.
    pub depth: Vec<f64>,
    /// Depth-averaged velocity, m/s, per cell: x and z, as of the last step
    /// whose water moved here.
    pub velocity: Vec<Vector2<f32>>,
    /// Faces to the +x and +z neighbours, each owned by the cell on its −
    /// side, so every face between two cells is held exactly once.
    pub faces: Vec<Face>,
    /// Faces through which water leaves the field: open map edges onto the
    /// sea, and edges onto the void.
    pub boundary: Vec<BoundaryFace>,
    /// Whether the solver steps this chunk.
    pub awake: bool,
    /// Seconds this chunk has been calm, towards sleeping.
    pub calm_for: f32,
}

impl FieldChunk {
    /// Dry cells over a span chunk's spans.
    pub fn from_spans(coord: SpanChunkCoord, chunk: &SpanChunk) -> Self {
        let mut offsets = Vec::with_capacity(COLUMNS_PER_CHUNK + 1);
        let mut bed = Vec::with_capacity(chunk.span_count());
        let mut ceiling = Vec::with_capacity(chunk.span_count());
        offsets.push(0);
        for local in 0..COLUMNS_PER_CHUNK {
            for span in chunk.column_spans(local) {
                bed.push(span.floor_c);
                ceiling.push(span.ceiling);
            }
            offsets.push(bed.len() as u32);
        }
        let depth = vec![0.0; bed.len()];
        let velocity = vec![Vector2::zeros(); bed.len()];
        Self {
            coord,
            offsets,
            bed,
            ceiling,
            depth,
            velocity,
            faces: Vec::new(),
            boundary: Vec::new(),
            awake: false,
            calm_for: 0.0,
        }
    }

    /// The flat index of a column's `ordinal`-th cell.
    pub fn flat(&self, local: usize, ordinal: u8) -> Option<usize> {
        let start = self.offsets[local] as usize;
        let index = start + ordinal as usize;
        (index < self.offsets[local + 1] as usize).then_some(index)
    }

    /// The column-local index and ordinal of every cell, in flat order.
    pub fn cell_columns(&self) -> impl Iterator<Item = (usize, u8)> + '_ {
        (0..COLUMNS_PER_CHUNK).flat_map(move |local| {
            let count = self.offsets[local + 1] - self.offsets[local];
            (0..count).map(move |ordinal| (local, ordinal as u8))
        })
    }

    /// The column-local index and ordinal of a cell.
    pub fn column_of(&self, flat: usize) -> (usize, u8) {
        let local = self.offsets.partition_point(|&o| o as usize <= flat) - 1;
        (local, (flat - self.offsets[local] as usize) as u8)
    }

    /// Water surface over a cell, m.
    pub fn surface(&self, flat: usize) -> f64 {
        self.bed[flat] as f64 + self.depth[flat]
    }

    /// Water held by the chunk, m³.
    pub fn volume(&self) -> f64 {
        self.depth.iter().sum::<f64>() * CELL_AREA
    }
}

/// Every chunk's cells, in the graph's slot order.
#[derive(Debug, Clone)]
pub struct FieldCells {
    pub chunks: Vec<FieldChunk>,
    /// Coordinate of `chunks[0]`, and the graph's width in chunks.
    origin: SpanChunkCoord,
    width: i32,
}

impl FieldCells {
    /// Dry cells over every span in the graph.
    pub fn build(graph: &SpanGraph) -> Self {
        let chunks: Vec<FieldChunk> = graph
            .chunk_coords()
            .enumerate()
            .map(|(slot, coord)| FieldChunk::from_spans(coord, graph.chunk_at_slot(slot)))
            .collect();
        let (min, max) = graph.column_bounds();
        let width = (max.i - min.i + 1) / CHUNK_COLUMNS;
        let origin = chunks.first().map_or(min.chunk(), |c| c.coord);
        Self {
            chunks,
            origin,
            width,
        }
    }

    /// The graph's width in chunks.
    pub fn width(&self) -> i32 {
        self.width
    }

    /// The slot of a chunk coordinate, if it is in the field.
    pub fn slot_of(&self, coord: SpanChunkCoord) -> Option<usize> {
        let x = coord.x - self.origin.x;
        let z = coord.z - self.origin.z;
        let depth = self.chunks.len() as i32 / self.width.max(1);
        (x >= 0 && z >= 0 && x < self.width && z < depth).then(|| (z * self.width + x) as usize)
    }

    /// The cell of a span in a column, if there is one.
    pub fn index(&self, graph: &SpanGraph, column: Column, ordinal: u8) -> Option<SpanIndex> {
        let slot = graph.slot(column.chunk())?;
        let flat = self.chunks[slot].flat(column.local_index(), ordinal)?;
        Some(SpanIndex {
            slot: slot as u32,
            flat: flat as u32,
        })
    }

    pub fn depth(&self, at: SpanIndex) -> f64 {
        self.chunks[at.slot as usize].depth[at.flat as usize]
    }

    pub fn depth_mut(&mut self, at: SpanIndex) -> &mut f64 {
        &mut self.chunks[at.slot as usize].depth[at.flat as usize]
    }

    pub fn bed(&self, at: SpanIndex) -> f32 {
        self.chunks[at.slot as usize].bed[at.flat as usize]
    }

    pub fn ceiling(&self, at: SpanIndex) -> f32 {
        self.chunks[at.slot as usize].ceiling[at.flat as usize]
    }

    pub fn surface(&self, at: SpanIndex) -> f64 {
        self.chunks[at.slot as usize].surface(at.flat as usize)
    }

    /// Water held by every cell, m³.
    pub fn volume(&self) -> f64 {
        self.chunks.iter().map(FieldChunk::volume).sum()
    }

    /// Wake a chunk, by slot.
    pub fn wake(&mut self, slot: usize) {
        let chunk = &mut self.chunks[slot];
        chunk.awake = true;
        chunk.calm_for = 0.0;
    }
}
