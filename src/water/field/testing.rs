//! Hand-laid fields for the field's own tests: one chunk, one span to a
//! column.

use crate::water::geometry::{
    Span, SpanChunk, SpanChunkCoord, SpanGraph, SpanIndex, CHUNK_COLUMNS, COLUMNS_PER_CHUNK,
};

use super::cells::FieldCells;
use super::faces::{lay_faces, link_faces};

/// Ground no water in a test reaches the top of.
const RIM: f32 = 100.0;

/// A span under open sky, its floor level at `bed`.
pub fn open(bed: f32) -> Span {
    Span {
        floor_c: bed,
        bed,
        quarters: [bed; 4],
        floor_min: bed,
        floor_max: bed,
        ceiling: f32::INFINITY,
    }
}

/// A span under a roof at `ceiling`, its floor level at `bed`.
pub fn roofed(bed: f32, ceiling: f32) -> Span {
    Span {
        ceiling,
        ..open(bed)
    }
}

/// One chunk whose column (i, k) holds `span(i, k)`, awake, with faces
/// laid. Its edges give onto the void.
pub fn chunk_of(span: impl Fn(i32, i32) -> Span) -> (SpanGraph, FieldCells) {
    let coord = SpanChunkCoord { x: 0, z: 0 };
    let columns: Vec<Vec<Span>> = (0..COLUMNS_PER_CHUNK as i32)
        .map(|local| vec![span(local % CHUNK_COLUMNS, local / CHUNK_COLUMNS)])
        .collect();
    let mut graph = SpanGraph::new(coord, coord);
    graph.replace_chunk(coord, SpanChunk::from_columns(&columns, 1));
    let mut cells = FieldCells::build(&graph);
    lay_faces(&mut cells.chunks[0], &graph, None);
    link_faces(&mut cells, &[0]);
    cells.chunks[0].awake = true;
    (graph, cells)
}

/// `chunk_of`, walled in: the columns round its edge are high ground,
/// unless `open_east`, which leaves its +x edge onto the void.
pub fn basin(open_east: bool, span: impl Fn(i32, i32) -> Span) -> (SpanGraph, FieldCells) {
    let last = CHUNK_COLUMNS - 1;
    chunk_of(|i, k| {
        let rim = i == 0 || k == 0 || k == last || (i == last && !open_east);
        if rim {
            open(RIM)
        } else {
            span(i, k)
        }
    })
}

/// The cell of column (i, k) of the one chunk.
pub fn cell(i: i32, k: i32) -> SpanIndex {
    SpanIndex {
        slot: 0,
        flat: (k * CHUNK_COLUMNS + i) as u32,
    }
}
