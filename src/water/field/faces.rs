//! Faces: where two cells meet, and where a cell meets the edge of the world.
//!
//! A face joins two spans in orthogonally adjacent columns whose air
//! intervals overlap, the span graph's own rule. It carries the discharge
//! per metre of face, which the solver integrates: momentum lives on faces,
//! volume in cells.

use crate::water::geometry::{
    saddle, Column, SpanChunkCoord, SpanGraph, SpanIndex, SpanRef, Step, ORTHOGONAL,
};

use rustc_hash::{FxHashMap, FxHashSet};

use super::cells::{FieldCells, FieldChunk};

/// Horizontal axis a face is normal to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Axis {
    X,
    Z,
}

/// The face between a cell and its neighbour on the + side.
#[derive(Debug, Clone, Copy)]
pub struct Face {
    /// The owning cell, by flat index in its chunk.
    pub a: u32,
    /// The neighbour, anywhere in the graph.
    pub b: SpanIndex,
    pub axis: Axis,
    /// Top of the opening between the two spans: the lower ceiling.
    pub roof: f32,
    /// Discharge per metre of face, m²/s; positive from `a` to `b`.
    pub q: f32,
    /// The face into `a` on the same axis, if there is one.
    pub behind: Option<FaceRef>,
    /// The face out of `b` on the same axis, if there is one.
    pub ahead: Option<FaceRef>,
}

/// A face, by its owning chunk's slot and its place in that chunk's list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaceRef {
    pub slot: u32,
    pub index: u32,
}

/// A face through which a cell's water leaves the field. On a map with a
/// sea, a closed edge is a wall below sea level, so the sea along it stays
/// in, and drains to the void above it, so a river still falls off the map.
#[derive(Debug, Clone, Copy)]
pub struct BoundaryFace {
    pub cell: u32,
    pub axis: Axis,
    /// +1 where the outside lies on the + side of the cell, −1 on the − side.
    pub outward: f32,
    /// The water level outside: the sea's, or `None` for the void, which
    /// takes whatever reaches it.
    pub outside: Option<f32>,
    /// Discharge per metre of face, m²/s; positive outwards.
    pub q: f32,
}

/// The sea beyond the map's open edges, as the faces need it.
#[derive(Debug, Clone, Copy)]
pub struct SeaBoundary {
    pub level: f32,
    /// Whether the map's edge in each orthogonal direction, in `ORTHOGONAL`
    /// order (+x, +z, −x, −z), opens onto the sea.
    pub open: [bool; 4],
}

/// The axis and sign of an orthogonal step.
fn axis_of(step: Step) -> (Axis, f32) {
    match (step.di, step.dk) {
        (1, 0) => (Axis::X, 1.0),
        (-1, 0) => (Axis::X, -1.0),
        (0, 1) => (Axis::Z, 1.0),
        _ => (Axis::Z, -1.0),
    }
}

/// Lay a chunk's faces from the graph as it stands. The chunk's cells must
/// already mirror the graph's spans, and so must every neighbour's.
pub fn lay_faces(chunk: &mut FieldChunk, graph: &SpanGraph, sea: Option<SeaBoundary>) {
    let coord = chunk.coord;
    let mut faces = Vec::new();
    let mut boundary = Vec::new();
    for (flat, (local, ordinal)) in chunk.cell_columns().enumerate() {
        let column = coord.column(local);
        let here = *graph.span(SpanRef::key(column, ordinal));
        for (direction, step) in ORTHOGONAL.iter().enumerate() {
            let next = column.offset(step.di, step.dk);
            let (axis, sign) = axis_of(*step);
            if !graph.contains_column(next) || graph.spans(next).is_empty() {
                let outside = sea.filter(|sea| sea.open[direction]).map(|sea| sea.level);
                let walled = outside.is_none() && sea.is_some_and(|sea| here.bed < sea.level);
                if walled {
                    continue;
                }
                boundary.push(BoundaryFace {
                    cell: flat as u32,
                    axis,
                    outward: sign,
                    outside,
                    q: 0.0,
                });
                continue;
            }
            if sign < 0.0 {
                continue;
            }
            for (other, there) in graph.spans(next).iter().enumerate() {
                if saddle(&here, there).is_none() {
                    continue;
                }
                let b = graph
                    .index_of(SpanRef::key(next, other as u8))
                    .expect("a span the graph just listed");
                faces.push(Face {
                    a: flat as u32,
                    b,
                    axis,
                    roof: here.ceiling.min(there.ceiling),
                    q: 0.0,
                    behind: None,
                    ahead: None,
                });
            }
        }
    }
    chunk.faces = faces;
    chunk.boundary = boundary;
}

/// Link each face of the chunks in `slots` to the faces behind and ahead of
/// it on its axis. Every chunk's faces must already be laid.
pub fn link_faces(cells: &mut FieldCells, slots: &[usize]) {
    let mut outgoing: FxHashMap<(u32, u32, Axis), u32> = FxHashMap::default();
    let mut incoming: FxHashMap<(u32, u32, Axis), FaceRef> = FxHashMap::default();
    let mut indexed: FxHashSet<usize> = FxHashSet::default();
    for &slot in slots {
        let coord = cells.chunks[slot].coord;
        let near = [
            (coord.x, coord.z),
            (coord.x - 1, coord.z),
            (coord.x, coord.z - 1),
            (coord.x + 1, coord.z),
            (coord.x, coord.z + 1),
        ];
        for (x, z) in near {
            let Some(s) = cells.slot_of(SpanChunkCoord { x, z }) else {
                continue;
            };
            if !indexed.insert(s) {
                continue;
            }
            for (index, face) in cells.chunks[s].faces.iter().enumerate() {
                let here = FaceRef {
                    slot: s as u32,
                    index: index as u32,
                };
                outgoing
                    .entry((s as u32, face.a, face.axis))
                    .or_insert(index as u32);
                incoming
                    .entry((face.b.slot, face.b.flat, face.axis))
                    .or_insert(here);
            }
        }
    }
    for &slot in slots {
        let mut faces = std::mem::take(&mut cells.chunks[slot].faces);
        for face in &mut faces {
            face.behind = incoming.get(&(slot as u32, face.a, face.axis)).copied();
            face.ahead = outgoing
                .get(&(face.b.slot, face.b.flat, face.axis))
                .map(|&index| FaceRef {
                    slot: face.b.slot,
                    index,
                });
        }
        cells.chunks[slot].faces = faces;
    }
}

/// The chunks whose faces reach into `coord`: itself and its −x and −z
/// neighbours, which own the faces on its − edges.
pub fn face_owners(coord: SpanChunkCoord) -> [SpanChunkCoord; 3] {
    [
        coord,
        SpanChunkCoord {
            x: coord.x - 1,
            z: coord.z,
        },
        SpanChunkCoord {
            x: coord.x,
            z: coord.z - 1,
        },
    ]
}

/// A face named by the spans it joins, stable across a rebuild of columns
/// it does not touch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FaceKey {
    pub a: (Column, u8),
    pub b: (Column, u8),
}
