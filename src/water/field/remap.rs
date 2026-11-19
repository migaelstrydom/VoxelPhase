//! Carrying the field across a terrain edit.
//!
//! An edit re-pairs a few columns' spans. The water standing in each old
//! span moves into the new span that holds its old floor, so a crater under
//! water leaves the water where it was, over a floor that has dropped away,
//! and the flow law does the rest. What a new span has no room for under
//! its roof goes to the span above it. Faces are laid again only in the chunks
//! the edit rebuilt and the chunks owning faces into them; a face between
//! two columns the edit left alone keeps its discharge.

use rustc_hash::{FxHashMap, FxHashSet};
use smallvec::SmallVec;

use crate::water::geometry::{
    Column, SpanChunkCoord, SpanGraph, SpanIndex, SpanRemap, COLUMNS_PER_CHUNK,
};

use super::cells::{FieldCells, FieldChunk, CELL_AREA};
use super::faces::{face_owners, lay_faces, link_faces, FaceKey, SeaBoundary};

/// Carry the cells over a rebuild the graph has already taken. Returns the
/// volume that fell into the void, m³.
pub fn carry(
    cells: &mut FieldCells,
    graph: &SpanGraph,
    remap: &SpanRemap,
    sea: Option<SeaBoundary>,
) -> f64 {
    let repaired: FxHashSet<Column> = remap.columns.iter().copied().collect();
    let rebuilt: FxHashSet<SpanChunkCoord> = remap.chunks.iter().copied().collect();
    let owners: Vec<usize> = rebuilt
        .iter()
        .flat_map(|&c| face_owners(c))
        .collect::<FxHashSet<_>>()
        .into_iter()
        .filter_map(|c| graph.slot(c))
        .collect();
    let kept = keep_discharges(cells, &owners, &repaired);

    let mut lost = 0.0;
    let mut moved: FxHashMap<SpanChunkCoord, Vec<(usize, Option<u8>, f64)>> = FxHashMap::default();
    for entry in &remap.entries {
        let coord = entry.column.chunk();
        let Some(slot) = graph.slot(coord) else {
            continue;
        };
        let old = &cells.chunks[slot];
        let local = entry.column.local_index();
        let depth = old
            .flat(local, entry.old_ordinal)
            .map_or(0.0, |flat| old.depth[flat]);
        moved
            .entry(coord)
            .or_default()
            .push((local, entry.new_ordinal, depth));
    }

    for &coord in &rebuilt {
        let Some(slot) = graph.slot(coord) else {
            continue;
        };
        let fresh = FieldChunk::from_spans(coord, graph.chunk_at_slot(slot));
        let old = cells.relay(slot, fresh);
        let new = &mut cells.chunks[slot];
        for local in 0..COLUMNS_PER_CHUNK {
            if repaired.contains(&coord.column(local)) {
                continue;
            }
            let count = (new.offsets[local + 1] - new.offsets[local]) as u8;
            for ordinal in 0..count {
                if let (Some(from), Some(to)) = (old.flat(local, ordinal), new.flat(local, ordinal))
                {
                    new.depth[to] = old.depth[from];
                }
            }
        }
        for &(local, to, depth) in moved.get(&coord).into_iter().flatten() {
            match to.and_then(|o| new.flat(local, o)) {
                Some(flat) => new.depth[flat] += depth,
                None => lost += depth * CELL_AREA,
            }
        }
        for &(local, ..) in moved.get(&coord).into_iter().flatten() {
            let column = new.offsets[local] as usize..new.offsets[local + 1] as usize;
            let rooms: SmallVec<[f64; 4]> = (column.clone())
                .map(|flat| (new.ceiling[flat] - new.bed[flat]) as f64)
                .collect();
            pass_up(&mut new.depth[column], &rooms);
        }
    }

    for &slot in &owners {
        lay_faces(&mut cells.chunks[slot], graph, sea);
        restore_discharges(cells, slot, &kept);
    }
    let relinked: Vec<usize> = owners
        .iter()
        .flat_map(|&slot| {
            let c = cells.chunks[slot].coord;
            [(0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)].map(|(dx, dz)| SpanChunkCoord {
                x: c.x + dx,
                z: c.z + dz,
            })
        })
        .collect::<FxHashSet<_>>()
        .into_iter()
        .filter_map(|c| cells.slot_of(c))
        .collect();
    link_faces(cells, &relinked);
    for &coord in &rebuilt {
        for dx in -1..=1 {
            for dz in -1..=1 {
                let near = SpanChunkCoord {
                    x: coord.x + dx,
                    z: coord.z + dz,
                };
                if let Some(slot) = graph.slot(near) {
                    cells.wake(slot);
                }
            }
        }
    }
    lost
}

/// Pass what each of a column's cells holds over the `room` under its roof
/// up to the cell above it, bottom to top. An edit can leave a span with
/// less room than the water it is given: a sheet of ground the re-mesh has
/// drawn across what was one span. The water that stood over the sheet is
/// the upper span's. The top cell has no roof to stand over.
fn pass_up(depth: &mut [f64], room: &[f64]) {
    for cell in 0..depth.len().saturating_sub(1) {
        let over = depth[cell] - room[cell];
        if over > 0.0 {
            depth[cell] -= over;
            depth[cell + 1] += over;
        }
    }
}

/// The discharge on every face of `owners` that joins two columns the edit
/// left alone, by the spans it joins.
fn keep_discharges(
    cells: &FieldCells,
    owners: &[usize],
    repaired: &FxHashSet<Column>,
) -> FxHashMap<FaceKey, f32> {
    let mut kept = FxHashMap::default();
    for &slot in owners {
        for face in &cells.chunks[slot].faces {
            if let Some(key) = face_key(cells, slot, face.a, face.b) {
                if !repaired.contains(&key.a.0) && !repaired.contains(&key.b.0) {
                    kept.insert(key, face.q);
                }
            }
        }
    }
    kept
}

fn restore_discharges(cells: &mut FieldCells, slot: usize, kept: &FxHashMap<FaceKey, f32>) {
    let keys: Vec<Option<FaceKey>> = cells.chunks[slot]
        .faces
        .iter()
        .map(|f| face_key(cells, slot, f.a, f.b))
        .collect();
    for (face, key) in cells.chunks[slot].faces.iter_mut().zip(keys) {
        if let Some(q) = key.and_then(|k| kept.get(&k)) {
            face.q = *q;
        }
    }
}

fn face_key(cells: &FieldCells, slot: usize, a: u32, b: SpanIndex) -> Option<FaceKey> {
    let here = &cells.chunks[slot];
    let there = cells.chunks.get(b.slot as usize)?;
    if a as usize >= here.depth.len() || b.flat as usize >= there.depth.len() {
        return None;
    }
    let (la, oa) = here.column_of(a as usize);
    let (lb, ob) = there.column_of(b.flat as usize);
    Some(FaceKey {
        a: (here.coord.column(la), oa),
        b: (there.coord.column(lb), ob),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn water_over_a_new_span_s_roof_goes_to_the_span_above() {
        // A pocket 0.27 m high given 0.4 m of water, under open water.
        let mut depth = [0.4, 2.0];
        pass_up(&mut depth, &[0.27, f64::INFINITY]);
        assert!((depth[0] - 0.27).abs() < 1.0e-12 && (depth[1] - 2.13).abs() < 1.0e-12);
        // Through a full span between them.
        let mut depth = [1.5, 0.5, 0.0];
        pass_up(&mut depth, &[1.0, 0.5, f64::INFINITY]);
        assert_eq!(depth, [1.0, 0.5, 0.5]);
        // Water that fits is left where it is.
        let mut depth = [0.2, 0.0];
        pass_up(&mut depth, &[0.27, f64::INFINITY]);
        assert_eq!(depth, [0.2, 0.0]);
    }
}
