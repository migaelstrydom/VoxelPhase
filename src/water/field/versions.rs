//! Per-chunk versions of the field's surface, for whatever draws it: a
//! chunk's version moves when its water has moved by more than can be seen
//! since it last moved, and so do those of the neighbours whose drawn edges
//! meet a border cell that has drifted as far since they last moved for it.
//! A jet landing in a chunk is drawn with its surface, so the chunk's version
//! moves when the jet does ([`SurfaceVersions::touch`]).

use super::cells::FieldCells;
use crate::water::geometry::{SpanChunkCoord, CHUNK_COLUMNS};

/// Least change in depth that moves a chunk's version, m.
const TOLERANCE: f64 = 1.0e-3;

/// The same, as a share of the water's depth, where that is more: deep
/// water simmers by millimetres wherever anything runs into it, and its
/// level a few millimetres out cannot be seen, while reading its whole
/// surface again every frame for them is most of what drawing a level's
/// rivers costs. A shore's thin water keeps `TOLERANCE`, where a
/// millimetre moves the waterline, and so does what a chunk tells its
/// neighbours of the cells along its border once it is drawn again, so
/// that their surfaces still meet.
const TOLERANCE_SHARE: f64 = 2.0e-3;

/// The most `TOLERANCE_SHARE` lets water of any depth go undrawn by, m: a
/// reservoir running out is seen to go down smoothly against its dam.
const MOST_TOLERANCE: f64 = 5.0e-3;

/// The least change in the depth of water `depth` deep that is drawn.
fn tolerance(depth: f64) -> f64 {
    (TOLERANCE_SHARE * depth).clamp(TOLERANCE, MOST_TOLERANCE)
}

/// One version per chunk, by slot.
#[derive(Debug, Clone, Default)]
pub struct SurfaceVersions {
    versions: Vec<u32>,
    /// Each chunk's depths when its version last moved: what it is drawn
    /// from, and what its neighbours read along their shared edges.
    drawn: Vec<Vec<f64>>,
    /// Each chunk's depths as its neighbours' versions last moved for them.
    /// A border cell moves its neighbours' versions once its drawn depth
    /// has drifted more than `TOLERANCE` from here, so two surfaces never
    /// draw a shared edge more than twice that apart.
    passed: Vec<Vec<f64>>,
    /// The update each chunk's drawn depths were last replaced in.
    replaced: Vec<u64>,
    /// Updates so far.
    updates: u64,
}

/// Each chunk's depths as its surface is drawn from them, and when they
/// were last replaced, so a surface read again can pass over the cells it
/// read since.
#[derive(Debug, Clone, Copy)]
pub struct DrawnDepths<'a> {
    /// Each chunk's depths when its version last moved, by slot.
    pub depths: &'a [Vec<f64>],
    /// The update each chunk's depths were last replaced in, by slot.
    pub replaced: &'a [u64],
    /// Updates so far: a surface read now has read every chunk replaced in
    /// this update or before.
    pub now: u64,
}

impl SurfaceVersions {
    pub fn new(cells: &FieldCells) -> Self {
        let depths: Vec<Vec<f64>> = cells.chunks.iter().map(|c| c.depth.clone()).collect();
        Self {
            versions: vec![0; cells.chunks.len()],
            drawn: depths.clone(),
            passed: depths,
            replaced: vec![0; cells.chunks.len()],
            updates: 0,
        }
    }

    /// Compare the chunks in `slots` with their depths when last moved, and
    /// move the versions of those that changed, and of the neighbours on
    /// the sides (and corners) where their border cells drifted.
    pub fn update(&mut self, cells: &FieldCells, slots: impl IntoIterator<Item = usize>) {
        self.updates += 1;
        let mut moved: Vec<(SpanChunkCoord, Sides)> = Vec::new();
        for slot in slots {
            let chunk = &cells.chunks[slot];
            let (drawn, passed) = (&mut self.drawn[slot], &mut self.passed[slot]);
            if drawn.len() != chunk.depth.len() {
                drawn.clone_from(&chunk.depth);
                passed.clone_from(&chunk.depth);
                self.replaced[slot] = self.updates;
                moved.push((chunk.coord, Sides::ALL));
                continue;
            }
            let changed = drawn
                .iter()
                .zip(&chunk.depth)
                .any(|(a, b)| (a - b).abs() > tolerance(*a));
            if !changed {
                continue;
            }
            drawn.clone_from(&chunk.depth);
            self.replaced[slot] = self.updates;
            let mut sides = Sides::OWN;
            for (flat, (told, now)) in passed.iter_mut().zip(drawn.iter()).enumerate() {
                if (*told - now).abs() > TOLERANCE {
                    *told = *now;
                    sides.add(chunk.column_of(flat).0);
                }
            }
            moved.push((chunk.coord, sides));
        }
        for (coord, sides) in moved {
            for dz in -1..=1 {
                for dx in -1..=1 {
                    if !sides.reaches(dx, dz) {
                        continue;
                    }
                    let near = SpanChunkCoord {
                        x: coord.x + dx,
                        z: coord.z + dz,
                    };
                    if let Some(slot) = cells.slot_of(near) {
                        self.versions[slot] = self.versions[slot].wrapping_add(1);
                    }
                }
            }
        }
    }

    /// Move chunk `slot`'s version for something drawn with its surface
    /// that is not its depths: a jet landing in it.
    pub fn touch(&mut self, slot: usize) {
        self.versions[slot] = self.versions[slot].wrapping_add(1);
    }

    /// Each chunk's depths as its surface is drawn from them.
    pub fn drawn(&self) -> DrawnDepths<'_> {
        DrawnDepths {
            depths: &self.drawn,
            replaced: &self.replaced,
            now: self.updates,
        }
    }

    /// Every chunk's version, by slot.
    pub fn versions(&self) -> &[u32] {
        &self.versions
    }
}

/// Which of a moved chunk's neighbours move with it: the chunk itself, and
/// each side (−1, 0 or +1 along each axis) whose border column drifted.
#[derive(Debug, Clone, Copy)]
struct Sides {
    /// Indexed by `(d + 1)` for d in −1..=1, per axis: whether a drifted
    /// cell lies on that border (index 1, the chunk itself, always does).
    x: [bool; 3],
    z: [bool; 3],
}

impl Sides {
    /// The chunk alone.
    const OWN: Self = Self {
        x: [false, true, false],
        z: [false, true, false],
    };
    const ALL: Self = Self {
        x: [true, true, true],
        z: [true, true, true],
    };

    /// Note a drifted cell in the column at chunk-local index `local`.
    fn add(&mut self, local: usize) {
        let last = CHUNK_COLUMNS as usize - 1;
        let (i, k) = (
            local % CHUNK_COLUMNS as usize,
            local / CHUNK_COLUMNS as usize,
        );
        self.x[0] |= i == 0;
        self.x[2] |= i == last;
        self.z[0] |= k == 0;
        self.z[2] |= k == last;
    }

    /// Whether the neighbour at (dx, dz) draws an edge that meets a change.
    fn reaches(&self, dx: i32, dz: i32) -> bool {
        self.x[(dx + 1) as usize] && self.z[(dz + 1) as usize]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::field::testing::{cell, chunk_of, open};

    #[test]
    fn deep_water_moves_its_version_by_a_share_of_its_depth_and_thin_water_by_a_millimetre() {
        let (_, mut cells) = chunk_of(|_, _| open(0.0));
        *cells.depth_mut(cell(5, 5)) = 2.0;
        *cells.depth_mut(cell(9, 9)) = 0.2;
        let mut versions = SurfaceVersions::new(&cells);
        let moved = |versions: &mut SurfaceVersions, cells: &FieldCells| {
            let before = versions.versions()[0];
            versions.update(cells, [0]);
            versions.versions()[0] != before
        };
        // 3 mm on 2 m of water is under two parts in a thousand of it.
        *cells.depth_mut(cell(5, 5)) += 0.003;
        assert!(!moved(&mut versions, &cells));
        // A little at a time, it adds up to more.
        *cells.depth_mut(cell(5, 5)) += 0.002;
        assert!(moved(&mut versions, &cells));
        assert!(!moved(&mut versions, &cells), "and is drawn as it stands");
        // On 20 cm, 1.5 mm is seen at the waterline.
        *cells.depth_mut(cell(9, 9)) += 0.0015;
        assert!(moved(&mut versions, &cells));
        // However deep, half a centimetre is drawn.
        *cells.depth_mut(cell(5, 5)) = 20.0;
        moved(&mut versions, &cells);
        *cells.depth_mut(cell(5, 5)) += 0.006;
        assert!(moved(&mut versions, &cells));
    }
}
