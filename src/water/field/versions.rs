//! Per-chunk versions of the field's surface, for whatever draws it: a
//! chunk's version moves when its water has moved by more than can be seen
//! since it last moved, and so do those of the neighbours whose drawn edges
//! meet the cells that moved.

use super::cells::FieldCells;
use crate::water::geometry::{SpanChunkCoord, CHUNK_COLUMNS};

/// Least change in depth that moves a chunk's version, m.
const TOLERANCE: f64 = 1.0e-3;

/// One version per chunk, by slot.
#[derive(Debug, Clone, Default)]
pub struct SurfaceVersions {
    versions: Vec<u32>,
    /// Each chunk's depths when its version last moved.
    drawn: Vec<Vec<f64>>,
}

impl SurfaceVersions {
    pub fn new(cells: &FieldCells) -> Self {
        Self {
            versions: vec![0; cells.chunks.len()],
            drawn: cells.chunks.iter().map(|c| c.depth.clone()).collect(),
        }
    }

    /// Compare the chunks in `slots` with their depths when last moved, and
    /// move the versions of those that changed, and of the neighbours on
    /// the sides (and corners) where cells changed.
    pub fn update(&mut self, cells: &FieldCells, slots: impl IntoIterator<Item = usize>) {
        let mut moved: Vec<(SpanChunkCoord, Sides)> = Vec::new();
        for slot in slots {
            let chunk = &cells.chunks[slot];
            let drawn = &mut self.drawn[slot];
            let sides = if drawn.len() != chunk.depth.len() {
                Sides::ALL
            } else {
                let mut sides = Sides::NONE;
                for (flat, (a, b)) in drawn.iter().zip(&chunk.depth).enumerate() {
                    if (a - b).abs() > TOLERANCE {
                        sides.add(chunk.column_of(flat).0);
                    }
                }
                sides
            };
            if sides.changed {
                drawn.clone_from(&chunk.depth);
                moved.push((chunk.coord, sides));
            }
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

    /// Every chunk's version, by slot.
    pub fn versions(&self) -> &[u32] {
        &self.versions
    }
}

/// Which edges of a chunk had cells change: the chunk itself, and each side
/// (−1, 0 or +1 along each axis) whose border column did.
#[derive(Debug, Clone, Copy)]
struct Sides {
    changed: bool,
    /// Indexed by `(d + 1)` for d in −1..=1, per axis: whether a changed
    /// cell lies on that border (index 1, the middle, is always true once
    /// anything changed).
    x: [bool; 3],
    z: [bool; 3],
}

impl Sides {
    const NONE: Self = Self {
        changed: false,
        x: [false, false, false],
        z: [false, false, false],
    };
    const ALL: Self = Self {
        changed: true,
        x: [true, true, true],
        z: [true, true, true],
    };

    /// Note a changed cell in the column at chunk-local index `local`.
    fn add(&mut self, local: usize) {
        let last = CHUNK_COLUMNS as usize - 1;
        let (i, k) = (
            local % CHUNK_COLUMNS as usize,
            local / CHUNK_COLUMNS as usize,
        );
        self.changed = true;
        self.x[1] = true;
        self.z[1] = true;
        self.x[0] |= i == 0;
        self.x[2] |= i == last;
        self.z[0] |= k == 0;
        self.z[2] |= k == last;
    }

    /// Whether the neighbour at (dx, dz) draws an edge that meets a change.
    fn reaches(&self, dx: i32, dz: i32) -> bool {
        self.changed && self.x[(dx + 1) as usize] && self.z[(dz + 1) as usize]
    }
}
