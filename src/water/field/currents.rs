//! Each cell's depth-averaged velocity, read off its faces once a step for
//! the chunks whose water moved, so that queries and the mesher look it up.
//!
//! On each axis, a cell's velocity is the mean over its faces of the
//! discharge over the mean depth either side (its own, at the map's edge),
//! taken as at least
//! `MEASURED_DEPTH`: dividing by a film's own depth would make a trickle a
//! torrent, and a film that moves still reads as moving.

use nalgebra::Vector2;

use super::cells::FieldCells;
use super::faces::{face_owners, Axis};
use super::field::WET_DEPTH;

/// Least depth a face's discharge is divided by, m.
const MEASURED_DEPTH: f64 = 0.01;

/// Per cell: the sum of its faces' velocities, and how many faces, by axis.
#[derive(Debug, Clone, Copy, Default)]
struct Tally {
    sum: Vector2<f32>,
    count: Vector2<f32>,
}

impl Tally {
    fn add(&mut self, axis: Axis, u: f32) {
        match axis {
            Axis::X => {
                self.sum.x += u;
                self.count.x += 1.0;
            }
            Axis::Z => {
                self.sum.y += u;
                self.count.y += 1.0;
            }
        }
    }

    fn mean(&self) -> Vector2<f32> {
        let mean = |s: f32, n: f32| if n > 0.0 { s / n } else { 0.0 };
        Vector2::new(
            mean(self.sum.x, self.count.x),
            mean(self.sum.y, self.count.y),
        )
    }
}

/// Read the velocity of every cell in `slots` off its faces: those its
/// chunk owns, and those owned by the chunks to its −x and −z.
pub fn refresh(cells: &mut FieldCells, slots: &[usize]) {
    let mut tallies: Vec<Option<Vec<Tally>>> = vec![None; cells.chunks.len()];
    for &slot in slots {
        tallies[slot] = Some(vec![Tally::default(); cells.chunks[slot].depth.len()]);
    }
    let mut owners: Vec<usize> = slots
        .iter()
        .flat_map(|&s| face_owners(cells.chunks[s].coord))
        .filter_map(|c| cells.slot_of(c))
        .collect();
    owners.sort_unstable();
    owners.dedup();
    for owner in owners {
        let chunk = &cells.chunks[owner];
        for face in &chunk.faces {
            let a = chunk.depth[face.a as usize];
            let b = cells.depth(face.b);
            let u = face.q / (0.5 * (a + b)).max(MEASURED_DEPTH) as f32;
            if let Some(tally) = tallies[owner].as_mut() {
                tally[face.a as usize].add(face.axis, u);
            }
            if let Some(tally) = tallies[face.b.slot as usize].as_mut() {
                tally[face.b.flat as usize].add(face.axis, u);
            }
        }
    }
    for &slot in slots {
        let chunk = &cells.chunks[slot];
        let Some(tally) = tallies[slot].as_mut() else {
            continue;
        };
        for face in &chunk.boundary {
            let depth = chunk.depth[face.cell as usize];
            let u = face.outward * face.q / depth.max(MEASURED_DEPTH) as f32;
            tally[face.cell as usize].add(face.axis, u);
        }
    }
    for &slot in slots {
        let Some(tally) = tallies[slot].take() else {
            continue;
        };
        let chunk = &mut cells.chunks[slot];
        for (flat, t) in tally.iter().enumerate() {
            chunk.velocity[flat] = if chunk.depth[flat] > WET_DEPTH {
                t.mean()
            } else {
                Vector2::zeros()
            };
        }
    }
}
