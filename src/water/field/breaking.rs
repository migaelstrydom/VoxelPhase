//! Breaking: water standing steeper than water can stand falls in on itself.
//!
//! The flow law turns the slope of a surface into speed and loses none of
//! its energy on the way: a wave run up a wall comes back as high as it
//! went, and a heap thrown up where a torrent lands sheds itself as a wave.
//! Real water that steep breaks. Where the surface steps more than
//! `STEEPEST` across a face, in one body of open water deep on both sides,
//! part of what stands over that slope goes to the lower side at once and
//! gains no speed by it: the energy a breaking wave loses to its own
//! turbulence.

use super::cells::FieldCells;
use super::faces::FaceRef;
use crate::water::geometry::{SpanIndex, COLUMN_SIZE};

/// The steepest a surface stands across a face, as a slope.
const STEEPEST: f64 = 0.6;

/// The share of what stands over that slope that comes down in a step.
const SHARE: f64 = 0.5;

/// The least depth on the lower side for the two to be one body of water
/// with a wave on it, m: shallower, the higher water is a front running out
/// over a floor, which the flow law carries.
const DEEP: f64 = 0.3;

/// Bring down the water standing too steep across each of `brinks`, the
/// faces the solver found a step across.
pub(super) fn settle(cells: &mut FieldCells, brinks: &[FaceRef]) {
    for &at in brinks {
        let chunk = &cells.chunks[at.slot as usize];
        let Some(face) = chunk.faces.get(at.index as usize) else {
            continue;
        };
        if face.flight.is_some() || face.conduit.is_some() || face.roof.is_finite() {
            continue;
        }
        let a = SpanIndex {
            slot: at.slot,
            flat: face.a,
        };
        let b = face.b;
        let (high, low) = if cells.surface(a) >= cells.surface(b) {
            (a, b)
        } else {
            (b, a)
        };
        // One body of water: the lower side is deep, and stands over the
        // higher side's floor. Over a ledge, the higher water falls.
        if cells.depth(low) < DEEP || cells.bed(high) as f64 >= cells.surface(low) {
            continue;
        }
        let over = cells.surface(high) - cells.surface(low) - STEEPEST * COLUMN_SIZE as f64;
        if over <= 0.0 {
            continue;
        }
        let down = (0.5 * SHARE * over).min(cells.depth(high));
        *cells.depth_mut(high) -= down;
        *cells.depth_mut(low) += down;
        for cell in [high, low] {
            if !cells.chunks[cell.slot as usize].awake {
                cells.wake(cell.slot as usize);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::field::testing::{cell, chunk_of, open};
    use crate::water::geometry::CHUNK_COLUMNS;

    fn every_face(cells: &FieldCells) -> Vec<FaceRef> {
        (0..cells.chunks[0].faces.len() as u32)
            .map(|index| FaceRef { slot: 0, index })
            .collect()
    }

    fn fill(cells: &mut FieldCells, depth: impl Fn(i32, i32) -> f64) {
        for k in 0..CHUNK_COLUMNS {
            for i in 0..CHUNK_COLUMNS {
                *cells.depth_mut(cell(i, k)) = depth(i, k);
            }
        }
    }

    #[test]
    fn a_heap_in_deep_water_comes_down_and_gains_no_speed() {
        let (_, mut cells) = chunk_of(|_, _| open(0.0));
        fill(&mut cells, |i, k| if (i, k) == (5, 5) { 5.0 } else { 2.0 });
        let before = cells.volume();
        let faces = every_face(&cells);
        settle(&mut cells, &faces);
        let heap = cells.depth(cell(5, 5));
        assert!(heap < 4.0 && heap > 2.0, "{heap}");
        for beside in [cell(4, 5), cell(6, 5), cell(5, 4), cell(5, 6)] {
            assert!(cells.depth(beside) > 2.0);
        }
        assert!((cells.volume() - before).abs() < 1.0e-9);
        assert!(cells.chunks[0].faces.iter().all(|face| face.q == 0.0));
        // What is left stands no steeper than water can, in the end.
        for _ in 0..200 {
            settle(&mut cells, &faces);
        }
        let steps = [cell(4, 5), cell(6, 5), cell(5, 4), cell(5, 6)]
            .map(|beside| cells.depth(cell(5, 5)) - cells.depth(beside));
        let steepest = STEEPEST * COLUMN_SIZE as f64;
        assert!(
            steps.iter().all(|&step| step <= steepest + 1.0e-3),
            "{steps:?}"
        );
    }

    #[test]
    fn water_on_a_ledge_and_a_front_on_a_film_are_left_to_the_flow_law() {
        // Half a metre of water on a ledge 4 m over a pool 2 m deep.
        let (_, mut cells) = chunk_of(|i, _| open(if i < 4 { 4.0 } else { 0.0 }));
        fill(&mut cells, |i, _| if i < 4 { 0.5 } else { 2.0 });
        let before = cells.chunks[0].depth.clone();
        let faces = every_face(&cells);
        settle(&mut cells, &faces);
        assert_eq!(cells.chunks[0].depth, before, "it falls");
        // Two metres of water running out over a film.
        let (_, mut cells) = chunk_of(|_, _| open(0.0));
        fill(&mut cells, |i, _| if i < 4 { 2.0 } else { 0.1 });
        let before = cells.chunks[0].depth.clone();
        let faces = every_face(&cells);
        settle(&mut cells, &faces);
        assert_eq!(cells.chunks[0].depth, before, "it runs");
    }
}
