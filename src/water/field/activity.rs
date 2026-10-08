//! Which chunks the solver steps: water that has stopped moving sleeps, and
//! any flow into it, an edit or a source wakes it again.

use super::cells::FieldCells;

/// Fastest a face may carry water in a chunk that counts as calm, m²/s: at a
/// metre's depth, a millimetre a second.
const CALM_DISCHARGE: f32 = 1.0e-3;

/// Seconds a chunk stays calm before it sleeps.
const SLEEP_AFTER: f32 = 2.0;

/// Count calm time for every awake chunk, and put to sleep those calm for
/// long enough. Returns the number of chunks still awake.
pub fn settle(cells: &mut FieldCells, dt: f32) -> usize {
    let mut awake = 0;
    for chunk in &mut cells.chunks {
        if !chunk.awake {
            continue;
        }
        let fastest = chunk
            .faces
            .iter()
            .map(|f| f.q.abs())
            .chain(chunk.boundary.iter().map(|f| f.q.abs()))
            .fold(0.0f32, f32::max);
        if fastest < CALM_DISCHARGE {
            chunk.calm_for += dt;
        } else {
            chunk.calm_for = 0.0;
        }
        if chunk.calm_for >= SLEEP_AFTER {
            chunk.awake = false;
            chunk.faces.iter_mut().for_each(|f| f.q = 0.0);
            chunk.boundary.iter_mut().for_each(|f| f.q = 0.0);
        } else {
            awake += 1;
        }
    }
    awake
}
