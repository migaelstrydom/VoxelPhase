//! Conduits: where the field's water runs full under a roof, driven by the
//! head between the open water at its two ends.
//!
//! ```text
//!   lake ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓            a straight run of faces, every cell
//!   ~~~~ ▓▓▓▓ rock ▓▓▓▓▓▓            between them full to its ceiling,
//!        ├──┬──┬──┬──┤  ~~~~ tail    from one cell with a free surface
//!   ─────┴──┴──┴──┴──┴──────         to another
//! ```
//!
//! The solver's momentum law reads the slope of a free surface, and water
//! pressed against a roof has none: two full cells stand level whatever
//! pushes on them. So the water in a conduit moves as one column, by the
//! difference in head between its ends against what its mouth takes of it
//! (Torricelli, with a discharge coefficient), and every face of the run
//! carries the same discharge, which keeps the cells between them full.
//!
//! Only a straight run between two free surfaces is a conduit. A tunnel
//! that bends, or ends blind, is left to the solver's own law.

use smallvec::SmallVec;

use crate::water::falling::GRAVITY;
use crate::water::geometry::{SpanIndex, COLUMN_SIZE};

use super::cells::FieldCells;
use super::faces::{Face, FaceRef};

/// Most faces a conduit runs through.
const LONGEST: usize = 16;

/// How near its ceiling a cell's water must stand for the cell to be full,
/// m.
const FULL_SLACK: f64 = 5.0e-3;

/// Least opening a conduit's narrowest face must leave, m.
const LEAST_THROAT: f32 = 0.02;

/// Share of the discharge its head would drive through a hole's whole
/// opening that passes: a short tube's, between a sharp-edged orifice's
/// 0.6 and a rounded mouth's 1.
const DISCHARGE: f32 = 0.8;

/// The fastest a conduit's water runs, m/s.
const MAX_SPEED: f32 = 14.0;

/// A run of faces along one axis whose water moves as one column.
#[derive(Debug, Clone, PartialEq)]
pub struct Conduit {
    /// Its faces, from its − end to its + end.
    pub faces: SmallVec<[FaceRef; 6]>,
    /// The cells with a free surface at its − and + ends.
    pub ends: [SpanIndex; 2],
    /// The middle of the opening at each end: the head its water leaves
    /// against where the water outside stands lower than that.
    pub mouths: [f32; 2],
    /// Its narrowest opening, m.
    pub throat: f32,
}

impl Conduit {
    /// The head driving its water towards its + end, m: the higher end's
    /// surface over the lower end's, or over the middle of the mouth it
    /// leaves by where that stands clear of the water outside.
    pub fn head(&self, levels: [f32; 2]) -> f32 {
        if levels[0] >= levels[1] {
            (levels[0] - levels[1].max(self.mouths[1])).max(0.0)
        } else {
            -(levels[1] - levels[0].max(self.mouths[0])).max(0.0)
        }
    }

    /// Its discharge per metre of face after `dt` seconds, from `q`: the
    /// column's momentum, pushed by its head and held back by the loss at
    /// its mouth and by `friction` (the solver's own, for water this deep
    /// and fast), both taken implicitly so neither reverses it. At rest
    /// against a steady head it carries `DISCHARGE · throat · √(2·g·head)`.
    pub fn driven(&self, q: f32, levels: [f32; 2], friction: f32, dt: f32) -> f32 {
        let length = self.faces.len() as f32 * COLUMN_SIZE;
        let speed = q / self.throat;
        let loss = dt * speed.abs() / (2.0 * length * DISCHARGE * DISCHARGE);
        let pushed = speed + dt * GRAVITY * self.head(levels) / length;
        (pushed / (1.0 + friction + loss)).clamp(-MAX_SPEED, MAX_SPEED) * self.throat
    }
}

fn face_at(cells: &FieldCells, at: FaceRef) -> &Face {
    &cells.chunks[at.slot as usize].faces[at.index as usize]
}

/// Whether a cell's water is pressed against its ceiling.
fn full(cells: &FieldCells, cell: SpanIndex) -> bool {
    let room = (cells.ceiling(cell) - cells.bed(cell)) as f64;
    room.is_finite() && cells.depth(cell) >= room - FULL_SLACK
}

/// The opening a face leaves, floor to roof.
fn opening(cells: &FieldCells, at: FaceRef) -> (f32, f32) {
    let face = face_at(cells, at);
    let owner = SpanIndex {
        slot: at.slot,
        flat: face.a,
    };
    (cells.bed(owner).max(cells.bed(face.b)), face.roof)
}

/// The conduit through `start`: the faces back and on along its axis
/// through every full cell, to a cell with a free surface at each end.
/// None where the run ends blind, or runs on too far to follow.
fn through(cells: &FieldCells, start: FaceRef) -> Option<Conduit> {
    let mut faces: SmallVec<[FaceRef; 6]> = smallvec::smallvec![start];
    let mut at = start;
    let low = loop {
        let face = face_at(cells, at);
        let cell = SpanIndex {
            slot: at.slot,
            flat: face.a,
        };
        if !full(cells, cell) {
            break cell;
        }
        at = face.behind?;
        faces.insert(0, at);
        if faces.len() > LONGEST {
            return None;
        }
    };
    let mut at = start;
    let high = loop {
        let cell = face_at(cells, at).b;
        if !full(cells, cell) {
            break cell;
        }
        at = face_at(cells, at).ahead?;
        faces.push(at);
        if faces.len() > LONGEST {
            return None;
        }
    };
    let middle = |at: FaceRef| {
        let (floor, roof) = opening(cells, at);
        0.5 * (floor + roof)
    };
    let throat = (faces.iter())
        .map(|&at| {
            let (floor, roof) = opening(cells, at);
            roof - floor
        })
        .fold(f32::INFINITY, f32::min);
    (throat >= LEAST_THROAT).then(|| Conduit {
        mouths: [middle(faces[0]), middle(faces[faces.len() - 1])],
        faces,
        ends: [low, high],
        throat,
    })
}

/// Take every face out of the conduit it was in. A face laid again since
/// is in none.
fn dissolve(cells: &mut FieldCells) {
    let mut conduits = std::mem::take(&mut cells.conduits);
    for at in conduits.drain(..).flat_map(|conduit| conduit.faces) {
        let chunk = cells.chunks.get_mut(at.slot as usize);
        if let Some(face) = chunk.and_then(|c| c.faces.get_mut(at.index as usize)) {
            face.conduit = None;
        }
    }
    cells.conduits = conduits;
}

/// Find the field's conduits again as its water stands. `pressed` are the
/// faces to look from: those whose water stood against their roofs as the
/// solver last stepped. Every conduit has at least one.
pub(super) fn find(cells: &mut FieldCells, pressed: &[FaceRef]) {
    dissolve(cells);
    for &start in pressed {
        let chunk = &cells.chunks[start.slot as usize];
        let Some(face) = chunk.faces.get(start.index as usize) else {
            continue;
        };
        if face.conduit.is_some() {
            continue;
        }
        let Some(conduit) = through(cells, start) else {
            continue;
        };
        let index = cells.conduits.len() as u32;
        for at in &conduit.faces {
            cells.chunks[at.slot as usize].faces[at.index as usize].conduit = Some(index);
        }
        cells.conduits.push(conduit);
    }
}

/// Forget every conduit, as when the ground they ran through has changed:
/// the next step finds them again.
pub(super) fn forget(cells: &mut FieldCells) {
    dissolve(cells);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::field::solver::FieldSolver;
    use crate::water::field::testing::{basin, cell, open, roofed};
    use crate::water::geometry::CHUNK_COLUMNS;

    /// Stand water at `level` in the columns `columns` of every row inside
    /// the rim, as far as their ceilings let it.
    fn fill(cells: &mut FieldCells, columns: std::ops::Range<i32>, level: f64) {
        for k in 1..CHUNK_COLUMNS - 1 {
            for i in columns.clone() {
                let at = cell(i, k);
                let depth = (level - cells.bed(at) as f64).max(0.0);
                let room = (cells.ceiling(at) - cells.bed(at)) as f64;
                *cells.depth_mut(at) = depth.min(room);
            }
        }
    }

    fn run(cells: &mut FieldCells, solver: &mut FieldSolver, seconds: f32) {
        let dt = 1.0 / 60.0;
        for _ in 0..(seconds / dt) as usize {
            solver.step(cells, dt);
            find(cells, solver.pressed());
        }
    }

    #[test]
    fn two_ponds_joined_under_water_come_level() {
        // A wall over columns 6 to 8 with a tunnel through it, 1 m to 1.5 m up.
        let (_, mut cells) = basin(false, |i, _| match i {
            6..=8 => roofed(1.0, 1.5),
            _ => open(0.0),
        });
        fill(&mut cells, 1..9, 4.0);
        fill(&mut cells, 9..15, 2.0);
        let before = cells.volume();
        let mut solver = FieldSolver::default();
        run(&mut cells, &mut solver, 30.0);
        let (left, right) = (cells.surface(cell(3, 5)), cells.surface(cell(12, 5)));
        assert!((left - right).abs() < 0.05, "{left} against {right}");
        assert!((cells.volume() - before).abs() < 1.0e-6 * before);
    }

    #[test]
    fn a_hole_run_full_passes_what_its_head_drives() {
        // A pond over columns 1 to 10 behind a wall three columns thick,
        // holed 1 m to 1.5 m up, with a drop to the map's edge beyond it.
        let (_, mut cells) = basin(true, |i, _| match i {
            11..=13 => roofed(1.0, 1.5),
            14.. => open(-4.0),
            _ => open(0.0),
        });
        fill(&mut cells, 1..11, 4.0);
        let mut solver = FieldSolver::default();
        run(&mut cells, &mut solver, 1.5);
        let conduit = &cells.conduits[cells.conduits.len() / 2];
        assert_eq!(conduit.faces.len(), 4, "from the pond to the open air");
        let level = cells.surface(conduit.ends[0]) as f32;
        let q = face_at(&cells, conduit.faces[3]).q;
        let rated = DISCHARGE * 0.5 * (2.0 * GRAVITY * (level - 1.25)).sqrt();
        assert!((q - rated).abs() < 0.1 * rated, "{q} against {rated}");
        let critical = 0.5 * (GRAVITY * 0.5).sqrt();
        assert!(q > 2.0 * critical, "more than a free surface would pass");
    }

    #[test]
    fn a_tunnel_that_ends_blind_is_no_conduit() {
        let (_, mut cells) = basin(false, |i, _| match i {
            6..=8 => roofed(1.0, 1.5),
            9.. => open(50.0),
            _ => open(0.0),
        });
        fill(&mut cells, 1..9, 4.0);
        let mut solver = FieldSolver::default();
        run(&mut cells, &mut solver, 2.0);
        assert!(cells.conduits.is_empty());
        assert!(cells.chunks[0].faces.iter().all(|f| f.conduit.is_none()));
    }
}
