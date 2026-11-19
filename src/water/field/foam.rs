//! Foam: the white water the field makes where it lands, breaks and surges,
//! carried on its currents until it fades.
//!
//! ```text
//!   falls' landings ────────────────────┐
//!   jumps: fast water run into slow ────┼─▶ foam in a cell ─▶ carried cell
//!   fronts: a surge over dry ground ────┘   (0 to MAX_FOAM)   to cell with
//!                                                             the water,
//!                                                             fading
//! ```
//!
//! Foam is made where water loses speed all at once and takes in air, not
//! wherever it is fast: a chute running smooth makes none, and the pool it
//! runs into does. Once made it is the water's own, so it trails away
//! downstream of a fall and turns in an eddy, and lasts a few seconds after
//! what made it has stopped. It weighs nothing and pushes nothing: only
//! what draws the water reads it.

use crate::water::falling::GRAVITY;
use crate::water::geometry::{
    SpanChunkCoord, SpanIndex, CHUNK_COLUMNS, COLUMNS_PER_CHUNK, COLUMN_SIZE,
};

use super::activity::CALM_DISCHARGE;
use super::cells::FieldCells;
use super::faces::{Face, FaceRef};
use super::field::WET_DEPTH;
use super::solver::flow_depth;

/// How long foam lasts, s: it fades to a third in this long.
const FOAM_LIFE: f32 = 2.5;

/// The most foam a cell holds. Over 1 it lies thicker than covers the
/// water, and takes that much longer to thin.
const MAX_FOAM: f32 = 1.5;

/// How fast whatever makes foam raises a cell's towards what it makes, 1/s.
const MAKING_RATE: f32 = 6.0;

/// Foam under which a cell has none.
const GONE: f32 = 0.01;

/// Least change in a cell's foam that can be seen, and the fewest steps
/// between two tellings of a chunk's drawers that its foam has moved: foam
/// fades smoothly, and a surface read again for it every frame is read
/// three times more than its foam needs.
const SEEN: f32 = 0.05;
const TOLD_EVERY: u8 = 3;

/// Most of a cell's foam one face carries off in a step.
const MOST_CARRIED: f32 = 0.5;

/// Least depth a face's discharge is divided by for its speed, m, so a
/// film's trickle is not taken for a torrent.
pub(super) const MEASURED_DEPTH: f32 = 0.05;

/// The power falling water lands with, per metre of its lip and per cell it
/// lands in, m³/s³ (what pours times the square of the speed it lands at),
/// from which it makes foam and at which it makes the most.
const LANDING_POWER: (f32, f32) = (1.0, 30.0);

/// Slowest water that breaks into foam where it is slowed, m/s, and the
/// speed it must lose from one face to the next to make any, and the most.
const JUMP_SPEED: f32 = 1.0;
const JUMP_DROP: (f32, f32) = (0.6, 3.0);

/// Least depth of the water a jump runs into, m.
const JUMP_DEPTH: f64 = 0.1;

/// Slowest water that makes foam by anything it does but land, m/s: the
/// solver notes the faces running this fast, and no others are looked at.
pub(super) const BREAKING_SPEED: f32 = 0.5;

/// Heights of a surge running out over dry ground, m, and its speeds, m/s,
/// from which its front makes foam and at which it makes the most; and the
/// most it makes, as a share of what a cell holds: a front is spray more
/// than foam, and its edge is its cells' own.
const FRONT_HEIGHT: (f32, f32) = (0.2, 0.8);
const FRONT_SPEED: (f32, f32) = (BREAKING_SPEED, 2.0);
const FRONT_FOAM: f32 = 0.5;

fn ramp((from, to): (f32, f32), x: f32) -> f32 {
    ((x - from) / (to - from)).clamp(0.0, 1.0)
}

/// How much foam water landing with `power` makes, 0 to 1.
fn landing_strength(power: f32) -> f32 {
    ramp(LANDING_POWER, power).sqrt()
}

/// The power water pouring `pour` m²/s lands with after leaving level at
/// `speed` m/s and falling `height` m, per metre of its lip.
pub(super) fn landing_power(pour: f32, speed: f32, height: f32) -> f32 {
    pour * (speed * speed + 2.0 * GRAVITY * height.max(0.0))
}

/// The field's foam: what steps it, and what tells its drawers it moved.
/// The foam itself is in the cells (`FieldChunk::foam`).
#[derive(Debug, Clone, Default)]
pub struct Foam {
    /// Per chunk slot, per cell: the foam its drawers were last told of.
    shown: Vec<Vec<f32>>,
    /// Per chunk slot, per cell: what the currents bring it this step, less
    /// what they take.
    carried: Vec<Vec<f32>>,
    /// Per chunk slot, per cell: the most any source would make there this
    /// step, 0 to 1.
    made: Vec<Vec<f32>>,
    /// Per chunk slot: whether its cells hold any foam.
    foamy: Vec<bool>,
    /// Per chunk slot: whether this step reads or writes its cells' foam.
    stirred: Vec<bool>,
    /// Per chunk slot: steps since its drawers were last told its foam had
    /// moved, up to `TOLD_EVERY`.
    untold: Vec<u8>,
    /// Per chunk slot: whether its foam has moved by more than can be seen
    /// and its drawers have yet to be told, so it is looked at again even
    /// once it holds none.
    owed: Vec<bool>,
    /// The chunks whose foam has changed by more than can be seen since
    /// they were last taken (`take_moved`), and whether each one's border
    /// cells hold or held any, which the chunks round it draw too.
    moved: Vec<(usize, bool)>,
}

impl Foam {
    /// Advance the foam by `dt` seconds over the water as it now runs: make
    /// it where `landings` (each a cell and the power water lands in it
    /// with) come down, and where the water through one of the `fast` faces
    /// (those the solver found running at `BREAKING_SPEED` or more) runs
    /// into slower water or out over dry ground; carry it on the faces of
    /// the chunks that hold any; and fade it.
    ///
    /// Only chunks with foam, or with something making it, are touched:
    /// still water and smooth water cost nothing.
    pub(super) fn step(
        &mut self,
        cells: &mut FieldCells,
        landings: &[(SpanIndex, f32)],
        fast: &[FaceRef],
        dt: f32,
    ) {
        let count = cells.chunks.len();
        for buffer in [&mut self.shown, &mut self.carried, &mut self.made] {
            buffer.resize_with(count, Vec::new);
        }
        self.foamy.resize(count, false);
        self.untold.resize(count, TOLD_EVERY);
        self.owed.resize(count, false);
        self.stirred.clear();
        self.stirred.resize(count, false);
        for &(cell, power) in landings {
            self.make(cells, cell, landing_strength(power));
        }
        for &at in fast {
            self.break_at(cells, at);
        }
        for slot in 0..count {
            if cells.chunks[slot].awake && self.reaches_foam(cells, slot) {
                self.carry(cells, slot, dt);
            }
        }
        let fade = (-dt / FOAM_LIFE).exp();
        let making = 1.0 - (-MAKING_RATE * dt).exp();
        for slot in 0..count {
            if !(self.stirred[slot] || self.foamy[slot] || self.owed[slot]) {
                continue;
            }
            self.stir(cells, slot);
            let chunk = &mut cells.chunks[slot];
            let (carried, made) = (&mut self.carried[slot], &mut self.made[slot]);
            let mut any = false;
            for (flat, foam) in chunk.foam.iter_mut().enumerate() {
                let wet = chunk.depth[flat] > WET_DEPTH;
                let mut now = if wet {
                    (*foam + carried[flat]).max(0.0)
                } else {
                    0.0
                };
                let target = MAX_FOAM * made[flat];
                if wet && target > now {
                    now += (target - now) * making;
                }
                now = (now * fade).min(MAX_FOAM);
                *foam = if now < GONE { 0.0 } else { now };
                any |= *foam > 0.0;
                carried[flat] = 0.0;
                made[flat] = 0.0;
            }
            self.foamy[slot] = any;
            self.untold[slot] = self.untold[slot].saturating_add(1).min(TOLD_EVERY);
            let shown = &mut self.shown[slot];
            // The last of a cell's foam going is seen however little it was:
            // nothing looks at the chunk again to say so.
            let seen = shown.len() != chunk.foam.len()
                || (shown.iter().zip(&chunk.foam))
                    .any(|(&was, &now)| (was - now).abs() > SEEN || (now == 0.0 && was != 0.0));
            self.owed[slot] = seen;
            if seen && self.untold[slot] >= TOLD_EVERY {
                let shared = (0..COLUMNS_PER_CHUNK)
                    .filter(|&local| on_border(local))
                    .any(|local| {
                        let cells =
                            chunk.offsets[local] as usize..chunk.offsets[local + 1] as usize;
                        let held = |foam: &[f32]| {
                            (foam.get(cells.clone())).is_some_and(|f| f.iter().any(|&f| f > 0.0))
                        };
                        held(&chunk.foam) || held(shown)
                    });
                shown.clone_from(&chunk.foam);
                self.untold[slot] = 0;
                self.owed[slot] = false;
                self.moved.push((slot, shared));
            }
        }
    }

    /// Have this step read and write chunk `slot`'s foam.
    fn stir(&mut self, cells: &FieldCells, slot: usize) {
        let cells = cells.chunks[slot].depth.len();
        if !self.stirred[slot] || self.carried[slot].len() != cells {
            self.stirred[slot] = true;
            self.carried[slot].resize(cells, 0.0);
            self.made[slot].resize(cells, 0.0);
        }
    }

    /// Note a source that would raise `cell`'s foam to `strength` (0 to 1)
    /// of the most a cell holds.
    fn make(&mut self, cells: &FieldCells, cell: SpanIndex, strength: f32) {
        if strength <= 0.0 {
            return;
        }
        self.stir(cells, cell.slot as usize);
        let made = &mut self.made[cell.slot as usize][cell.flat as usize];
        *made = made.max(strength);
    }

    /// Whether chunk `slot`'s faces run between cells any of which may hold
    /// foam: its own, or those of the chunks its faces reach into on its +x
    /// and +z sides.
    fn reaches_foam(&self, cells: &FieldCells, slot: usize) -> bool {
        let coord = cells.chunks[slot].coord;
        let holds = |slot: usize| self.foamy[slot] || self.stirred[slot];
        let beside = |x: i32, z: i32| {
            let near = SpanChunkCoord {
                x: coord.x + x,
                z: coord.z + z,
            };
            cells.slot_of(near).is_some_and(holds)
        };
        holds(slot) || beside(1, 0) || beside(0, 1)
    }

    /// Note what the water through fast face `at` makes: a front where it
    /// surges out over dry ground, a jump where it runs into water going
    /// slower. Water going over a fall makes its foam where it lands.
    fn break_at(&mut self, cells: &FieldCells, at: FaceRef) {
        let Some(face) = cells.chunks[at.slot as usize].faces.get(at.index as usize) else {
            return;
        };
        let owner = SpanIndex {
            slot: at.slot,
            flat: face.a,
        };
        let forward = face.q > 0.0;
        let (giver, taker) = if forward {
            (owner, face.b)
        } else {
            (face.b, owner)
        };
        let held = cells.depth(giver);
        if held <= WET_DEPTH || falls_over(cells, face, forward) {
            return;
        }
        let speed = face.q.abs() / flow_depth(cells, owner, face).max(MEASURED_DEPTH);
        let ahead = cells.depth(taker);
        if ahead <= WET_DEPTH {
            let made = FRONT_FOAM * ramp(FRONT_HEIGHT, held as f32) * ramp(FRONT_SPEED, speed);
            self.make(cells, giver, made);
        } else if speed > JUMP_SPEED && ahead >= JUMP_DEPTH {
            let next = if forward { face.ahead } else { face.behind };
            let onward = next.map_or(0.0, |next| speed_through(cells, next));
            let onward = if forward { onward } else { -onward };
            self.make(cells, taker, ramp(JUMP_DROP, speed - onward.max(0.0)));
        }
    }

    /// Carry foam across the faces of awake chunk `slot`, with the water.
    /// Foam going over a fall is lost in it.
    fn carry(&mut self, cells: &FieldCells, slot: usize, dt: f32) {
        let chunk = &cells.chunks[slot];
        for face in &chunk.faces {
            if face.q.abs() < CALM_DISCHARGE {
                continue;
            }
            let owner = SpanIndex {
                slot: slot as u32,
                flat: face.a,
            };
            let forward = face.q > 0.0;
            let (giver, taker) = if forward {
                (owner, face.b)
            } else {
                (face.b, owner)
            };
            let foam = cells.chunks[giver.slot as usize].foam[giver.flat as usize];
            let held = cells.depth(giver);
            if foam <= 0.0 || held <= WET_DEPTH {
                continue;
            }
            let share = (face.q.abs() * dt / (held as f32 * COLUMN_SIZE)).min(MOST_CARRIED);
            self.stir(cells, giver.slot as usize);
            self.carried[giver.slot as usize][giver.flat as usize] -= foam * share;
            if !falls_over(cells, face, forward) {
                self.stir(cells, taker.slot as usize);
                self.carried[taker.slot as usize][taker.flat as usize] += foam * share;
            }
        }
    }

    /// The chunks whose foam has changed by more than can be seen since
    /// this was last asked, by slot, and whether each one's change reaches
    /// the cells along its border, which the chunks round it draw too.
    pub fn take_moved(&mut self) -> Vec<(usize, bool)> {
        let mut moved = std::mem::take(&mut self.moved);
        // A chunk moved more than once is shared if it ever was.
        moved.sort_unstable_by_key(|&(slot, shared)| (slot, !shared));
        moved.dedup_by_key(|&mut (slot, _)| slot);
        moved
    }
}

/// Whether the column at chunk-local index `local` lies along its chunk's
/// border.
fn on_border(local: usize) -> bool {
    let last = CHUNK_COLUMNS as usize - 1;
    let (i, k) = (
        local % CHUNK_COLUMNS as usize,
        local / CHUNK_COLUMNS as usize,
    );
    i == 0 || k == 0 || i == last || k == last
}

/// Whether the water through `face`, running the way `forward` says, goes
/// over a fall there.
fn falls_over(cells: &FieldCells, face: &Face, forward: bool) -> bool {
    face.flight
        .is_some_and(|flight| cells.flights[flight as usize].from_owner == forward)
}

/// How fast water runs through a face, m/s: positive from its `a` to its
/// `b`.
fn speed_through(cells: &FieldCells, at: FaceRef) -> f32 {
    let face: &Face = &cells.chunks[at.slot as usize].faces[at.index as usize];
    let owner = SpanIndex {
        slot: at.slot,
        flat: face.a,
    };
    face.q / flow_depth(cells, owner, face).max(MEASURED_DEPTH)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::field::faces::Axis;
    use crate::water::field::testing::{basin, cell, open};

    /// A walled basin of still water 1 m deep.
    fn pond() -> FieldCells {
        let (_, mut cells) = basin(false, |_, _| open(0.0));
        for i in 1..CHUNK_COLUMNS - 1 {
            for k in 1..CHUNK_COLUMNS - 1 {
                *cells.depth_mut(cell(i, k)) = 1.0;
            }
        }
        cells
    }

    fn foam(cells: &FieldCells, at: SpanIndex) -> f32 {
        cells.chunks[at.slot as usize].foam[at.flat as usize]
    }

    /// Set the discharge of the +x face of cell (i, k).
    fn run(cells: &mut FieldCells, i: i32, k: i32, q: f32) {
        let flat = cell(i, k).flat;
        let face = (cells.chunks[0].faces.iter_mut())
            .find(|f| f.a == flat && f.axis == Axis::X)
            .unwrap();
        face.q = q;
    }

    /// Every face carrying water, as the solver would note the fast ones.
    fn fast_faces(cells: &FieldCells) -> Vec<FaceRef> {
        (cells.chunks[0].faces.iter().enumerate())
            .filter(|(_, face)| face.q != 0.0)
            .map(|(index, _)| FaceRef {
                slot: 0,
                index: index as u32,
            })
            .collect()
    }

    #[test]
    fn landing_water_makes_foam_that_fades_when_it_stops() {
        let mut cells = pond();
        let mut field = Foam::default();
        let at = cell(5, 5);
        for _ in 0..60 {
            field.step(&mut cells, &[(at, 100.0)], &[], 1.0 / 60.0);
        }
        assert!(foam(&cells, at) > 1.2, "{}", foam(&cells, at));
        assert_eq!(foam(&cells, cell(9, 9)), 0.0);
        assert_eq!(field.take_moved(), vec![(0, false)], "far from its border");
        for _ in 0..(4.0 * FOAM_LIFE * 60.0) as usize {
            field.step(&mut cells, &[], &[], 1.0 / 60.0);
        }
        assert!(foam(&cells, at) < 0.05);
        for _ in 0..(4.0 * FOAM_LIFE * 60.0) as usize {
            field.step(&mut cells, &[], &[], 1.0 / 60.0);
        }
        assert_eq!(foam(&cells, at), 0.0);
        assert!(
            field.shown[0].iter().all(|&f| f == 0.0),
            "and its drawers know"
        );
        field.take_moved();
        field.step(&mut cells, &[], &[], 1.0 / 60.0);
        assert!(field.take_moved().is_empty(), "nothing left to move");
    }

    #[test]
    fn a_trickle_landing_makes_none() {
        let mut cells = pond();
        let mut field = Foam::default();
        field.step(&mut cells, &[(cell(5, 5), 0.5)], &[], 1.0);
        assert_eq!(foam(&cells, cell(5, 5)), 0.0);
    }

    #[test]
    fn foam_is_carried_the_way_the_water_runs() {
        let mut cells = pond();
        let mut field = Foam::default();
        field.step(&mut cells, &[(cell(5, 5), 100.0)], &[], 0.5);
        // 0.5 m/s to +x through every face of the row.
        for i in 1..CHUNK_COLUMNS - 2 {
            run(&mut cells, i, 5, 0.5);
        }
        for _ in 0..60 {
            field.step(&mut cells, &[], &[], 1.0 / 60.0);
        }
        let ahead = foam(&cells, cell(6, 5));
        assert!(ahead > 0.1, "{ahead}");
        assert_eq!(foam(&cells, cell(4, 5)), 0.0, "none goes upstream");
        assert_eq!(foam(&cells, cell(5, 6)), 0.0, "nor across the current");
    }

    #[test]
    fn fast_water_run_into_slow_breaks_into_foam_and_a_smooth_run_does_not() {
        let mut cells = pond();
        let mut field = Foam::default();
        // 3 m/s along row 5 up to column 8, where it stops; the same along
        // row 9, running on to the wall.
        for i in 1..8 {
            run(&mut cells, i, 5, 3.0);
        }
        for i in 1..CHUNK_COLUMNS - 2 {
            run(&mut cells, i, 9, 3.0);
        }
        let fast = fast_faces(&cells);
        field.step(&mut cells, &[], &fast, 0.1);
        assert!(foam(&cells, cell(8, 5)) > 0.3, "where it is slowed");
        assert_eq!(foam(&cells, cell(4, 5)), 0.0, "not where it runs on");
        assert_eq!(foam(&cells, cell(6, 9)), 0.0);
    }

    #[test]
    fn a_surge_over_dry_ground_foams_at_its_front() {
        let mut cells = pond();
        let mut field = Foam::default();
        for i in 8..CHUNK_COLUMNS - 1 {
            *cells.depth_mut(cell(i, 5)) = 0.0;
        }
        run(&mut cells, 7, 5, 2.0);
        let fast = fast_faces(&cells);
        field.step(&mut cells, &[], &fast, 0.1);
        assert!(foam(&cells, cell(7, 5)) > 0.15);
    }
}
