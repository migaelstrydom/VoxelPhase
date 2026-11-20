//! Falls: where the field's water goes over an edge, and where and when it
//! comes down.
//!
//! ```text
//!   awake chunk ──▶ each face pouring from water that `falls` to what lies
//!                   across it, and each boundary face pouring off the map
//!               ──▶ a Fall: its lip, the velocity it leaves with, and its
//!                   arc followed column by column down to the water or the
//!                   ground it meets
//!               ──▶ the face's Flight: the solver sends its water into the
//!                   air instead of into the cell across
//!               ──▶ the fall brings it down: into the cells its arc lands
//!                   in, once the first of it has had time to get there,
//!                   with the momentum it lands with
//! ```
//!
//! Water leaves a lip at the speed it runs over it. Out of a hole, where
//! open water stands over the hole's roof behind it, it bursts out at the
//! speed that head gives it (Torricelli), from the middle of the opening and
//! thinner than the opening by as much: a jet, though the same `Fall`. So
//! does deep water through a slot down in its side. A jet that comes out
//! under the water outside does not fly, and is kept only to be seen
//! filling its hole while the hole's top stands clear (`Fall::drowned`).
//!
//! Water going over an edge flies. It does not arrive in the cell at the
//! foot of the cliff and run on from there: a breach's torrent lands metres
//! out, a trickle at the cliff's foot, and the ledges an arc clears stay
//! dry. Everything that shows a fall (the drawn curtain, the streams that
//! strike bodies, the spray where it lands) reads the same `Fall`, so the
//! water is seen to come down where it does. Streams flying at each other,
//! as into a hole off all its sides, meet in the air and come down
//! together.
//!
//! A fall has a life. New, its water has only fallen as far as its age lets
//! it, and none has landed. Once its water stops going over, what is
//! already in the air goes on falling: the fall is `spent`, and is kept
//! until the last of it is down. Where the water rocks across a line a fall
//! is judged by, as it does where a torrent bursts out, a fall holds to
//! what it was: it would otherwise be drawn one way and the other on
//! alternate frames.

use nalgebra::{Point3, Vector2};
use rustc_hash::FxHashMap;
use smallvec::SmallVec;

use crate::water::falling::{
    along_edge, edge_ends, fall_time, falls, launch, ledge_reach, off_map, GRAVITY,
};
use crate::water::geometry::{
    Column, SpanGraph, SpanIndex, SpanRef, Step, COLUMN_SIZE, ORTHOGONAL,
};

use super::cells::{FieldCells, CELL_AREA, FULL_SLACK};
use super::faces::{Axis, Face, FaceRef, Flight, Push};
use super::field::WET_DEPTH;
use super::foam::landing_power;

/// Least flow over an edge, per metre of it, that is a fall, m²/s.
const MIN_POUR: f32 = 0.005;

/// Least step between two cells' surfaces that water can fall or burst
/// across, m: under both `falls`' step and a jet's clearance.
const LEAST_STEP: f64 = 0.25;

/// How far past each line a fall is judged by the water must go for a
/// fall found the step before to be judged otherwise: the least step it is
/// still found across, m, and how far a jet's top must pass its clearance,
/// and the water behind it fall from its hole's roof, m (`JetLines`). The
/// water at a brink rocks from one step to the next, by a hand's depth
/// where a torrent bursts out into the water heaped at its mouth, and
/// judged each time against one line, a fall near it would be drawn one
/// way and the other on alternate frames.
const HELD_STEP: f64 = 0.1;
const HELD_CLEARANCE: f32 = 0.2;
const HELD_FILL: f64 = 0.1;

/// How long a fall is kept on as it was while the water over its face,
/// still pouring its way, no longer meets the lines it is judged by, s.
/// Where a torrent bursts out the water surges and ebbs by more than any
/// margin from one step to the next, and a fall lost and found again over
/// a few frames would be drawn flickering.
const GRACE: f32 = 0.15;

/// How quickly a fall found again takes up the discharge its face carries
/// now, from what it carried, s: the time over which it closes all but a
/// third of the difference. The discharge over a torrent's lip rocks by a
/// few hundredths from one step to the next, and a jet's thickness and the
/// height of its top, taken from it, would shimmer.
const POUR_EASE: f32 = 0.05;

/// Least depth a face's discharge is divided by for the speed its water
/// leaves at, m, so a film's trickle is not launched as a torrent.
const MEASURED_DEPTH: f64 = 0.02;

/// Most cells back along its axis a roofed cell is followed to the open
/// water driving it.
const LONGEST_CONDUIT: usize = 8;

/// Least head, m, the open water behind a hole must stand over the hole's
/// roof to drive its water out as a jet.
const PRESSED_HEAD: f32 = 0.05;

/// Least height, m, the top of a jet must stand over the water or ground
/// outside for the hole's water to burst out as one, and the least opening
/// that does. A hole half drowned by the water it has let out still throws
/// the top of its jet out over it.
const JET_CLEARANCE: f32 = 0.25;
const JET_OPENING: f32 = 0.1;

/// Share of the speed its head would give it that a jet leaves at: what the
/// hole's edges leave of it.
const JET_SPEED_SHARE: f32 = 0.95;

/// How far past its edge an arc starts, m: inside the column it falls into.
const NUDGE: f32 = 1.0e-3;

/// Most columns an arc is followed across before it is taken to have left
/// the world.
const MAX_COLUMNS: usize = 96;

/// Share of the level momentum it lands with that falling water gives the
/// water it lands in: the rest goes in the plunge.
const CARRIED: f32 = 0.5;

/// Slowest a fall's water moves along an axis as it lands for it to push
/// the water there along it, m/s, and the fastest it drives that water.
const PUSHING_SPEED: f32 = 0.2;
const PUSHED_SPEED: f32 = 1.0;

/// How far apart, level, the lips of a fall found after an edit and one
/// found before it may lie for the two to be the same fall, m.
const SAME_LIP: f32 = 0.5 * COLUMN_SIZE;

/// How far under the roof of the cell it comes out into an arc starts, at
/// the highest, m.
const UNDER_ROOF: f32 = 0.01;

/// How many cells out from where it lands a spent fall's last water looks
/// for room, and how deep it lies in each before it looks further, m.
const SPILL_REACH: usize = 48;
const BACKED_UP: f64 = 0.1;

/// Most cells under roofs counted as where falls' water spreads to from
/// where it lands (`fill`).
const BODY_REACH: usize = 512;

/// How many times what the falls landing under roofs ask for (`Roofed::need`)
/// the room found for them is looked for: where there is that much, each
/// one's share holds what it has in the air and as much again.
const ROOM_TO_SPARE: f64 = 2.0;

/// Most columns out across the gap it falls into that a fall looks for one
/// flying back at it (`Falls::meet`).
const MEET_REACH: i32 = 16;

/// Water going over one edge of a cell into the air.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fall {
    /// The cell its water leaves.
    pub from: SpanIndex,
    /// Which of the cell's edges it goes over, as an index into
    /// `ORTHOGONAL`.
    pub direction: u8,
    /// The middle of its lip, on top of the water going over it: out past
    /// the cell's own edge where the column below carries the ledge on.
    pub lip: Point3<f32>,
    /// How far past the cell's edge the ledge runs at each end of the edge
    /// (`edge_ends`), in columns.
    pub reach: [f32; 2],
    /// Its level velocity as it leaves, m/s.
    pub velocity: Vector2<f32>,
    /// How thick the water is as it leaves, m.
    pub depth: f32,
    /// How much of the opening it leaves by it fills behind its mouth, m:
    /// the whole height of a hole run full, however thin its jet flies.
    /// Its depth, for water going over a lip, or out of a hole it does not
    /// fill.
    pub opening: f32,
    /// Whether the water behind its mouth stands against the opening's
    /// roof, so that it fills its hole there (`opening`).
    pub fills: bool,
    /// What it carries over each metre of its lip, m²/s.
    pub pour: f32,
    /// Whether it bursts out of a hole, driven by open water standing over
    /// the hole's roof behind it, rather than running over a lip.
    pub driven: bool,
    /// Whether it comes out under the water outside, with the top of the
    /// hole it fills still standing clear of that water. None of it flies:
    /// its water runs on into the water outside as over any face. It is
    /// kept to be seen, filling the hole's mouth over the water.
    pub drowned: bool,
    /// Where the top of its water would come down, flying free: the end of
    /// its arc. Behind a wall, where one stands in its way.
    pub foot: Point3<f32>,
    /// Where its water meets the world: its foot, or where it strikes a
    /// wall on the way there.
    pub landing: Point3<f32>,
    /// The cell it lands in; none where it leaves the world.
    pub into: Option<SpanIndex>,
    /// Whether it falls off the world into the void, where nothing catches
    /// it: followed down until it has broken up in the air (`VOID_DROP`),
    /// it lands nowhere.
    pub into_void: bool,
    /// How long its water has been going over, s: its water has fallen no
    /// further than that lets it.
    pub age: f32,
    /// How long it has been kept on as it was, its water going over
    /// without meeting the lines a fall is judged by (`GRACE`), s.
    pub lapsed: f32,
    /// How long ago its water stopped going over, s, while the last of it
    /// is still in the air. None while it pours.
    pub spent: Option<f32>,
    /// The water it has in the air, m³.
    pub aloft: f64,
    /// How long the water it has in the air takes to come down, s: its
    /// flight time, but where its landing comes suddenly nearer, shrinking
    /// no faster than time passes, since the water already past there is
    /// still on its way down.
    pub settling: f32,
}

impl Fall {
    /// The step across the edge it goes over.
    pub fn step(&self) -> Step {
        ORTHOGONAL[self.direction as usize]
    }

    /// The way its edge faces, level.
    pub fn face(&self) -> Vector2<f32> {
        let step = self.step();
        Vector2::new(step.di as f32, step.dk as f32)
    }

    /// The top of its water at the mouth of the opening it leaves by: over
    /// its own top by half of what the opening is taller than it is thick,
    /// since a jet leaves from the opening's middle.
    pub fn roof(&self) -> f32 {
        self.lip.y + 0.5 * (self.opening - self.depth).max(0.0)
    }

    /// How far its top falls, m.
    pub fn height(&self) -> f32 {
        (self.lip.y - self.foot.y).max(0.0)
    }

    /// How far out from its lip its top lands, level.
    pub fn throw(&self) -> Vector2<f32> {
        Vector2::new(self.foot.x - self.lip.x, self.foot.z - self.lip.z)
    }

    /// How long its water takes from its lip to its foot, s.
    pub fn flight_time(&self) -> f32 {
        fall_time(self.height())
    }

    /// The stretch of its fall that holds water, as seconds since the water
    /// there left the lip: from the last of it, at the lip while it pours,
    /// to the first of it, at the foot once it has had time to get there.
    pub fn flying(&self) -> (f32, f32) {
        (self.spent.unwrap_or(0.0), self.age.min(self.flight_time()))
    }

    /// How long after it leaves the lip the last of the water it has in
    /// the air is down, s.
    fn coming_down(&self) -> f32 {
        self.flight_time().max(self.settling)
    }

    /// Whether its water flies: it pours, and does not come out drowned.
    pub fn flies(&self) -> bool {
        self.spent.is_none() && !self.drowned
    }

    /// Whether its water is coming down at its foot.
    pub fn lands(&self) -> bool {
        self.age >= self.flight_time()
    }

    /// How much of the water it has in the air comes down over the next
    /// `dt` seconds, m³: none until the first of it has reached its foot;
    /// then, while it pours, what it holds in the air over the time that
    /// takes to come down (`settling`), and once spent, what is left evenly
    /// until the last of it is down.
    fn due(&self, dt: f32) -> f64 {
        let flight = self.flight_time();
        if self.age < flight {
            return 0.0;
        }
        let left = self.coming_down() - self.spent.unwrap_or(0.0);
        if left <= dt {
            self.aloft
        } else {
            self.aloft * (dt / left) as f64
        }
    }

    /// Whether this fall is drawn as `earlier`, the same fall as it was
    /// drawn before, still is: the same water over the same edge, to within
    /// what can be seen, begun and ended when that was.
    pub fn drawn_alike(&self, earlier: &Fall) -> bool {
        let other = earlier;
        const NEAR: f32 = 0.02;
        const SHARE: f32 = 0.05;
        let close = |a: f32, b: f32| (a - b).abs() <= NEAR + SHARE * a.abs().max(b.abs());
        self.from == other.from
            && self.direction == other.direction
            && self.driven == other.driven
            && self.drowned == other.drowned
            && self.spent.is_some() == other.spent.is_some()
            && self.age >= other.age
            && self.into.is_some() == other.into.is_some()
            && (self.lip - other.lip).norm() <= NEAR
            && (self.foot - other.foot).norm() <= NEAR + SHARE * self.height()
            && close(self.depth, other.depth)
            && close(self.opening, other.opening)
            && close(self.pour, other.pour)
    }
}

/// Every fall in the field, by the cell it leaves: those pouring, and those
/// spent whose last water is still coming down.
#[derive(Debug, Clone, Default)]
pub struct Falls {
    /// Per chunk slot: the falls leaving its cells, by cell and direction.
    chunks: Vec<Vec<Fall>>,
    /// Per chunk slot, for each of its falls: what it falls over.
    edges: Vec<Vec<Edge>>,
    /// Per chunk slot: how many times its cells had been laid out when its
    /// falls were last found (`FieldCells::generation`).
    generations: Vec<u32>,
    /// Each cell water came down in when the falls were last found, and
    /// the power it landed there with, per metre of its fall's lip, m³/s³.
    landings: Vec<(SpanIndex, f32)>,
}

/// What a fall goes over.
#[derive(Debug, Clone, Copy)]
enum Edge {
    /// A face (by its owning chunk's slot and its place there), to the cell
    /// across it, sending its water into `flight` (an index into
    /// `FieldCells::flights`) once it is followed down.
    Face {
        slot: u32,
        index: u32,
        below: SpanIndex,
        flight: Option<u32>,
    },
    /// The map's edge, down to the water at `level`, or into the void
    /// where there is none.
    Map { level: f32, void: bool },
    /// Nothing any more: it is spent.
    Spent,
}

impl Edge {
    /// The flight of the face it is, once its fall has been followed down.
    fn flight(&self) -> Option<u32> {
        match self {
            Edge::Face { flight, .. } => *flight,
            Edge::Map { .. } | Edge::Spent => None,
        }
    }
}

impl Falls {
    /// The fall from cell `from` over its edge in `direction` (an index
    /// into `ORTHOGONAL`), if its water falls there.
    pub fn get(&self, from: SpanIndex, direction: usize) -> Option<&Fall> {
        let list = self.chunks.get(from.slot as usize)?;
        let key = (from.flat, direction as u8);
        list.binary_search_by_key(&key, |f| (f.from.flat, f.direction))
            .ok()
            .map(|at| &list[at])
    }

    /// The falls leaving the cells of chunk `slot`, by cell and direction.
    pub fn of_slot(&self, slot: usize) -> &[Fall] {
        self.chunks.get(slot).map_or(&[], Vec::as_slice)
    }

    /// Every fall.
    pub fn iter(&self) -> impl Iterator<Item = &Fall> + '_ {
        self.chunks.iter().flatten()
    }

    /// The water in the air, m³.
    pub fn aloft(&self) -> f64 {
        self.iter().map(|fall| fall.aloft).sum()
    }

    /// Each cell water came down in when the falls were last found, and
    /// the power it landed there with, per metre of its fall's lip, m³/s³.
    pub fn landings(&self) -> &[(SpanIndex, f32)] {
        &self.landings
    }

    pub fn len(&self) -> usize {
        self.chunks.iter().map(Vec::len).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.chunks.iter().all(Vec::is_empty)
    }

    /// The fall whose lip runs on from `fall`'s at its end `end`
    /// (`edge_ends`), and which of that fall's ends meets it there: the two
    /// are one curtain. None where the fall ends, and where more lips meet
    /// at that corner than two falls can share. A pouring fall and a spent
    /// one are not one curtain: the spent one is on its way down. Nor are a
    /// jet that flies and one drowned at its hole's mouth.
    pub fn beside(
        &self,
        graph: &SpanGraph,
        cells: &FieldCells,
        fall: &Fall,
        end: usize,
    ) -> Option<(&Fall, usize)> {
        let (other, meets) = self.next_round(graph, cells, fall, end)?;
        let (back, _) = self.next_round(graph, cells, other, meets)?;
        let mutual = back.from == fall.from && back.direction == fall.direction;
        let alike = other.spent.is_some() == fall.spent.is_some() && other.drowned == fall.drowned;
        (mutual && alike).then_some((other, meets))
    }

    /// The fall `fall`'s lip would run on into at its end `end`, and that
    /// fall's end there, whether or not that fall would run on into this
    /// one: straight on, where the water beside it goes over the same way;
    /// else round an outer corner, where its own water goes over the edge
    /// beside this one; else round an inner corner, where the water across
    /// the corner falls into the same column.
    fn next_round(
        &self,
        graph: &SpanGraph,
        cells: &FieldCells,
        fall: &Fall,
        end: usize,
    ) -> Option<(&Fall, usize)> {
        let step = fall.step();
        let along = along_edge(step, edge_ends(step)[end]);
        let back = Step {
            di: -along.di,
            dk: -along.dk,
        };
        let own = span_of(cells, fall.from);
        for beside in graph.joined(own, along) {
            let cell = cells.index(graph, beside.span.column, beside.span.ordinal);
            if let Some(next) = cell.and_then(|c| self.get(c, fall.direction as usize)) {
                return Some((next, end_towards(step, back)));
            }
        }
        if let Some(next) = self.get(fall.from, direction_index(along)) {
            return Some((next, end_towards(along, step)));
        }
        let across = own.column.offset(step.di + along.di, step.dk + along.dk);
        if !graph.contains_column(across) {
            return None;
        }
        let inward = Step {
            di: -step.di,
            dk: -step.dk,
        };
        (0..graph.spans(across).len())
            .filter_map(|ordinal| cells.index(graph, across, ordinal as u8))
            .find_map(|cell| self.get(cell, direction_index(back)))
            .map(|next| (next, end_towards(back, inward)))
    }

    /// Find every fall again as the water stands `dt` seconds on, give each
    /// face its flight, and bring down the water that is due. Returns the
    /// volume that came down outside the world, m³. Only an awake chunk's
    /// faces carry water, and every flight is found afresh, so none outlives
    /// the cells it names.
    ///
    /// Each fall's velocity is first taken from its own water, then evened
    /// out along its lip with the falls it runs on into: a cell's velocity
    /// at a brink is the mean of a few faces, and thrown each its own way
    /// the falls of one lip would cross in the air. Only then is each
    /// followed down.
    ///
    /// A fall found where one was before is that fall, older, with the
    /// water it had in the air, and is held to by a margin (`HELD_STEP`,
    /// `JetLines`). One whose face still pours its way, though the water no
    /// longer meets the lines a fall is judged by, is kept on as it was for
    /// a moment (`GRACE`). One no longer found is spent, and kept while its
    /// water is still coming down.
    ///
    /// Falls that fly at each other across a gap meet there (`meet`), and
    /// a fall landing under roofs sends no more into the air than the body
    /// of cells it lands in has room for (`make_room`).
    ///
    /// `brinks` are the faces to look at: those the solver found a step
    /// across as it last stepped. None looks at every awake chunk's faces,
    /// as after an edit, when the solver has not seen the ground as it is.
    pub(super) fn find(
        &mut self,
        graph: &SpanGraph,
        cells: &mut FieldCells,
        brinks: Option<&[FaceRef]>,
        dt: f32,
    ) -> f64 {
        let count = cells.chunks.len();
        self.take_flights(cells);
        cells.pushes.clear();
        let mut found: Vec<Vec<(Fall, Edge)>> = vec![Vec::new(); count];
        let every: Vec<FaceRef>;
        let brinks = match brinks {
            Some(brinks) => brinks,
            None => {
                every = (cells.chunks.iter().enumerate())
                    .filter(|(_, chunk)| chunk.awake)
                    .flat_map(|(slot, chunk)| {
                        (0..chunk.faces.len() as u32).map(move |index| FaceRef {
                            slot: slot as u32,
                            index,
                        })
                    })
                    .collect();
                &every
            }
        };
        // The faces of the falls found the step before are looked at again
        // whatever the solver saw across them, so each is held to by a
        // margin (`HELD_STEP`).
        let laid_again = self.laid_again(cells);
        let held = self.held_faces(&laid_again);
        for &FaceRef { slot, index } in brinks.iter().chain(&held) {
            let chunk = &cells.chunks[slot as usize];
            let Some(face) = chunk.faces.get(index as usize).filter(|_| chunk.awake) else {
                continue;
            };
            if face.q.abs() < MIN_POUR {
                continue;
            }
            let owner = SpanIndex { slot, flat: face.a };
            let (upper, lower, sign) = if face.q > 0.0 {
                (owner, face.b, 1.0)
            } else {
                (face.b, owner, -1.0)
            };
            let direction = direction_of(face.axis, sign);
            let was = self.was(&laid_again, upper, lower, direction);
            // Water falls only where it pours between surfaces a step apart.
            // A jet found the step before is judged by its lines alone: the
            // step out of a hole run full is the height its top stands clear.
            let step = chunk.surface(face.a as usize) - cells.surface(face.b);
            let least = match was {
                Some(was) if was.driven => 0.0,
                Some(_) => HELD_STEP,
                None => LEAST_STEP,
            };
            if step.abs() < least {
                continue;
            }
            // Nor into water pressed against a roof, which has no surface
            // to fall to: the two are one body, run full.
            if cells.full(lower) {
                continue;
            }
            let head = driving_head(cells, upper, face, face.q > 0.0);
            let pour = match was {
                Some(was) => {
                    let take = 1.0 - (-dt / POUR_EASE).exp();
                    was.pour + (face.q.abs() - was.pour) * take
                }
                None => face.q.abs(),
            };
            let lines = JetLines::after(was);
            let Some(mut fall) =
                over_face(graph, cells, upper, lower, direction, pour, head, lines)
            else {
                continue;
            };
            if fall.driven && !fall.drowned {
                match met(graph, cells, &fall, lower, lines.flies) {
                    None => {}
                    Some(level) if fall.roof() - level >= lines.seen => fall.drowned = true,
                    Some(_) => continue,
                }
            }
            let edge = Edge::Face {
                slot,
                index,
                below: lower,
                flight: None,
            };
            // Two spans across one edge: the cell's water falls to one.
            let list = &mut found[upper.slot as usize];
            let same = |f: &Fall| f.from == upper && f.direction == fall.direction;
            match list.iter_mut().find(|(f, _)| same(f)) {
                Some(held) if held.0.pour >= fall.pour => {}
                Some(held) => *held = (fall, edge),
                None => list.push((fall, edge)),
            }
        }
        self.keep_lapsed(cells, &laid_again, &mut found, dt);
        for (slot, (chunk, list)) in cells.chunks.iter().zip(&mut found).enumerate() {
            if !chunk.awake {
                continue;
            }
            for edge in &chunk.boundary {
                let from = SpanIndex {
                    slot: slot as u32,
                    flat: edge.cell,
                };
                let direction = direction_of(edge.axis, edge.outward);
                list.extend(off_edge(cells, from, direction, edge.q, edge.outside));
            }
        }
        self.chunks.resize_with(count, Vec::new);
        self.edges.resize_with(count, Vec::new);
        self.generations.resize(count, 0);
        let mut lost = 0.0;
        for (slot, mut list) in found.into_iter().enumerate() {
            let key = |(fall, _): &(Fall, Edge)| (fall.from.flat, fall.direction);
            list.sort_unstable_by_key(key);
            list.dedup_by_key(|held| key(held));
            let before = std::mem::take(&mut self.chunks[slot]);
            self.generations[slot] = cells.generation(slot);
            lost += carry_on(graph, cells, before, &mut list, laid_again[slot], dt);
            list.sort_unstable_by_key(key);
            (self.chunks[slot], self.edges[slot]) = list.into_iter().unzip();
        }
        // How low each fall's water may lie as it leaves: the floor of the
        // hole a jet fills, the underside of one that does not, the bed
        // under water going over a lip.
        let floors: Vec<Vec<f32>> = (self.chunks.iter())
            .map(|list| list.iter().map(|fall| fall.roof() - fall.opening).collect())
            .collect();
        for _ in 0..EVENINGS {
            self.even_out(graph, cells, &floors);
        }
        self.land(graph, cells, dt);
        lost + self.bring_down(graph, cells, dt)
    }

    /// Which chunk slots have been laid out again since the falls were
    /// last found: what was found there names cells and faces that may be
    /// others now.
    fn laid_again(&self, cells: &FieldCells) -> Vec<bool> {
        (0..cells.chunks.len())
            .map(|slot| self.generations.get(slot) != Some(&cells.generation(slot)))
            .collect()
    }

    /// Keep on, as it was, each pouring fall found the step before and not
    /// found again, while the water over its face still pours its way into
    /// a cell with room for it, for no longer than `GRACE` in all. None is
    /// kept across a chunk laid out again (`laid_again`).
    fn keep_lapsed(
        &self,
        cells: &FieldCells,
        laid_again: &[bool],
        found: &mut [Vec<(Fall, Edge)>],
        dt: f32,
    ) {
        for (list, edges) in self.chunks.iter().zip(&self.edges) {
            for (fall, edge) in list.iter().zip(edges) {
                let Edge::Face {
                    slot, index, below, ..
                } = *edge
                else {
                    continue;
                };
                let lapsed = fall.lapsed + dt;
                let stale = [slot, fall.from.slot, below.slot]
                    .iter()
                    .any(|&s| laid_again[s as usize]);
                if fall.spent.is_some() || lapsed > GRACE || stale {
                    continue;
                }
                let again = &mut found[fall.from.slot as usize];
                if again
                    .iter()
                    .any(|(f, _)| f.from == fall.from && f.direction == fall.direction)
                {
                    continue;
                }
                let chunk = &cells.chunks[slot as usize];
                let Some(face) = chunk.faces.get(index as usize) else {
                    continue;
                };
                let owner = SpanIndex { slot, flat: face.a };
                let pouring = if (owner, face.b) == (fall.from, below) {
                    face.q
                } else if (owner, face.b) == (below, fall.from) {
                    -face.q
                } else {
                    continue;
                };
                if !chunk.awake || pouring < MIN_POUR || cells.full(below) {
                    continue;
                }
                let edge = Edge::Face {
                    slot,
                    index,
                    below,
                    flight: None,
                };
                let fall = Fall {
                    pour: pouring,
                    age: 0.0,
                    lapsed,
                    aloft: 0.0,
                    ..*fall
                };
                again.push((fall, edge));
            }
        }
    }

    /// The faces of the pouring falls found the step before, but for those
    /// in chunks laid out again since (`laid_again`).
    fn held_faces(&self, laid_again: &[bool]) -> Vec<FaceRef> {
        (self.chunks.iter().zip(&self.edges))
            .flat_map(|(list, edges)| list.iter().zip(edges))
            .filter(|(fall, _)| fall.spent.is_none())
            .filter_map(|(_, edge)| match *edge {
                Edge::Face { slot, index, .. } if !laid_again[slot as usize] => {
                    Some(FaceRef { slot, index })
                }
                _ => None,
            })
            .collect()
    }

    /// The fall found the step before from `cell` over its edge in
    /// `direction`, to `across`, if it poured and neither cell's chunk has
    /// been laid out again since (`laid_again`).
    fn was(
        &self,
        laid_again: &[bool],
        cell: SpanIndex,
        across: SpanIndex,
        direction: u8,
    ) -> Option<&Fall> {
        if laid_again[cell.slot as usize] || laid_again[across.slot as usize] {
            return None;
        }
        self.get(cell, direction as usize)
            .filter(|fall| fall.spent.is_none())
    }

    /// Take what each fall's face has sent into the air since the falls
    /// were last found, and end last time's flights. A face laid again
    /// since has none.
    fn take_flights(&mut self, cells: &mut FieldCells) {
        for (list, edges) in self.chunks.iter_mut().zip(&self.edges) {
            for (fall, edge) in list.iter_mut().zip(edges) {
                if let Edge::Face {
                    flight: Some(flight),
                    ..
                } = *edge
                {
                    fall.aloft += cells.flights[flight as usize].aloft;
                }
            }
        }
        let mut flights = std::mem::take(&mut cells.flights);
        for flight in flights.drain(..) {
            let chunk = cells.chunks.get_mut(flight.face.slot as usize);
            if let Some(face) = chunk.and_then(|c| c.faces.get_mut(flight.face.index as usize)) {
                face.flight = None;
            }
        }
        cells.flights = flights;
    }

    /// Even each fall's velocity out with those of the falls its lip runs
    /// on into: half its own, a quarter each of theirs, its own again where
    /// its lip ends. Jets side by side even out how thick they are and how
    /// high their tops stand as well, and how high they fill their holes
    /// at the mouth, which only ever comes up to the jets' beside: each
    /// hole's water is driven its own way, by the flow law or as a column
    /// run full, and drawn each at its own height a row of them is a row of
    /// teeth. None sinks below its `floors`, the floor of its hole: round a
    /// corner, a jet's neighbour may come out of a hole a metre lower. A
    /// spent fall's water is already in the air, and keeps what it left
    /// with.
    fn even_out(&mut self, graph: &SpanGraph, cells: &FieldCells, floors: &[Vec<f32>]) {
        let evened: Vec<Vec<(Vector2<f32>, [f32; 3])>> = self
            .chunks
            .iter()
            .map(|list| {
                list.iter()
                    .map(|fall| {
                        let shape = |fall: &Fall| [fall.lip.y, fall.depth, fall.roof()];
                        let own = (fall.velocity, shape(fall));
                        if fall.spent.is_some() {
                            return own;
                        }
                        let beside = |end: usize| match self.beside(graph, cells, fall, end) {
                            Some((other, _)) if fall.driven && other.driven => {
                                (other.velocity, shape(other))
                            }
                            Some((other, _)) => (other.velocity, own.1),
                            None => own,
                        };
                        let (a, b) = (beside(0), beside(1));
                        (
                            0.5 * own.0 + 0.25 * (a.0 + b.0),
                            std::array::from_fn(|i| 0.5 * own.1[i] + 0.25 * (a.1[i] + b.1[i])),
                        )
                    })
                    .collect()
            })
            .collect();
        for ((list, evened), floors) in self.chunks.iter_mut().zip(evened).zip(floors) {
            for ((fall, (velocity, [top, depth, roof])), floor) in
                list.iter_mut().zip(evened).zip(floors)
            {
                // The top of its body at the mouth comes up to its
                // neighbours', and never down from the roof of its own hole.
                let roof = roof.max(fall.roof());
                let top = top.max(floor + depth);
                fall.velocity = velocity;
                fall.lip.y = top;
                fall.depth = depth;
                fall.opening = depth + 2.0 * (roof - top).max(0.0);
            }
        }
    }

    /// Follow every pouring fall down from its lip to where it lands, `dt`
    /// seconds after it was last followed, bring together those that fly
    /// at each other (`meet`), and give the face each goes over its flight.
    fn land(&mut self, graph: &SpanGraph, cells: &mut FieldCells, dt: f32) {
        for (list, edges) in self.chunks.iter_mut().zip(&self.edges) {
            for (fall, edge) in list.iter_mut().zip(edges) {
                follow(graph, cells, fall, edge);
            }
        }
        self.meet(graph, cells);
        for (list, edges) in self.chunks.iter_mut().zip(&mut self.edges) {
            for (fall, edge) in list.iter_mut().zip(edges) {
                if matches!(edge, Edge::Spent) {
                    continue;
                }
                fall.settling = fall.flight_time().max(fall.settling - dt);
                let Edge::Face {
                    slot,
                    index,
                    flight,
                    ..
                } = edge
                else {
                    continue;
                };
                // A drowned jet's water goes over its face as over any
                // other: none of it flies.
                if fall.drowned {
                    continue;
                }
                *flight = Some(cells.flights.len() as u32);
                let face = &mut cells.chunks[*slot as usize].faces[*index as usize];
                face.flight = *flight;
                let from_owner = fall.from.slot == *slot && fall.from.flat == face.a;
                cells.flights.push(Flight {
                    face: FaceRef {
                        slot: *slot,
                        index: *index,
                    },
                    from_owner,
                    aloft: 0.0,
                    room: f64::INFINITY,
                });
            }
        }
    }

    /// Bring together the falls that fly at each other across a gap, as
    /// water pouring into a hole off all its sides does, or into a ditch
    /// off both its banks. Two such streams meet in the air and come down
    /// together from there, neither carried on past the other: each is
    /// slowed to come down no further out than where they meet, a point
    /// across the gap that splits it as their level momenta do. Flown on
    /// through each other, they would cross and land beyond the far side:
    /// out under its roof, where the gap is a hole down into a cave.
    fn meet(&mut self, graph: &SpanGraph, cells: &FieldCells) {
        // How far out along its edge's facing a fall meets the world, and
        // its arc flown free comes down; and the level momentum it carries
        // out that way per metre of its lip.
        let out = |fall: &Fall, to: Point3<f32>| {
            Vector2::new(to.x - fall.lip.x, to.z - fall.lip.z)
                .dot(&fall.face())
                .max(0.0)
        };
        let thrust = |fall: &Fall| fall.pour * fall.velocity.dot(&fall.face()).max(0.0);
        let mut slowed = Vec::new();
        for (slot, (list, edges)) in self.chunks.iter().zip(&self.edges).enumerate() {
            for (index, (fall, edge)) in list.iter().zip(edges).enumerate() {
                let Edge::Face { below, .. } = *edge else {
                    continue;
                };
                if !fall.flies() {
                    continue;
                }
                let Some(other) = self.facing(graph, cells, fall, below) else {
                    continue;
                };
                let across = other.lip - fall.lip;
                let gap = Vector2::new(across.x, across.z).dot(&fall.face());
                let (own, theirs) = (out(fall, fall.landing), out(other, other.landing));
                if gap <= 0.0 || own + theirs <= gap {
                    continue;
                }
                let (push, against) = (thrust(fall), thrust(other));
                let share = if push + against > 0.0 {
                    push / (push + against)
                } else {
                    0.5
                };
                let meets = (share * gap).clamp(gap - theirs, own);
                let free = out(fall, fall.foot);
                if meets < own && free > 0.0 {
                    slowed.push((slot, index, meets / free));
                }
            }
        }
        for (slot, index, by) in slowed {
            let edge = self.edges[slot][index];
            let fall = &mut self.chunks[slot][index];
            fall.velocity *= by;
            follow(graph, cells, fall, &edge);
        }
    }

    /// The fall that flies back at `fall` across the gap it falls into,
    /// `below` being the cell across its edge: the nearest along its row
    /// going over the opposite edge of its cell, out to `MEET_REACH`
    /// columns.
    fn facing(
        &self,
        graph: &SpanGraph,
        cells: &FieldCells,
        fall: &Fall,
        below: SpanIndex,
    ) -> Option<&Fall> {
        let step = fall.step();
        let back = direction_index(Step {
            di: -step.di,
            dk: -step.dk,
        });
        let start = span_of(cells, below).column;
        for n in 1..=MEET_REACH {
            let column = start.offset(step.di * n, step.dk * n);
            if !graph.contains_column(column) {
                return None;
            }
            let found = (0..graph.spans(column).len())
                .filter_map(|ordinal| cells.index(graph, column, ordinal as u8))
                .find_map(|cell| self.get(cell, back).filter(|other| other.flies()));
            if found.is_some() {
                return found;
            }
        }
        None
    }

    /// Bring down what each fall's water is due over `dt` seconds (`due`),
    /// into the cells it lands in, and have each pouring fall whose water
    /// is landing push the water there the way it flies. What those cells
    /// have no room for under their roofs stays in the air, and each face
    /// is told how much more may follow it there (`Flight::room`). Returns
    /// the volume that came down outside the world, m³.
    fn bring_down(&mut self, graph: &SpanGraph, cells: &mut FieldCells, dt: f32) -> f64 {
        let mut lost = 0.0;
        self.landings.clear();
        let mut roofed: Vec<Roofed> = Vec::new();
        for (list, edges) in self.chunks.iter_mut().zip(&self.edges) {
            for (fall, edge) in list.iter_mut().zip(edges) {
                if fall.spent.is_some() {
                    fall.into = fall.into.and_then(|_| cell_under(graph, cells, fall));
                }
                let into = footprint(graph, cells, fall);
                let due = fall.due(dt);
                if into.is_empty() {
                    fall.aloft -= due;
                    lost += due;
                } else {
                    fall.aloft -= due - cells.pour_into(&into, due);
                }
                if fall.lands() && !into.is_empty() {
                    let speed = fall.velocity.norm();
                    let height = fall.lip.y - fall.landing.y;
                    let power = landing_power(fall.pour, speed, height) / into.len() as f32;
                    self.landings.extend(into.iter().map(|&cell| (cell, power)));
                }
                if fall.flies() && fall.lands() && fall.landing == fall.foot {
                    push(graph, cells, fall, &into);
                }
                // A drowned jet throws nothing into the air to land.
                let all_roofed = into.iter().all(|&cell| cells.ceiling(cell).is_finite());
                if !into.is_empty() && all_roofed && !fall.drowned {
                    // A spent fall sends no more: it asks only for what it
                    // still has in the air.
                    let pouring = fall.pour as f64 * COLUMN_SIZE as f64;
                    let steady = match fall.spent {
                        None => pouring * fall.coming_down() as f64,
                        Some(_) => 0.0,
                    };
                    roofed.push(Roofed {
                        behind: behind(graph, cells, fall),
                        flight: edge.flight(),
                        aloft: fall.aloft,
                        need: fall.aloft.max(steady),
                        into,
                    });
                }
            }
        }
        make_room(graph, cells, &roofed);
        lost
    }
}

/// A fall that comes down under roofs, as its landing's room is shared out.
struct Roofed {
    /// The cell it leaves, and those its arc flies through on its way
    /// down: its water does not spread back into them, which the water
    /// behind it fills as soon as it stops.
    behind: SmallVec<[SpanIndex; 8]>,
    /// Its face's flight, while it pours.
    flight: Option<u32>,
    /// The water it has in the air, m³.
    aloft: f64,
    /// The room it asks of where it lands, m³: the water it has in the
    /// air, or, while it pours, as much as it holds there pouring as it
    /// pours now, if more.
    need: f64,
    /// The cells it lands in, every one under a roof.
    into: SmallVec<[SpanIndex; 4]>,
}

/// The room under roofs that falls' water spreads through from where it
/// lands, and what the falls landing there ask of it.
struct Body {
    room: f64,
    need: f64,
}

/// What `fill` found from where a fall lands: room of its own, or the
/// cells of a body found from another fall's landing.
enum Filled {
    Room(f64),
    Joins(usize),
}

/// Tell each pouring fall's flight that comes down under roofs how much
/// more of its water has room to: its share of the room in the body of
/// roofed cells it lands in (`fill`), which the falls landing there share
/// as they ask for it, less what it already has in the air. A cave that
/// carries off what lands in it has room to spare for every one of them;
/// one filling up holds them back.
fn make_room(graph: &SpanGraph, cells: &mut FieldCells, roofed: &[Roofed]) {
    if roofed.is_empty() {
        return;
    }
    let need: f64 = roofed.iter().map(|fall| fall.need).sum();
    let enough = ROOM_TO_SPARE * need;
    let leaving: SmallVec<[SpanIndex; 16]> = (roofed.iter())
        .flat_map(|fall| fall.behind.iter().copied())
        .collect();
    let mut body_of: FxHashMap<SpanIndex, usize> = FxHashMap::default();
    let mut bodies: Vec<Body> = Vec::new();
    let mut joins = Vec::with_capacity(roofed.len());
    let mut reached = Vec::new();
    for fall in roofed {
        let body = match fall.into.iter().find_map(|cell| body_of.get(cell)) {
            Some(&body) => body,
            None => {
                let body = bodies.len();
                let filled = fill(
                    graph,
                    cells,
                    &fall.into,
                    &leaving,
                    enough,
                    body,
                    &mut body_of,
                    &mut reached,
                );
                match filled {
                    Filled::Room(room) => {
                        bodies.push(Body { room, need: 0.0 });
                        body
                    }
                    Filled::Joins(other) => {
                        for cell in &reached {
                            body_of.insert(*cell, other);
                        }
                        other
                    }
                }
            }
        };
        bodies[body].need += fall.need;
        joins.push(body);
    }
    for (fall, body) in roofed.iter().zip(joins) {
        let (Some(flight), body) = (fall.flight, &bodies[body]) else {
            continue;
        };
        if body.room.is_finite() && body.need > 0.0 {
            let share = body.room * fall.need / body.need;
            cells.flights[flight as usize].room = (share - fall.aloft).max(0.0);
        }
    }
}

/// Mark as body `body` the cells under roofs that water landing in `into`
/// spreads through: those, and the roofed cells with room joined to them,
/// but for the cells falls leave and fly through (`leaving`), each put in
/// `reached`. Finds their room, m³, until it covers `enough` or
/// `BODY_REACH` cells have been taken in; without end where they open onto
/// a cell under the sky, up which the water can rise. Where they reach a body already marked,
/// they are part of it. Counted only where it lands, the room would choke
/// a fall into a cave: a second's water in the air is more than the few
/// cells it lands in hold, while the cave round them carries it off as it
/// lands.
#[allow(clippy::too_many_arguments)]
fn fill(
    graph: &SpanGraph,
    cells: &FieldCells,
    into: &[SpanIndex],
    leaving: &[SpanIndex],
    enough: f64,
    body: usize,
    body_of: &mut FxHashMap<SpanIndex, usize>,
    reached: &mut Vec<SpanIndex>,
) -> Filled {
    reached.clear();
    reached.extend_from_slice(into);
    for &cell in into {
        body_of.insert(cell, body);
    }
    let mut room = 0.0;
    let mut at = 0;
    while at < reached.len() {
        let cell = reached[at];
        at += 1;
        room += cells.room(cell).max(0.0) * CELL_AREA;
        if room >= enough || reached.len() >= BODY_REACH {
            continue;
        }
        for beside in graph.orthogonal_neighbours(span_of(cells, cell)) {
            let Some(beside) = graph.index_of(beside.span) else {
                continue;
            };
            if leaving.contains(&beside) {
                continue;
            }
            match body_of.get(&beside) {
                Some(&other) if other != body => return Filled::Joins(other),
                Some(_) => continue,
                None => {}
            }
            if !cells.ceiling(beside).is_finite() {
                return Filled::Room(f64::INFINITY);
            }
            if !cells.full(beside) {
                body_of.insert(beside, body);
                reached.push(beside);
            }
        }
    }
    Filled::Room(room)
}

/// The cell `fall` leaves, and the cells its arc flies through from its lip
/// to where it meets the world, sampled every half column.
fn behind(graph: &SpanGraph, cells: &FieldCells, fall: &Fall) -> SmallVec<[SpanIndex; 8]> {
    let mut behind: SmallVec<[SpanIndex; 8]> = smallvec::smallvec![fall.from];
    let lip = Vector2::new(fall.lip.x, fall.lip.z);
    let speed = fall.velocity.norm();
    let far = (Vector2::new(fall.landing.x, fall.landing.z) - lip).norm();
    if speed <= 0.0 {
        return behind;
    }
    let paces = ((far / (0.5 * COLUMN_SIZE)).ceil() as usize).min(MAX_COLUMNS);
    let along = fall.velocity / speed;
    for k in 0..paces {
        let out = (k as f32 + 0.5) * far / paces as f32;
        let at = lip + along * (out + NUDGE);
        let height = fall.lip.y - 0.5 * GRAVITY * (out / speed).powi(2);
        let column = Column::containing(at.x, at.y);
        if !graph.contains_column(column) {
            continue;
        }
        let cell = graph
            .span_at(column, height)
            .and_then(|span| cells.index(graph, column, span.ordinal));
        if let Some(cell) = cell.filter(|cell| !behind.contains(cell)) {
            behind.push(cell);
        }
    }
    behind
}

/// Carry the falls of chunk `slot`, as they were `before`, on into those
/// `found` there now (in order of cell and direction), `dt` seconds later.
/// One found again is the same fall, older, with the water it had in the
/// air. One not found is spent: it is added to `found` while its water is
/// still coming down, and otherwise what it had left is brought down at
/// once. Returns the volume that came down outside the world, m³.
///
/// A chunk laid out again since (`relaid`) names its cells afresh, so its
/// falls are matched by where their lips are, and none of them is kept
/// spent, since nothing says which cell it left.
fn carry_on(
    graph: &SpanGraph,
    cells: &mut FieldCells,
    before: Vec<Fall>,
    found: &mut Vec<(Fall, Edge)>,
    relaid: bool,
    dt: f32,
) -> f64 {
    let mut lost = 0.0;
    let mut matched = vec![false; found.len()];
    let mut waiting = vec![false; found.len()];
    let mut spent = Vec::new();
    for old in before {
        let level = |fall: &Fall| Vector2::new(fall.lip.x, fall.lip.z);
        let again = if relaid {
            (0..matched.len()).find(|&i| {
                let now = &found[i].0;
                !matched[i]
                    && now.direction == old.direction
                    && (level(now) - level(&old)).norm() <= SAME_LIP
            })
        } else {
            let key = (old.from.flat, old.direction);
            found[..matched.len()]
                .binary_search_by_key(&key, |(f, _)| (f.from.flat, f.direction))
                .ok()
        };
        // A jet found drowned with water it threw still in the air is not
        // seen so yet: that water comes down first, where it was going,
        // as a spent fall's does.
        let thrown = |i: usize| found[i].0.drowned && !old.drowned && old.aloft > 0.0;
        match again {
            Some(i) if !thrown(i) => {
                matched[i] = true;
                let now = &mut found[i].0;
                now.age = old.age + dt;
                now.settling = old.settling;
                now.aloft += old.aloft;
            }
            drowned => {
                let mut ghost = Fall {
                    age: old.age + dt,
                    spent: Some(old.spent.unwrap_or(0.0) + dt),
                    ..old
                };
                if relaid || ghost.spent >= Some(ghost.coming_down()) {
                    ghost.into = ghost.into.and_then(|_| cell_under(graph, cells, &ghost));
                    let into = footprint(graph, cells, &ghost);
                    lost += set_down(graph, cells, &into, ghost.aloft);
                    if let Some(i) = drowned {
                        matched[i] = true;
                    }
                } else {
                    spent.push((ghost, Edge::Spent));
                    if let Some(i) = drowned {
                        matched[i] = true;
                        waiting[i] = true;
                    }
                }
            }
        }
    }
    let mut waiting = waiting.into_iter();
    found.retain(|_| !waiting.next().unwrap_or(false));
    found.extend(spent);
    lost
}

/// Follow `fall`'s arc from its lip over what it falls over, `edge`, to
/// where it comes down. A spent fall keeps the arc it had.
fn follow(graph: &SpanGraph, cells: &FieldCells, fall: &mut Fall, edge: &Edge) {
    match *edge {
        Edge::Face { below, .. } => {
            let landed = land(
                graph,
                cells,
                fall.lip,
                fall.velocity,
                fall.face(),
                below,
                MAX_COLUMNS,
            );
            fall.foot = landed.foot;
            fall.landing = landed.at;
            fall.into = landed.into;
            fall.into_void = false;
        }
        Edge::Map { level, void } => {
            let out = fall.velocity * fall_time(fall.lip.y - level);
            fall.foot = Point3::new(fall.lip.x + out.x, level, fall.lip.z + out.y);
            fall.landing = fall.foot;
            fall.into_void = void;
        }
        Edge::Spent => {}
    }
}

/// The cell under where a spent fall's water lands, as the ground stands
/// now: the cells it was found landing in may have been laid out again
/// since. Where it struck a wall, the column it came down the side of; and
/// where the ground now stands over the place it landed, the column it
/// came from.
fn cell_under(graph: &SpanGraph, cells: &FieldCells, fall: &Fall) -> Option<SpanIndex> {
    let level = Vector2::new(fall.landing.x, fall.landing.z);
    let along = fall.velocity.try_normalize(1.0e-6).unwrap_or(fall.face());
    let struck = fall.landing != fall.foot;
    let ahead = if struck { -NUDGE } else { NUDGE };
    [ahead, -ahead].into_iter().find_map(|nudge| {
        let at = level + along * nudge;
        let column = Column::containing(at.x, at.y);
        if !graph.contains_column(column) {
            return None;
        }
        // A height inside the ground is in no span, and the graph answers
        // with the one over it: water does not land up there.
        let span = graph.span_at(column, fall.landing.y)?;
        let cell = cells.index(graph, column, span.ordinal)?;
        (cells.bed(cell) <= fall.landing.y + FOOTPRINT_RISE).then_some(cell)
    })
}

/// Bring the last `volume` m³ of a fall down into the cells `into`, with
/// no fall left to hold in the air what they have no room for: that backs
/// up into the nearest cells joined to them that have room, ring by ring,
/// as a sheet no deeper than `BACKED_UP` in each, and what they still have
/// no room for is shared out over all of them. Poured whole into the first
/// open cell it reached, it would stand there as a column metres high.
/// Only where none within `SPILL_REACH` cells has room does it stand in
/// `into` over their roofs, until the solver has carried it off. Returns
/// what came down outside the world: all of it where there are no cells.
fn set_down(graph: &SpanGraph, cells: &mut FieldCells, into: &[SpanIndex], volume: f64) -> f64 {
    if into.is_empty() {
        return volume.max(0.0);
    }
    let mut left = cells.pour_into(into, volume);
    let mut reached: SmallVec<[SpanIndex; 32]> = into.iter().copied().collect();
    let mut ring = 0..reached.len();
    while left > 0.0 && !ring.is_empty() && reached.len() < SPILL_REACH {
        let outer = reached.len();
        for at in ring {
            for beside in graph.orthogonal_neighbours(span_of(cells, reached[at])) {
                let Some(beside) = graph.index_of(beside.span) else {
                    continue;
                };
                if !reached.contains(&beside) {
                    reached.push(beside);
                }
            }
        }
        ring = outer..reached.len();
        left = cells.pour_up_to(&reached[ring.clone()], left, BACKED_UP);
    }
    left = cells.pour_into(&reached[into.len()..], left);
    if left > 0.0 {
        let over = left / into.len() as f64 / CELL_AREA;
        for &cell in into {
            *cells.depth_mut(cell) += over;
        }
    }
    0.0
}

/// Have `fall`'s water push the water it lands in the way it flies: on each
/// axis it moves along, the face of each cell it lands in on the side it
/// is headed for takes a share of the momentum it lands with, until its
/// water runs at a share of the speed the fall lands at.
fn push(graph: &SpanGraph, cells: &mut FieldCells, fall: &Fall, into: &[SpanIndex]) {
    if into.is_empty() {
        return;
    }
    let share = CARRIED * fall.pour / (into.len() as f32 * COLUMN_SIZE);
    for &cell in into {
        for (speed, step) in [
            (fall.velocity.x, Step { di: 1, dk: 0 }),
            (fall.velocity.y, Step { di: 0, dk: 1 }),
        ] {
            if speed.abs() < PUSHING_SPEED {
                continue;
            }
            let ahead = if speed > 0.0 {
                step
            } else {
                Step {
                    di: -step.di,
                    dk: -step.dk,
                }
            };
            if let Some(face) = face_towards(graph, cells, cell, ahead) {
                cells.pushes.push(Push {
                    face,
                    rate: share * speed,
                    speed: (CARRIED * speed).clamp(-PUSHED_SPEED, PUSHED_SPEED),
                });
            }
        }
    }
}

/// The face between `cell` and the cell joined to it across `step`, if
/// there is one.
fn face_towards(
    graph: &SpanGraph,
    cells: &FieldCells,
    cell: SpanIndex,
    step: Step,
) -> Option<FaceRef> {
    let beside = graph
        .joined(span_of(cells, cell), step)
        .into_iter()
        .next()?;
    let other = cells.index(graph, beside.span.column, beside.span.ordinal)?;
    let (owner, across) = if step.di + step.dk > 0 {
        (cell, other)
    } else {
        (other, cell)
    };
    let faces = &cells.chunks[owner.slot as usize].faces;
    let first = faces.partition_point(|face| face.a < owner.flat);
    faces[first..]
        .iter()
        .take_while(|face| face.a == owner.flat)
        .position(|face| face.b == across)
        .map(|at| FaceRef {
            slot: owner.slot,
            index: (first + at) as u32,
        })
}

/// How many times each fall's velocity is evened out along its lip.
const EVENINGS: usize = 2;

/// The index into `ORTHOGONAL` of the step along `axis` in the way of
/// `sign`.
fn direction_of(axis: Axis, sign: f32) -> u8 {
    match (axis, sign > 0.0) {
        (Axis::X, true) => 0,
        (Axis::Z, true) => 1,
        (Axis::X, false) => 2,
        (Axis::Z, false) => 3,
    }
}

/// The index into `ORTHOGONAL` of an orthogonal step.
fn direction_index(step: Step) -> usize {
    ORTHOGONAL
        .iter()
        .position(|s| *s == step)
        .expect("an orthogonal step")
}

/// Which end (`edge_ends`) of the edge across `step` lies towards `along`.
fn end_towards(step: Step, along: Step) -> usize {
    edge_ends(step)
        .iter()
        .position(|&sides| along_edge(step, sides) == along)
        .expect("an edge has an end each way along it")
}

/// The span a cell is.
fn span_of(cells: &FieldCells, at: SpanIndex) -> SpanRef {
    let chunk = &cells.chunks[at.slot as usize];
    let (local, ordinal) = chunk.column_of(at.flat as usize);
    SpanRef::key(chunk.coord.column(local), ordinal)
}

/// The middle of the edge of `column` facing `face`, x and z.
fn edge_middle(column: Column, face: Vector2<f32>) -> Vector2<f32> {
    let (cx, cz) = column.centre();
    Vector2::new(cx, cz) + face * (0.5 * COLUMN_SIZE)
}

/// The surface of the open water driving the water of cell `upper` out
/// over `face` (from its owner, or to it), if any stands over the opening.
/// Under open sky that is `upper`'s own, where it stands over the face's
/// roof: the opening is a slot down in its side. Under a roof it is the
/// first cell under open sky back along the face's axis, through cells
/// that are roofed, if its water stands over `upper`'s roof.
fn driving_head(
    cells: &FieldCells,
    upper: SpanIndex,
    face: &Face,
    from_owner: bool,
) -> Option<f32> {
    let roof = cells.ceiling(upper);
    if !roof.is_finite() {
        let head = cells.surface(upper) as f32;
        return (head > face.roof + PRESSED_HEAD).then_some(head);
    }
    let mut link = if from_owner { face.behind } else { face.ahead };
    for _ in 0..LONGEST_CONDUIT {
        let back = link?;
        let behind = &cells.chunks[back.slot as usize].faces[back.index as usize];
        let (cell, further) = if from_owner {
            let owner = SpanIndex {
                slot: back.slot,
                flat: behind.a,
            };
            (owner, behind.behind)
        } else {
            (behind.b, behind.ahead)
        };
        if !cells.ceiling(cell).is_finite() {
            let head = cells.surface(cell) as f32;
            return (head > roof + PRESSED_HEAD).then_some(head);
        }
        link = further;
    }
    None
}

/// The lines a jet is judged by: fixed for a new one, and for one found
/// the step before moved each way from what it was by a margin, so that it
/// holds to it.
#[derive(Debug, Clone, Copy)]
struct JetLines {
    /// How far its top must stand over the water outside for it to fly, m.
    flies: f32,
    /// How far its top, or the top of the hole it fills, must stand over
    /// the water outside for it to be seen at all, m.
    seen: f32,
    /// How near its opening's roof the water behind must stand for it to
    /// fill its hole at the mouth, m.
    fills: f64,
}

impl JetLines {
    /// The lines for a jet that was `was` the step before.
    fn after(was: Option<&Fall>) -> Self {
        match was.filter(|was| was.driven) {
            None => Self {
                flies: JET_CLEARANCE,
                seen: JET_CLEARANCE,
                fills: FULL_SLACK,
            },
            Some(was) => Self {
                flies: if was.drowned {
                    JET_CLEARANCE + HELD_CLEARANCE
                } else {
                    JET_CLEARANCE - HELD_CLEARANCE
                },
                seen: JET_CLEARANCE - HELD_CLEARANCE,
                fills: if was.fills { HELD_FILL } else { FULL_SLACK },
            },
        }
    }
}

/// How water leaves a lip.
struct Leaving {
    /// The top of the water as it leaves.
    top: f32,
    /// How thick it is.
    depth: f32,
    /// How tall the opening it leaves by is.
    opening: f32,
    /// Whether it fills its hole at the mouth (`Fall::fills`).
    fills: bool,
    /// Its level velocity.
    velocity: Vector2<f32>,
    /// Whether open water behind it drives it out.
    driven: bool,
    /// Whether it comes out under the water outside (`Fall::drowned`).
    drowned: bool,
}

/// How the water of cell `upper` leaves over an edge facing `face` to the
/// cell `lower` across it, carrying `pour` m²/s, if it falls there.
///
/// Driven by open water standing at `head` over its roof, it is a jet out
/// of the opening between the two cells: from the opening's middle, at the
/// speed its head gives it there, as thick as carries its flow at that
/// speed. Its top need only come out clear of what lies outside. Otherwise it
/// goes over as it runs: off the top of its own water, or of as much of it
/// as the opening lets through, where that `falls`. Where it does neither,
/// but fills a hole whose top stands clear, it is a jet drowned
/// (`Fall::drowned`).
fn leaving(
    cells: &FieldCells,
    upper: SpanIndex,
    lower: SpanIndex,
    face: Vector2<f32>,
    pour: f32,
    head: Option<f32>,
    lines: JetLines,
) -> Option<Leaving> {
    if cells.depth(upper) <= WET_DEPTH {
        return None;
    }
    let roof = cells.ceiling(upper).min(cells.ceiling(lower)) as f64;
    let level = cells.surface(upper).min(roof);
    let depth = cells.depth(upper).min(level - cells.bed(upper) as f64);
    if depth <= WET_DEPTH {
        return None;
    }
    let below = cells.surface(lower);
    let running = pour / depth.max(MEASURED_DEPTH) as f32;
    let jet = head.and_then(|head| {
        let floor = cells.bed(upper).max(cells.bed(lower));
        let opening = level as f32 - floor;
        let middle = floor + 0.5 * opening;
        let driven = JET_SPEED_SHARE * (2.0 * GRAVITY * (head - middle).max(0.0)).sqrt();
        let speed = driven.max(running);
        let thickness = (pour / speed).min(opening);
        let top = middle + 0.5 * thickness;
        // Where the water behind it stands against the opening's roof it
        // fills the hole to its mouth, however thin it flies: it is seen
        // there, drowned, for as long as the top of the hole stands clear
        // of the water outside, though its own top does not.
        let pressed = cells.surface(upper) >= roof - lines.fills;
        let clear = |top: f32, by: f32| top - below as f32 >= by;
        let seen = clear(top, lines.seen) || (pressed && clear(floor + opening, lines.seen));
        (opening >= JET_OPENING && seen).then(|| Leaving {
            top,
            depth: thickness,
            opening: if pressed { opening } else { thickness },
            fills: pressed,
            velocity: launch(face, speed, Vector2::zeros()),
            driven: true,
            drowned: !clear(top, lines.flies),
        })
    });
    if jet.as_ref().is_some_and(|jet| !jet.drowned) {
        return jet;
    }
    let flow = cells.chunks[upper.slot as usize].velocity[upper.flat as usize];
    let over = falls(level, cells.bed(upper) as f64, below).then(|| Leaving {
        top: level as f32,
        depth: depth as f32,
        opening: depth as f32,
        fills: false,
        velocity: launch(face, running, flow),
        driven: false,
        drowned: false,
    });
    over.or(jet)
}

/// The fall from cell `upper` over its edge in `direction` to the cell
/// `lower` across it, carrying `pour` m²/s, if its water falls there: as it
/// leaves its lip, not yet followed down. `head` is the open water driving
/// it, if any (`driving_head`); `lines` what a jet is judged by.
#[allow(clippy::too_many_arguments)]
fn over_face(
    graph: &SpanGraph,
    cells: &FieldCells,
    upper: SpanIndex,
    lower: SpanIndex,
    direction: u8,
    pour: f32,
    head: Option<f32>,
    lines: JetLines,
) -> Option<Fall> {
    let step = ORTHOGONAL[direction as usize];
    let face = Vector2::new(step.di as f32, step.dk as f32);
    let leaving = leaving(cells, upper, lower, face, pour, head, lines)?;
    // Where the column below carries the ledge on, the water leaves where
    // the ledge really ends.
    let lower_span = graph.span(span_of(cells, lower));
    let ledge = 0.5 * (cells.bed(upper) + lower_span.bed);
    let reach = edge_ends(step).map(|sides| ledge_reach(step, sides, &lower_span.quarters, ledge));
    let out = 0.5 * (reach[0] + reach[1]) * COLUMN_SIZE;
    let middle = edge_middle(span_of(cells, upper).column, face) + face * out;
    let lip = Point3::new(middle.x, leaving.top, middle.y);
    Some(Fall {
        from: upper,
        direction,
        lip,
        reach,
        velocity: leaving.velocity,
        depth: leaving.depth,
        opening: leaving.opening,
        fills: leaving.fills,
        pour,
        driven: leaving.driven,
        drowned: leaving.drowned,
        foot: lip,
        landing: lip,
        into: None,
        into_void: false,
        age: 0.0,
        lapsed: 0.0,
        spent: None,
        aloft: 0.0,
        settling: 0.0,
    })
}

/// The fall from cell `from` off the map's edge in `direction`, carrying
/// `pour` m²/s to the water `outside` (the void, where there is none), if
/// its water falls there: as it leaves its lip, not yet followed down.
fn off_edge(
    cells: &FieldCells,
    from: SpanIndex,
    direction: u8,
    pour: f32,
    outside: Option<f32>,
) -> Option<(Fall, Edge)> {
    let depth = cells.depth(from);
    let level = cells.surface(from);
    let below = off_map(level, outside.map(f64::from));
    if pour < MIN_POUR || depth <= WET_DEPTH || !falls(level, cells.bed(from) as f64, below) {
        return None;
    }
    let step = ORTHOGONAL[direction as usize];
    let face = Vector2::new(step.di as f32, step.dk as f32);
    let middle = edge_middle(span_of(cells, from).column, face);
    let lip = Point3::new(middle.x, level as f32, middle.y);
    let speed = pour / depth.max(MEASURED_DEPTH) as f32;
    let flow = cells.chunks[from.slot as usize].velocity[from.flat as usize];
    let fall = Fall {
        from,
        direction,
        lip,
        reach: [0.0; 2],
        velocity: launch(face, speed, flow),
        depth: depth as f32,
        opening: depth as f32,
        fills: false,
        pour,
        driven: false,
        drowned: false,
        foot: lip,
        landing: lip,
        into: None,
        into_void: false,
        age: 0.0,
        lapsed: 0.0,
        spent: None,
        aloft: 0.0,
        settling: 0.0,
    };
    Some((
        fall,
        Edge::Map {
            level: below as f32,
            void: outside.is_none(),
        },
    ))
}

/// How many columns out from its hole a jet is followed to see whether it
/// comes out into the air.
const DROWNED_REACH: usize = 6;

/// The level of the water outside that the jet `fall`, bursting towards
/// the cell `below`, meets before its top has dropped `clearance`: water
/// heaped at the hole's mouth, or a pool risen to it. It is then no jet
/// flying but the hole's water running on into the water outside, which
/// its head must push aside. None for a jet that comes out into the air.
fn met(
    graph: &SpanGraph,
    cells: &FieldCells,
    fall: &Fall,
    below: SpanIndex,
    clearance: f32,
) -> Option<f32> {
    let landed = land(
        graph,
        cells,
        fall.lip,
        fall.velocity,
        fall.face(),
        below,
        DROWNED_REACH,
    );
    let into = landed.into?;
    (fall.lip.y - landed.foot.y < clearance).then(|| cells.surface(into) as f32)
}

/// How far above the level its water lands at a cell's floor may stand for
/// the cell to take a share of it, m: a thick stream spreads over the water
/// it comes down in, not up a bank beside it.
const FOOTPRINT_RISE: f32 = 0.1;

/// Most cells a stream comes down over.
const LONGEST_FOOTPRINT: usize = 8;

/// The cells a fall's water comes down in, once its arc has been followed:
/// the cell its top lands in, and those along its track back to where its
/// underside lands, a thickness's fall sooner. A thick stream lands over
/// metres of ground; poured into one cell, it would stand there as a
/// column of water. Empty where it leaves the world.
fn footprint(graph: &SpanGraph, cells: &FieldCells, fall: &Fall) -> SmallVec<[SpanIndex; 4]> {
    let Some(landed) = fall.into else {
        return SmallVec::new();
    };
    let mut into: SmallVec<[SpanIndex; 4]> = smallvec::smallvec![landed];
    let level = fall.foot.y;
    let lip = Vector2::new(fall.lip.x, fall.lip.z);
    let speed = fall.velocity.norm();
    let far = (Vector2::new(fall.landing.x, fall.landing.z) - lip).norm();
    let near = (speed * fall_time(fall.lip.y - fall.depth - level)).min(far);
    let pace = 0.5 * COLUMN_SIZE;
    let paces = (((far - near) / pace).ceil() as usize).min(2 * LONGEST_FOOTPRINT);
    if speed <= 0.0 || paces == 0 {
        return into;
    }
    let along = fall.velocity / speed;
    for k in 0..paces {
        let at = lip + along * (near + (k as f32 + 0.5) * (far - near) / paces as f32);
        let column = Column::containing(at.x, at.y);
        if !graph.contains_column(column) {
            continue;
        }
        let cell = graph
            .span_at(column, level)
            .and_then(|span| cells.index(graph, column, span.ordinal));
        if let Some(cell) = cell {
            let open = cells.bed(cell) <= level + FOOTPRINT_RISE;
            if open && !into.contains(&cell) && into.len() < LONGEST_FOOTPRINT {
                into.push(cell);
            }
        }
    }
    into
}

/// Where an arc comes down.
struct Landed {
    /// The end of the arc flown free, down to the water it lands in.
    foot: Point3<f32>,
    /// Where it meets the world.
    at: Point3<f32>,
    /// The cell that takes its water; none where it leaves the world.
    into: Option<SpanIndex>,
}

/// Where water leaving `lip` level at `velocity`, over an edge facing
/// `face`, comes down.
///
/// The arc is followed across the columns it passes over. In each it is
/// either still in the air over that column's water, or comes down in it,
/// or has met the side of ground that stands higher than it flies, or of
/// water pressed against a roof: then it runs down that wall into the
/// column it came from, and its arc is drawn on into the wall, which hides
/// the rest of it. An arc that meets a wall
/// before it has left its lip falls into `below`, the cell across its edge.
/// One still in the air after `columns` columns is taken to leave the
/// world.
fn land(
    graph: &SpanGraph,
    cells: &FieldCells,
    lip: Point3<f32>,
    velocity: Vector2<f32>,
    face: Vector2<f32>,
    below: SpanIndex,
    columns: usize,
) -> Landed {
    // A jet's top, evened with the jets beside it, may stand at or over
    // the roof of the cell it comes out into, and a height in the roof is
    // answered with the ground over it: a wall met at once.
    let lip = Point3::new(lip.x, lip.y.min(cells.ceiling(below) - UNDER_ROOF), lip.z);
    let start = Vector2::new(lip.x, lip.z) + face * NUDGE;
    let height = |t: f32| lip.y - 0.5 * GRAVITY * t * t;
    let at = |t: f32, y: f32| {
        let p = start + velocity * t;
        Point3::new(p.x, y, p.y)
    };
    let gone = |t: f32| Landed {
        foot: at(t, height(t)),
        at: at(t, height(t)),
        into: None,
    };

    let mut column = Column::containing(start.x, start.y);
    // The time the arc crosses the next column boundary on each axis, and
    // how long it takes to cross a column.
    let crossing = |position: f32, index: i32, speed: f32| {
        if speed > 0.0 {
            ((index + 1) as f32 * COLUMN_SIZE - position) / speed
        } else if speed < 0.0 {
            (index as f32 * COLUMN_SIZE - position) / speed
        } else {
            f32::INFINITY
        }
    };
    let mut next = Vector2::new(
        crossing(start.x, column.i, velocity.x),
        crossing(start.y, column.k, velocity.y),
    );
    let across = Vector2::new(
        COLUMN_SIZE / velocity.x.abs().max(1.0e-9),
        COLUMN_SIZE / velocity.y.abs().max(1.0e-9),
    );

    let mut entered = 0.0f32;
    // The cell the arc last flew over, and its water's level.
    let mut flown: Option<(SpanIndex, f32)> = None;
    for _ in 0..columns {
        let entering = height(entered);
        if !graph.contains_column(column) {
            return gone(entered);
        }
        let Some(cell) = graph
            .span_at(column, entering)
            .and_then(|span| cells.index(graph, column, span.ordinal))
        else {
            return gone(entered);
        };
        let level = (cells.surface(cell) as f32).min(cells.ceiling(cell));
        // The side of higher ground, or of water pressed against a roof,
        // which has no surface to come down on.
        if entering < cells.bed(cell) || (entering < level && cells.full(cell)) {
            return match flown {
                Some((cell, level)) => Landed {
                    foot: at(fall_time(lip.y - level).max(entered), level.min(entering)),
                    at: at(entered, entering),
                    into: Some(cell),
                },
                None => {
                    let level = (cells.surface(below) as f32).min(lip.y);
                    Landed {
                        foot: at(0.0, level),
                        at: at(0.0, level),
                        into: Some(below),
                    }
                }
            };
        }
        let leaving = next.x.min(next.y);
        if height(leaving) <= level {
            // Water standing higher than the arc enters the column at takes
            // it there, at the height it comes in.
            let foot = at(
                fall_time(lip.y - level).clamp(entered, leaving),
                level.min(entering),
            );
            return Landed {
                foot,
                at: foot,
                into: Some(cell),
            };
        }
        flown = Some((cell, level));
        entered = leaving;
        if next.x <= next.y {
            column = column.offset(velocity.x.signum() as i32, 0);
            next.x += across.x;
        } else {
            column = column.offset(0, velocity.y.signum() as i32);
            next.y += across.y;
        }
    }
    gone(entered)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::field::conduits;
    use crate::water::field::solver::FieldSolver;
    use crate::water::field::testing::{cell, chunk_of, open, roofed, stacked};
    use crate::water::geometry::CHUNK_COLUMNS;

    /// One chunk whose floor is `floor(i)` along x, the same on every row,
    /// awake, with faces laid.
    fn terrace(floor: impl Fn(i32) -> f32) -> (SpanGraph, FieldCells) {
        chunk_of(|i, _| open(floor(i)))
    }

    /// Set the discharge of the +x face of cell (i, k).
    fn pour(cells: &mut FieldCells, i: i32, k: i32, q: f32) {
        let flat = cell(i, k).flat;
        let face = cells.chunks[0]
            .faces
            .iter_mut()
            .find(|f| f.a == flat && f.axis == Axis::X)
            .expect("a face to the +x side");
        face.q = q;
    }

    /// A cliff 4 m high at x = 2 m: columns 0..4 stand on it.
    fn cliff() -> (SpanGraph, FieldCells) {
        terrace(|i| if i < 4 { 4.0 } else { 0.0 })
    }

    #[test]
    fn water_over_a_cliff_lands_out_from_its_foot_by_how_fast_it_left() {
        let (graph, mut cells) = cliff();
        *cells.depth_mut(cell(3, 5)) = 0.5;
        pour(&mut cells, 3, 5, 1.0);
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let fall = *found.get(cell(3, 5), 0).expect("it falls to +x");
        // 1 m²/s over 0.5 m is 2 m/s; 4.5 m takes √(2·4.5/g) s.
        let flight = fall_time(4.5);
        assert!((fall.lip - Point3::new(2.0, 4.5, 2.75)).norm() < 1.0e-5);
        assert!(
            (fall.foot.x - (2.0 + 2.0 * flight)).abs() < 1.0e-2,
            "{:?}",
            fall.foot
        );
        assert!(fall.foot.y.abs() < 1.0e-5);
        let landed = Column::containing(fall.foot.x, fall.foot.z);
        assert_eq!(fall.into, Some(cell(landed.i, landed.k)));
        assert!(landed.i > 4, "it clears the cell at the cliff's foot");
        let face = cells.chunks[0]
            .faces
            .iter()
            .find(|f| f.a == cell(3, 5).flat && f.axis == Axis::X)
            .unwrap();
        let flight = &cells.flights[face.flight.expect("its face flies") as usize];
        assert!(flight.from_owner);
        assert_eq!(footprint(&graph, &cells, &fall)[0], fall.into.unwrap());
        assert_eq!(found.len(), 1);
    }

    #[test]
    fn a_fall_its_water_still_pours_over_is_kept_on_through_a_surge_and_ends_after_it() {
        let (graph, mut cells) = cliff();
        *cells.depth_mut(cell(3, 5)) = 0.5;
        pour(&mut cells, 3, 5, 1.0);
        let mut found = Falls::default();
        let dt = 1.0 / 60.0;
        found.find(&graph, &mut cells, None, dt);
        let lip = found.get(cell(3, 5), 0).expect("it falls").lip;
        // The water below the cliff surges up to 0.3 m under its lip: no
        // fall would be found there, but this one is kept on as it was...
        for i in 4..CHUNK_COLUMNS {
            *cells.depth_mut(cell(i, 5)) = 4.2;
        }
        found.find(&graph, &mut cells, None, dt);
        let kept = found.get(cell(3, 5), 0).expect("kept on");
        assert!(kept.spent.is_none() && kept.lip == lip);
        // ...until the water has stood there longer than a surge would.
        for _ in 0..=(GRACE / dt) as usize {
            found.find(&graph, &mut cells, None, dt);
        }
        let ended = found.get(cell(3, 5), 0);
        assert!(ended.is_none_or(|fall| fall.spent.is_some()));
    }

    #[test]
    fn falls_into_a_gap_off_both_its_sides_meet_and_come_down_together() {
        // Ground at 4 m but for a gap 2 m wide, from x = 2 m to 4 m, with
        // water 0.5 m deep pouring into it off both sides.
        let land_from = |left: f32, right: f32| -> (Fall, Fall) {
            let (graph, mut cells) = terrace(|i| if (4..8).contains(&i) { 0.0 } else { 4.0 });
            for i in [3, 8] {
                *cells.depth_mut(cell(i, 5)) = 0.5;
            }
            pour(&mut cells, 3, 5, left);
            pour(&mut cells, 7, 5, -right);
            let mut found = Falls::default();
            found.find(&graph, &mut cells, None, 0.0);
            let left = *found
                .get(cell(3, 5), 0)
                .expect("water falls in off the left");
            let right = *found.get(cell(8, 5), 2).expect("and off the right");
            (left, right)
        };
        // Alone, each would fly 1.9 m: on past where the other comes down.
        // Pouring alike, they meet in the middle...
        let (left, right) = land_from(1.0, 1.0);
        for fall in [left, right] {
            assert!((fall.foot.x - 3.0).abs() < 0.02, "{:?}", fall.foot);
            assert!(fall.foot.y.abs() < 1.0e-5);
        }
        // ...and the one carrying four times the momentum carries the
        // meeting four fifths of the way across.
        let (left, right) = land_from(2.0, 1.0);
        for fall in [left, right] {
            assert!((fall.foot.x - 3.6).abs() < 0.02, "{:?}", fall.foot);
        }
    }

    #[test]
    fn a_fall_into_a_cave_wider_than_where_it_lands_pours_on_unchecked() {
        // A ledge at 4 m over the mouth of a cave 6 m high and 6 m deep
        // that runs the chunk's width: a second's water in the air is more
        // than the cells it lands in hold, but nothing to the cave.
        let (graph, mut cells) = chunk_of(|i, _| match i {
            0..=3 => open(4.0),
            _ => roofed(0.0, 6.0),
        });
        let ledge: Vec<SpanIndex> = (4..=6).map(|k| cell(3, k)).collect();
        let mut solver = FieldSolver::default();
        let mut found = Falls::default();
        let dt = 1.0 / 60.0;
        let mut poured = 0;
        for _ in 0..(3.0 / dt) as usize {
            for &at in &ledge {
                *cells.depth_mut(at) = 1.0;
            }
            solver.step(&mut cells, dt);
            found.find(&graph, &mut cells, Some(solver.brinks()), dt);
            let pouring = |at: &SpanIndex| found.get(*at, 0).is_some_and(|f| f.spent.is_none());
            if poured > 0 || ledge.iter().all(pouring) {
                assert!(
                    ledge.iter().all(pouring),
                    "a fall stopped after {poured} steps"
                );
                poured += 1;
            }
        }
        assert!(poured > 150, "it poured for {poured} steps");
    }

    #[test]
    fn a_thick_stream_comes_down_over_the_ground_between_its_underside_and_its_top() {
        let (graph, mut cells) = cliff();
        *cells.depth_mut(cell(3, 5)) = 2.0;
        pour(&mut cells, 3, 5, 8.0);
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let fall = *found.get(cell(3, 5), 0).expect("it falls");
        let into = footprint(&graph, &cells, &fall);
        // Its top falls 6 m and its underside 4 m, at 4 m/s: 0.8 m apart.
        assert!(into.len() >= 2, "{into:?}");
        assert_eq!(into[0], fall.into.unwrap());
        let nearest = (into.iter().map(|c| c.flat % CHUNK_COLUMNS as u32))
            .min()
            .unwrap();
        let landed = Column::containing(fall.foot.x, fall.foot.z);
        assert!(
            (nearest as i32) < landed.i,
            "none nearer than its top lands"
        );
        assert!(into.iter().all(|c| c.flat / CHUNK_COLUMNS as u32 == 5));
    }

    #[test]
    fn it_comes_down_on_the_water_standing_below() {
        let (graph, mut cells) = cliff();
        *cells.depth_mut(cell(3, 5)) = 0.5;
        for i in 4..CHUNK_COLUMNS {
            *cells.depth_mut(cell(i, 5)) = 1.5;
        }
        pour(&mut cells, 3, 5, 1.0);
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let fall = found.get(cell(3, 5), 0).expect("it falls");
        assert!((fall.foot.y - 1.5).abs() < 1.0e-5);
        assert!((fall.foot.x - (2.0 + 2.0 * fall_time(3.0))).abs() < 1.0e-2);
    }

    #[test]
    fn a_wall_in_its_way_sends_it_down_to_the_cell_before_the_wall() {
        // A slot 1 m wide at the cliff's foot, then ground as high as the
        // ledge again.
        let (graph, mut cells) = terrace(|i| if (4..6).contains(&i) { 0.0 } else { 4.0 });
        *cells.depth_mut(cell(3, 5)) = 0.5;
        pour(&mut cells, 3, 5, 1.0);
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let fall = found.get(cell(3, 5), 0).expect("it falls");
        assert_eq!(fall.into, Some(cell(5, 5)));
        // It strikes the wall 1 m out, 0.5 s on, and its arc runs on behind
        // the wall down to the slot's floor.
        assert!((fall.landing.x - 3.0).abs() < 1.0e-2, "{:?}", fall.landing);
        assert!((fall.landing.y - (4.5 - 0.5 * GRAVITY * 0.25)).abs() < 1.0e-2);
        assert!((fall.foot.x - (2.0 + 2.0 * fall_time(4.5))).abs() < 1.0e-2);
        assert!(fall.foot.y.abs() < 1.0e-5);
    }

    #[test]
    fn water_that_slopes_or_barely_pours_does_not_fall() {
        let (graph, mut cells) = terrace(|i| if i < 4 { 0.15 } else { 0.0 });
        *cells.depth_mut(cell(3, 5)) = 0.5;
        pour(&mut cells, 3, 5, 1.0);
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        assert!(found.is_empty(), "deep water over a low step slopes");

        let (graph, mut cells) = cliff();
        *cells.depth_mut(cell(3, 5)) = 0.5;
        pour(&mut cells, 3, 5, 0.5 * MIN_POUR);
        found.find(&graph, &mut cells, None, 0.0);
        assert!(found.is_empty(), "a seep is no fall");
    }

    #[test]
    fn falls_side_by_side_run_on_into_each_other_and_the_last_ends() {
        let (graph, mut cells) = cliff();
        for k in 5..8 {
            *cells.depth_mut(cell(3, k)) = 0.5;
            pour(&mut cells, 3, k, 1.0);
        }
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let middle = *found.get(cell(3, 6), 0).unwrap();
        // The +x edge's ends lie towards −z, then +z.
        let (before, meets) = found.beside(&graph, &cells, &middle, 0).unwrap();
        assert_eq!((before.from, meets), (cell(3, 5), 1));
        let (after, meets) = found.beside(&graph, &cells, &middle, 1).unwrap();
        assert_eq!((after.from, meets), (cell(3, 7), 0));
        let last = *found.get(cell(3, 7), 0).unwrap();
        assert!(found.beside(&graph, &cells, &last, 1).is_none());
    }

    #[test]
    fn a_fall_runs_on_round_an_outer_corner_into_its_own_cell_s_other_edge() {
        // A pillar one column wide standing 4 m over the floor.
        let (graph, mut cells) = chunk_of(|i, k| open(if (i, k) == (3, 5) { 4.0 } else { 0.0 }));
        *cells.depth_mut(cell(3, 5)) = 0.5;
        for face in &mut cells.chunks[0].faces {
            if face.a == cell(3, 5).flat {
                face.q = 1.0;
            }
        }
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let east = *found.get(cell(3, 5), 0).expect("over its +x edge");
        let north = *found.get(cell(3, 5), 1).expect("over its +z edge");
        // The +x edge's end towards +z meets the +z edge's end towards +x.
        let (round, meets) = found.beside(&graph, &cells, &east, 1).unwrap();
        assert_eq!((round.direction, meets), (north.direction, 1));
        let (back, meets) = found.beside(&graph, &cells, &north, 1).unwrap();
        assert_eq!((back.direction, meets), (east.direction, 1));
    }

    /// A dam 1 m thick at x = 1.5 m, holed from 1 m to 2.5 m up, holding
    /// water 5 m deep behind it over a dry floor in front: the hole runs
    /// full, carrying `q` m²/s.
    fn holed_dam(q: f32) -> (SpanGraph, FieldCells) {
        let (graph, mut cells) = chunk_of(|i, _| match i {
            3 | 4 => roofed(1.0, 2.5),
            _ => open(0.0),
        });
        for i in 0..3 {
            *cells.depth_mut(cell(i, 5)) = 5.0;
        }
        for i in 3..5 {
            *cells.depth_mut(cell(i, 5)) = 1.5;
            pour(&mut cells, i - 1, 5, q);
        }
        pour(&mut cells, 4, 5, q);
        (graph, cells)
    }

    #[test]
    fn water_driven_through_a_hole_bursts_out_as_a_jet_at_the_speed_of_its_head() {
        let (graph, mut cells) = holed_dam(4.0);
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let jet = *found.get(cell(4, 5), 0).expect("it bursts out of the hole");
        assert!(jet.driven);
        // 3.25 m of head over the middle of the opening.
        let speed = JET_SPEED_SHARE * (2.0 * GRAVITY * 3.25).sqrt();
        assert!((jet.velocity - Vector2::new(speed, 0.0)).norm() < 1.0e-3);
        assert!(
            (jet.depth - 4.0 / speed).abs() < 1.0e-4,
            "as thick as carries it"
        );
        assert!((jet.lip.y - (1.75 + 0.5 * jet.depth)).abs() < 1.0e-4);
        assert!(jet.foot.x > 2.5 + 3.0, "thrown well clear: {:?}", jet.foot);
        assert_eq!(found.len(), 1, "nothing falls inside the hole");
    }

    #[test]
    fn a_hole_half_drowned_by_its_own_water_still_throws_the_top_of_its_jet() {
        let (graph, mut cells) = holed_dam(4.0);
        // The jet's middle is at 1.75 m and its top a quarter metre higher.
        for i in 5..CHUNK_COLUMNS {
            *cells.depth_mut(cell(i, 5)) = 1.7;
        }
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let jet = *found.get(cell(4, 5), 0).expect("its top clears the water");
        assert!(jet.driven && !jet.drowned && jet.lip.y > 1.95);
        assert!((jet.roof() - 2.5).abs() < 1.0e-3, "it fills its hole");
        assert_eq!(cells.flights.len(), 1);
    }

    #[test]
    fn a_jet_drowned_under_its_hole_s_roof_is_kept_to_be_seen_and_none_of_it_flies() {
        let (graph, mut cells) = holed_dam(4.0);
        let mut found = Falls::default();
        // Water over the jet's top, at 2 m, but well under the hole's roof,
        // at 2.5 m.
        for i in 5..CHUNK_COLUMNS {
            *cells.depth_mut(cell(i, 5)) = 2.1;
        }
        found.find(&graph, &mut cells, None, 0.0);
        let jet = *found.get(cell(4, 5), 0).expect("the hole's top is clear");
        assert!(jet.driven && jet.drowned && jet.spent.is_none());
        assert!(cells.flights.is_empty(), "its water runs on over its face");
        // Water up to the hole's roof: nothing of it is left to see, once
        // it has stood there longer than a surge would.
        for i in 5..CHUNK_COLUMNS {
            *cells.depth_mut(cell(i, 5)) = 2.5;
        }
        found.find(&graph, &mut cells, None, 2.0 * GRACE);
        assert!(found.iter().all(|fall| fall.spent.is_some()));
    }

    #[test]
    fn a_jet_found_before_holds_to_how_it_was_until_the_water_outside_passes_its_line_by_a_margin()
    {
        // Each case: the water outside the hole as the jet is first found,
        // then as it is found again; and whether a jet found afresh in the
        // second is seen, and drowned.
        let found_at = |first: Option<f64>, then: f64| -> Option<Fall> {
            let (graph, mut cells) = holed_dam(4.0);
            let mut found = Falls::default();
            for outside in first.into_iter().chain([then]) {
                for i in 5..CHUNK_COLUMNS {
                    *cells.depth_mut(cell(i, 5)) = outside;
                }
                found.find(&graph, &mut cells, None, 0.0);
            }
            found
                .get(cell(4, 5), 0)
                .filter(|jet| jet.spent.is_none())
                .copied()
        };
        // The jet's top stands at 2.01 m and its hole's roof at 2.5 m. With
        // the water outside 0.2 m under its top, a new jet is drowned...
        let fresh = found_at(None, 1.8).expect("seen");
        assert!(fresh.drowned);
        // ...but one that was flying flies on.
        let flew = found_at(Some(1.7), 1.8).expect("seen");
        assert!(!flew.drowned);
        // With the water 0.1 m under the hole's roof, a new jet is not seen
        // at all, but one that was drowned under it still is.
        assert!(found_at(None, 2.4).is_none());
        let kept = found_at(Some(2.1), 2.4).expect("still seen");
        assert!(kept.drowned);
    }

    #[test]
    fn a_jet_that_meets_water_before_it_has_dropped_does_not_fly() {
        let (graph, mut cells) = holed_dam(4.0);
        // The cell at the hole's mouth is dry, but water stands as high as
        // the jet's top from the next one on: under the hole's roof.
        for i in 6..CHUNK_COLUMNS {
            *cells.depth_mut(cell(i, 5)) = 2.0;
        }
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let jet = found
            .get(cell(4, 5), 0)
            .expect("it is seen running into it");
        assert!(jet.drowned && cells.flights.is_empty());
        // Up to the hole's roof, and once it has stood there longer than a
        // surge would, it is no fall at all.
        for i in 6..CHUNK_COLUMNS {
            *cells.depth_mut(cell(i, 5)) = 2.5;
        }
        found.find(&graph, &mut cells, None, 2.0 * GRACE);
        assert!(found.iter().all(|fall| fall.spent.is_some()));
    }

    #[test]
    fn jets_side_by_side_even_out_how_high_their_tops_stand() {
        let (graph, mut cells) = chunk_of(|i, _| match i {
            3 | 4 => roofed(1.0, 2.5),
            _ => open(0.0),
        });
        for (k, q) in [(5, 4.0), (6, 8.0)] {
            for i in 0..3 {
                *cells.depth_mut(cell(i, k)) = 5.0;
            }
            for i in 3..5 {
                *cells.depth_mut(cell(i, k)) = 1.5;
                pour(&mut cells, i - 1, k, q);
            }
            pour(&mut cells, 4, k, q);
        }
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let (thin, thick) = (
            *found.get(cell(4, 5), 0).expect("a jet"),
            *found.get(cell(4, 6), 0).expect("a jet"),
        );
        assert!(thin.driven && thick.driven);
        // Each as thick as carries its own flow, their tops would stand a
        // quarter metre apart.
        let speed = JET_SPEED_SHARE * (2.0 * GRAVITY * 3.25).sqrt();
        let apart = 0.5 * (8.0 - 4.0) / speed;
        assert!(thick.lip.y > thin.lip.y);
        assert!(thick.lip.y - thin.lip.y < 0.3 * apart);
        assert!(thick.depth - thin.depth < 0.3 * 2.0 * apart);
    }

    #[test]
    fn a_jet_into_a_cave_sends_over_no_more_than_has_room_to_land() {
        // Water 4 m deep behind a dam holed 1 m to 1.5 m up, and beyond the
        // dam a cave under the same ground, its roof no higher than the
        // hole's: two columns long, so the jet has more in the air than
        // the cave has room for before the first of it is down. What lands
        // in the cave does not spread back into the hole it came through:
        // that fills from the lake as soon as the jet stops.
        let (graph, mut cells) = stacked(|i, k| match i {
            1..=6 if k == 5 => vec![open(0.0)],
            7..=8 if k == 5 => vec![roofed(1.0, 1.5), open(6.0)],
            9..=10 if k == 5 => vec![roofed(0.0, 1.5), open(6.0)],
            _ => vec![open(50.0)],
        });
        let row = |columns: std::ops::RangeInclusive<i32>| -> Vec<SpanIndex> {
            columns
                .map(|i| cells.index(&graph, Column::new(i, 5), 0).unwrap())
                .collect()
        };
        let (pond, hole, cave) = (row(1..=6), row(7..=8), row(9..=10));
        for &at in &pond {
            *cells.depth_mut(at) = 4.0;
        }
        let mut solver = FieldSolver::default();
        let mut found = Falls::default();
        let dt = 1.0 / 60.0;
        let mut jetted = 0.0;
        for _ in 0..(10.0 / dt) as usize {
            solver.step(&mut cells, dt);
            conduits::find(&mut cells, solver.pressed());
            let lost = found.find(&graph, &mut cells, Some(solver.brinks()), dt);
            assert_eq!(lost, 0.0);
            let room: f64 = (cave.iter())
                .map(|&at| cells.room(at).max(0.0) * CELL_AREA)
                .sum();
            // What a jet that has stopped still has in the air was sent
            // while there was room for it; the lake, driving its water on
            // through the hole, may fill the cave under it before it lands.
            let pouring = found.iter().filter(|fall| fall.spent.is_none());
            let aloft: f64 = pouring.map(|fall| fall.aloft).sum::<f64>()
                + cells.flights.iter().map(|f| f.aloft).sum::<f64>();
            if found.iter().any(|fall| fall.driven) {
                jetted += aloft;
            }
            assert!(aloft <= room + 0.05, "{aloft} m³ aloft, room for {room}");
            let roofed = hole.iter().chain(&cave);
            let over = roofed.map(|&at| -cells.room(at)).fold(0.0, f64::max);
            assert!(over < 0.01, "{over} m over a roof");
        }
        assert!(jetted > 0.0, "a jet flew into the cave");
        assert!(cave.iter().all(|&at| cells.full(at)), "and filled it");
        assert!(found.aloft() < 1.0e-9, "and all it threw came down");
    }

    #[test]
    fn deep_water_over_a_slot_in_its_side_jets_through_it_and_fills_the_pocket_beyond() {
        // A lake 10 m deep over a floor at 4 m. Beside its first five
        // columns the floor is a slab from 4.2 m to 4.5 m, with a pocket
        // under it down to 2 m: the lake reaches the pocket through a slot
        // 0.2 m high at its foot.
        let (graph, mut cells) = stacked(|i, k| match i {
            1..=5 if k == 5 => vec![open(4.0)],
            6..=8 if k == 5 => vec![roofed(2.0, 4.2), open(4.5)],
            _ => vec![open(50.0)],
        });
        let at = |i: i32, ordinal: u8| cells.index(&graph, Column::new(i, 5), ordinal).unwrap();
        let lake: Vec<SpanIndex> = (1..=5)
            .map(|i| at(i, 0))
            .chain((6..=8).map(|i| at(i, 1)))
            .collect();
        let pocket: Vec<SpanIndex> = (6..=8).map(|i| at(i, 0)).collect();
        for &cell in &lake {
            *cells.depth_mut(cell) = 14.0 - cells.bed(cell) as f64;
        }
        let mut solver = FieldSolver::default();
        let mut found = Falls::default();
        let dt = 1.0 / 60.0;
        let mut jetted = false;
        for _ in 0..(20.0 / dt) as usize {
            solver.step(&mut cells, dt);
            conduits::find(&mut cells, solver.pressed());
            let lost = found.find(&graph, &mut cells, Some(solver.brinks()), dt);
            assert_eq!(lost, 0.0);
            for fall in found.iter().filter(|fall| fall.spent.is_none()) {
                assert!(fall.driven, "nothing goes over the top of the lake");
                assert!(fall.lip.y < 4.2 && fall.lip.y > 4.0, "{:?}", fall.lip);
                assert!(pocket.contains(&fall.into.expect("it lands")));
                jetted = true;
            }
        }
        assert!(jetted, "a jet came through the slot");
        assert!(pocket.iter().all(|&cell| cells.full(cell)), "and filled it");
        assert_eq!(found.len(), 0, "then nothing falls");
    }

    #[test]
    fn the_last_of_a_fall_backs_up_out_of_a_full_cave() {
        // A cave two columns long, full, that opens onto a pool.
        let (graph, mut cells) = chunk_of(|i, k| match i {
            4..=5 if k == 5 => roofed(0.0, 1.0),
            6..=9 if k == 5 => open(0.0),
            _ => open(50.0),
        });
        for i in 4..=5 {
            *cells.depth_mut(cell(i, 5)) = 1.0;
        }
        let lost = set_down(&graph, &mut cells, &[cell(4, 5)], 0.5);
        assert_eq!(lost, 0.0);
        assert!(cells.room(cell(4, 5)).abs() < 1.0e-9 && cells.room(cell(5, 5)).abs() < 1.0e-9);
        // It spreads out of the cave as a sheet over the nearest cells
        // with room, not a column in the first of them.
        let held: f64 = cells.chunks[0].depth.iter().sum::<f64>() * CELL_AREA;
        assert!((held - 1.0).abs() < 1.0e-9, "all of it is down: {held}");
        let outside = (0..CHUNK_COLUMNS).flat_map(|i| (0..CHUNK_COLUMNS).map(move |k| (i, k)));
        for (i, k) in outside.filter(|&(i, k)| k != 5 || !(4..=5).contains(&i)) {
            assert!(cells.depth(cell(i, k)) <= BACKED_UP + 1.0e-9, "({i}, {k})");
        }
        assert!((cells.depth(cell(6, 5)) - BACKED_UP).abs() < 1.0e-9);
        assert!((cells.depth(cell(7, 5)) - BACKED_UP).abs() < 1.0e-9);

        // With nowhere in reach to back up into, it stands over the roof.
        let (graph, mut cells) = chunk_of(|i, k| match i {
            4..=5 if k == 5 => roofed(0.0, 1.0),
            _ => open(50.0),
        });
        for i in 4..=5 {
            *cells.depth_mut(cell(i, 5)) = 1.0;
        }
        set_down(&graph, &mut cells, &[cell(4, 5)], 0.5);
        assert!((cells.depth(cell(4, 5)) - 3.0).abs() < 1.0e-9);
    }

    #[test]
    fn an_arc_that_meets_water_pressed_under_a_roof_comes_down_before_it() {
        // Shallow open water below a ledge 1 m high, and from 2 m out a
        // cavity under a roof at 1 m, filled to it.
        let (graph, mut cells) = chunk_of(|i, _| match i {
            0..=3 => open(1.0),
            4..=7 => open(0.0),
            _ => roofed(0.0, 1.0),
        });
        for i in 4..CHUNK_COLUMNS {
            *cells.depth_mut(cell(i, 5)) = if i < 8 { 0.3 } else { 1.0 };
        }
        *cells.depth_mut(cell(3, 5)) = 0.5;
        // 5 m/s off the ledge: 2 m out its top has dropped 0.8 m, to
        // between the open water and the roof.
        pour(&mut cells, 3, 5, 2.5);
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let fall = *found.get(cell(3, 5), 0).expect("it falls");
        assert_eq!(fall.into, Some(cell(7, 5)), "the last cell under open sky");
        assert!((fall.landing.x - 4.0).abs() < 1.0e-2 && fall.landing != fall.foot);
    }

    #[test]
    fn a_hole_with_no_water_over_its_roof_pours_as_a_lip_does() {
        let (graph, mut cells) = holed_dam(1.0);
        for i in 0..3 {
            *cells.depth_mut(cell(i, 5)) = 2.2;
        }
        *cells.depth_mut(cell(4, 5)) = 0.6;
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        let fall = *found.get(cell(4, 5), 0).expect("it pours out");
        assert!(!fall.driven);
        assert!(
            (fall.lip.y - 1.6).abs() < 1.0e-5,
            "off the top of its water"
        );
    }

    /// Send 1 m²/s over the cliff's +x face of cell (3, 5) for `seconds`,
    /// in frames, finding the falls again after each, and holding the water
    /// below the cliff `pool` deep. Returns the volume that came down.
    fn pour_for(
        graph: &SpanGraph,
        cells: &mut FieldCells,
        found: &mut Falls,
        seconds: f32,
        pool: f64,
    ) -> f64 {
        let dt = 1.0 / 60.0;
        let mut landed = 0.0;
        for _ in 0..(seconds / dt).round() as usize {
            let flight = (cells.chunks[0].faces.iter())
                .find(|f| f.a == cell(3, 5).flat && f.axis == Axis::X)
                .and_then(|f| f.flight);
            if let Some(flight) = flight {
                cells.flights[flight as usize].aloft += 1.0 * COLUMN_SIZE as f64 * dt as f64;
            }
            landed += settle(graph, cells, found, dt, pool);
        }
        landed
    }

    /// Find the falls again `dt` seconds on, and take what came down out of
    /// the water below the cliff, which is held `pool` deep. Returns it.
    fn settle(
        graph: &SpanGraph,
        cells: &mut FieldCells,
        found: &mut Falls,
        dt: f32,
        pool: f64,
    ) -> f64 {
        let lost = found.find(graph, cells, None, dt);
        assert_eq!(lost, 0.0, "none leaves the world");
        let mut landed = 0.0;
        for i in 4..CHUNK_COLUMNS {
            for k in 0..CHUNK_COLUMNS {
                let depth = cells.depth_mut(cell(i, k));
                landed += (*depth - pool) * CELL_AREA;
                *depth = pool;
            }
        }
        landed
    }

    /// Water 0.5 m deep going over the cliff at 2 m/s into water `pool`
    /// deep, as its fall is first found.
    fn poured(pool: f64) -> (SpanGraph, FieldCells, Falls) {
        let (graph, mut cells) = cliff();
        *cells.depth_mut(cell(3, 5)) = 0.5;
        for i in 4..CHUNK_COLUMNS {
            for k in 0..CHUNK_COLUMNS {
                *cells.depth_mut(cell(i, k)) = pool;
            }
        }
        pour(&mut cells, 3, 5, 1.0);
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        (graph, cells, found)
    }

    #[test]
    fn its_water_comes_down_only_once_it_has_had_time_to_fall() {
        let (graph, mut cells, mut found) = poured(0.0);
        // 4.5 m takes 0.96 s.
        let landed = pour_for(&graph, &mut cells, &mut found, 0.8, 0.0);
        let fall = *found.get(cell(3, 5), 0).unwrap();
        assert!(!fall.lands() && (fall.age - 0.8).abs() < 0.02);
        assert_eq!(fall.flying(), (0.0, fall.age));
        assert_eq!(landed, 0.0, "none of it is down yet");
        assert!(
            (found.aloft() - 0.8 * 0.5).abs() < 0.02,
            "{}",
            found.aloft()
        );
        assert!(cells.pushes.is_empty());

        let landed = pour_for(&graph, &mut cells, &mut found, 4.0, 0.0);
        let fall = *found.get(cell(3, 5), 0).unwrap();
        assert!(fall.lands());
        // As much comes down as goes over: what it holds in the air is what
        // goes over in the time it takes to fall.
        let steady = 0.5 * fall.flight_time() as f64;
        assert!(
            (found.aloft() - steady).abs() < 0.05 * steady,
            "{}",
            found.aloft()
        );
        assert!((landed + found.aloft() - 4.8 * 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn a_fall_whose_landing_comes_suddenly_nearer_brings_its_water_down_no_faster() {
        let (graph, mut cells, mut found) = poured(0.0);
        pour_for(&graph, &mut cells, &mut found, 3.0, 0.0);
        let aloft = found.aloft();
        // The water below rises to 1.5 m under the lip: little over half
        // the flight. What was in the air is still a second from landing.
        for i in 4..CHUNK_COLUMNS {
            for k in 0..CHUNK_COLUMNS {
                *cells.depth_mut(cell(i, k)) = 3.0;
            }
        }
        let dt = 1.0 / 60.0;
        let landed = settle(&graph, &mut cells, &mut found, dt, 3.0);
        let fall = *found.get(cell(3, 5), 0).unwrap();
        assert!(fall.flight_time() < 0.6 && fall.settling > 0.9, "{fall:?}");
        let over = 0.5 * dt as f64;
        assert!(landed < 1.3 * over, "{landed} of {aloft} in a step");
        // Once the last of that is down, it holds what its shorter fall
        // does.
        pour_for(&graph, &mut cells, &mut found, 2.0, 3.0);
        let steady = 0.5 * found.get(cell(3, 5), 0).unwrap().flight_time() as f64;
        let left = found.aloft();
        assert!(
            (left - steady).abs() < 0.1 * steady,
            "{left} against {steady}"
        );
    }

    #[test]
    fn a_fall_that_stops_is_kept_until_the_last_of_its_water_is_down() {
        let (graph, mut cells, mut found) = poured(0.0);
        let mut landed = pour_for(&graph, &mut cells, &mut found, 3.0, 0.0);
        pour(&mut cells, 3, 5, 0.0);
        landed += settle(&graph, &mut cells, &mut found, 0.25, 0.0);
        let spent = *found
            .get(cell(3, 5), 0)
            .expect("its water is still falling");
        assert_eq!(spent.spent, Some(0.25));
        assert_eq!(spent.flying().0, 0.25);
        assert!(spent.aloft > 0.0);
        assert!(cells.flights.is_empty(), "nothing more goes over");
        assert!(cells.chunks[0].faces.iter().all(|f| f.flight.is_none()));
        landed += settle(&graph, &mut cells, &mut found, 1.0, 0.0);
        assert!(found.is_empty(), "the last of it is down");
        assert!((landed - 3.0 * 0.5).abs() < 1.0e-6, "{landed}");
    }

    #[test]
    fn a_fall_found_again_while_spent_is_the_same_fall() {
        let (graph, mut cells, mut found) = poured(0.0);
        pour_for(&graph, &mut cells, &mut found, 3.0, 0.0);
        pour(&mut cells, 3, 5, 0.0);
        settle(&graph, &mut cells, &mut found, 0.1, 0.0);
        let aloft = found.aloft();
        pour(&mut cells, 3, 5, 1.0);
        settle(&graph, &mut cells, &mut found, 0.0, 0.0);
        let fall = found.get(cell(3, 5), 0).unwrap();
        assert!(fall.spent.is_none() && fall.age > 3.0);
        assert_eq!(found.aloft(), aloft);
    }

    #[test]
    fn landing_water_pushes_the_water_it_lands_in_the_way_it_flies() {
        let (graph, mut cells, mut found) = poured(1.5);
        pour_for(&graph, &mut cells, &mut found, 2.0, 1.5);
        let fall = *found.get(cell(3, 5), 0).unwrap();
        let into = footprint(&graph, &cells, &fall);
        assert_eq!(cells.pushes.len(), into.len());
        for push in &cells.pushes {
            let face = &cells.chunks[0].faces[push.face.index as usize];
            assert_eq!(face.axis, Axis::X);
            let from = SpanIndex {
                slot: 0,
                flat: face.a,
            };
            assert!(into.contains(&from), "the far face of a cell it lands in");
        }
        // 1 m²/s at 2 m/s over cells half a metre long, half of it kept.
        let whole: f32 = cells.pushes.iter().map(|push| push.rate).sum();
        assert!((whole - CARRIED * 1.0 * 2.0 / COLUMN_SIZE).abs() < 1.0e-4);
    }

    #[test]
    fn a_chunk_falling_asleep_gives_up_its_flights() {
        let (graph, mut cells) = cliff();
        *cells.depth_mut(cell(3, 5)) = 0.5;
        pour(&mut cells, 3, 5, 1.0);
        let mut found = Falls::default();
        found.find(&graph, &mut cells, None, 0.0);
        cells.chunks[0].awake = false;
        found.find(&graph, &mut cells, None, 0.0);
        assert!(found.iter().all(|fall| fall.spent.is_some()));
        assert!(cells.chunks[0].faces.iter().all(|f| f.flight.is_none()));
        assert!(cells.flights.is_empty());
    }
}
