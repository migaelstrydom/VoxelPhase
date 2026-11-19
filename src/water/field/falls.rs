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
//! thinner than the opening by as much: a jet, though the same `Fall`.
//!
//! Water going over an edge flies. It does not arrive in the cell at the
//! foot of the cliff and run on from there: a breach's torrent lands metres
//! out, a trickle at the cliff's foot, and the ledges an arc clears stay
//! dry. Everything that shows a fall (the drawn curtain, the streams that
//! strike bodies, the spray where it lands) reads the same `Fall`, so the
//! water is seen to come down where it does.
//!
//! A fall has a life. New, its water has only fallen as far as its age lets
//! it, and none has landed. Once its water stops going over, what is
//! already in the air goes on falling: the fall is `spent`, and is kept
//! until the last of it is down.

use nalgebra::{Point3, Vector2};
use smallvec::SmallVec;

use crate::water::falling::{
    along_edge, edge_ends, fall_time, falls, launch, ledge_reach, off_map, GRAVITY,
};
use crate::water::geometry::{
    Column, SpanGraph, SpanIndex, SpanRef, Step, COLUMN_SIZE, ORTHOGONAL,
};

use super::cells::{FieldCells, CELL_AREA};
use super::faces::{Axis, Face, FaceRef, Flight, Push};
use super::field::WET_DEPTH;

/// Least flow over an edge, per metre of it, that is a fall, m²/s.
const MIN_POUR: f32 = 0.005;

/// Least step between two cells' surfaces that water can fall or burst
/// across, m: under both `falls`' step and a jet's clearance.
const LEAST_STEP: f64 = 0.25;

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
/// the water there along it, m/s.
const PUSHING_SPEED: f32 = 0.2;

/// How far apart, level, the lips of a fall found after an edit and one
/// found before it may lie for the two to be the same fall, m.
const SAME_LIP: f32 = 0.5 * COLUMN_SIZE;

/// Depth landing water must add to a sleeping chunk's cell to wake the
/// chunk, m.
const WAKE_DEPTH: f64 = 1.0e-5;

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
    /// What it carries over each metre of its lip, m²/s.
    pub pour: f32,
    /// Whether it bursts out of a hole, driven by open water standing over
    /// the hole's roof behind it, rather than running over a lip.
    pub driven: bool,
    /// Where the top of its water would come down, flying free: the end of
    /// its arc. Behind a wall, where one stands in its way.
    pub foot: Point3<f32>,
    /// Where its water meets the world: its foot, or where it strikes a
    /// wall on the way there.
    pub landing: Point3<f32>,
    /// The cell it lands in; none where it leaves the world.
    pub into: Option<SpanIndex>,
    /// How long its water has been going over, s: its water has fallen no
    /// further than that lets it.
    pub age: f32,
    /// How long ago its water stopped going over, s, while the last of it
    /// is still in the air. None while it pours.
    pub spent: Option<f32>,
    /// The water it has in the air, m³.
    pub aloft: f64,
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

    /// Whether its water is coming down at its foot.
    pub fn lands(&self) -> bool {
        self.age >= self.flight_time()
    }

    /// How much of the water it has in the air comes down over the next
    /// `dt` seconds, m³: none until the first of it has reached its foot;
    /// then, while it pours, what it holds in the air over the time that
    /// takes to fall, and once spent, what is left evenly until the last of
    /// it is down.
    fn due(&self, dt: f32) -> f64 {
        let flight = self.flight_time();
        if self.age < flight {
            return 0.0;
        }
        let left = flight - self.spent.unwrap_or(0.0);
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
            && self.spent.is_some() == other.spent.is_some()
            && self.age >= other.age
            && self.into.is_some() == other.into.is_some()
            && (self.lip - other.lip).norm() <= NEAR
            && (self.foot - other.foot).norm() <= NEAR + SHARE * self.height()
            && close(self.depth, other.depth)
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
    /// The map's edge, down to the water at `level`.
    Map { level: f32 },
    /// Nothing any more: it is spent.
    Spent,
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

    pub fn len(&self) -> usize {
        self.chunks.iter().map(Vec::len).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.chunks.iter().all(Vec::is_empty)
    }

    /// The fall whose lip runs on from `fall`'s at its end `end`
    /// (`edge_ends`), and which of that fall's ends meets it there: the two
    /// are one curtain. None where the fall ends, and where more lips meet
    /// at that corner than two falls can share.
    pub fn beside(
        &self,
        graph: &SpanGraph,
        cells: &FieldCells,
        fall: &Fall,
        end: usize,
    ) -> Option<(&Fall, usize)> {
        let (other, meets) = self.next_round(graph, cells, fall, end)?;
        let (back, _) = self.next_round(graph, cells, other, meets)?;
        (back.from == fall.from && back.direction == fall.direction).then_some((other, meets))
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
    /// water it had in the air. One no longer found is spent, and kept
    /// while its water is still coming down.
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
        for &FaceRef { slot, index } in brinks {
            let chunk = &cells.chunks[slot as usize];
            let Some(face) = chunk.faces.get(index as usize).filter(|_| chunk.awake) else {
                continue;
            };
            // Water falls only where it pours between surfaces a step apart.
            let step = chunk.surface(face.a as usize) - cells.surface(face.b);
            if face.q.abs() < MIN_POUR || step.abs() < LEAST_STEP {
                continue;
            }
            let owner = SpanIndex { slot, flat: face.a };
            let (upper, lower, sign) = if face.q > 0.0 {
                (owner, face.b, 1.0)
            } else {
                (face.b, owner, -1.0)
            };
            let direction = direction_of(face.axis, sign);
            let head = driving_head(cells, upper, face, face.q > 0.0);
            let pour = face.q.abs();
            let Some(fall) = over_face(graph, cells, upper, lower, direction, pour, head) else {
                continue;
            };
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
            let relaid = self.generations[slot] != cells.generation(slot);
            self.generations[slot] = cells.generation(slot);
            lost += carry_on(graph, cells, before, &mut list, relaid, dt);
            list.sort_unstable_by_key(key);
            (self.chunks[slot], self.edges[slot]) = list.into_iter().unzip();
        }
        for _ in 0..EVENINGS {
            self.even_out(graph, cells);
        }
        self.land(graph, cells);
        lost + self.bring_down(graph, cells, dt)
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
    /// its lip ends. A spent fall's water is already in the air, and keeps
    /// the velocity it left with.
    fn even_out(&mut self, graph: &SpanGraph, cells: &FieldCells) {
        let evened: Vec<Vec<Vector2<f32>>> = self
            .chunks
            .iter()
            .map(|list| {
                list.iter()
                    .map(|fall| {
                        if fall.spent.is_some() {
                            return fall.velocity;
                        }
                        let beside = |end: usize| {
                            self.beside(graph, cells, fall, end)
                                .map_or(fall.velocity, |(other, _)| other.velocity)
                        };
                        0.5 * fall.velocity + 0.25 * (beside(0) + beside(1))
                    })
                    .collect()
            })
            .collect();
        for (list, evened) in self.chunks.iter_mut().zip(evened) {
            for (fall, velocity) in list.iter_mut().zip(evened) {
                fall.velocity = velocity;
            }
        }
    }

    /// Follow every pouring fall down from its lip to where it lands, and
    /// give the face it goes over its flight.
    fn land(&mut self, graph: &SpanGraph, cells: &mut FieldCells) {
        for (list, edges) in self.chunks.iter_mut().zip(&mut self.edges) {
            for (fall, edge) in list.iter_mut().zip(edges) {
                match edge {
                    Edge::Face {
                        slot,
                        index,
                        below,
                        flight,
                    } => {
                        let landed =
                            land(graph, cells, fall.lip, fall.velocity, fall.face(), *below);
                        fall.foot = landed.foot;
                        fall.landing = landed.at;
                        fall.into = landed.into;
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
                        });
                    }
                    Edge::Map { level } => {
                        let out = fall.velocity * fall_time(fall.lip.y - *level);
                        fall.foot = Point3::new(fall.lip.x + out.x, *level, fall.lip.z + out.y);
                        fall.landing = fall.foot;
                    }
                    Edge::Spent => {}
                }
            }
        }
    }

    /// Bring down what each fall's water is due over `dt` seconds (`due`),
    /// into the cells it lands in, and have each pouring fall whose water
    /// is landing push the water there the way it flies. Returns the volume
    /// that came down outside the world, m³.
    fn bring_down(&mut self, graph: &SpanGraph, cells: &mut FieldCells, dt: f32) -> f64 {
        let mut lost = 0.0;
        for fall in self.chunks.iter_mut().flatten() {
            if fall.spent.is_some() {
                fall.into = fall.into.and_then(|_| cell_under(graph, cells, fall));
            }
            let into = footprint(graph, cells, fall);
            let due = fall.due(dt);
            fall.aloft -= due;
            lost += pour_into(cells, &into, due);
            if fall.spent.is_none() && fall.lands() && fall.landing == fall.foot {
                push(graph, cells, fall, &into);
            }
        }
        lost
    }
}

/// Carry the falls of one chunk, as they were `before`, on into those
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
        match again {
            Some(i) => {
                matched[i] = true;
                let now = &mut found[i].0;
                now.age = old.age + dt;
                now.aloft += old.aloft;
            }
            None => {
                let mut ghost = Fall {
                    age: old.age + dt,
                    spent: Some(old.spent.unwrap_or(0.0) + dt),
                    ..old
                };
                if relaid || ghost.spent >= Some(ghost.flight_time()) {
                    ghost.into = ghost.into.and_then(|_| cell_under(graph, cells, &ghost));
                    let into = footprint(graph, cells, &ghost);
                    lost += pour_into(cells, &into, ghost.aloft);
                } else {
                    spent.push((ghost, Edge::Spent));
                }
            }
        }
    }
    found.extend(spent);
    lost
}

/// The cell under where a spent fall's water lands, as the ground stands
/// now: the cells it was found landing in may have been laid out again
/// since. Where it struck a wall, the column it came down the side of.
fn cell_under(graph: &SpanGraph, cells: &FieldCells, fall: &Fall) -> Option<SpanIndex> {
    let level = Vector2::new(fall.landing.x, fall.landing.z);
    let along = fall.velocity.try_normalize(1.0e-6).unwrap_or(fall.face());
    let struck = fall.landing != fall.foot;
    let at = level + along * if struck { -NUDGE } else { NUDGE };
    let column = Column::containing(at.x, at.y);
    if !graph.contains_column(column) {
        return None;
    }
    let span = graph.span_at(column, fall.landing.y)?;
    cells.index(graph, column, span.ordinal)
}

/// Pour `volume` m³ into the cells `into`, each an equal share. Returns
/// what came down outside the world: all of it where there are none.
fn pour_into(cells: &mut FieldCells, into: &[SpanIndex], volume: f64) -> f64 {
    if volume <= 0.0 {
        return 0.0;
    }
    if into.is_empty() {
        return volume;
    }
    let each = volume / into.len() as f64 / CELL_AREA;
    for &cell in into {
        *cells.depth_mut(cell) += each;
        if each > WAKE_DEPTH && !cells.chunks[cell.slot as usize].awake {
            cells.wake(cell.slot as usize);
        }
    }
    0.0
}

/// Have `fall`'s water push the water it lands in the way it flies: on each
/// axis it moves along, the face of each cell it lands in on the side it
/// is headed for takes a share of the momentum it lands with.
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

/// The surface of the open water driving the roofed cell `upper`, whose
/// water goes over `face` (from its owner, or to it): the first cell under
/// open sky back along the face's axis, through cells that are roofed, if
/// its water stands over `upper`'s roof. None for a cell under open sky,
/// and where nothing stands over its roof.
fn driving_head(
    cells: &FieldCells,
    upper: SpanIndex,
    face: &Face,
    from_owner: bool,
) -> Option<f32> {
    let roof = cells.ceiling(upper);
    if !roof.is_finite() {
        return None;
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

/// How water leaves a lip.
struct Leaving {
    /// The top of the water as it leaves.
    top: f32,
    /// How thick it is.
    depth: f32,
    /// Its level velocity.
    velocity: Vector2<f32>,
    /// Whether open water behind it drives it out.
    driven: bool,
}

/// How the water of cell `upper` leaves over an edge facing `face` to the
/// cell `lower` across it, carrying `pour` m²/s, if it falls there.
///
/// Driven by open water standing at `head` over its roof, it is a jet out
/// of the opening between the two cells: from the opening's middle, at the
/// speed its head gives it there, as thick as carries its flow at that
/// speed. Its top need only come out clear of what lies outside. Otherwise it
/// goes over as it runs: off the top of its own water, where that `falls`.
fn leaving(
    cells: &FieldCells,
    upper: SpanIndex,
    lower: SpanIndex,
    face: Vector2<f32>,
    pour: f32,
    head: Option<f32>,
) -> Option<Leaving> {
    let depth = cells.depth(upper);
    let (level, below) = (cells.surface(upper), cells.surface(lower));
    if depth <= WET_DEPTH {
        return None;
    }
    let running = pour / depth.max(MEASURED_DEPTH) as f32;
    if let Some(head) = head {
        let floor = cells.bed(upper).max(cells.bed(lower));
        let opening = (level as f32).min(cells.ceiling(upper)) - floor;
        let middle = floor + 0.5 * opening;
        let driven = JET_SPEED_SHARE * (2.0 * GRAVITY * (head - middle).max(0.0)).sqrt();
        let speed = driven.max(running);
        let thickness = (pour / speed).min(opening);
        let top = middle + 0.5 * thickness;
        if opening >= JET_OPENING && top - below as f32 >= JET_CLEARANCE {
            return Some(Leaving {
                top,
                depth: thickness,
                velocity: launch(face, speed, Vector2::zeros()),
                driven: true,
            });
        }
    }
    let flow = cells.chunks[upper.slot as usize].velocity[upper.flat as usize];
    falls(level, cells.bed(upper) as f64, below).then(|| Leaving {
        top: level as f32,
        depth: depth as f32,
        velocity: launch(face, running, flow),
        driven: false,
    })
}

/// The fall from cell `upper` over its edge in `direction` to the cell
/// `lower` across it, carrying `pour` m²/s, if its water falls there: as it
/// leaves its lip, not yet followed down. `head` is the open water driving
/// it, if any (`driving_head`).
fn over_face(
    graph: &SpanGraph,
    cells: &FieldCells,
    upper: SpanIndex,
    lower: SpanIndex,
    direction: u8,
    pour: f32,
    head: Option<f32>,
) -> Option<Fall> {
    let step = ORTHOGONAL[direction as usize];
    let face = Vector2::new(step.di as f32, step.dk as f32);
    let leaving = leaving(cells, upper, lower, face, pour, head)?;
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
        pour,
        driven: leaving.driven,
        foot: lip,
        landing: lip,
        into: None,
        age: 0.0,
        spent: None,
        aloft: 0.0,
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
        pour,
        driven: false,
        foot: lip,
        landing: lip,
        into: None,
        age: 0.0,
        spent: None,
        aloft: 0.0,
    };
    Some((
        fall,
        Edge::Map {
            level: below as f32,
        },
    ))
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
/// or has met the side of ground that stands higher than it flies: then it
/// runs down that wall into the column it came from, and its arc is drawn
/// on into the wall, which hides the rest of it. An arc that meets a wall
/// before it has left its lip falls into `below`, the cell across its edge.
fn land(
    graph: &SpanGraph,
    cells: &FieldCells,
    lip: Point3<f32>,
    velocity: Vector2<f32>,
    face: Vector2<f32>,
    below: SpanIndex,
) -> Landed {
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
    for _ in 0..MAX_COLUMNS {
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
        let level = cells.surface(cell) as f32;
        if entering < cells.bed(cell) {
            // The side of higher ground.
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
            let foot = at(fall_time(lip.y - level).clamp(entered, leaving), level);
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
    use crate::water::field::testing::{cell, chunk_of, open, roofed};
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
        assert!(jet.driven && jet.lip.y > 1.95);
        for i in 5..CHUNK_COLUMNS {
            *cells.depth_mut(cell(i, 5)) = 1.9;
        }
        found.find(&graph, &mut cells, None, 0.0);
        assert!(
            found.iter().all(|fall| fall.spent.is_some()),
            "drowned, it runs on under the water"
        );
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
