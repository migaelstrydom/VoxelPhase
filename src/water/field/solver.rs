//! The flow law: shallow water on the span cells, by the local inertial
//! scheme (Bates, Horritt & Fewtrell 2010).
//!
//! ```text
//!   faces  q ← (q − g·h_f·dt·∂η/∂x) / (1 + g·dt·n²·|q| / h_f^(7/3))    momentum
//!          |q| ≤ Fr_max · h_f · √(g·h_f)                             a brink's limit
//!          q ← its conduit's, where it runs full under a roof        (`conduits`)
//!   cells  h += dt · Σ q·w / A                                       volume
//! ```
//!
//! Volume moves only across faces, and each face moves the same amount out
//! of one cell as into another, so water is conserved exactly up to
//! rounding. A cell never gives more than it holds, nor takes more than it
//! has room for under its ceiling: its outflows, or its inflows, are scaled
//! down together when they would.
//!
//! The cell that takes a face's water is the one across it, unless the
//! water falls there (`Face::flight`): then it goes into the air, the cell
//! at the foot of the cliff gets none of it, and its fall brings it down
//! where its arc does (`falls`), with the momentum it lands with
//! (`FieldCells::pushes`).

use super::cells::{FieldCells, CELL_AREA};
use super::faces::{Face, FaceRef};
use super::foam::{BREAKING_SPEED, MEASURED_DEPTH};
use crate::water::geometry::{SpanIndex, COLUMN_SIZE};

/// Gravity, m/s².
const G: f32 = 9.81;

/// Least step between two cells' surfaces at which their face is noted as
/// a brink, m: a little under the least water falls across
/// (`falls::LEAST_STEP`), since the water moves on before falls are found.
pub(super) const BRINK_STEP: f32 = 0.2;

/// How near a face's roof the higher of its cells' surfaces must stand for
/// the face to be noted as pressed, m: as near as a full cell's
/// (`conduits`).
const PRESSED_SLACK: f32 = 5.0e-3;

/// Steepest slope of the surface whose pull a push on a face may match:
/// landing water tilts the pool away from its foot by no more.
const PUSH_SLOPE: f32 = 0.05;

/// Face width and the distance between cell centres, m.
const DX: f32 = COLUMN_SIZE;

/// The constants of the flow law.
#[derive(Debug, Clone, Copy)]
pub struct FlowParams {
    /// Manning's roughness, s/m^(1/3): a natural channel's.
    pub manning_n: f32,
    /// The fastest flow a face carries, as a Froude number over its depth.
    /// The local inertial scheme has no upwinding and grows unstable above
    /// about 1; over a cliff's edge the cap stands in for critical flow.
    pub froude_limit: f32,
    /// Flow depth at a face below which it carries nothing, m.
    pub dry_depth: f32,
    /// Share of the stable step `dx / (√(g·h) + |u|)` a substep takes.
    pub courant: f32,
    /// Weight of a face's own discharge against the mean of the faces behind
    /// and ahead of it on its axis (de Almeida et al. 2012). Below 1 the
    /// update diffuses discharge along the flow, which damps the grid-scale
    /// oscillation the scheme shows over deep, slow water.
    pub theta: f32,
    /// Linear drag on slow, deep water, 1/s, scaled by
    /// `exp(−|u| / still_speed) · h / (h + still_depth)`. It stands in for
    /// the turbulence and wind that stop a pond sloshing, which the scheme
    /// has no term for: without it a seiche of a centimetre runs for minutes
    /// and keeps its water awake. Where Manning friction is strong, in a
    /// river metres a second or a sheet centimetres deep, it fades away.
    pub still_drag: f32,
    /// Speed over which `still_drag` fades, m/s.
    pub still_speed: f32,
    /// Depth at which `still_drag` is half on, m.
    pub still_depth: f32,
}

impl Default for FlowParams {
    fn default() -> Self {
        Self {
            manning_n: 0.035,
            froude_limit: 1.0,
            dry_depth: 1.0e-3,
            courant: 0.7,
            theta: 0.9,
            still_drag: 0.3,
            still_speed: 0.1,
            still_depth: 0.5,
        }
    }
}

/// What one frame's stepping did.
#[derive(Debug, Clone, Copy, Default)]
pub struct StepStats {
    pub substeps: u32,
    /// Chunks stepped, and faces evaluated over all substeps.
    pub chunks: u32,
    pub faces: u64,
    /// Volume let out through boundary faces, m³: positive to the void or
    /// the sea, negative when the sea flowed in.
    pub boundary_out: f64,
}

/// Steps the field. Holds only scratch space between calls.
#[derive(Debug, Default)]
pub struct FieldSolver {
    pub params: FlowParams,
    /// Per chunk, per cell: volume asked of the cell this substep, m³.
    demand: Vec<Vec<f64>>,
    /// Per chunk, per cell: volume offered to the cell this substep, as the
    /// givers can give it, m³.
    offered: Vec<Vec<f64>>,
    /// Per chunk, per cell: the share of its asked outflow a cell can give.
    give: Vec<Vec<f64>>,
    /// Per chunk, per cell: the share of its offered inflow a cell has room
    /// for under its ceiling.
    take: Vec<Vec<f64>>,
    /// Chunks stepped this frame: the awake ones and those owning faces into
    /// them.
    stepped: Vec<usize>,
    /// Chunks with a cell a stepped face gives to or takes from: the
    /// stepped ones and their +x and +z neighbours.
    touched: Vec<usize>,
    /// Per stepped chunk, per face: its discharge and the depth it carries
    /// it at (`carried`) at the start of the substep, so every face's
    /// update reads its neighbours' old values.
    previous: Vec<Vec<(f32, f32)>>,
    /// Per chunk: whether it is stepped this frame.
    is_stepped: Vec<bool>,
    /// The faces whose cells' surfaces stood a step apart as the last
    /// substep began: the only ones water may be falling over.
    brinks: Vec<FaceRef>,
    /// The faces whose water stood against their roofs as the last step
    /// ended: every conduit runs through at least one.
    pressed: Vec<FaceRef>,
    /// The faces whose water ran at `BREAKING_SPEED` or more as the last
    /// step ended: the only ones whose water may be breaking into foam.
    fast: Vec<FaceRef>,
}

impl FieldSolver {
    pub fn new(params: FlowParams) -> Self {
        Self {
            params,
            ..Self::default()
        }
    }

    /// The longest stable substep over the stepped chunks.
    fn stable_step(&self, cells: &FieldCells) -> f32 {
        let mut deepest = 0.0f64;
        for &slot in &self.stepped {
            for &h in &cells.chunks[slot].depth {
                deepest = deepest.max(h);
            }
        }
        let wave = (G * deepest as f32).sqrt() * (1.0 + self.params.froude_limit);
        if wave <= 0.0 {
            f32::INFINITY
        } else {
            self.params.courant * DX / wave
        }
    }

    /// Choose this frame's chunks: every awake chunk, and every chunk whose
    /// faces reach into one.
    fn choose_chunks(&mut self, cells: &FieldCells, width: i32) {
        self.stepped.clear();
        self.touched.clear();
        let count = cells.chunks.len();
        let neighbours = |slot: usize| {
            let x = slot as i32 % width;
            let east = (x + 1 < width).then_some(slot + 1);
            let north = Some(slot + width as usize).filter(|&s| s < count);
            [east, north]
        };
        let mut marked = vec![false; count];
        for slot in 0..count {
            let awake = |s: usize| cells.chunks[s].awake;
            if awake(slot) || neighbours(slot).into_iter().flatten().any(awake) {
                self.stepped.push(slot);
                for s in std::iter::once(slot).chain(neighbours(slot).into_iter().flatten()) {
                    if !marked[s] {
                        marked[s] = true;
                        self.touched.push(s);
                    }
                }
            }
        }
        for buffer in [
            &mut self.demand,
            &mut self.offered,
            &mut self.give,
            &mut self.take,
        ] {
            buffer.resize_with(count, Vec::new);
        }
        self.is_stepped.clear();
        self.is_stepped.resize(count, false);
        for &slot in &self.stepped {
            self.is_stepped[slot] = true;
        }
        for &slot in &self.touched {
            let n = cells.chunks[slot].depth.len();
            for buffer in [
                &mut self.demand,
                &mut self.offered,
                &mut self.give,
                &mut self.take,
            ] {
                buffer[slot].resize(n, 0.0);
            }
        }
    }

    /// Advance the field by `dt` seconds, in as many substeps as stability
    /// needs.
    pub fn step(&mut self, cells: &mut FieldCells, dt: f32) -> StepStats {
        self.choose_chunks(cells, cells.width());
        self.brinks.clear();
        self.pressed.clear();
        self.fast.clear();
        let mut stats = StepStats {
            chunks: self.stepped.len() as u32,
            ..StepStats::default()
        };
        if self.stepped.is_empty() {
            return stats;
        }
        let substeps = (dt / self.stable_step(cells)).ceil().max(1.0) as u32;
        let h = dt / substeps as f32;
        for _ in 0..substeps {
            stats.faces += self.update_faces(cells, h);
            self.limit_outflows(cells, h);
            stats.boundary_out += self.move_water(cells, h);
        }
        self.note_faces(cells);
        stats.substeps = substeps;
        stats
    }

    /// The momentum update on every stepped face, and on every conduit.
    fn update_faces(&mut self, cells: &mut FieldCells, dt: f32) -> u64 {
        let p = self.params;
        self.push_faces(cells, dt);
        self.previous.resize_with(cells.chunks.len(), Vec::new);
        for &slot in &self.stepped {
            let previous = &mut self.previous[slot];
            previous.clear();
            let a = |flat| SpanIndex {
                slot: slot as u32,
                flat,
            };
            previous.extend(
                cells.chunks[slot]
                    .faces
                    .iter()
                    .map(|f| carried(cells, a(f.a), f)),
            );
        }
        let mut count = 0;
        let mut brinks = std::mem::take(&mut self.brinks);
        brinks.clear();
        for &slot in &self.stepped {
            let mut faces = std::mem::take(&mut cells.chunks[slot].faces);
            let stepped = faces.iter_mut().zip(&self.previous[slot]);
            for (index, (face, &(_, depth))) in stepped.enumerate() {
                let in_conduit = face.conduit.is_some();
                if !in_conduit && depth <= p.dry_depth {
                    face.q = 0.0;
                    continue;
                }
                let a = SpanIndex {
                    slot: slot as u32,
                    flat: face.a,
                };
                let step = (cells.surface(face.b) - cells.surface(a)) as f32;
                if step.abs() >= BRINK_STEP {
                    brinks.push(FaceRef {
                        slot: slot as u32,
                        index: index as u32,
                    });
                }
                if !in_conduit {
                    let blended = self.blended(cells, face, depth);
                    face.q = inertial(blended, depth, step / DX, dt, &p);
                }
            }
            count += faces.len() as u64;
            let chunk = &mut cells.chunks[slot];
            chunk.faces = faces;
            for i in 0..chunk.boundary.len() {
                let face = chunk.boundary[i];
                let flat = face.cell as usize;
                let (bed, surface) = (chunk.bed[flat], chunk.surface(flat) as f32);
                chunk.boundary[i].q = boundary_discharge(bed, surface, face.outside, &p);
            }
        }
        self.brinks = brinks;
        self.drive_conduits(cells, dt);
        count
    }

    /// Note, as the step ends, the stepped faces whose water stands against
    /// their roofs and those whose water runs fast. Kept out of the momentum
    /// update's own loop, which it slows by a quarter.
    fn note_faces(&mut self, cells: &FieldCells) {
        self.pressed.clear();
        self.fast.clear();
        for &slot in &self.stepped {
            let chunk = &cells.chunks[slot];
            let faces = chunk.faces.iter().zip(&self.previous[slot]);
            for (index, (face, &(_, depth))) in faces.enumerate() {
                let at = FaceRef {
                    slot: slot as u32,
                    index: index as u32,
                };
                if face.roof.is_finite() {
                    let level = chunk.surface(face.a as usize).max(cells.surface(face.b));
                    if level as f32 >= face.roof - PRESSED_SLACK {
                        self.pressed.push(at);
                    }
                }
                let running = face.conduit.is_none()
                    && face.q.abs() >= BREAKING_SPEED * depth.max(MEASURED_DEPTH);
                if running {
                    self.fast.push(at);
                }
            }
        }
        // A conduit's water runs into whatever stands where it comes out.
        for conduit in &cells.conduits {
            let face = |at: FaceRef| cells.chunks[at.slot as usize].faces[at.index as usize].q;
            let (first, last) = (conduit.faces[0], conduit.faces[conduit.faces.len() - 1]);
            let q = face(first);
            if q.abs() >= BREAKING_SPEED * conduit.throat {
                self.fast.push(if q > 0.0 { last } else { first });
            }
        }
    }

    /// Have landing water shove the faces beside it (`FieldCells::pushes`),
    /// before they are read: each towards the speed its push drives it to
    /// and no faster, and no harder than gravity down `PUSH_SLOPE` would
    /// push back. The scheme carries no momentum downstream, so what a
    /// torrent lands with would otherwise heap the pool up at its foot.
    fn push_faces(&self, cells: &mut FieldCells, dt: f32) {
        for i in 0..cells.pushes.len() {
            let push = cells.pushes[i];
            let (slot, index) = (push.face.slot as usize, push.face.index as usize);
            let Some(&face) = cells.chunks[slot].faces.get(index) else {
                continue;
            };
            let owner = SpanIndex {
                slot: push.face.slot,
                flat: face.a,
            };
            let depth = flow_depth(cells, owner, &face);
            if depth <= self.params.dry_depth {
                continue;
            }
            let most = G * depth * PUSH_SLOPE;
            let pushed = face.q + push.rate.clamp(-most, most) * dt;
            let driven = push.speed * depth;
            let q = if push.speed >= 0.0 {
                pushed.min(driven.max(face.q))
            } else {
                pushed.max(driven.min(face.q))
            };
            cells.chunks[slot].faces[index].q = q;
        }
    }

    /// Move each conduit's water as one column: every face of it takes the
    /// discharge its head drives (`Conduit::driven`), from the mean of what
    /// they carried.
    fn drive_conduits(&self, cells: &mut FieldCells, dt: f32) {
        let conduits = std::mem::take(&mut cells.conduits);
        for conduit in &conduits {
            let face = |at: &FaceRef| cells.chunks[at.slot as usize].faces[at.index as usize].q;
            let q = conduit.faces.iter().map(face).sum::<f32>() / conduit.faces.len() as f32;
            let levels = conduit.ends.map(|end| cells.surface(end) as f32);
            let held = friction(q, conduit.throat, dt, &self.params);
            let q = conduit.driven(q, levels, held, dt);
            for at in &conduit.faces {
                cells.chunks[at.slot as usize].faces[at.index as usize].q = q;
            }
        }
        cells.conduits = conduits;
    }

    /// The faces whose cells' surfaces stood `BRINK_STEP` apart as the last
    /// substep began.
    pub fn brinks(&self) -> &[FaceRef] {
        &self.brinks
    }

    /// The faces whose water stood against their roofs as the last step
    /// ended.
    pub fn pressed(&self) -> &[FaceRef] {
        &self.pressed
    }

    /// The faces whose water ran at `foam::BREAKING_SPEED` or more as the
    /// last step ended.
    pub fn fast(&self) -> &[FaceRef] {
        &self.fast
    }

    /// A face's discharge at the start of the substep, blended with the mean
    /// of the faces behind and ahead of it, the face's flow `depth` deep.
    /// What blends is each face's velocity, carried at this face's depth:
    /// blended as discharge, a film's face beside a river's took a share of
    /// the river's flow each substep and pushed it up the bank. A missing or
    /// dry neighbour counts as the face itself, as a wall does: blended
    /// towards a dry face's nothing, every face of a river running
    /// diagonally, each with dry bank ahead of it, dragged and held the
    /// river 20 cm high. A neighbour in a chunk not
    /// stepped this frame has not moved, so its discharge now is its
    /// discharge then.
    fn blended(&self, cells: &FieldCells, face: &Face, depth: f32) -> f32 {
        let own = face.q;
        let theta = self.params.theta;
        if theta >= 1.0 {
            return own;
        }
        let dry = self.params.dry_depth;
        let old = |r: Option<FaceRef>| {
            r.and_then(|r| {
                let (slot, index) = (r.slot as usize, r.index as usize);
                let (q, d) = if self.is_stepped[slot] {
                    self.previous[slot][index]
                } else {
                    let f = &cells.chunks[slot].faces[index];
                    let a = SpanIndex {
                        slot: r.slot,
                        flat: f.a,
                    };
                    carried(cells, a, f)
                };
                (d > dry).then(|| q * depth / d)
            })
            .unwrap_or(own)
        };
        theta * own + 0.5 * (1.0 - theta) * (old(face.behind) + old(face.ahead))
    }

    /// Scale each face's flow so that no cell gives more than it holds, and
    /// none takes more than it has room for under its ceiling. Water does not
    /// flow under pressure: a full cave stays full and no fuller.
    fn limit_outflows(&mut self, cells: &FieldCells, dt: f32) {
        for &slot in &self.touched {
            self.demand[slot].iter_mut().for_each(|d| *d = 0.0);
            self.offered[slot].iter_mut().for_each(|d| *d = 0.0);
        }
        let volume = |q: f32| (q * DX * dt) as f64;
        self.for_each_flow(cells, volume, |this, giver, _, v| {
            if let Some((slot, flat)) = giver {
                this.demand[slot][flat] += v;
            }
        });
        for &slot in &self.touched {
            let chunk = &cells.chunks[slot];
            for (i, give) in self.give[slot].iter_mut().enumerate() {
                let asked = self.demand[slot][i];
                let held = chunk.depth[i] * CELL_AREA;
                *give = if asked > held { held / asked } else { 1.0 };
            }
        }
        self.for_each_flow(cells, volume, |this, giver, taker, v| {
            if let Some((slot, flat)) = taker {
                let give = giver.map_or(1.0, |(s, f)| this.give[s][f]);
                this.offered[slot][flat] += v * give;
            }
        });
        for &slot in &self.touched {
            let chunk = &cells.chunks[slot];
            for (i, take) in self.take[slot].iter_mut().enumerate() {
                let offered = self.offered[slot][i];
                let room = ((chunk.ceiling[i] - chunk.bed[i]) as f64 - chunk.depth[i]) * CELL_AREA
                    + self.demand[slot][i] * self.give[slot][i];
                *take = if offered > room {
                    (room / offered).max(0.0)
                } else {
                    1.0
                };
            }
        }
    }

    /// Visit every stepped flow this substep as (giver, taker, volume), each
    /// end `None` where it lies outside the field.
    fn for_each_flow(
        &mut self,
        cells: &FieldCells,
        volume: impl Fn(f32) -> f64,
        mut visit: impl FnMut(&mut Self, Option<(usize, usize)>, Option<(usize, usize)>, f64),
    ) {
        for i in 0..self.stepped.len() {
            let slot = self.stepped[i];
            let chunk = &cells.chunks[slot];
            for face in &chunk.faces {
                let v = volume(face.q);
                if v == 0.0 {
                    continue;
                }
                match ends(cells, slot, face, v > 0.0) {
                    (giver, Taker::Across(taker)) => {
                        visit(self, Some(giver), Some(taker), v.abs());
                    }
                    (giver, Taker::Air(_)) => visit(self, Some(giver), None, v.abs()),
                }
            }
            for face in &chunk.boundary {
                let v = volume(face.q);
                let cell = (slot, face.cell as usize);
                if v > 0.0 {
                    visit(self, Some(cell), None, v);
                } else if v < 0.0 {
                    visit(self, None, Some(cell), -v);
                }
            }
        }
    }

    /// The share of a flow from `giver` to `taker` that moves.
    fn share(&self, giver: Option<(usize, usize)>, taker: Option<(usize, usize)>) -> f64 {
        giver.map_or(1.0, |(s, f)| self.give[s][f]) * taker.map_or(1.0, |(s, f)| self.take[s][f])
    }

    /// Move water across every stepped face, as limited: into the cell
    /// across, or into the air where it falls. Returns the volume that left
    /// the field through boundary faces.
    fn move_water(&self, cells: &mut FieldCells, dt: f32) -> f64 {
        let mut boundary_out = 0.0;
        for &slot in &self.stepped {
            let mut faces = std::mem::take(&mut cells.chunks[slot].faces);
            for face in &mut faces {
                let v = (face.q.abs() * DX * dt) as f64;
                if v == 0.0 {
                    continue;
                }
                let (giver, taker) = ends(cells, slot, face, face.q > 0.0);
                let moved = match taker {
                    Taker::Across(taker) => {
                        let part = v * self.share(Some(giver), Some(taker));
                        cells.chunks[taker.0].depth[taker.1] += part / CELL_AREA;
                        wake_on_flow(cells, taker.0, part);
                        part
                    }
                    Taker::Air(flight) => {
                        let part = v * self.share(Some(giver), None);
                        cells.flights[flight as usize].aloft += part;
                        part
                    }
                };
                if moved < v {
                    face.q *= (moved / v) as f32;
                }
                cells.chunks[giver.0].depth[giver.1] -= moved / CELL_AREA;
                wake_on_flow(cells, giver.0, moved);
            }
            cells.chunks[slot].faces = faces;
            let mut boundary = std::mem::take(&mut cells.chunks[slot].boundary);
            for face in &mut boundary {
                let mut v = (face.q * DX * dt) as f64;
                let cell = Some((slot, face.cell as usize));
                let s = if v > 0.0 {
                    self.share(cell, None)
                } else {
                    self.share(None, cell)
                };
                if s < 1.0 {
                    v *= s;
                    face.q *= s as f32;
                }
                cells.chunks[slot].depth[face.cell as usize] -= v / CELL_AREA;
                boundary_out += v;
            }
            cells.chunks[slot].boundary = boundary;
        }
        boundary_out
    }
}

/// What takes the water a face moves.
enum Taker {
    /// The cell across the face, as (slot, flat).
    Across((usize, usize)),
    /// The air, while it falls: its flight, by index.
    Air(u32),
}

/// What a face in chunk `slot` moves water between when it flows from its
/// owner to the cell across (`forward`) or back: the giver, as (slot,
/// flat), and the taker.
fn ends(cells: &FieldCells, slot: usize, face: &Face, forward: bool) -> ((usize, usize), Taker) {
    let at = |c: SpanIndex| (c.slot as usize, c.flat as usize);
    let owner = SpanIndex {
        slot: slot as u32,
        flat: face.a,
    };
    let (giver, across) = if forward {
        (owner, face.b)
    } else {
        (face.b, owner)
    };
    let flight = face
        .flight
        .filter(|&index| cells.flights[index as usize].from_owner == forward);
    let taker = match flight {
        None => Taker::Across(at(across)),
        Some(flight) => Taker::Air(flight),
    };
    (at(giver), taker)
}

/// Depth a flux must change a sleeping chunk's cell by to wake the chunk, m.
const WAKE_DEPTH: f64 = 1.0e-5;

fn wake_on_flow(cells: &mut FieldCells, slot: usize, volume: f64) {
    let chunk = &mut cells.chunks[slot];
    if !chunk.awake && volume.abs() / CELL_AREA > WAKE_DEPTH {
        chunk.awake = true;
        chunk.calm_for = 0.0;
    }
}

/// How deep the water flows through a face whose owning cell is `a`: from
/// the higher bed to the higher surface, under its roof.
pub(super) fn flow_depth(cells: &FieldCells, a: SpanIndex, face: &Face) -> f32 {
    let floor = cells.bed(a).max(cells.bed(face.b));
    let surface = (cells.surface(a) as f32).max(cells.surface(face.b) as f32);
    surface.min(face.roof) - floor
}

/// A face's discharge and the depth it carries it at, as its neighbours on
/// its axis blend with it: its flow depth, or none in a conduit, whose
/// water is driven by its ends and stands to the open water beside it as a
/// wall does.
fn carried(cells: &FieldCells, a: SpanIndex, face: &Face) -> (f32, f32) {
    let depth = if face.conduit.is_some() {
        0.0
    } else {
        flow_depth(cells, a, face)
    };
    (face.q, depth)
}

/// The update of a boundary face. To the void, a free overfall. To the sea,
/// a radiating edge (Flather 1976): the outflow is the long-wave speed times
/// the surface's height over sea level, so a wave reaching the edge leaves
/// the map instead of reflecting off a wall of water held at one level.
fn boundary_discharge(bed: f32, surface: f32, outside: Option<f32>, p: &FlowParams) -> f32 {
    match outside {
        None => {
            let depth = surface - bed;
            if depth <= p.dry_depth {
                0.0
            } else {
                p.froude_limit * depth * (G * depth).sqrt()
            }
        }
        Some(sea) => {
            let depth = surface.max(sea) - bed;
            if depth <= p.dry_depth {
                return 0.0;
            }
            let limit = p.froude_limit * depth * (G * depth).sqrt();
            ((G * depth).sqrt() * (surface - sea)).clamp(-limit, limit)
        }
    }
}

/// What holds back a discharge `q` flowing `depth` deep over `dt` seconds,
/// as the share of itself it loses: Manning friction and the still drag.
fn friction(q: f32, depth: f32, dt: f32, p: &FlowParams) -> f32 {
    let still = p.still_drag * dt * (-(q.abs() / depth) / p.still_speed).exp() * depth
        / (depth + p.still_depth);
    still + G * dt * p.manning_n * p.manning_n * q.abs() / depth.powf(7.0 / 3.0)
}

/// One face's momentum: gravity down the surface's slope against Manning
/// friction and the still drag, treated implicitly so they never reverse the
/// flow, then capped.
fn inertial(q: f32, depth: f32, slope: f32, dt: f32, p: &FlowParams) -> f32 {
    let q = (q - G * depth * dt * slope) / (1.0 + friction(q, depth, dt, p));
    let limit = p.froude_limit * depth * (G * depth).sqrt();
    q.clamp(-limit, limit)
}
