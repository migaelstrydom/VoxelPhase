//! Where each vertex of a built field surface reads its height, depth and
//! flow, so that a surface whose water has moved but kept its shape is read
//! again from the cells instead of built again from the graph.
//!
//! ```text
//!   build ──▶ vertices + sources (cells per vertex, which were wet, which
//!             joined the corner's group; each slot's layout generation)
//!   frame ──▶ refresh: every vertex read again from its cells
//!             ├─ shape held  ──▶ up to date
//!             └─ a cell went wet or dry, a group split or joined, a slot
//!                was laid out again ──▶ built again, under the budget
//! ```
//!
//! A refresh writes what it can even where the shape no longer holds, so
//! the surface keeps moving while it waits to be built again.

use nalgebra::Vector2;
use smallvec::SmallVec;

use crate::water::field::{DrawnDepths, FieldCells, WaterField};
use crate::water::geometry::{SpanGraph, SpanIndex, SpanRef, ORTHOGONAL};

use super::jet::Jet;
use super::nappe::{Face, Nappe, NAPPE_ROWS};
use super::river_mesh::RiverMesh;
use super::scene::Refreshed;
use super::vertex::RiverVertex;

/// Depth a cell must hold to be drawn, m.
const DRAWN_DEPTH: f64 = 0.002;

/// Least room a cell must leave under its ceiling to have a surface, m: a
/// cell filled to its roof, as water pressed into a sealed cavity is, has
/// none to draw.
const ROOF_GAP: f64 = 0.01;

/// How far a neighbour's water must stand into a column, above its lowest
/// floor and below its bed, and how thin the water on that bed must be, for
/// the column to be drawn as the neighbour's, m.
const PERCH: f64 = 0.1;

/// The field's cells as the surfaces are drawn from them: each depth as it
/// stood when its chunk's version last moved (`SurfaceVersions`), so every
/// surface reading a cell reads it alike, whichever was drawn last.
#[derive(Clone, Copy)]
pub struct DrawnCells<'a> {
    pub cells: &'a FieldCells,
    pub graph: &'a SpanGraph,
    pub depths: DrawnDepths<'a>,
}

impl DrawnCells<'_> {
    pub(super) fn depth(&self, at: SpanIndex) -> f64 {
        self.depths.depths[at.slot as usize][at.flat as usize]
    }

    /// Whether a slot's depths were replaced after update `read`.
    fn replaced_since(&self, slot: u32, read: u64) -> bool {
        self.depths.replaced[slot as usize] > read
    }

    pub(super) fn bed(&self, at: SpanIndex) -> f32 {
        self.cells.bed(at)
    }

    pub(super) fn surface(&self, at: SpanIndex) -> f64 {
        self.bed(at) as f64 + self.depth(at)
    }

    /// Whether a cell has a surface to draw: deep enough, not filled to its
    /// roof, and not perched.
    pub(super) fn drawn(&self, at: SpanIndex) -> bool {
        let depth = self.depth(at);
        depth > DRAWN_DEPTH
            && self.surface(at) < self.cells.ceiling(at) as f64 - ROOF_GAP
            && !self.perched(at)
    }

    /// Whether a cell holds only a film perched on its bed while a
    /// neighbour's water stands in the column below it, as where a terrace's
    /// riser crosses a column whose upper tread is the wider: the column is
    /// then the neighbour's water, drawn on to the riser as its shore, since
    /// one quad over the column would tilt the neighbour's surface up to the
    /// film's in front of the riser.
    fn perched(&self, at: SpanIndex) -> bool {
        let bed = self.bed(at) as f64;
        if self.depth(at) >= PERCH {
            return false;
        }
        let chunk = &self.cells.chunks[at.slot as usize];
        let (local, ordinal) = chunk.column_of(at.flat as usize);
        let spans = self
            .graph
            .chunk_at_slot(at.slot as usize)
            .column_spans(local);
        let Some(floor) = spans.get(ordinal as usize).map(|s| s.floor_min as f64) else {
            return false;
        };
        if floor + 2.0 * PERCH > bed {
            return false;
        }
        let span = SpanRef::key(chunk.coord.column(local), ordinal);
        ORTHOGONAL.iter().any(|&step| {
            self.graph.joined(span, step).iter().any(|n| {
                self.cells
                    .index(self.graph, n.span.column, n.span.ordinal)
                    .is_some_and(|c| {
                        let s = self.surface(c);
                        self.depth(c) > DRAWN_DEPTH && s > floor + PERCH && s < bed - PERCH
                    })
            })
        })
    }
}

/// Surfaces further apart than this do not share a corner, m: a step this
/// high between two wet cells is water going over an edge, drawn as a
/// falling sheet. Lower steps are a sheet running down a slope, up to 200 %,
/// unless the water `falls`.
pub(super) const JOIN: f64 = 1.0;

/// Least step, m, at which water standing on a floor above the water below
/// it is drawn falling rather than sloping: steeper than 45° across a
/// column.
const FALL: f64 = 0.5;

/// Whether water at `upper` on a bed at `bed` goes over an edge to water at
/// `lower`, drawn as a sheet between them rather than a slope: a step over
/// `JOIN`, or over `FALL` where the upper water's own floor stands above the
/// lower water, which a slope between them would run through.
pub(super) fn falls(upper: f64, bed: f64, lower: f64) -> bool {
    let step = upper - lower;
    step > JOIN || (step > FALL && bed > lower)
}

/// Whether a dry cell beside water at `level` is under rock there: its floor
/// at its column's centre stands at or above the water, so a surface carried
/// into its column runs into the terrain, which hides it. A column with no
/// cell at that height is rock. Otherwise the dry cell is open, a hollow the
/// water has not reached, and a surface carried over it is cut.
pub(super) fn covered(cells: &DrawnCells, bank: Option<SpanIndex>, level: f64) -> bool {
    bank.is_none_or(|b| cells.bed(b) as f64 >= level)
}

/// The depth a shore vertex carried from a corner `depth` deep draws over a
/// dry `bank`: the corner's own where the bank is covered, so the surface
/// runs on into the terrain and is never cut; its opposite where it is open,
/// so the draw cuts the surface midway out over the hollow.
pub(super) fn shore_depth(
    cells: &DrawnCells,
    bank: Option<SpanIndex>,
    level: f64,
    depth: f32,
) -> f32 {
    if covered(cells, bank, level) {
        depth
    } else {
        -depth
    }
}

/// Least flow over an edge, per metre of it, that a falling sheet draws as
/// full, m²/s: a sheet is as deep as the water pours over its edge, so the
/// draw cuts it where it only trickles.
const FULL_POUR: f64 = 1.0;

/// How far a sheet falling into the void past the map's edge hangs, m.
const VOID_DROP: f64 = 6.0;

/// The water a surface at `level` falls to off the map's edge: the sea's
/// level `outside`, or `VOID_DROP` below where there is none.
pub(super) fn off_map(level: f64, outside: Option<f64>) -> f64 {
    outside.unwrap_or(level - VOID_DROP)
}

/// How fast water is thrown out over a lip when it barely moves, m/s, so
/// even a trickle clears the face it falls down.
const LIP_SPEED: f32 = 0.8;

/// The furthest a sheet's foot is thrown out from its lip, m.
const MAX_THROW: f32 = 1.6;

/// Gravity, m/s².
const GRAVITY: f32 = 9.81;

/// How far out from its lip a sheet's foot lands: water leaving the lip at
/// its velocity falls `fall` metres in sqrt(2 fall / g) seconds. The sheet
/// leans out to it, clear of rock that leans out below the lip.
///
/// It is thrown the way its water moves, which the lip's corner shares with
/// the sheets beside it: a lip running across the lattice is a staircase of
/// sheets facing two ways, and thrown each its own way they would cross.
/// The face only keeps it going out.
pub(super) fn thrown(face: Vector2<f32>, flow: Vector2<f32>, fall: f32) -> Vector2<f32> {
    if fall <= 0.0 {
        return Vector2::zeros();
    }
    let velocity = flow + face * (LIP_SPEED - flow.dot(&face)).max(0.0) * 0.5;
    let speed = velocity.norm();
    let direction = if speed > 1.0e-4 {
        velocity / speed
    } else {
        face
    };
    direction * (speed * (2.0 * fall / GRAVITY).sqrt()).min(MAX_THROW)
}

/// The foot of a sheet falling `fall` from `lip`, thrown out from it,
/// on its top face; notes the fall on the lip.
pub(super) fn foot(lip: &mut RiverVertex, fall: f32) -> RiverVertex {
    lip.sheet.y = fall;
    Nappe::new(*lip, thrown(lip.face, lip.flow, fall), fall).vertex(NAPPE_ROWS, Face::Top)
}

/// How deep a sheet falling from cell `own` over the edge facing `face` is
/// drawn: the water's depth there by its speed out over the edge, over
/// `FULL_POUR`.
pub(super) fn pour(
    field: &WaterField,
    cells: &DrawnCells,
    own: SpanIndex,
    face: Vector2<f32>,
) -> f32 {
    let out = field.velocity(own).dot(&face).max(0.0) as f64;
    (cells.depth(own) * out / FULL_POUR) as f32
}

/// One cell meeting at a corner, as the corner sees it.
#[derive(Debug, Clone, Copy)]
pub(super) struct Meeting {
    /// Its surface, if it has one to draw.
    pub surface: Option<f64>,
    /// Its floor.
    pub bed: f64,
    /// Its depth.
    pub depth: f64,
}

impl Meeting {
    pub fn of(cells: &DrawnCells, at: SpanIndex) -> Self {
        let depth = cells.depth(at);
        Self {
            surface: cells.drawn(at).then(|| cells.surface(at)),
            bed: cells.bed(at) as f64,
            depth,
        }
    }
}

/// A corner of a cell's quad.
#[derive(Debug, Clone, Copy)]
pub(super) struct Corner {
    pub height: f64,
    /// Depth of water there: positive under water, negative over a bank
    /// standing above it. The shader cuts the surface where it falls below
    /// the shore depth, so a shore is a contour, not the edge of a column.
    pub depth: f64,
}

/// A corner's shape: which of its meetings are wet, and which of those
/// joined the group whose surface it takes, one bit per meeting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Shape {
    wet: u8,
    grouped: u8,
}

/// A corner from the cells meeting there, the cell's own first, for a cell
/// whose surface stands at `level`.
///
/// Its height is the mean surface of the cell's group among the wet cells,
/// each weighed by its depth: sorted by surface, split wherever one `falls`
/// to the next, so every cell at a corner sees the same groups.
/// Weighed so, a film draining down a bank above deep water barely lifts
/// the deep water's corner, where a plain mean would stand it in the air
/// over the slope between them; and a cell only just wet barely moves it. Its depth is the mean over
/// the cells of each one's depth: a dry one's is zero, or left out where its
/// floor stands above the water, since the terrain hides the surface there.
pub(super) fn gather(meetings: &[Meeting], level: f64) -> (Corner, Shape) {
    let mut surfaces: SmallVec<[(f64, f64, f64); 4]> = meetings
        .iter()
        .filter_map(|m| Some((m.surface?, m.depth, m.bed)))
        .collect();
    surfaces.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut group: SmallVec<[(f64, f64); 4]> = SmallVec::new();
    for (s, d, bed) in surfaces {
        if group.last().is_some_and(|&(last, _)| falls(s, bed, last)) {
            if group.iter().any(|&(g, _)| g == level) {
                break;
            }
            group.clear();
        }
        group.push((s, d));
    }
    let held: f64 = group.iter().map(|&(_, d)| d).sum();
    let height = group.iter().map(|&(s, d)| s * d).sum::<f64>() / held;
    let (low, high) = (group[0].0, group[group.len() - 1].0);

    let mut shape = Shape { wet: 0, grouped: 0 };
    let (mut sum, mut count) = (0.0, 0.0);
    for (i, m) in meetings.iter().enumerate() {
        let depth = match m.surface {
            Some(s) if s >= low && s <= high => {
                shape.wet |= 1 << i;
                shape.grouped |= 1 << i;
                m.depth
            }
            Some(_) => {
                shape.wet |= 1 << i;
                continue;
            }
            None if m.bed >= height => continue,
            None => 0.0,
        };
        sum += depth;
        count += 1.0;
    }
    let corner = Corner {
        height,
        depth: sum / count,
    };
    (corner, shape)
}

/// A corner's flow: the mean velocity of the cells in its group, so that the
/// shader's flow mapping runs smoothly across cells instead of switching at
/// every edge.
pub(super) fn corner_flow(field: &WaterField, cells: &[SpanIndex], shape: Shape) -> Vector2<f32> {
    let (mut sum, mut count) = (Vector2::zeros(), 0.0);
    for (i, &c) in cells.iter().enumerate() {
        if shape.grouped & (1 << i) != 0 {
            sum += field.velocity(c);
            count += 1.0;
        }
    }
    if count > 0.0 {
        sum / count
    } else {
        Vector2::zeros()
    }
}

/// Where one vertex reads its height, depth and flow.
#[derive(Debug, Clone)]
pub(super) enum VertexSource {
    /// A corner of a cell's quad: the cells meeting there, the cell's own
    /// first, and the corner's shape when built.
    Corner {
        cells: SmallVec<[SpanIndex; 4]>,
        shape: Shape,
    },
    /// A shore patch's corner on its cell's own edge: that cell's corner,
    /// vertex `from` of the same surface.
    Edge { from: u32 },
    /// A shore patch's corner out over the dry column beside its cell:
    /// vertex `from`'s height and flow, and its depth as `shore_depth` takes
    /// it over the dry cell `bank` there (none for rock). The patch stands
    /// while that cell stays dry.
    Shore { from: u32, bank: Option<SpanIndex> },
    /// A falling sheet's top: vertex `from`'s corner, drawn as deep as the
    /// wet cell `own` pours over the sheet's edge.
    Lip { from: u32, own: SpanIndex },
    /// A falling sheet's foot: the lower cell's corner below the sheet's
    /// top, vertex `from`, whose depth and flow it takes. The cells are the
    /// corner's, the lower cell's first. The sheet stands while the wet cell
    /// `own` it falls from `falls` to the lower one.
    Foot {
        from: u32,
        cells: SmallVec<[SpanIndex; 4]>,
        shape: Shape,
        own: SpanIndex,
    },
    /// The foot of a sheet falling off the map's edge: vertex `from`, the
    /// sheet's top, lowered to the water `outside` (`off_map`). The sheet
    /// stands while the wet cell `own` it falls from `falls` to it.
    Brink {
        from: u32,
        own: SpanIndex,
        outside: Option<f64>,
    },
    /// Vertex `around` of row `row` of the jet of the field's source
    /// `source`, along its arc down to the water in the cell `landing`. The
    /// jet stands while its water still lands there.
    Jet {
        source: u32,
        landing: SpanIndex,
        row: u32,
        around: u32,
    },
    /// A point on `face` of a falling sheet's body, on row `row` down from
    /// its lip, vertex `lip`, to its foot, vertex `foot`, both on its top
    /// face (`Nappe::between`).
    Nappe {
        lip: u32,
        foot: u32,
        row: u32,
        face: Face,
    },
}

/// A built surface's vertex sources, and what else must hold for its shape
/// to: the layout of every slot they read, by generation, and which of its
/// own chunk's cells were drawn. A corner whose cells' depths have not been
/// replaced since it was last read is passed over, but for its flow.
#[derive(Debug, Clone, Default)]
pub struct SurfaceSources {
    pub(super) vertices: Vec<VertexSource>,
    pub(super) layouts: SmallVec<[(u32, u32); 9]>,
    pub(super) slot: u32,
    pub(super) drawn: Vec<bool>,
    /// Pairs of wet cells, the first falling to the second, that a fall is
    /// drawn on past (`FieldScene::passed`): the shape holds while each
    /// still falls.
    pub(super) passes: Vec<(SpanIndex, SpanIndex)>,
    /// The update whose drawn depths the surface was last read from.
    pub(super) read: u64,
}

impl SurfaceSources {
    /// Note the slot of a cell a vertex reads.
    pub(super) fn reads(&mut self, cells: &FieldCells, at: SpanIndex) {
        self.reads_slot(cells, at.slot);
    }

    /// Note that wet cell `upper` falls to `lower`, which the shape takes.
    pub(super) fn passes(&mut self, cells: &FieldCells, upper: SpanIndex, lower: SpanIndex) {
        self.reads(cells, upper);
        self.reads(cells, lower);
        self.passes.push((upper, lower));
    }

    /// Note a slot whose layout a vertex depends on.
    pub(super) fn reads_slot(&mut self, cells: &FieldCells, slot: u32) {
        if !self.layouts.iter().any(|&(s, _)| s == slot) {
            let generation = cells.generation(slot as usize);
            self.layouts.push((slot, generation));
        }
    }
}

/// Read every vertex of a built surface again from the field. Where it no
/// longer keeps the shape it was built with, what could be read is written
/// anyway.
pub(super) fn refresh(field: &WaterField, cells: &DrawnCells, mesh: &mut RiverMesh) -> Refreshed {
    let sources = &mesh.sources;
    if sources.vertices.len() != mesh.vertices.len() {
        return Refreshed::Unreadable;
    }
    if sources
        .layouts
        .iter()
        .any(|&(slot, generation)| cells.cells.generation(slot as usize) != generation)
    {
        return Refreshed::Unreadable;
    }
    let (slot, read) = (sources.slot, sources.read);
    let mut held = cells.depths.depths[slot as usize].len() == sources.drawn.len()
        && (!sources
            .layouts
            .iter()
            .any(|&(s, _)| cells.replaced_since(s, read))
            || sources.drawn.iter().enumerate().all(|(flat, &was)| {
                cells.drawn(SpanIndex {
                    slot,
                    flat: flat as u32,
                }) == was
            }));
    held &= sources.passes.iter().all(|&(upper, lower)| {
        cells.drawn(upper)
            && cells.drawn(lower)
            && falls(
                cells.surface(upper),
                cells.bed(upper) as f64,
                cells.surface(lower),
            )
    });
    for (v, source) in sources.vertices.iter().enumerate() {
        match source {
            VertexSource::Corner { cells: at, shape } => {
                if !at.iter().any(|c| cells.replaced_since(c.slot, read)) {
                    mesh.vertices[v].flow = corner_flow(field, at, *shape);
                    continue;
                }
                let meetings: SmallVec<[Meeting; 4]> =
                    at.iter().map(|&c| Meeting::of(cells, c)).collect();
                let Some(level) = meetings[0].surface else {
                    held = false;
                    continue;
                };
                let (corner, now) = gather(&meetings, level);
                held &= now == *shape;
                let vertex = &mut mesh.vertices[v];
                vertex.surface = corner.height as f32;
                vertex.depth = corner.depth as f32;
                vertex.flow = corner_flow(field, at, now);
            }
            &VertexSource::Edge { from } => {
                let source = mesh.vertices[from as usize];
                let vertex = &mut mesh.vertices[v];
                vertex.surface = source.surface;
                vertex.depth = source.depth;
                vertex.flow = source.flow;
            }
            &VertexSource::Shore { from, bank } => {
                held &= bank.is_none_or(|b| !cells.drawn(b));
                let source = mesh.vertices[from as usize];
                let vertex = &mut mesh.vertices[v];
                vertex.surface = source.surface;
                vertex.depth = shore_depth(cells, bank, source.surface as f64, source.depth);
                vertex.flow = source.flow;
            }
            &VertexSource::Lip { from, own } => {
                let source = mesh.vertices[from as usize];
                let vertex = &mut mesh.vertices[v];
                let pour = pour(field, cells, own, vertex.face);
                vertex.surface = source.surface;
                vertex.depth = source.depth.min(pour);
                vertex.flow = source.flow;
            }
            VertexSource::Foot {
                from,
                cells: at,
                shape,
                own,
            } => {
                let meetings: SmallVec<[Meeting; 4]> =
                    at.iter().map(|&c| Meeting::of(cells, c)).collect();
                let source = Meeting::of(cells, *own);
                let (Some(lower), Some(upper)) = (meetings[0].surface, source.surface) else {
                    held = false;
                    continue;
                };
                let (corner, now) = gather(&meetings, lower);
                held &= now == *shape && falls(upper, source.bed, lower);
                let fall = mesh.vertices[*from as usize].surface - corner.height as f32;
                mesh.vertices[v] = foot(&mut mesh.vertices[*from as usize], fall);
            }
            &VertexSource::Jet {
                source,
                landing,
                row,
                around,
            } => {
                let source = &field.sources()[source as usize];
                held &= source.landing.first() == Some(&landing);
                match Jet::new(source, cells.surface(landing) as f32) {
                    Some(jet) => mesh.vertices[v] = jet.vertex(row, around),
                    None => held = false,
                }
            }
            &VertexSource::Brink { from, own, outside } => {
                let source = Meeting::of(cells, own);
                let Some(upper) = source.surface else {
                    held = false;
                    continue;
                };
                held &= falls(upper, source.bed, off_map(upper, outside));
                let top = mesh.vertices[from as usize].surface;
                let fall = top - off_map(top as f64, outside) as f32;
                mesh.vertices[v] = foot(&mut mesh.vertices[from as usize], fall);
            }
            &VertexSource::Nappe {
                lip,
                foot,
                row,
                face,
            } => {
                let lip = mesh.vertices[lip as usize];
                mesh.vertices[v] =
                    Nappe::between(lip, &mesh.vertices[foot as usize]).vertex(row, face);
            }
        }
    }
    // A surface that lost its shape reads every cell again until built.
    if held {
        mesh.sources.read = cells.depths.now;
        Refreshed::Held
    } else {
        Refreshed::Reshaped
    }
}
