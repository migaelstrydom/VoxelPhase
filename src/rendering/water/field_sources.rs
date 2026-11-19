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

use crate::water::falling::falls;
use crate::water::field::{DrawnDepths, Falls, FieldCells, Jet as FieldJet, WaterField};
use crate::water::geometry::{SpanGraph, SpanIndex, SpanRef, ORTHOGONAL};

use super::curtain::Curtain;
use super::field_mesher::skirt_vertex;
use super::jet::Jet;
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
    /// The water going over edges: a cell a jet bursts from has no surface.
    pub falls: &'a Falls,
    /// The water's clock, s: what a fall's age is counted back from.
    pub clock: f32,
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
    /// roof, not perched, and not bursting out of a hole as a jet, which
    /// comes from the middle of the hole it fills, under where a surface
    /// would lie. A cell without one stands to the water beside it as dry
    /// ground does: it lifts none of their corners towards its roof.
    pub(super) fn drawn(&self, at: SpanIndex) -> bool {
        let depth = self.depth(at);
        depth > DRAWN_DEPTH
            && self.surface(at) < self.cells.ceiling(at) as f64 - ROOF_GAP
            && !self.perched(at)
            && !self.bursts(at)
    }

    /// Whether a cell's water bursts out of it as a jet. Asked of every
    /// cell at every corner, so it looks once into its chunk's falls, which
    /// lie in order of cell.
    fn bursts(&self, at: SpanIndex) -> bool {
        if !self.cells.ceiling(at).is_finite() {
            return false;
        }
        let falls = self.falls.of_slot(at.slot as usize);
        let first = falls.partition_point(|fall| fall.from.flat < at.flat);
        falls[first..]
            .iter()
            .take_while(|fall| fall.from.flat == at.flat)
            .any(|fall| fall.driven)
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

/// The falls pouring from the cells of chunk `slot`, as a surface's shape
/// takes them: by cell and direction, and whether each is driven. A spent
/// fall shapes nothing: its curtain is drawn until its water is down, and
/// its going is no change of shape.
pub(super) fn falls_of(
    field: &WaterField,
    slot: u32,
) -> impl Iterator<Item = (u32, u8, bool)> + '_ {
    (field.falls().of_slot(slot as usize).iter())
        .filter(|fall| fall.spent.is_none())
        .map(|fall| (fall.from.flat, fall.direction, fall.driven))
}

/// The surface vertex midway between `from` and `to`.
pub(super) fn midway(from: RiverVertex, to: &RiverVertex) -> RiverVertex {
    RiverVertex {
        xz: 0.5 * (from.xz + to.xz),
        surface: 0.5 * (from.surface + to.surface),
        depth: 0.5 * (from.depth + to.depth),
        flow: 0.5 * (from.flow + to.flow),
        air: 0.5 * (from.air + to.air),
        ..from
    }
}

/// Where a curtain hangs from: the place and height of each of the surface
/// vertices `hung` along its lip.
pub(super) fn hung_from(vertices: &[RiverVertex], hung: [u32; 3]) -> [(Vector2<f32>, f32); 3] {
    hung.map(|v| {
        let vertex = &vertices[v as usize];
        (vertex.xz, vertex.surface)
    })
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

/// What the water at a corner carries: its flow, and the foam riding it.
#[derive(Debug, Clone, Copy)]
pub(super) struct Carried {
    /// Depth-averaged velocity, m/s, x and z.
    pub flow: Vector2<f32>,
    /// Foam, as the field has it (`FieldChunk::foam`).
    pub foam: f32,
}

/// What a corner's water carries: the mean of the cells in its group, so
/// that the shader's flow mapping and its foam run smoothly across cells
/// instead of switching at every edge.
pub(super) fn corner_carried(field: &WaterField, cells: &[SpanIndex], shape: Shape) -> Carried {
    let (mut flow, mut foam, mut count) = (Vector2::zeros(), 0.0, 0.0);
    for (i, &c) in cells.iter().enumerate() {
        if shape.grouped & (1 << i) != 0 {
            flow += field.velocity(c);
            foam += field.foam(c);
            count += 1.0;
        }
    }
    if count > 0.0 {
        Carried {
            flow: flow / count,
            foam: foam / count,
        }
    } else {
        Carried {
            flow: Vector2::zeros(),
            foam: 0.0,
        }
    }
}

/// Where one vertex reads its height, depth, flow and foam.
#[derive(Debug, Clone)]
pub(super) enum VertexSource {
    /// A corner of a cell's quad: the cells meeting there, the cell's own
    /// first, and the corner's shape when built.
    Corner {
        cells: SmallVec<[SpanIndex; 4]>,
        shape: Shape,
    },
    /// A copy of vertex `from` of the same surface, moved: a ledge's
    /// surface carried out to meet a sheet hung at its lip.
    Edge { from: u32 },
    /// A skirt's corner out over a dry column: the surface of the cell
    /// whose quad's corners are vertices `first..first + 4`, at `within`
    /// its square, `past` columns out from it (`skirt_vertex`). The skirt
    /// stands while the `dry` cells stay dry: the one under it, and those
    /// that would lay it if wet.
    Skirt {
        first: u32,
        within: Vector2<f32>,
        past: f32,
        dry: SmallVec<[SpanIndex; 4]>,
    },
    /// Midway between two vertices of the same surface, `of`: the middle of
    /// a lip, or of a corner cut off between two lips.
    Between { of: [u32; 2] },
    /// A vertex of a fall's curtain, read with the rest of its curtain
    /// (`SurfaceSources::curtains`).
    Curtain,
    /// Vertex `around` of row `row` of jet `source` of those landing in the
    /// surface's chunk (`SurfaceSources::jets`), along its arc down to the
    /// water in the cell `landing`. The jet stands while it flies that arc.
    Jet {
        source: u32,
        landing: SpanIndex,
        row: u32,
        around: u32,
    },
    /// Corner `corner` of the foam where jet `source` of those landing in
    /// the surface's chunk lands in the cell `landing` (`Jet::plunge`). It
    /// stands while the jet flies that arc.
    JetPlunge {
        source: u32,
        landing: SpanIndex,
        corner: u32,
    },
}

/// One fall's curtain in a built surface: a run of vertices read again
/// together from the fall, while it and the falls it runs on into stand.
/// A spent fall's is left as built: its water is in the air, the draw
/// shows less of it by the clock (`RiverVertex::life`), and once the last
/// of it is down it is folded away, with no surface built again for it.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct CurtainSource {
    /// The fall it is drawn for, by its cell and direction.
    pub fall: (SpanIndex, u8),
    /// The fall its lip runs on into at each end, if any.
    pub beside: [Option<(SpanIndex, u8)>; 2],
    /// The surface vertices it hangs from, at its lip's two ends and its
    /// middle; none where the water it leaves has no surface.
    pub hung: Option<[u32; 3]>,
    /// Its first vertex, and how many it has.
    pub first: u32,
    pub count: u32,
    /// Whether its fall was spent when it was built.
    pub spent: bool,
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
    /// Its falls' curtains.
    pub(super) curtains: Vec<CurtainSource>,
    /// The falls leaving its chunk's cells when it was built (`falls_of`):
    /// its shape holds while they are the falls leaving them.
    pub(super) falls: Vec<(u32, u8, bool)>,
    /// The update whose drawn depths the surface was last read from.
    pub(super) read: u64,
    /// The jets landing in its chunk when it was built, in the field's
    /// order: a jet vertex's `source` counts into these.
    pub(super) jets: Vec<FieldJet>,
}

/// The field's jets landing in chunk `slot`, in the field's order.
pub(super) fn landing_jets(field: &WaterField, slot: u32) -> Vec<FieldJet> {
    field
        .jets()
        .filter(|jet| jet.landing.slot == slot)
        .collect()
}

impl SurfaceSources {
    /// Note the slot of a cell a vertex reads.
    pub(super) fn reads(&mut self, cells: &FieldCells, at: SpanIndex) {
        self.reads_slot(cells, at.slot);
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
                let at = SpanIndex {
                    slot,
                    flat: flat as u32,
                };
                cells.drawn(at) == was
            }));
    held &= falls_of(field, slot).eq(sources.falls.iter().copied());
    // The jets landing in its chunk must be the ones it was built with,
    // flying the arcs they flew, for its jet vertices to be theirs.
    let jets = landing_jets(field, slot);
    held &= jets.len() == sources.jets.len()
        && jets.iter().zip(&sources.jets).all(|(a, b)| a.same_arc(b));
    for (v, source) in sources.vertices.iter().enumerate() {
        match source {
            VertexSource::Corner { cells: at, shape } => {
                if !at.iter().any(|c| cells.replaced_since(c.slot, read)) {
                    let carried = corner_carried(field, at, *shape);
                    mesh.vertices[v].flow = carried.flow;
                    mesh.vertices[v].air = carried.foam;
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
                let carried = corner_carried(field, at, now);
                vertex.flow = carried.flow;
                vertex.air = carried.foam;
            }
            &VertexSource::Edge { from } => {
                let source = mesh.vertices[from as usize];
                let vertex = &mut mesh.vertices[v];
                vertex.surface = source.surface;
                vertex.depth = source.depth;
                vertex.flow = source.flow;
                vertex.air = source.air;
            }
            VertexSource::Skirt {
                first,
                within,
                past,
                dry,
            } => {
                held &= dry.iter().all(|&c| !cells.drawn(c));
                let from = skirt_vertex(&mesh.vertices, *first, *within, *past);
                let vertex = &mut mesh.vertices[v];
                vertex.surface = from.surface;
                vertex.depth = from.depth;
                vertex.flow = from.flow;
                vertex.air = from.air;
            }
            &VertexSource::Between { of: [a, b] } => {
                let from = mesh.vertices[a as usize];
                let between = midway(from, &mesh.vertices[b as usize]);
                let vertex = &mut mesh.vertices[v];
                vertex.surface = between.surface;
                vertex.depth = between.depth;
                vertex.flow = between.flow;
                vertex.air = between.air;
            }
            VertexSource::Curtain => {}
            &VertexSource::Jet {
                source,
                landing,
                row,
                around,
            } => {
                match jets
                    .get(source as usize)
                    .and_then(|jet| Jet::new(jet, cells.surface(landing) as f32))
                {
                    Some(jet) => mesh.vertices[v] = jet.vertex(row, around),
                    None => held = false,
                }
            }
            &VertexSource::JetPlunge {
                source,
                landing,
                corner,
            } => {
                match jets
                    .get(source as usize)
                    .and_then(|jet| Jet::new(jet, cells.surface(landing) as f32))
                {
                    Some(jet) => mesh.vertices[v] = jet.plunge(corner),
                    None => held = false,
                }
            }
        }
    }
    // Each curtain again from its fall, once the surface it hangs from has
    // been read. A spent fall's is as it was built until its water is
    // down, then folded to a point.
    for curtain in &sources.curtains {
        let first = curtain.first as usize;
        let fall = field.fall(curtain.fall.0, curtain.fall.1 as usize);
        if curtain.spent {
            if fall.is_none_or(|fall| fall.spent.is_none()) {
                let folded = mesh.vertices[first];
                mesh.vertices[first..first + curtain.count as usize].fill(folded);
            }
            continue;
        }
        let Some(fall) = fall.filter(|fall| fall.spent.is_none()) else {
            held = false;
            continue;
        };
        let beside = [0, 1].map(|end| field.fall_beside(fall, end));
        if beside.map(|b| b.map(|(other, _)| (other.from, other.direction))) != curtain.beside {
            held = false;
            continue;
        }
        let top = curtain.hung.map(|hung| hung_from(&mesh.vertices, hung));
        let beside = beside.map(|b| b.map(|(other, _)| other));
        let vertices = Curtain::new(fall, beside, top, cells.clock).vertices();
        mesh.vertices[first..first + vertices.len()].copy_from_slice(&vertices);
    }
    // A surface that lost its shape reads every cell again until built.
    if held {
        mesh.sources.read = cells.depths.now;
        Refreshed::Held
    } else {
        Refreshed::Reshaped
    }
}
