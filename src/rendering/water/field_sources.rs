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
use crate::water::geometry::SpanIndex;

use super::river_mesh::RiverMesh;
use super::scene::Refreshed;

/// Depth a cell must hold to be drawn, m.
const DRAWN_DEPTH: f64 = 0.002;

/// Least room a cell must leave under its ceiling to have a surface, m: a
/// cell filled to its roof, as water pressed into a sealed cavity is, has
/// none to draw.
const ROOF_GAP: f64 = 0.01;

/// The field's cells as the surfaces are drawn from them: each depth as it
/// stood when its chunk's version last moved (`SurfaceVersions`), so every
/// surface reading a cell reads it alike, whichever was drawn last.
#[derive(Clone, Copy)]
pub struct DrawnCells<'a> {
    pub cells: &'a FieldCells,
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

    /// Whether a cell has a surface to draw: deep enough, and not filled to
    /// its roof.
    pub(super) fn drawn(&self, at: SpanIndex) -> bool {
        let depth = self.depth(at);
        depth > DRAWN_DEPTH && self.surface(at) < self.cells.ceiling(at) as f64 - ROOF_GAP
    }
}

/// Surfaces further apart than this do not share a corner, m: a step this
/// high between two wet cells is water going over an edge, drawn as a
/// falling sheet. Lower steps are a sheet running down a slope, up to 200 %.
pub(super) const JOIN: f64 = 1.0;

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
/// Its height is the mean surface of the cell's group among the wet cells:
/// sorted by surface, split wherever two stand more than `JOIN` apart, so
/// every cell at a corner sees the same groups. Its depth is the mean over
/// the cells of each one's depth: a dry one's is zero, or left out where its
/// floor stands above the water, since the terrain hides the surface there.
pub(super) fn gather(meetings: &[Meeting], level: f64) -> (Corner, Shape) {
    let mut surfaces: SmallVec<[f64; 4]> = meetings.iter().filter_map(|m| m.surface).collect();
    surfaces.sort_by(f64::total_cmp);
    let mut group: SmallVec<[f64; 4]> = SmallVec::new();
    for s in surfaces {
        if group.last().is_some_and(|&last| s - last > JOIN) {
            if group.contains(&level) {
                break;
            }
            group.clear();
        }
        group.push(s);
    }
    let height = group.iter().sum::<f64>() / group.len() as f64;
    let (low, high) = (group[0], group[group.len() - 1]);

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
    /// `own` it falls from stands more than `JOIN` over the lower one.
    Foot {
        from: u32,
        cells: SmallVec<[SpanIndex; 4]>,
        shape: Shape,
        own: SpanIndex,
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
    /// The update whose drawn depths the surface was last read from.
    pub(super) read: u64,
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
        && (!cells.replaced_since(slot, read)
            || sources.drawn.iter().enumerate().all(|(flat, &was)| {
                cells.drawn(SpanIndex {
                    slot,
                    flat: flat as u32,
                }) == was
            }));
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
                let (Some(lower), Some(upper)) =
                    (meetings[0].surface, Meeting::of(cells, *own).surface)
                else {
                    held = false;
                    continue;
                };
                let (corner, now) = gather(&meetings, lower);
                held &= now == *shape && upper - lower > JOIN;
                let top = mesh.vertices[*from as usize];
                let vertex = &mut mesh.vertices[v];
                vertex.surface = corner.height as f32;
                vertex.depth = top.depth;
                vertex.flow = top.flow;
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
