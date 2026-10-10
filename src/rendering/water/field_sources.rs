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

use crate::water::field::{FieldCells, WaterField};
use crate::water::geometry::SpanIndex;

use super::river_mesh::RiverMesh;

/// Depth a cell must hold to be drawn, m.
pub(super) const DRAWN_DEPTH: f64 = 0.002;

/// Farthest below the water a dry cell beside it counts, m, in a corner's
/// depth: a wall is a bank whose foot the water meets, so a channel between
/// high walls still has corners deep enough to draw.
const BANK_DEPTH: f64 = 0.1;

/// Surfaces further apart than this do not share a corner, m: a step this
/// high between two wet cells is water going over an edge, drawn as a
/// curtain. Lower steps are a sheet running down a slope, up to 200 %.
pub(super) const JOIN: f64 = 1.0;

/// One cell meeting at a corner, as the corner sees it.
#[derive(Debug, Clone, Copy)]
pub(super) struct Meeting {
    /// Its surface, if it is wet enough to draw.
    pub surface: Option<f64>,
    /// Its floor.
    pub bed: f64,
    /// Its depth.
    pub depth: f64,
}

impl Meeting {
    pub fn of(cells: &FieldCells, at: SpanIndex) -> Self {
        let depth = cells.depth(at);
        Self {
            surface: (depth > DRAWN_DEPTH).then(|| cells.surface(at)),
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
/// the cells of each one's depth, a dry one's taken as no more than zero and
/// as far below as its floor stands above the water, down to `BANK_DEPTH`.
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
            None => (height - m.bed).clamp(-BANK_DEPTH, 0.0),
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
    /// A skirt's corner on the edge it shares with a cell: that cell's
    /// corner, vertex `from` of the same surface.
    Edge { from: u32 },
    /// A skirt's far corner: vertex `from`'s height, as deep as the dry
    /// `bank`'s floor stands under the surface of the wet cell `own`. The
    /// skirt stands while that surface is above the bank's lowest floor.
    Bank {
        from: u32,
        own: SpanIndex,
        bank: SpanIndex,
        floor: f32,
    },
    /// A curtain's corner: the higher or lower of two wet cells' surfaces,
    /// with the flow of the first.
    Curtain {
        own: SpanIndex,
        other: SpanIndex,
        top: bool,
    },
}

/// A built surface's vertex sources, and what else must hold for its shape
/// to: the layout of every slot they read, by generation; which of its own
/// chunk's cells were drawn; and the lowest floor of the dry banks beside a
/// wet cell that were too high for a skirt.
#[derive(Debug, Clone, Default)]
pub struct SurfaceSources {
    pub(super) vertices: Vec<VertexSource>,
    pub(super) layouts: SmallVec<[(u32, u32); 9]>,
    pub(super) slot: u32,
    pub(super) drawn: Vec<bool>,
    pub(super) high_banks: Vec<(SpanIndex, f32)>,
}

impl SurfaceSources {
    /// Note the slot of a cell a vertex reads.
    pub(super) fn reads(&mut self, cells: &FieldCells, at: SpanIndex) {
        if !self.layouts.iter().any(|&(slot, _)| slot == at.slot) {
            let generation = cells.generation(at.slot as usize);
            self.layouts.push((at.slot, generation));
        }
    }
}

/// Read every vertex of a built surface again from the field. Returns
/// whether the surface kept the shape it was built with; where it did not,
/// what could be read is written anyway.
pub(super) fn refresh(field: &WaterField, mesh: &mut RiverMesh) -> bool {
    let cells = field.cells();
    let sources = &mesh.sources;
    if sources.vertices.is_empty() || sources.vertices.len() != mesh.vertices.len() {
        return false;
    }
    if sources
        .layouts
        .iter()
        .any(|&(slot, generation)| cells.generation(slot as usize) != generation)
    {
        return false;
    }
    let chunk = &cells.chunks[sources.slot as usize];
    let mut held = chunk.depth.len() == sources.drawn.len()
        && chunk
            .depth
            .iter()
            .zip(&sources.drawn)
            .all(|(&d, &drawn)| (d > DRAWN_DEPTH) == drawn)
        && sources
            .high_banks
            .iter()
            .all(|&(own, floor)| cells.surface(own) <= floor as f64);
    for (v, source) in sources.vertices.iter().enumerate() {
        match source {
            VertexSource::Corner { cells: at, shape } => {
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
            &VertexSource::Bank {
                from,
                own,
                bank,
                floor,
            } => {
                let level = cells.surface(own);
                held &= cells.depth(bank) <= DRAWN_DEPTH && level > floor as f64;
                let source = mesh.vertices[from as usize];
                let vertex = &mut mesh.vertices[v];
                vertex.surface = source.surface;
                vertex.depth = (level - cells.bed(bank) as f64).min(0.0) as f32;
                vertex.flow = source.flow;
            }
            &VertexSource::Curtain { own, other, top } => {
                let (a, b) = (Meeting::of(cells, own), Meeting::of(cells, other));
                let (Some(a), Some(b)) = (a.surface, b.surface) else {
                    held = false;
                    continue;
                };
                held &= (a - b).abs() > JOIN;
                let vertex = &mut mesh.vertices[v];
                vertex.surface = if top { a.max(b) } else { a.min(b) } as f32;
                vertex.flow = field.velocity(own);
            }
        }
    }
    held
}
