//! The shallow-water field's surface, drawn through the river pipeline.
//!
//! ```text
//!   wet cell ──▶ one quad over its column, each corner the mean surface of
//!                its group of the wet cells meeting there: split where one
//!                falls to the next
//!   two wet cells, the upper falling to the lower
//!            ──▶ a sheet down their shared edge, from the upper surface's
//!                corners to the lower's: water going over the edge, hung
//!                out at the lip, and foam where it lands
//!   wet cell at the map's edge, its water leaving
//!            ──▶ a sheet off the edge, down to the sea or into the void
//!   a source landing in the chunk
//!            ──▶ a tube along its jet's arc, from where it comes out
//!                down to the water it lands in, and foam where it lands
//!   dry column beside one
//!            ──▶ a skirt carrying the cell's surface over it, each quarter
//!                of the column laid by one wet cell
//! ```
//!
//! The mesher does not decide where a shore is. The skirt reaches past it,
//! and the draw finds it to the pixel: the terrain hides the skirt wherever
//! the ground stands above the water, and where the water is shallow
//! `water.frag` fades the surface as the ground behind it, read from the
//! scene's depth, comes up to it.
//!
//! Each vertex also carries the water's depth there, as the field has it;
//! within the water's own cells the draw cuts the surface at `river.vert`'s
//! shore depth too.
//!
//! Every vertex carries its own height and a velocity, the mean of the cells
//! its corner joins, so the river shader's flow mapping runs on the field's
//! currents and runs smoothly across cells. A surface is built from the graph
//! when its shape changes; while only its water moves, every vertex is read
//! again from the cells it was built from (`field_sources`). Past the map's
//! open edges, the sea runs out to the horizon as a ring.

use nalgebra::{Vector2, Vector3, Vector4};
use smallvec::SmallVec;

use crate::water::falling::{falls, ledge_reach, off_map};
use crate::water::field::{past_edge, DrawnDepths, Source, WaterField};
use crate::water::geometry::{Column, SpanIndex, SpanRef, Step, COLUMN_SIZE, ORTHOGONAL};
use crate::water::ids::SurfaceId;
use crate::water::surface::Swell;
use crate::water::waves::{still_level, Sea, SurfaceWaves};
use crate::water::{WaterWorld, FIELD_BODY};

use super::divide::WaterPatch;
use super::field_sources::{self, corner_flow, gather, Corner, DrawnCells, Meeting, VertexSource};
use super::jet::{Jet, JET_RING, JET_ROWS, PLUNGE_CORNERS};
use super::nappe::{fan, plunge, strip, Face, Nappe, NAPPE_ROWS};
use super::ocean_ring::{append_ring, occupied_bounds, MapRect};
use super::river_mesh::{RiverDraw, RiverMesh};
use super::scene::{MeshKey, Refreshed, RippleTileView, WaterMesh, WaterScene};
use super::vertex::RiverVertex;

/// The field, as the water renderer sees it.
pub struct FieldScene<'a> {
    pub field: &'a WaterField,
    /// Each chunk's surface version, by slot: changes whenever its surface
    /// has moved since it was last drawn.
    pub versions: &'a [u32],
    /// Each chunk's depths as its surface is drawn from them.
    pub depths: DrawnDepths<'a>,
    /// Seconds simulated: the clock the drawn currents run on.
    pub clock: f32,
    /// The ripples and swell over the field, if it has any.
    pub waves: Option<&'a SurfaceWaves>,
}

impl<'a> FieldScene<'a> {
    /// The game's water, as drawn.
    pub fn of(world: &'a WaterWorld) -> Self {
        Self {
            field: world.field(),
            versions: world.versions(),
            depths: world.drawn_depths(),
            clock: world.clock(),
            waves: Some(world.waves()),
        }
    }
}

impl<'a> FieldScene<'a> {
    /// Each awake tile, over still water, drawn on its chunk's surface.
    fn ripple_views(&self) -> Vec<RippleTileView<'a>> {
        let Some(waves) = self.waves else {
            return Vec::new();
        };
        let cells = self.field.cells();
        waves
            .ripples()
            .active()
            .filter_map(|(key, tile)| {
                let level = tile.mask.level;
                let surface = cells.slot_of(key.0).map(|slot| SurfaceId(slot as u32));
                RippleTileView::new(waves.ripples(), key, level, waves.swell_at(level), surface)
            })
            .collect()
    }

    fn drawn_cells(&self) -> DrawnCells<'a> {
        DrawnCells {
            cells: self.field.cells(),
            graph: self.field.graph(),
            depths: self.depths,
        }
    }

    fn cell(&self, span: SpanRef) -> Option<SpanIndex> {
        self.field
            .cells()
            .index(self.field.graph(), span.column, span.ordinal)
    }

    fn meeting(&self, span: SpanRef) -> Option<Meeting> {
        Some(Meeting::of(&self.drawn_cells(), self.cell(span)?))
    }

    /// The span joined to `span` across one orthogonal step whose water a
    /// surface at `level` would meet: the wet one whose surface stands
    /// nearest, else the one whose floor does.
    fn across(&self, span: SpanRef, di: i32, dk: i32, level: f64) -> Option<SpanRef> {
        let near = |m: &Meeting| (m.surface.unwrap_or(m.bed) - level).abs();
        let mut wet: Option<(f64, SpanRef)> = None;
        let mut any: Option<(f64, SpanRef)> = None;
        for n in self.field.graph().joined(span, Step { di, dk }) {
            let key = SpanRef::key(n.span.column, n.span.ordinal);
            let Some(m) = self.meeting(key) else {
                continue;
            };
            let d = near(&m);
            if m.surface.is_some() && wet.is_none_or(|(best, _)| d < best) {
                wet = Some((d, key));
            }
            if any.is_none_or(|(best, _)| d < best) {
                any = Some((d, key));
            }
        }
        wet.or(any).map(|(_, key)| key)
    }

    /// The corner of `span`'s column on the (sx, sz) side, for a cell whose
    /// surface stands at `level`, its flow, and where it reads from: the cell and the
    /// cells across from it on each side and diagonally, as `gather` joins
    /// them.
    fn corner(
        &self,
        span: SpanRef,
        level: f64,
        sx: i32,
        sz: i32,
    ) -> (Corner, Vector2<f32>, VertexSource) {
        let x = self.across(span, sx, 0, level);
        let z = self.across(span, 0, sz, level);
        let diagonal = x
            .and_then(|n| self.across(n, 0, sz, level))
            .or_else(|| z.and_then(|n| self.across(n, sx, 0, level)));
        let cells: SmallVec<[SpanIndex; 4]> = [Some(span), x, z, diagonal]
            .into_iter()
            .flatten()
            .filter_map(|s| self.cell(s))
            .collect();
        let drawn = self.drawn_cells();
        let meetings: SmallVec<[Meeting; 4]> =
            cells.iter().map(|&c| Meeting::of(&drawn, c)).collect();
        let (corner, shape) = gather(&meetings, level);
        let flow = corner_flow(self.field, &cells, shape);
        (corner, flow, VertexSource::Corner { cells, shape })
    }

    /// Every wet cell joined to `span` across one orthogonal step.
    fn wet_across(&self, span: SpanRef, di: i32, dk: i32) -> SmallVec<[(SpanRef, SpanIndex); 2]> {
        let drawn = self.drawn_cells();
        self.field
            .graph()
            .joined(span, Step { di, dk })
            .into_iter()
            .map(|n| SpanRef::key(n.span.column, n.span.ordinal))
            .filter_map(|key| Some((key, self.cell(key)?)))
            .filter(|&(_, c)| Meeting::of(&drawn, c).surface.is_some())
            .collect()
    }

    /// A cell's quad, and the falling sheets and skirt around it.
    fn cell_surface(&self, mesh: &mut RiverMesh, span: SpanRef, at: SpanIndex, level: f64) {
        let cells = self.field.cells();
        let drawn = self.drawn_cells();
        if let Some((upper, lower)) = self.passed(span, at) {
            mesh.sources.passes(cells, upper, at);
            mesh.sources.passes(cells, at, lower);
            return;
        }
        let column = span.column;
        let built =
            [(-1, -1), (1, -1), (1, 1), (-1, 1)].map(|(sx, sz)| self.corner(span, level, sx, sz));
        let corners = built.clone().map(|(corner, _, _)| corner);
        let flows = built.clone().map(|(_, flow, _)| flow);
        let first = mesh.vertices.len() as u32;
        for (_, _, source) in &built {
            if let VertexSource::Corner { cells: read, .. } = source {
                for &c in read {
                    mesh.sources.reads(cells, c);
                }
            }
        }
        quad(
            mesh,
            column,
            corners,
            flows,
            built.map(|(_, _, source)| source),
        );

        for step in ORTHOGONAL {
            for (other_span, other) in self.wet_across(span, step.di, step.dk) {
                let other_level = drawn.surface(other);
                if falls(level, drawn.bed(at) as f64, other_level) {
                    let (landing, land) = self.landing(mesh, (other_span, other));
                    let landing = (landing, drawn.surface(land));
                    self.sheet(mesh, step, first, (span, at), other_span, landing);
                }
            }
        }

        let graph = self.field.graph();
        for (direction, step) in ORTHOGONAL.iter().enumerate() {
            let next = column.offset(step.di, step.dk);
            if graph.contains_column(next) && !graph.spans(next).is_empty() {
                continue;
            }
            if let Some(outside) = past_edge(self.field.sea(), direction, drawn.bed(at)) {
                self.brink(mesh, span, *step, first, at, outside);
            }
        }

        self.skirt(mesh, column, level, first);
    }

    /// The wet cell `span` (cell `at`) pours on to over one of its edges,
    /// if any: the lowest it `falls` to.
    fn pours_to(&self, span: SpanRef, at: SpanIndex) -> Option<(SpanRef, SpanIndex)> {
        let drawn = self.drawn_cells();
        let (level, bed) = (drawn.surface(at), drawn.bed(at) as f64);
        ORTHOGONAL
            .iter()
            .flat_map(|s| self.wet_across(span, s.di, s.dk))
            .filter(|&(_, c)| falls(level, bed, drawn.surface(c)))
            .min_by(|a, b| drawn.surface(a.1).total_cmp(&drawn.surface(b.1)))
    }

    /// Whether the wet cell `span` (cell `at`) is a step a fall runs over:
    /// water falls on to it from a cell beside it and it pours straight on
    /// over another edge, as where a cliff leans across a column and its
    /// face holds a film on each column it crosses. Water falling so fast
    /// would clear it, so the fall above is drawn on past it (`landing`)
    /// and it is not drawn. The cell it is fed from and the one it pours to.
    fn passed(&self, span: SpanRef, at: SpanIndex) -> Option<(SpanIndex, SpanIndex)> {
        let (_, lower) = self.pours_to(span, at)?;
        let drawn = self.drawn_cells();
        let level = drawn.surface(at);
        let upper = ORTHOGONAL
            .iter()
            .flat_map(|s| self.wet_across(span, s.di, s.dk))
            .map(|(_, c)| c)
            .find(|&c| falls(drawn.surface(c), drawn.bed(c) as f64, level))?;
        Some((upper, lower))
    }

    /// Where water falling on to the wet cell `lower` lands: that cell,
    /// or past every step it runs over (`passed`), the first that does not
    /// pour on. Each step passed is noted for the surface's shape to hold.
    fn landing(
        &self,
        mesh: &mut RiverMesh,
        mut lower: (SpanRef, SpanIndex),
    ) -> (SpanRef, SpanIndex) {
        for _ in 0..MAX_STEPS_PASSED {
            let Some(next) = self.pours_to(lower.0, lower.1) else {
                break;
            };
            mesh.sources.passes(self.field.cells(), lower.1, next.1);
            lower = next;
        }
        lower
    }

    /// What stands in `column` at `level`, beside a wet cell there: water,
    /// a dry cell (none where the column is rock at that height), or the
    /// map's edge.
    fn beside(&self, column: Column, level: f64) -> Beside {
        let graph = self.field.graph();
        let Some(slot) = graph.slot(column.chunk()) else {
            return Beside::Outside;
        };
        let cell = graph
            .span_at(column, level as f32)
            .and_then(|span| self.cell(span));
        match cell {
            Some(cell) if self.drawn_cells().drawn(cell) => Beside::Wet,
            bank => Beside::Dry(slot as u32, bank),
        }
    }

    /// The cell's surface carried out over the dry columns around it, as a
    /// skirt the depth test and the fragment shader trim: the terrain hides
    /// it wherever the ground stands above the water, and `water.frag` fades
    /// it where the ground behind it comes up to it. Nothing here decides
    /// where the shore is.
    ///
    /// Each quarter of a dry column beside water is laid once, by the wet
    /// cell nearest it (`skirt_owner`), so skirts never overlap. Its corners
    /// take the cell's surface where they lie, clamped to the cell's square:
    /// along the cell's edge they meet its quad exactly.
    fn skirt(&self, mesh: &mut RiverMesh, column: Column, level: f64, first: u32) {
        let near = AROUND.map(|(di, dk)| self.beside(column.offset(di, dk), level));
        if !near.iter().any(|b| matches!(b, Beside::Dry(..))) {
            return;
        }
        // Who lays a quarter can turn on a column two out.
        let around: [[Beside; 5]; 5] = [-2, -1, 0, 1, 2].map(|dk| {
            [-2, -1, 0, 1, 2].map(|di| match AROUND.iter().position(|&o| o == (di, dk)) {
                Some(i) => near[i],
                None if (di, dk) == (0, 0) => Beside::Wet,
                None => self.beside(column.offset(di, dk), level),
            })
        });
        let at = |di: i32, dk: i32| around[(dk + 2) as usize][(di + 2) as usize];
        let (x0, z0) = column.min_corner();
        for (di, dk) in AROUND {
            let Beside::Dry(slot, bank) = at(di, dk) else {
                continue;
            };
            let owned: SmallVec<[((i32, i32), SmallVec<[SpanIndex; 4]>); 4]> = QUARTERS
                .into_iter()
                .filter_map(|side| Some((side, skirt_owner((di, dk), side, at)?)))
                .collect();
            if owned.is_empty() {
                continue;
            }
            mesh.sources.reads_slot(self.field.cells(), slot);
            // A column the cell lays whole is one quad, not four.
            let pieces: SmallVec<[(Option<(i32, i32)>, SmallVec<[SpanIndex; 4]>); 4]> =
                if owned.len() == QUARTERS.len() {
                    let mut dry: SmallVec<[SpanIndex; 4]> = SmallVec::new();
                    for c in owned.into_iter().flat_map(|(_, d)| d) {
                        if !dry.contains(&c) {
                            dry.push(c);
                        }
                    }
                    smallvec::smallvec![(None, dry)]
                } else {
                    owned.into_iter().map(|(side, d)| (Some(side), d)).collect()
                };
            for (side, dry) in pieces {
                let held: SmallVec<[SpanIndex; 4]> = bank.into_iter().chain(dry).collect();
                // The piece's span along one axis, in columns from the cell's
                // minimum corner: the column, or its half on side `s`.
                let range = |d: i32, s: Option<i32>| match s {
                    None => (d as f32, d as f32 + 1.0),
                    Some(s) => {
                        let near = d as f32 + if s < 0 { 0.0 } else { 0.5 };
                        (near, near + 0.5)
                    }
                };
                let ((xa, xb), (za, zb)) = (
                    range(di, side.map(|(sx, _)| sx)),
                    range(dk, side.map(|(_, sz)| sz)),
                );
                let base = mesh.vertices.len() as u32;
                for (x, z) in [(xa, za), (xb, za), (xb, zb), (xa, zb)] {
                    let within = Vector2::new(x.clamp(0.0, 1.0), z.clamp(0.0, 1.0));
                    let from = skirt_vertex(&mesh.vertices, first, within);
                    mesh.vertices.push(RiverVertex {
                        xz: Vector2::new(x0 + x * COLUMN_SIZE, z0 + z * COLUMN_SIZE),
                        ..from
                    });
                    mesh.sources.vertices.push(VertexSource::Skirt {
                        first,
                        within,
                        dry: held.clone(),
                    });
                }
                mesh.indices.extend_from_slice(&[
                    base,
                    base + 2,
                    base + 1,
                    base,
                    base + 3,
                    base + 2,
                ]);
            }
        }
    }
}

/// The eight columns around a cell, as (di, dk).
const AROUND: [(i32, i32); 8] = [
    (-1, -1),
    (0, -1),
    (1, -1),
    (-1, 0),
    (1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
];

/// The quarters of a column, as the side of its centre each lies on.
const QUARTERS: [(i32, i32); 4] = [(-1, -1), (1, -1), (1, 1), (-1, 1)];

/// Whether the cell at (0, 0) lays the skirt over the quarter on the
/// (sx, sz) side of the dry column `(di, dk)` from it, as `at` tells what
/// stands in each column around it at its level: the first wet column of
/// those touching the quarter beside it (on x, then on z), diagonal to it,
/// then the column's far sides (x, then z) lays it. Every cell around the
/// column asks the same question of the same columns, so each quarter has
/// one owner. If it is the cell's, the dry cells that would have taken it
/// had they been wet: the skirt stands while they stay dry.
fn skirt_owner(
    (di, dk): (i32, i32),
    (sx, sz): (i32, i32),
    at: impl Fn(i32, i32) -> Beside,
) -> Option<SmallVec<[SpanIndex; 4]>> {
    let mut passed = SmallVec::new();
    for (cx, cz) in [(sx, 0), (0, sz), (sx, sz), (-sx, 0), (0, -sz)] {
        let (x, z) = (di + cx, dk + cz);
        match at(x, z) {
            Beside::Wet if (x, z) == (0, 0) => return Some(passed),
            Beside::Wet => return None,
            Beside::Dry(_, Some(cell)) => passed.push(cell),
            _ => {}
        }
    }
    None
}

/// The cell's surface at `within` its square, in columns from its minimum
/// corner: its quad's corners (vertex `first + i`, counter-clockwise from
/// the minimum) blended across it.
pub(super) fn skirt_vertex(
    vertices: &[RiverVertex],
    first: u32,
    within: Vector2<f32>,
) -> RiverVertex {
    let corner = |i: usize| vertices[first as usize + i];
    let (u, v) = (within.x, within.y);
    let weights = [(1.0 - u) * (1.0 - v), u * (1.0 - v), u * v, (1.0 - u) * v];
    let mut out = corner(0);
    out.surface = 0.0;
    out.depth = 0.0;
    out.flow = Vector2::zeros();
    for (i, w) in weights.into_iter().enumerate() {
        let c = corner(i);
        out.surface += w * c.surface;
        out.depth += w * c.depth;
        out.flow += c.flow * w;
    }
    out
}

/// Most steps a fall is drawn on past (`FieldScene::landing`).
const MAX_STEPS_PASSED: usize = 16;

/// What a wet cell's surface meets in a column beside it.
#[derive(Debug, Clone, Copy)]
enum Beside {
    /// Water drawn there: its own surface covers the column.
    Wet,
    /// No water: the column's slot, and its dry cell at the water's height,
    /// none where the column is rock there.
    Dry(u32, Option<SpanIndex>),
    /// Past the map's edge.
    Outside,
}

impl FieldScene<'_> {
    /// A sheet of water falling over the edge a cell shares with the lower
    /// cell across `step`: from the cell's own corners on that edge (vertex
    /// `first + i`) down to the lower cell's surface at its corners there,
    /// so it meets both surfaces, its foot thrown out from the lip and its
    /// body curling over between them (`nappe`). It is as deep as the water pours over the edge, no deeper
    /// than the water it falls from, so it is cut where little or nothing
    /// pours.
    ///
    /// Where the lower cell is a step the fall runs over (`passed`), the
    /// sheet falls on past it to the water it `landing`s in.
    ///
    /// Where the lower column's own floor carries the ledge on
    /// (`ledge_reach`), each
    /// end of the sheet hangs out at the lip instead, and the cell's surface
    /// runs on over the ledge to meet it. The ledge is told by the two beds
    /// alone, so the sheet keeps its place while the water moves.
    fn sheet(
        &self,
        mesh: &mut RiverMesh,
        step: Step,
        first: u32,
        (own_span, own): (SpanRef, SpanIndex),
        lower_span: SpanRef,
        (landing, landing_level): (SpanRef, f64),
    ) {
        let ends = edge_ends(step);
        let face = Vector2::new(step.di as f32, step.dk as f32);
        let lower = self.field.graph().span(lower_span);
        let ledge = 0.5 * (self.field.cells().bed(own) + lower.bed);
        let quarters = lower.quarters;
        let shift =
            ends.map(|(_, sides)| face * ledge_reach(step, sides, &quarters, ledge) * COLUMN_SIZE);
        if shift.iter().any(|s| *s != Vector2::zeros()) {
            let base = mesh.vertices.len() as u32;
            for (&(i, _), shift) in ends.iter().zip(shift) {
                let top = mesh.vertices[(first as usize) + i];
                mesh.vertices.push(RiverVertex {
                    xz: top.xz + shift,
                    ..top
                });
                mesh.sources.vertices.push(VertexSource::Edge {
                    from: first + i as u32,
                });
            }
            let [a, b] = ends.map(|(i, _)| first + i as u32);
            mesh.indices
                .extend_from_slice(&[a, b, base + 1, a, base + 1, base]);
        }
        let frays = self.frays(own_span, own, step, ends);
        let base = self.lip_vertices(mesh, first, own, ends, shift, face, frays);
        let mut feet = [0; 2];
        for (end, &(_, (sx, sz))) in ends.iter().enumerate() {
            let (foot, _, source) = self.corner(landing, landing_level, sx, sz);
            let VertexSource::Corner { cells, shape } = source else {
                unreachable!("a corner reads its cells");
            };
            for &c in &cells {
                mesh.sources.reads(self.field.cells(), c);
            }
            let fall = mesh.vertices[base as usize + end].surface - foot.height as f32;
            feet[end] = foot_vertex(mesh, base + end as u32, fall);
            mesh.sources.vertices.push(VertexSource::Foot {
                from: base + end as u32,
                cells,
                shape,
                own,
            });
        }
        nappe(mesh, [base, base + 1], feet);
    }
}

impl FieldScene<'_> {
    /// The tops of a sheet falling from cell `own` over the edge facing
    /// `face`: its corners at `ends` (vertex `first + i`) moved out by
    /// `shift`, as deep as the water pours over the edge, each fraying out
    /// where `frays` says. Returns the first.
    #[allow(clippy::too_many_arguments)]
    fn lip_vertices(
        &self,
        mesh: &mut RiverMesh,
        first: u32,
        own: SpanIndex,
        ends: [(usize, (i32, i32)); 2],
        shift: [Vector2<f32>; 2],
        face: Vector2<f32>,
        frays: [bool; 2],
    ) -> u32 {
        let base = mesh.vertices.len() as u32;
        let pour = field_sources::pour(self.field, &self.drawn_cells(), own, face);
        let edge = Vector2::new(-face.y, face.x);
        let xz = ends.map(|(i, _)| mesh.vertices[(first as usize) + i].xz);
        for (end, (&(i, _), shift)) in ends.iter().zip(shift).enumerate() {
            let top = mesh.vertices[(first as usize) + i];
            let side = (xz[end] - xz[1 - end]).dot(&edge).signum();
            let along = (top.xz + shift).dot(&edge);
            mesh.vertices.push(RiverVertex {
                xz: top.xz + shift,
                face,
                depth: top.depth.min(pour),
                sheet: Vector4::new(0.0, 0.0, side, along),
                fray: frays[end] as u32 as f32,
                ..top
            });
            mesh.sources.vertices.push(VertexSource::Lip {
                from: first + i as u32,
                own,
            });
        }
        base
    }

    /// Whether each end of the sheet falling from `span` (cell `at`) across
    /// `step` is an end of its fall, frayed out, rather than joining the
    /// sheet beside it: no other water falls over an edge meeting at that
    /// end's corner, between the four columns round it.
    fn frays(
        &self,
        span: SpanRef,
        at: SpanIndex,
        step: Step,
        ends: [(usize, (i32, i32)); 2],
    ) -> [bool; 2] {
        let graph = self.field.graph();
        let drawn = self.drawn_cells();
        let wet = |column: Column| -> SmallVec<[(SpanRef, SpanIndex); 2]> {
            if !graph.contains_column(column) {
                return SmallVec::new();
            }
            (0..graph.spans(column).len())
                .map(|ordinal| SpanRef::key(column, ordinal as u8))
                .filter_map(|key| Some((key, self.cell(key)?)))
                .filter(|&(_, c)| drawn.drawn(c))
                .collect()
        };
        // Whether water in `from` falls to `to` across (di, dk), other than
        // this sheet's own.
        let pours = |from: Column, di: i32, dk: i32| {
            wet(from).into_iter().any(|(a_span, a)| {
                self.wet_across(a_span, di, dk).into_iter().any(|(_, b)| {
                    let own = a == at && (di, dk) == (step.di, step.dk);
                    !own && falls(drawn.surface(a), drawn.bed(a) as f64, drawn.surface(b))
                })
            })
        };
        let column = span.column;
        ends.map(|(_, sides)| {
            let (ei, ek) = along_edge(step, sides);
            let along = column.offset(ei, ek);
            let below = column.offset(step.di, step.dk);
            let edges = [
                (column, ei, ek),
                (along, step.di, step.dk),
                (column, step.di, step.dk),
                (below, ei, ek),
            ];
            !edges
                .iter()
                .any(|&(from, di, dk)| pours(from, di, dk) || pours(from.offset(di, dk), -di, -dk))
        })
    }

    /// A sheet of water falling off the map's edge across `step`, from the
    /// cell's own corners on that edge (vertex `first + i`) down to the
    /// water `outside`: the sea's level, or `VOID_DROP` below the cell's
    /// surface where nothing lies beyond. Drawn where the cell `falls` to it,
    /// as deep as the water pours out.
    fn brink(
        &self,
        mesh: &mut RiverMesh,
        span: SpanRef,
        step: Step,
        first: u32,
        own: SpanIndex,
        outside: Option<f32>,
    ) {
        let drawn = self.drawn_cells();
        let level = drawn.surface(own);
        let outside = outside.map(f64::from);
        if !falls(level, drawn.bed(own) as f64, off_map(level, outside)) {
            return;
        }
        let ends = edge_ends(step);
        let face = Vector2::new(step.di as f32, step.dk as f32);
        let frays = ends.map(|(_, sides)| {
            let (di, dk) = along_edge(step, sides);
            self.wet_across(span, di, dk).is_empty()
        });
        let shift = [Vector2::zeros(); 2];
        let base = self.lip_vertices(mesh, first, own, ends, shift, face, frays);
        let mut feet = [0; 2];
        for (end, foot) in feet.iter_mut().enumerate() {
            let top = mesh.vertices[base as usize + end];
            let fall = top.surface - off_map(top.surface as f64, outside) as f32;
            *foot = foot_vertex(mesh, base + end as u32, fall);
            mesh.sources.vertices.push(VertexSource::Brink {
                from: base + end as u32,
                own,
                outside,
            });
        }
        nappe(mesh, [base, base + 1], feet);
    }
}

impl FieldScene<'_> {
    /// Source `index`'s jet, down to the water in the cell it lands in.
    fn jet(&self, mesh: &mut RiverMesh, index: usize, source: &Source) {
        let landing = source.landing[0];
        let Some(jet) = Jet::new(source, self.drawn_cells().surface(landing) as f32) else {
            return;
        };
        let mut rows = Vec::with_capacity(JET_ROWS as usize + 1);
        for row in 0..=JET_ROWS {
            let first = mesh.vertices.len() as u32;
            for around in 0..JET_RING {
                mesh.vertices.push(jet.vertex(row, around));
                mesh.sources.vertices.push(VertexSource::Jet {
                    source: index as u32,
                    landing,
                    row,
                    around,
                });
            }
            let ring: SmallVec<[u32; JET_RING as usize + 1]> =
                (0..=JET_RING).map(|k| first + k % JET_RING).collect();
            rows.push(ring);
        }
        strip(mesh, &rows);

        let first = mesh.vertices.len() as u32;
        for corner in 0..=PLUNGE_CORNERS {
            mesh.vertices.push(jet.plunge(corner));
            mesh.sources.vertices.push(VertexSource::JetPlunge {
                source: index as u32,
                landing,
                corner,
            });
        }
        let ring: SmallVec<[u32; PLUNGE_CORNERS as usize + 1]> = (0..=PLUNGE_CORNERS)
            .map(|k| first + 1 + k % PLUNGE_CORNERS)
            .collect();
        fan(mesh, first, &ring);
    }
}

/// Push the foot of a sheet whose lip is vertex `lip`, falling `fall`
/// (`field_sources::foot`), and return its index.
fn foot_vertex(mesh: &mut RiverMesh, lip: u32, fall: f32) -> u32 {
    let foot = field_sources::foot(&mut mesh.vertices[lip as usize], fall);
    mesh.vertices.push(foot);
    mesh.vertices.len() as u32 - 1
}

/// The body of a sheet whose top face runs from its `lips` to its `feet`,
/// one of each at either end of its edge: both its faces, row by row
/// between them, and the foam where it lands.
fn nappe(mesh: &mut RiverMesh, lips: [u32; 2], feet: [u32; 2]) {
    let nappes = [0, 1].map(|end| {
        Nappe::between(
            mesh.vertices[lips[end] as usize],
            &mesh.vertices[feet[end] as usize],
        )
    });
    for face in Face::BOTH {
        let mut rows = Vec::with_capacity(NAPPE_ROWS as usize + 1);
        for row in 0..=NAPPE_ROWS {
            let mut pair = [0; 2];
            for (end, index) in pair.iter_mut().enumerate() {
                *index = match (face, row) {
                    (Face::Top, 0) => lips[end],
                    (Face::Top, NAPPE_ROWS) => feet[end],
                    _ => {
                        mesh.vertices.push(nappes[end].vertex(row, face));
                        mesh.sources.vertices.push(VertexSource::Nappe {
                            lip: lips[end],
                            foot: feet[end],
                            row,
                            face,
                        });
                        mesh.vertices.len() as u32 - 1
                    }
                };
            }
            rows.push(pair);
        }
        strip(mesh, &rows);
    }
    let mut rows = Vec::with_capacity(2);
    for share in [0.0, 1.0] {
        let mut pair = [0; 2];
        for (end, index) in pair.iter_mut().enumerate() {
            let lip = mesh.vertices[lips[end] as usize];
            mesh.vertices
                .push(plunge(lip, &mesh.vertices[feet[end] as usize], share));
            mesh.sources.vertices.push(VertexSource::Plunge {
                lip: lips[end],
                foot: feet[end],
                share,
            });
            *index = mesh.vertices.len() as u32 - 1;
        }
        rows.push(pair);
    }
    strip(mesh, &rows);
}

/// The ends of a cell's edge across `step`: as the cell's corner indices,
/// and as the sides of the neighbouring column they lie on.
fn edge_ends(step: Step) -> [(usize, (i32, i32)); 2] {
    match (step.di, step.dk) {
        (1, _) => [(1, (-1, -1)), (2, (-1, 1))],
        (-1, _) => [(0, (1, -1)), (3, (1, 1))],
        (_, 1) => [(3, (-1, -1)), (2, (1, -1))],
        _ => [(0, (-1, 1)), (1, (1, 1))],
    }
}

/// The step along a cell's edge across `step` towards its end on the
/// neighbouring column's `sides` (`edge_ends`).
fn along_edge(step: Step, (sx, sz): (i32, i32)) -> (i32, i32) {
    if step.di != 0 {
        (0, sz)
    } else {
        (sx, 0)
    }
}

fn quad(
    mesh: &mut RiverMesh,
    column: Column,
    corners: [Corner; 4],
    flows: [Vector2<f32>; 4],
    sources: [VertexSource; 4],
) {
    let (x0, z0) = column.min_corner();
    let offsets = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    let base = mesh.vertices.len() as u32;
    for (((cx, cz), corner), flow) in offsets.iter().zip(corners).zip(flows) {
        mesh.vertices.push(RiverVertex {
            xz: Vector2::new(x0 + cx * COLUMN_SIZE, z0 + cz * COLUMN_SIZE),
            surface: corner.height as f32,
            depth: corner.depth as f32,
            flow,
            face: Vector2::zeros(),
            sheet: Vector4::zeros(),
            normal: Vector3::y(),
            fray: 0.0,
        });
    }
    mesh.sources.vertices.extend(sources);
    mesh.indices
        .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
}

impl WaterScene for FieldScene<'_> {
    fn mesh_key(&self) -> MeshKey {
        Vec::new()
    }

    fn build_mesh(&self) -> WaterMesh {
        WaterMesh::default()
    }

    /// The sea's level, for its ring past the map's edges.
    fn level(&self, body: SurfaceId) -> Option<f32> {
        self.field
            .sea()
            .filter(|_| body == FIELD_BODY)
            .map(|sea| sea.level)
    }

    fn clock(&self) -> f32 {
        self.clock
    }

    /// The sea's swell, for its ring past the map's edges.
    fn swell(&self, body: SurfaceId) -> Swell {
        self.sea()
            .filter(|_| body == FIELD_BODY)
            .map_or(Swell::default(), |sea| sea.swell)
    }

    fn sea(&self) -> Option<Sea> {
        self.waves?.sea()
    }

    fn ripple_tiles(&self) -> Vec<RippleTileView<'_>> {
        self.ripple_views()
    }

    /// Every chunk whose water stands still, at its level, and every drawn
    /// column of the chunks whose water runs, at its surface.
    fn surface_patches(&self) -> Vec<WaterPatch> {
        let drawn = self.drawn_cells();
        let mut patches = Vec::new();
        for (slot, chunk) in self.field.cells().chunks.iter().enumerate() {
            if let Some(level) = still_level(self.field, chunk.coord) {
                patches.push(WaterPatch::tile(chunk.coord.x, chunk.coord.z, level));
                continue;
            }
            for flat in 0..chunk.depth.len() {
                let at = SpanIndex {
                    slot: slot as u32,
                    flat: flat as u32,
                };
                if drawn.drawn(at) {
                    let column = chunk.coord.column(chunk.column_of(flat).0);
                    patches.push(WaterPatch::column(column, drawn.surface(at) as f32));
                }
            }
        }
        patches
    }

    fn ocean_key(&self) -> MeshKey {
        self.field
            .sea()
            .map(|_| (FIELD_BODY, 0))
            .into_iter()
            .collect()
    }

    /// The ring past the map's open edges only: the sea inside them is the
    /// field's own water.
    fn build_ocean(&self) -> WaterMesh {
        let mut mesh = WaterMesh::default();
        let graph = self.field.graph();
        if let (Some(sea), Some((min, max))) = (self.field.sea(), occupied_bounds(graph)) {
            let map = MapRect {
                i0: min.i,
                k0: min.k,
                i1: max.i + 1,
                k1: max.k + 1,
            };
            // The sea's edges run +x, +z, −x, −z; the ring's −x, +x, −z, +z.
            let open = [sea.open[2], sea.open[0], sea.open[3], sea.open[1]];
            append_ring(&mut mesh, FIELD_BODY, sea.level, map, open);
        }
        mesh
    }

    /// One surface per chunk, by slot.
    fn river_key(&self) -> MeshKey {
        self.versions
            .iter()
            .enumerate()
            .map(|(slot, &v)| (SurfaceId(slot as u32), v))
            .collect()
    }

    fn build_river(&self, id: SurfaceId) -> RiverMesh {
        let slot = id.0 as usize;
        let mut mesh = RiverMesh::default();
        let Some(chunk) = self.field.cells().chunks.get(slot) else {
            return mesh;
        };
        let drawn = self.drawn_cells();
        let at = |flat: usize| SpanIndex {
            slot: slot as u32,
            flat: flat as u32,
        };
        mesh.sources.slot = slot as u32;
        mesh.sources.read = self.depths.now;
        // Its own layout, which an empty surface reads nothing else to note.
        mesh.sources.reads(self.field.cells(), at(0));
        mesh.sources.drawn = (0..chunk.depth.len())
            .map(|flat| drawn.drawn(at(flat)))
            .collect();
        for (flat, (local, ordinal)) in chunk.cell_columns().enumerate() {
            if !mesh.sources.drawn[flat] {
                continue;
            }
            let span = SpanRef::key(chunk.coord.column(local), ordinal);
            self.cell_surface(&mut mesh, span, at(flat), drawn.surface(at(flat)));
        }
        for (index, source) in self.field.sources().iter().enumerate() {
            if source
                .landing
                .first()
                .is_some_and(|c| c.slot == slot as u32)
            {
                self.jet(&mut mesh, index, source);
            }
        }
        if !mesh.indices.is_empty() || !mesh.sheet_indices.is_empty() {
            mesh.draws.push(RiverDraw {
                id,
                first_index: 0,
                index_count: mesh.indices.len() as u32,
                first_sheet_index: 0,
                sheet_index_count: mesh.sheet_indices.len() as u32,
            });
        }
        mesh
    }

    fn refresh_river(&self, _id: SurfaceId, mesh: &mut RiverMesh) -> Refreshed {
        field_sources::refresh(self.field, &self.drawn_cells(), mesh)
    }
}

impl WaterScene for WaterWorld {
    fn mesh_key(&self) -> MeshKey {
        FieldScene::of(self).mesh_key()
    }

    fn build_mesh(&self) -> WaterMesh {
        FieldScene::of(self).build_mesh()
    }

    fn level(&self, body: SurfaceId) -> Option<f32> {
        FieldScene::of(self).level(body)
    }

    fn clock(&self) -> f32 {
        FieldScene::of(self).clock()
    }

    fn swell(&self, body: SurfaceId) -> Swell {
        FieldScene::of(self).swell(body)
    }

    fn sea(&self) -> Option<Sea> {
        FieldScene::of(self).sea()
    }

    fn ripple_tiles(&self) -> Vec<RippleTileView<'_>> {
        FieldScene::of(self).ripple_views()
    }

    fn surface_patches(&self) -> Vec<WaterPatch> {
        FieldScene::of(self).surface_patches()
    }

    fn ocean_key(&self) -> MeshKey {
        FieldScene::of(self).ocean_key()
    }

    fn build_ocean(&self) -> WaterMesh {
        FieldScene::of(self).build_ocean()
    }

    fn river_key(&self) -> MeshKey {
        FieldScene::of(self).river_key()
    }

    fn build_river(&self, id: SurfaceId) -> RiverMesh {
        FieldScene::of(self).build_river(id)
    }

    fn refresh_river(&self, id: SurfaceId, mesh: &mut RiverMesh) -> Refreshed {
        FieldScene::of(self).refresh_river(id, mesh)
    }
}
