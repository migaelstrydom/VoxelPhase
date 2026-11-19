//! The shallow-water field's surface, drawn through the river pipeline.
//!
//! ```text
//!   wet cell ──▶ one quad over its column, each corner the mean surface of
//!                its group of the wet cells meeting there: split where one
//!                falls to the next; a corner where its water goes over
//!                both edges is cut off, so a lip running across the
//!                lattice is straight
//!   a fall from the cell (`water::field::Fall`)
//!            ──▶ a curtain hung from the cell's edge down the fall's arc
//!                to where it lands, one with the curtains of the falls
//!                beside it, and foam where it lands (`curtain`)
//!   a source landing in the chunk
//!            ──▶ a tube along its jet's arc, from where it comes out
//!                down to the water it lands in, and foam where it lands
//!   dry column beside one
//!            ──▶ a skirt carrying the cell's surface over it, each quarter
//!                of the column laid by one wet cell; a film's running out
//!                across the column
//! ```
//!
//! The mesher does not decide where a shore is. The skirt reaches past it,
//! and the draw finds it to the pixel: the terrain hides the skirt wherever
//! the ground stands above the water, and where the water is shallow
//! `water.frag` fades the surface as the ground behind it, read from the
//! scene's depth, comes up to it. Nor does it decide where water falls: the
//! field does, and the curtain is drawn along the arc its water flies.
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

use crate::water::field::{DrawnDepths, Fall, WaterField};
use crate::water::geometry::{Column, SpanIndex, SpanRef, Step, COLUMN_SIZE};
use crate::water::ids::SurfaceId;
use crate::water::surface::{Swell, WindExposure};
use crate::water::waves::{still_level, Sea, SurfaceWaves};
use crate::water::{WaterWorld, FIELD_BODY};

use super::curtain::{fan, strip, Curtain};
use super::divide::WaterPatch;
use super::field_sources::{
    self, corner_carried, gather, landing_jets, Carried, Corner, CurtainSource, DrawnCells,
    Meeting, VertexSource,
};
use super::jet::{Jet, JET_RING, JET_ROWS, PLUNGE_CORNERS};
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
            falls: self.field.falls(),
            clock: self.clock,
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
    /// surface stands at `level`, what its water carries, and where it
    /// reads from: the cell and the cells across from it on each side and
    /// diagonally, as `gather` joins them.
    fn corner(
        &self,
        span: SpanRef,
        level: f64,
        sx: i32,
        sz: i32,
    ) -> (Corner, Carried, VertexSource) {
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
        let carried = corner_carried(self.field, &cells, shape);
        (corner, carried, VertexSource::Corner { cells, shape })
    }

    /// A cell's surface, the curtains of the water falling from it, and the
    /// skirt around it.
    fn cell_surface(&self, mesh: &mut RiverMesh, span: SpanRef, at: SpanIndex, level: f64) {
        let cells = self.field.cells();
        let column = span.column;
        let built =
            [(-1, -1), (1, -1), (1, 1), (-1, 1)].map(|(sx, sz)| self.corner(span, level, sx, sz));
        for (_, _, source) in &built {
            if let VertexSource::Corner { cells: read, .. } = source {
                for &c in read {
                    mesh.sources.reads(cells, c);
                }
            }
        }
        let over = [0, 1, 2, 3].map(|direction| self.field.fall(at, direction).copied());
        let first = mesh.vertices.len() as u32;
        // Only water still going over an edge shapes the surface there: a
        // spent fall's curtain hangs from where its water left. Nor does
        // water leaving by a slot down in its side, which goes from the
        // slot and not from the surface over it.
        let drawn = self.drawn_cells();
        let off_the_top = |fall: &Fall| drawn.off_the_top(fall);
        let pouring =
            over.map(|fall| fall.is_some_and(|fall| fall.spent.is_none() && off_the_top(&fall)));
        let lips = lay_cell(mesh, column, built, pouring);
        for (fall, lip) in over.iter().zip(lips) {
            if let Some(fall) = fall {
                self.curtain(mesh, fall, lip.filter(|_| off_the_top(fall)));
            }
        }
        self.skirt(mesh, column, level, first);
    }

    /// The curtain of `fall`, hung along its lip from the surface vertices
    /// `lip` (its two ends and, between them, its middle), or from its own
    /// arc where the water it leaves has no surface drawn: water filling a
    /// hole to its roof and bursting out of it.
    fn curtain(&self, mesh: &mut RiverMesh, fall: &Fall, lip: Option<[u32; 3]>) {
        let cells = self.field.cells();
        let beside = [0, 1].map(|end| self.field.fall_beside(fall, end));
        mesh.sources.reads(cells, fall.from);
        for (other, _) in beside.iter().flatten() {
            mesh.sources.reads(cells, other.from);
        }
        let hung = lip.map(|lip| carried_out(mesh, fall, &beside, lip));
        let top = hung.map(|hung| field_sources::hung_from(&mesh.vertices, hung));
        let others = beside.map(|b| b.map(|(other, _)| other));
        let curtain = Curtain::new(fall, others, top, self.clock);
        let first = mesh.vertices.len() as u32;
        let vertices = curtain.vertices();
        mesh.sources
            .vertices
            .extend(std::iter::repeat_n(VertexSource::Curtain, vertices.len()));
        mesh.vertices.extend(vertices);
        Curtain::join(mesh, first, beside.map(|b| b.is_none()));
        mesh.sources.curtains.push(CurtainSource {
            fall: (fall.from, fall.direction),
            beside: beside.map(|b| b.map(|(other, _)| (other.from, other.direction))),
            hung,
            first,
            count: mesh.vertices.len() as u32 - first,
            spent: fall.spent.is_some(),
        });
    }

    /// Whether water run full in a hole in `column` is drawn about `level`:
    /// over its own roof, where the graph has only the ground above it.
    fn run_full_at(&self, column: Column, level: f64) -> bool {
        let (cells, graph) = (self.field.cells(), self.field.graph());
        // A hole has ground over it: a span of its own above the hole's.
        let spans = graph.spans(column).len();
        if cells.run_full.is_empty() || spans < 2 {
            return false;
        }
        let drawn = self.drawn_cells();
        (0..spans - 1).any(|ordinal| {
            cells.index(graph, column, ordinal as u8).is_some_and(|c| {
                cells.run_full.contains_key(&c)
                    && (drawn.level(c)).is_some_and(|head| (head - level).abs() < RUN_FULL_NEAR)
            })
        })
    }

    /// What stands in `column` at `level`, beside a wet cell there: water,
    /// a dry cell (none where the column is rock at that height), a drop,
    /// or the map's edge.
    fn beside(&self, column: Column, level: f64) -> Beside {
        let graph = self.field.graph();
        let Some(slot) = graph.slot(column.chunk()) else {
            return Beside::Outside;
        };
        let span = graph.span_at(column, level as f32);
        let cell = span.and_then(|span| self.cell(span));
        match cell {
            Some(cell) if self.drawn_cells().drawn(cell) => Beside::Wet,
            _ if self.run_full_at(column, level) => Beside::Wet,
            // Dry ground a fall below the water all over the column: no
            // bank the water could end against, but the foot of a drop its
            // water goes over.
            Some(_)
                if span.is_some_and(|span| {
                    (graph.span(span).floor_max as f64) < level - SKIRTED_DROP
                }) =>
            {
                Beside::Below
            }
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
                    let past = (x - within.x).abs().max((z - within.y).abs());
                    let from = skirt_vertex(&mesh.vertices, first, within, past);
                    mesh.vertices.push(RiverVertex {
                        xz: Vector2::new(x0 + x * COLUMN_SIZE, z0 + z * COLUMN_SIZE),
                        ..from
                    });
                    mesh.sources.vertices.push(VertexSource::Skirt {
                        first,
                        within,
                        past,
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
/// the minimum) blended across it, `past` columns out from it (0 on its
/// edge, 1 at the dry column's far side).
///
/// A film's depth runs out to nothing across the dry column, so the draw's
/// shore depth ends it there wherever the ground does not end it first: a
/// front's thin edge on a floor level with it, which no bank rises to meet.
/// Deeper water keeps its depth to be cut by the bank beside it, and to
/// meet the sea's ring past the map's edge.
pub(super) fn skirt_vertex(
    vertices: &[RiverVertex],
    first: u32,
    within: Vector2<f32>,
    past: f32,
) -> RiverVertex {
    let corner = |i: usize| vertices[first as usize + i];
    let (u, v) = (within.x, within.y);
    let weights = [(1.0 - u) * (1.0 - v), u * (1.0 - v), u * v, (1.0 - u) * v];
    let mut out = corner(0);
    out.surface = 0.0;
    out.depth = 0.0;
    out.flow = Vector2::zeros();
    out.air = 0.0;
    for (i, w) in weights.into_iter().enumerate() {
        let c = corner(i);
        out.surface += w * c.surface;
        out.depth += w * c.depth;
        out.flow += c.flow * w;
        out.air += w * c.air;
    }
    if out.depth < FILM_DEPTH {
        out.depth *= 1.0 - past;
    }
    out
}

/// Depth under which a skirt's water is a film that runs out across the dry
/// column, m: the depth from which `water.frag` reads the ground behind the
/// water as its depth (`GROUND_TRUSTED_DEPTH`).
const FILM_DEPTH: f32 = 0.3;

/// How near a level the head of water run full in a hole must be drawn for
/// the hole's column to be covered at that level, m.
const RUN_FULL_NEAR: f64 = 1.0;

/// How far below a cell's water the whole floor of a dry column beside it
/// must lie for the column to be a drop, not a bank, m: no skirt is laid
/// out over it, since no shore can cross it.
const SKIRTED_DROP: f64 = 0.5;

/// What a wet cell's surface meets in a column beside it.
#[derive(Debug, Clone, Copy)]
enum Beside {
    /// Water drawn there: its own surface covers the column.
    Wet,
    /// No water: the column's slot, and its dry cell at the water's height,
    /// none where the column is rock there.
    Dry(u32, Option<SpanIndex>),
    /// Dry ground far below the water: the foot of a drop.
    Below,
    /// Past the map's edge.
    Outside,
}

impl FieldScene<'_> {
    /// The jet `index` of those landing in the mesh's chunk, down to the
    /// water in the cell it lands in.
    fn jet(&self, mesh: &mut RiverMesh, index: usize) {
        let source = mesh.sources.jets[index];
        let landing = source.landing;
        let Some(jet) = Jet::new(&source, self.drawn_cells().surface(landing) as f32) else {
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

/// The corners of a cell's quad, counter-clockwise from its minimum, in
/// columns from it.
const CORNERS: [(f32, f32); 4] = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];

/// For each direction (an index into `ORTHOGONAL`), the corners at the two
/// ends of a cell's edge that way, in `edge_ends`' order.
const EDGE_CORNERS: [[usize; 2]; 4] = [[1, 2], [3, 2], [0, 3], [0, 1]];

/// For each corner, the directions of the two edges meeting there: the one
/// that comes to it going round the quad, and the one that leaves it.
const CORNER_EDGES: [[usize; 2]; 4] = [[2, 3], [3, 0], [0, 1], [1, 2]];

/// Lay a cell's surface over `column` from its four `built` corners, its
/// water going over the edges `falls` says (by direction). Its four corner
/// vertices come first, in order. Returns, for each edge its water goes
/// over, the vertices along that lip: its two ends (`edge_ends`) and,
/// between them, its middle.
///
/// A corner where the water goes over both edges is cut off between their
/// middles, and those edges' lips end at the middle of the cut: a lip
/// running across the lattice is a staircase of such corners, and cut so it
/// is straight.
fn lay_cell(
    mesh: &mut RiverMesh,
    column: Column,
    built: [(Corner, Carried, VertexSource); 4],
    falls: [bool; 4],
) -> [Option<[u32; 3]>; 4] {
    let (x0, z0) = column.min_corner();
    let base = mesh.vertices.len() as u32;
    for ((cx, cz), (corner, carried, source)) in CORNERS.into_iter().zip(built) {
        mesh.vertices.push(RiverVertex {
            xz: Vector2::new(x0 + cx * COLUMN_SIZE, z0 + cz * COLUMN_SIZE),
            surface: corner.height as f32,
            depth: corner.depth as f32,
            flow: carried.flow,
            face: Vector2::zeros(),
            sheet: Vector4::zeros(),
            normal: Vector3::y(),
            fray: 0.0,
            air: carried.foam,
            life: RiverVertex::ALWAYS,
        });
        mesh.sources.vertices.push(source);
    }
    let middles: [Option<u32>; 4] = std::array::from_fn(|direction| {
        falls[direction].then(|| {
            let [a, b] = EDGE_CORNERS[direction].map(|corner| base + corner as u32);
            between(mesh, a, b)
        })
    });
    let cuts: [Option<u32>; 4] = std::array::from_fn(|corner| {
        let [before, after] = CORNER_EDGES[corner];
        Some(between(mesh, middles[before]?, middles[after]?))
    });
    let mut outline: SmallVec<[u32; 8]> = SmallVec::new();
    for corner in 0..4 {
        match (cuts[corner], CORNER_EDGES[corner]) {
            (Some(_), [before, after]) => {
                outline.extend(middles[before].into_iter().chain(middles[after]))
            }
            _ => outline.push(base + corner as u32),
        }
    }
    for pair in outline[1..].windows(2) {
        mesh.indices
            .extend_from_slice(&[outline[0], pair[1], pair[0]]);
    }
    std::array::from_fn(|direction| {
        let end = |corner: usize| cuts[corner].unwrap_or(base + corner as u32);
        let [a, b] = EDGE_CORNERS[direction];
        Some([end(a), middles[direction]?, end(b)])
    })
}

/// A surface vertex midway between vertices `a` and `b`.
fn between(mesh: &mut RiverMesh, a: u32, b: u32) -> u32 {
    let (from, to) = (mesh.vertices[a as usize], mesh.vertices[b as usize]);
    mesh.vertices
        .push(field_sources::midway(from, &mesh.vertices[b as usize]));
    debug_assert_eq!(to.sheet, Vector4::zeros());
    mesh.sources
        .vertices
        .push(VertexSource::Between { of: [a, b] });
    mesh.vertices.len() as u32 - 1
}

/// How far past its cell's edge the lip of `fall` stands at its end `end`,
/// in columns, running on there into the fall `beside` it: the mean of the
/// two where they go over the same way, so both hang from one point; none
/// where the lip turns a corner.
fn end_reach(fall: &Fall, end: usize, beside: Option<(&Fall, usize)>) -> f32 {
    match beside {
        None => fall.reach[end],
        Some((other, meets)) if other.direction == fall.direction => {
            0.5 * (fall.reach[end] + other.reach[meets])
        }
        Some(_) => 0.0,
    }
}

/// Where the curtain of `fall` hangs from: the surface vertices along its
/// lip, `lip`, or where the column below carries the ledge on past the
/// cell's edge, copies of them out where the ledge really ends, with the
/// cell's surface carried out over the ledge to meet them.
fn carried_out(
    mesh: &mut RiverMesh,
    fall: &Fall,
    beside: &[Option<(&Fall, usize)>; 2],
    lip: [u32; 3],
) -> [u32; 3] {
    let reach = [
        end_reach(fall, 0, beside[0]),
        0.5 * (fall.reach[0] + fall.reach[1]),
        end_reach(fall, 1, beside[1]),
    ];
    if reach.iter().all(|&r| r == 0.0) {
        return lip;
    }
    let base = mesh.vertices.len() as u32;
    for (from, reach) in lip.into_iter().zip(reach) {
        let top = mesh.vertices[from as usize];
        mesh.vertices.push(RiverVertex {
            xz: top.xz + fall.face() * (reach * COLUMN_SIZE),
            ..top
        });
        mesh.sources.vertices.push(VertexSource::Edge { from });
    }
    let out = [base, base + 1, base + 2];
    for triangle in [
        [lip[0], out[0], out[1]],
        [lip[0], out[1], lip[1]],
        [lip[1], out[1], out[2]],
        [lip[1], out[2], lip[2]],
    ] {
        face_up(mesh, triangle);
    }
    out
}

/// Add a triangle of surface, wound to face up; nothing if it has no area.
fn face_up(mesh: &mut RiverMesh, [a, b, c]: [u32; 3]) {
    let at = |i: u32| mesh.vertices[i as usize].xz;
    let (p, q, r) = (at(a), at(b), at(c));
    // The upward part of (q − p) × (r − p), with z for the second axis.
    let up = (q.y - p.y) * (r.x - p.x) - (q.x - p.x) * (r.y - p.y);
    if up > 1.0e-9 {
        mesh.indices.extend_from_slice(&[a, b, c]);
    } else if up < -1.0e-9 {
        mesh.indices.extend_from_slice(&[a, c, b]);
    }
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

    fn wind(&self) -> Option<&WindExposure> {
        self.waves.map(SurfaceWaves::exposure)
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
        // Water with no surface of its own still falls: a hole run full.
        let falls = self.field.falls().of_slot(slot);
        for fall in falls {
            if !mesh.sources.drawn[fall.from.flat as usize] {
                self.curtain(&mut mesh, fall, None);
            }
        }
        mesh.sources.falls = field_sources::falls_of(self.field, &drawn, slot as u32).collect();
        mesh.sources.jets = landing_jets(self.field, slot as u32);
        for index in 0..mesh.sources.jets.len() {
            self.jet(&mut mesh, index);
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

    fn wind(&self) -> Option<&WindExposure> {
        Some(self.waves().exposure())
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
