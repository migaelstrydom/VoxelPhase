//! The sea's surface: every column it owns inside the map, and a ring out to
//! the horizon beyond each open edge (§12).
//!
//! ```text
//!   span ownership ──▶ one quad per sea column, plus the shore ring: the
//!                      columns beside it whose ground stands above the sea,
//!                      under the terrain, so the depth test cuts the shore
//!   open edges ──────▶ a band of column-wide quads along the map's edge,
//!                      across which the swell's height fades out, then
//!                      strips of quads coarser towards the horizon, flat
//! ```
//!
//! The band meets the sea's own columns vertex for vertex, and the flat
//! strips beyond it at no height at all, so neither join can crack. The
//! swell's normal is the fragment shader's, so the flat sea still reads as
//! swell.
//!
//! Built at load and again only when the sea claims a lowland. Like a
//! basin, the level and swell are pushed per draw.

use nalgebra::Vector2;
use rustc_hash::FxHashMap;

use crate::water::geometry::{Column, Outlets, SpanChunkCoord, SpanGraph, COLUMN_SIZE, ORTHOGONAL};
use crate::water::ids::StoreId;
use crate::water::network::Ocean;

use super::basin_mesher::{append_columns, WaterDraw, WaterMesh};
use super::vertex::BasinVertex;

/// How far the ring reaches beyond the map, m.
const HORIZON: f32 = 1024.0;

/// Width of the band of column-wide quads along the map's edge, m.
const FINE_BAND: f32 = 8.0;

/// Distances from the map's edge at which the flat rows past the band start,
/// m, coarser towards the horizon.
const RING_ROWS: [f32; 8] = [FINE_BAND, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, HORIZON];

/// Width of the ring's quads along the edge, m. The map's edge is a whole
/// number of these, so the ring meets the sea inside it.
const RING_STEP: f32 = 8.0;

/// Depth given to the ring's floor: open water, deep enough that the swell
/// runs at full height and the tint is the sea's.
const RING_DEPTH: f32 = 100.0;

/// The ring's draws are keyed to a tile no ripple tile can have.
const RING_TILE: SpanChunkCoord = SpanChunkCoord {
    x: i32::MIN,
    z: i32::MIN,
};

/// Append the sea's surface to `mesh`.
pub fn append_ocean(
    mesh: &mut WaterMesh,
    id: StoreId,
    ocean: &Ocean,
    graph: &SpanGraph,
    outlets: &Outlets,
) {
    let (columns, wet) = sea_columns(id, ocean, graph);
    append_columns(mesh, id, &columns, &wet);
    if let Some(sea) = outlets.sea() {
        append_ring(mesh, id, ocean, graph, sea.open);
    }
}

/// Columns the sea owns, with the floor under each, and the shore ring;
/// and the owned columns' floors alone.
fn sea_columns(
    id: StoreId,
    ocean: &Ocean,
    graph: &SpanGraph,
) -> (Vec<(Column, f32)>, FxHashMap<Column, f32>) {
    let mut floors: FxHashMap<Column, f32> = FxHashMap::default();
    for span in graph.all_refs() {
        if graph.owner(span).body == Some(id) {
            let floor = floors.entry(span.column).or_insert(f32::INFINITY);
            *floor = floor.min(graph.span(span).floor_min);
        }
    }
    // Beside the sea, a column whose ground holds the sea back: under the
    // terrain at the waterline. A column the sea could stand in but does not
    // own (a lowland behind a wall) is left out.
    let mut ring: Vec<(Column, f32)> = Vec::new();
    for (&column, &floor) in &floors {
        for step in ORTHOGONAL {
            let next = column.offset(step.di, step.dk);
            if floors.contains_key(&next) || !graph.contains_column(next) {
                continue;
            }
            let dry = graph
                .span_at(next, ocean.level)
                .is_none_or(|s| graph.span(s).floor_min >= ocean.level);
            if dry {
                ring.push((next, floor));
            }
        }
    }
    let mut out: Vec<(Column, f32)> = floors.iter().map(|(&c, &f)| (c, f)).collect();
    ring.sort_by(|a, b| a.0.cmp(&b.0));
    ring.dedup_by(|a, b| a.0 == b.0);
    out.extend(ring);
    out.sort_by(|a, b| a.0.chunk().cmp(&b.0.chunk()).then(a.0.cmp(&b.0)));
    (out, floors)
}

/// The first and last columns with any spans: the terrain's own edge, which
/// the graph's chunk-aligned bounds can overhang.
fn occupied_bounds(graph: &SpanGraph) -> Option<(Column, Column)> {
    let (min, max) = graph.column_bounds();
    let mut out: Option<(Column, Column)> = None;
    for k in min.k..=max.k {
        for i in min.i..=max.i {
            let column = Column::new(i, k);
            if graph.spans(column).is_empty() {
                continue;
            }
            out = Some(match out {
                None => (column, column),
                Some((lo, hi)) => (
                    Column::new(lo.i.min(i), lo.k.min(k)),
                    Column::new(hi.i.max(i), hi.k.max(k)),
                ),
            });
        }
    }
    out
}

/// The ring beyond each open edge: the fine band, then the flat strips. The
/// −x and +x sides span the map's depth, −z and +z its width plus the
/// corners.
fn append_ring(
    mesh: &mut WaterMesh,
    id: StoreId,
    ocean: &Ocean,
    graph: &SpanGraph,
    open: [bool; 4],
) {
    let Some((min, max)) = occupied_bounds(graph) else {
        return;
    };
    let map = MapRect {
        x0: min.i as f32 * COLUMN_SIZE,
        z0: min.k as f32 * COLUMN_SIZE,
        x1: (max.i + 1) as f32 * COLUMN_SIZE,
        z1: (max.k + 1) as f32 * COLUMN_SIZE,
    };
    let floor = ocean.level - RING_DEPTH;
    let first_index = mesh.indices.len() as u32;
    append_fine_band(mesh, &map, floor, open);
    let MapRect { x0, z0, x1, z1 } = map;
    // (outward normal, spans along the edge, the first row)
    let beside_corners = vec![
        (x0 - HORIZON, x0 - FINE_BAND),
        (x1 + FINE_BAND, x1 + HORIZON),
    ];
    let sides = [
        (open[0], (-1.0, 0.0), vec![(z0, z1)], x0, &RING_ROWS[..]),
        (open[1], (1.0, 0.0), vec![(z0, z1)], x1, &RING_ROWS[..]),
        (
            open[2],
            (0.0, -1.0),
            vec![(x0 - HORIZON, x1 + HORIZON)],
            z0,
            &RING_ROWS[..],
        ),
        (
            open[3],
            (0.0, 1.0),
            vec![(x0 - HORIZON, x1 + HORIZON)],
            z1,
            &RING_ROWS[..],
        ),
        // Beside the corners, where the band stops, the −z and +z sides'
        // rows reach in to the map's edge.
        (
            open[2],
            (0.0, -1.0),
            beside_corners.clone(),
            z0,
            &[0.0, FINE_BAND][..],
        ),
        (
            open[3],
            (0.0, 1.0),
            beside_corners,
            z1,
            &[0.0, FINE_BAND][..],
        ),
    ];
    for (is_open, (nx, nz), spans, edge, rows) in sides {
        if !is_open {
            continue;
        }
        for (a, b) in spans {
            let along = ((b - a) / RING_STEP).round() as usize;
            for row in rows.windows(2) {
                // A coarse row takes coarse steps along the edge too.
                let stride = ((row[1] - row[0]) / RING_STEP).max(1.0) as usize;
                let mut i = 0;
                while i < along {
                    let (s0, s1) = (
                        a + i as f32 * RING_STEP,
                        a + (i + stride).min(along) as f32 * RING_STEP,
                    );
                    let corner = |s: f32, d: f32| {
                        if nx != 0.0 {
                            Vector2::new(edge + nx * d, s)
                        } else {
                            Vector2::new(s, edge + nz * d)
                        }
                    };
                    let base = mesh.vertices.len() as u32;
                    for p in [
                        corner(s0, row[0]),
                        corner(s1, row[0]),
                        corner(s1, row[1]),
                        corner(s0, row[1]),
                    ] {
                        mesh.vertices.push(BasinVertex {
                            xz: p,
                            floor,
                            swell_share: 0.0,
                        });
                    }
                    mesh.indices.extend_from_slice(&[
                        base,
                        base + 1,
                        base + 2,
                        base,
                        base + 2,
                        base + 3,
                    ]);
                    i += stride;
                }
            }
        }
    }
    if mesh.indices.len() as u32 > first_index {
        mesh.draws.push(WaterDraw {
            body: id,
            tile: RING_TILE,
            first_index,
            index_count: mesh.indices.len() as u32 - first_index,
        });
    }
}

/// The map's plan extent, m.
#[derive(Debug, Clone, Copy)]
struct MapRect {
    x0: f32,
    z0: f32,
    x1: f32,
    z1: f32,
}

impl MapRect {
    /// How far a point lies outside the map, along whichever axis is
    /// farther, m.
    fn distance(&self, p: Vector2<f32>) -> f32 {
        let dx = (self.x0 - p.x).max(p.x - self.x1).max(0.0);
        let dz = (self.z0 - p.y).max(p.y - self.z1).max(0.0);
        dx.max(dz)
    }

    /// Which open side's band a point just outside the map lies in: the −z
    /// and +z sides take the corners, as their strips do.
    fn in_band(&self, p: Vector2<f32>, open: [bool; 4]) -> bool {
        let d = self.distance(p);
        if d <= 0.0 || d >= FINE_BAND {
            return false;
        }
        if p.y < self.z0 {
            open[2]
        } else if p.y > self.z1 {
            open[3]
        } else if p.x < self.x0 {
            open[0]
        } else {
            open[1]
        }
    }
}

/// Column-wide quads along each open edge, out to [`FINE_BAND`]. The swell's
/// height fades from the whole of it at the map's edge to none at the
/// band's outer edge.
fn append_fine_band(mesh: &mut WaterMesh, map: &MapRect, floor: f32, open: [bool; 4]) {
    let cells = (FINE_BAND / COLUMN_SIZE) as i32;
    let lattice = |x: f32| (x / COLUMN_SIZE).round() as i32;
    let (i0, k0) = (lattice(map.x0) - cells, lattice(map.z0) - cells);
    let (i1, k1) = (lattice(map.x1) + cells, lattice(map.z1) + cells);
    let mut corners: FxHashMap<(i32, i32), u32> = FxHashMap::default();
    let mut corner = |i: i32, k: i32, mesh: &mut WaterMesh| {
        *corners.entry((i, k)).or_insert_with(|| {
            let xz = Vector2::new(i as f32 * COLUMN_SIZE, k as f32 * COLUMN_SIZE);
            let t = (map.distance(xz) / FINE_BAND).clamp(0.0, 1.0);
            mesh.vertices.push(BasinVertex {
                xz,
                floor,
                swell_share: 1.0 - t * t * (3.0 - 2.0 * t),
            });
            mesh.vertices.len() as u32 - 1
        })
    };
    for k in k0..k1 {
        for i in i0..i1 {
            let centre = Vector2::new(
                (i as f32 + 0.5) * COLUMN_SIZE,
                (k as f32 + 0.5) * COLUMN_SIZE,
            );
            if !map.in_band(centre, open) {
                continue;
            }
            let a = corner(i, k, mesh);
            let b = corner(i + 1, k, mesh);
            let c = corner(i + 1, k + 1, mesh);
            let d = corner(i, k + 1, mesh);
            mesh.indices.extend_from_slice(&[a, d, c, a, c, b]);
        }
    }
}
