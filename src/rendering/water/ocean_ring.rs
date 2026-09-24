//! The sea past the map's open edges, out to the horizon (§12).
//!
//! ```text
//!   the map's edge ──▶ nested rectangles around the map: 0.5 m apart, with
//!                      a vertex every 0.5 m, out to 8 m; then twice as far
//!                      out each time, their vertices spaced to match
//!   each pair of neighbouring rectangles ──▶ stitched side by side with
//!                      triangles that share every vertex
//! ```
//!
//! No vertex lies partway along another triangle's edge, so no join can
//! crack, not even by a pixel. The innermost rectangle is the map's edge,
//! with a vertex at every column corner, so the sea's own quads meet it
//! vertex for vertex. Across the first 8 m the swell's height fades out;
//! beyond, the sea is flat, and its normal is the fragment shader's.

use nalgebra::Vector2;
use rustc_hash::FxHashMap;

use crate::water::geometry::{SpanChunkCoord, COLUMN_SIZE};
use crate::water::ids::StoreId;

use super::basin_mesher::{WaterDraw, WaterMesh};
use super::vertex::BasinVertex;

/// Width of the band whose swell height fades out, in columns (8 m).
const FADE_BAND: i32 = 16;

/// Distance of the farthest rectangle from the map's edge, in columns (1 km).
const HORIZON: i32 = 2048;

/// Depth given to the ring's floor: open water, deep enough that the swell
/// runs at full height and the tint is the sea's.
const RING_DEPTH: f32 = 100.0;

/// The ring's draws are keyed to a tile no ripple tile can have.
pub const RING_TILE: SpanChunkCoord = SpanChunkCoord {
    x: i32::MIN,
    z: i32::MIN,
};

/// The map's plan extent, in column corners: `i0..=i1` across x, `k0..=k1`
/// across z.
#[derive(Debug, Clone, Copy)]
pub struct MapRect {
    pub i0: i32,
    pub k0: i32,
    pub i1: i32,
    pub k1: i32,
}

/// One rectangle: its distance from the map's edge and the spacing of its
/// vertices, both in columns.
#[derive(Debug, Clone, Copy)]
struct Ring {
    distance: i32,
    spacing: i32,
}

/// Every rectangle, inside out.
fn rings() -> Vec<Ring> {
    let mut out: Vec<Ring> = (0..=FADE_BAND)
        .map(|distance| Ring {
            distance,
            spacing: 1,
        })
        .collect();
    let mut distance = FADE_BAND * 2;
    while distance <= HORIZON {
        out.push(Ring {
            distance,
            spacing: distance / 2,
        });
        distance *= 2;
    }
    out
}

/// Append the ring beyond each open edge (−x, +x, −z, +z) to `mesh`, for
/// a sea at `level`.
pub fn append_ring(mesh: &mut WaterMesh, id: StoreId, level: f32, map: MapRect, open: [bool; 4]) {
    if !open.iter().any(|&o| o) {
        return;
    }
    let first_index = mesh.indices.len() as u32;
    let mut vertices = Vertices {
        mesh,
        index: FxHashMap::default(),
        floor: level - RING_DEPTH,
    };
    let rings = rings();
    let sides: Vec<[Vec<u32>; 4]> = rings
        .iter()
        .map(|ring| [0, 1, 2, 3].map(|side| vertices.side(map, *ring, side)))
        .collect();
    let mesh = vertices.mesh;
    for pair in sides.windows(2) {
        for side in 0..4 {
            if open[side] {
                stitch(mesh, &pair[0][side], &pair[1][side]);
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

/// The ring's vertices, each made once and shared by every triangle at it.
struct Vertices<'a> {
    mesh: &'a mut WaterMesh,
    /// By position, in eighths of a column.
    index: FxHashMap<(i64, i64), u32>,
    floor: f32,
}

impl Vertices<'_> {
    /// One side of a rectangle, corner to corner, as vertex indices: −x and
    /// +x run along z, −z and +z along x.
    fn side(&mut self, map: MapRect, ring: Ring, side: usize) -> Vec<u32> {
        let d = ring.distance;
        let (lo_i, lo_k, hi_i, hi_k) = (map.i0 - d, map.k0 - d, map.i1 + d, map.k1 + d);
        let (from, to) = match side {
            0 => ((lo_i, lo_k), (lo_i, hi_k)),
            1 => ((hi_i, lo_k), (hi_i, hi_k)),
            2 => ((lo_i, lo_k), (hi_i, lo_k)),
            _ => ((lo_i, hi_k), (hi_i, hi_k)),
        };
        let length = (to.0 - from.0) + (to.1 - from.1);
        let n = ((length as f32 / ring.spacing as f32).round() as i32).max(1);
        let t = d as f32 / FADE_BAND as f32;
        let share = if t >= 1.0 {
            0.0
        } else {
            1.0 - t * t * (3.0 - 2.0 * t)
        };
        (0..=n)
            .map(|j| {
                // Exact at a column's spacing: the sea's own corners.
                let along = |a: i32, b: i32| a as f64 + (b - a) as f64 * j as f64 / n as f64;
                self.vertex(along(from.0, to.0), along(from.1, to.1), share)
            })
            .collect()
    }

    fn vertex(&mut self, i: f64, k: f64, swell_share: f32) -> u32 {
        let key = ((i * 8.0).round() as i64, (k * 8.0).round() as i64);
        let mesh = &mut *self.mesh;
        let floor = self.floor;
        *self.index.entry(key).or_insert_with(|| {
            mesh.vertices.push(BasinVertex {
                xz: Vector2::new(
                    (i * COLUMN_SIZE as f64) as f32,
                    (k * COLUMN_SIZE as f64) as f32,
                ),
                floor,
                swell_share,
            });
            mesh.vertices.len() as u32 - 1
        })
    }
}

/// Triangles between the same side of two neighbouring rectangles, walking
/// both corner to corner and stepping whichever's next vertex comes first.
fn stitch(mesh: &mut WaterMesh, inner: &[u32], outer: &[u32]) {
    let (m, n) = (inner.len() - 1, outer.len() - 1);
    let (mut i, mut j) = (0, 0);
    while i < m || j < n {
        let inner_next = (i + 1) as f64 / m as f64;
        let outer_next = (j + 1) as f64 / n as f64;
        if j == n || (i < m && inner_next <= outer_next) {
            triangle(mesh, inner[i], inner[i + 1], outer[j]);
            i += 1;
        } else {
            triangle(mesh, inner[i], outer[j + 1], outer[j]);
            j += 1;
        }
    }
}

/// A triangle wound as the sea's quads are: clockwise in (x, z), which is
/// counter-clockwise seen from above.
fn triangle(mesh: &mut WaterMesh, a: u32, b: u32, c: u32) {
    let p = |v: u32| mesh.vertices[v as usize].xz;
    let (pa, pb, pc) = (p(a), p(b), p(c));
    let cross = (pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x);
    if cross < 0.0 {
        mesh.indices.extend_from_slice(&[a, b, c]);
    } else {
        mesh.indices.extend_from_slice(&[a, c, b]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ring(open: [bool; 4]) -> WaterMesh {
        let mut mesh = WaterMesh::default();
        let map = MapRect {
            i0: -8,
            k0: -6,
            i1: 8,
            k1: 6,
        };
        append_ring(&mut mesh, StoreId(0), 0.0, map, open);
        mesh
    }

    #[test]
    fn no_vertex_lies_along_another_triangle_s_edge() {
        // Out past the fade band's join to the first coarse rectangle.
        const WINDOW: f32 = 24.0;
        let mesh = ring([true; 4]);
        let inside = |p: Vector2<f32>| p.x.abs() < WINDOW && p.y.abs() < WINDOW;
        let mut near: Vec<Vector2<f32>> = mesh
            .vertices
            .iter()
            .map(|v| v.xz)
            .filter(|&p| inside(p))
            .collect();
        near.sort_by(|a, b| a.x.total_cmp(&b.x));
        for t in mesh.indices.chunks(3) {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let (a, b) = (mesh.vertices[a as usize].xz, mesh.vertices[b as usize].xz);
                if !inside(a) || !inside(b) {
                    continue;
                }
                let lo = near.partition_point(|p| p.x < a.x.min(b.x) - 1e-3);
                let hi = near.partition_point(|p| p.x <= a.x.max(b.x) + 1e-3);
                for &p in &near[lo..hi] {
                    let (ab, ap) = (b - a, p - a);
                    let cross = ab.x * ap.y - ab.y * ap.x;
                    let along = ab.dot(&ap) / ab.norm_squared();
                    assert!(
                        cross.abs() > 1e-4 || along <= 1e-4 || along >= 1.0 - 1e-4,
                        "{p:?} lies along {a:?}-{b:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn every_triangle_faces_up_the_same_way() {
        let mesh = ring([true; 4]);
        for t in mesh.indices.chunks(3) {
            let p = |v: u32| mesh.vertices[v as usize].xz;
            let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
            let cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            assert!(cross < 0.0);
        }
    }

    #[test]
    fn the_inner_edge_has_a_vertex_at_every_column_corner() {
        let mesh = ring([true; 4]);
        for i in -8..=8 {
            let x = i as f32 * COLUMN_SIZE;
            for z in [-3.0, 3.0] {
                assert!(
                    mesh.vertices.iter().any(|v| v.xz == Vector2::new(x, z)),
                    "no vertex at ({x}, {z})"
                );
            }
        }
    }

    #[test]
    fn a_closed_edge_has_no_ring() {
        let mesh = ring([false, false, true, false]);
        let used: Vec<Vector2<f32>> = mesh
            .indices
            .iter()
            .map(|&v| mesh.vertices[v as usize].xz)
            .collect();
        assert!(!used.is_empty());
        assert!(used.iter().all(|p| p.y <= -3.0 + 1e-3), "only the −z side");
    }
}
