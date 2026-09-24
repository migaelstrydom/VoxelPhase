//! Where the terrain surface crosses each column: the raw material of spans.
//!
//! Every terrain triangle is projected onto the XZ plane and rasterised
//! against the column centres it covers. A centre inside a triangle records a
//! [`Crossing`]: the height of the surface there, and whether it faces up (a
//! floor) or down (a ceiling). Upward-facing triangles are also clipped to
//! each column square they touch, recording the [`FloorPiece`] of floor that
//! lies inside it — what a span's `floor_min` and `floor_max` come from.
//!
//! **Fill rule.** A centre on an edge shared by two triangles belongs to
//! exactly one of them. Each projected triangle is oriented counter-clockwise
//! and a centre exactly on an edge counts only for the edge's owning side, by
//! a rule that flips with the edge's direction. The edge function is always
//! evaluated with the edge's endpoints in a canonical order, so the two
//! triangles on either side compute exactly negated values, whatever the
//! rounding.
//!
//! Crossings are cached per (terrain chunk, tile), a tile being one span
//! chunk's 8 m square: a terrain edit recomputes only the tiles its changed
//! region touches, while pairing into spans reads every chunk stacked in a
//! column.

use nalgebra::Point3;

use rustc_hash::FxHashMap;

use super::span::{Column, SpanChunkCoord, COLUMN_SIZE};

/// Which way a surface faces where it crosses a column centre.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Facing {
    /// Solid below, air above.
    Floor,
    /// Air below, solid above.
    Ceiling,
}

/// The surface crossing a column centre.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Crossing {
    pub y: f32,
    pub facing: Facing,
}

/// The height range of one upward-facing triangle, clipped to one column's
/// square.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FloorPiece {
    pub y_min: f32,
    pub y_max: f32,
}

/// What one terrain chunk contributes to one column.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ColumnCrossings<'a> {
    pub crossings: &'a [(Column, Crossing)],
    pub pieces: &'a [(Column, FloorPiece)],
}

/// One terrain chunk's crossings and floor pieces within one span chunk's
/// 8 m tile, each sorted by column.
#[derive(Debug, Clone, Default)]
pub struct TileCrossings {
    crossings: Vec<(Column, Crossing)>,
    pieces: Vec<(Column, FloorPiece)>,
}

/// Rasterise triangles, bucketed into the tiles they touch.
pub fn rasterise_by_tile(
    triangles: impl Iterator<Item = [Point3<f32>; 3]>,
) -> FxHashMap<SpanChunkCoord, TileCrossings> {
    let mut all = TileCrossings::default();
    for triangle in triangles {
        all.add_triangle(triangle, |_| true);
    }
    let mut tiles: FxHashMap<SpanChunkCoord, TileCrossings> = FxHashMap::default();
    for entry in all.crossings {
        tiles
            .entry(entry.0.chunk())
            .or_default()
            .crossings
            .push(entry);
    }
    for entry in all.pieces {
        tiles.entry(entry.0.chunk()).or_default().pieces.push(entry);
    }
    for tile in tiles.values_mut() {
        tile.finish();
    }
    tiles
}

impl TileCrossings {
    /// Rasterise triangles, keeping only what lands in one tile.
    pub fn for_tile(
        triangles: impl Iterator<Item = [Point3<f32>; 3]>,
        tile: SpanChunkCoord,
    ) -> Self {
        let mut out = Self::default();
        for triangle in triangles {
            out.add_triangle(triangle, |column| column.chunk() == tile);
        }
        out.finish();
        out
    }

    /// Rasterise one world-space triangle into the columns `keep` accepts.
    fn add_triangle(&mut self, vertices: [Point3<f32>; 3], keep: impl Fn(Column) -> bool) {
        let Some(projected) = ProjectedTriangle::new(vertices) else {
            return;
        };
        let (first, last) = projected.centre_range();
        for k in first.k..=last.k {
            for i in first.i..=last.i {
                let column = Column::new(i, k);
                if !keep(column) {
                    continue;
                }
                let (x, z) = column.centre();
                if let Some(y) = projected.height_at(x as f64, z as f64) {
                    self.crossings.push((
                        column,
                        Crossing {
                            y,
                            facing: projected.facing,
                        },
                    ));
                }
            }
        }

        if projected.facing == Facing::Floor {
            let (first, last) = projected.square_range();
            for k in first.k..=last.k {
                for i in first.i..=last.i {
                    let column = Column::new(i, k);
                    if !keep(column) {
                        continue;
                    }
                    if let Some(piece) = clip_to_square(&vertices, column) {
                        self.pieces.push((column, piece));
                    }
                }
            }
        }
    }

    /// Sort by column, so lookups and diffs are merges of sorted runs.
    fn finish(&mut self) {
        self.crossings.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then(a.1.y.total_cmp(&b.1.y))
                .then(a.1.facing.cmp(&b.1.facing))
        });
        self.pieces.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then(a.1.y_min.total_cmp(&b.1.y_min))
                .then(a.1.y_max.total_cmp(&b.1.y_max))
        });
    }

    pub fn is_empty(&self) -> bool {
        self.crossings.is_empty() && self.pieces.is_empty()
    }

    /// Columns this tile's entries touch, sorted and unique.
    pub fn touched(&self) -> Vec<Column> {
        let mut out: Vec<Column> = self
            .crossings
            .iter()
            .map(|c| c.0)
            .chain(self.pieces.iter().map(|p| p.0))
            .collect();
        out.sort_unstable();
        out.dedup();
        out
    }

    /// What this tile contributes to one column.
    pub fn column(&self, column: Column) -> ColumnCrossings<'_> {
        ColumnCrossings {
            crossings: run(&self.crossings, column),
            pieces: run(&self.pieces, column),
        }
    }

    /// Columns whose contribution differs between two rasterisations of a
    /// tile, sorted and unique.
    pub fn changed_columns(&self, other: &Self) -> Vec<Column> {
        let mut out = differing_runs(&self.crossings, &other.crossings);
        out.extend(differing_runs(&self.pieces, &other.pieces));
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// Columns whose runs differ between two column-sorted lists, by a single
/// merge walk.
fn differing_runs<T: PartialEq>(a: &[(Column, T)], b: &[(Column, T)]) -> Vec<Column> {
    let mut out = Vec::new();
    let (mut i, mut j) = (0, 0);
    while i < a.len() || j < b.len() {
        let column = match (a.get(i), b.get(j)) {
            (Some(x), Some(y)) => x.0.min(y.0),
            (Some(x), None) => x.0,
            (None, Some(y)) => y.0,
            (None, None) => unreachable!(),
        };
        let a_end = i + a[i..].partition_point(|e| e.0 <= column);
        let b_end = j + b[j..].partition_point(|e| e.0 <= column);
        if a[i..a_end] != b[j..b_end] {
            out.push(column);
        }
        i = a_end;
        j = b_end;
    }
    out
}

/// The run of a column-sorted list belonging to one column.
fn run<T>(list: &[(Column, T)], column: Column) -> &[(Column, T)] {
    let start = list.partition_point(|e| e.0 < column);
    let end = start + list[start..].partition_point(|e| e.0 <= column);
    &list[start..end]
}

/// A triangle projected onto XZ and oriented counter-clockwise there.
struct ProjectedTriangle {
    /// Vertices in counter-clockwise order, as f64.
    v: [[f64; 3]; 3],
    /// Twice the projected area, positive.
    area2: f64,
    facing: Facing,
}

impl ProjectedTriangle {
    /// `None` for a triangle with no projected area: a vertical wall crosses
    /// no column centre.
    fn new(vertices: [Point3<f32>; 3]) -> Option<Self> {
        let mut v = vertices.map(|p| [p.x as f64, p.y as f64, p.z as f64]);
        let area2 = orient(&v[0], &v[1], &v[2]);
        if area2 == 0.0 {
            return None;
        }
        // Outward normals: n.y = e1.z·e2.x − e1.x·e2.z = −area2, so a
        // clockwise projection faces up.
        let facing = if area2 < 0.0 {
            Facing::Floor
        } else {
            Facing::Ceiling
        };
        if area2 < 0.0 {
            v.swap(1, 2);
        }
        Some(Self {
            v,
            area2: area2.abs(),
            facing,
        })
    }

    /// Columns whose centres lie in the triangle's XZ bounding box.
    fn centre_range(&self) -> (Column, Column) {
        let (min_x, max_x, min_z, max_z) = self.bounds();
        let lo = |v: f64| ((v / COLUMN_SIZE as f64) - 0.5).ceil() as i32;
        let hi = |v: f64| ((v / COLUMN_SIZE as f64) - 0.5).floor() as i32;
        (
            Column::new(lo(min_x), lo(min_z)),
            Column::new(hi(max_x), hi(max_z)),
        )
    }

    /// Columns whose squares overlap the triangle's XZ bounding box.
    fn square_range(&self) -> (Column, Column) {
        let (min_x, max_x, min_z, max_z) = self.bounds();
        let cell = |v: f64| (v / COLUMN_SIZE as f64).floor() as i32;
        (
            Column::new(cell(min_x), cell(min_z)),
            Column::new(cell(max_x), cell(max_z)),
        )
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        let xs = self.v.map(|p| p[0]);
        let zs = self.v.map(|p| p[2]);
        (
            xs.iter().copied().fold(f64::INFINITY, f64::min),
            xs.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            zs.iter().copied().fold(f64::INFINITY, f64::min),
            zs.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        )
    }

    /// Surface height at (x, z) if the point belongs to this triangle under
    /// the fill rule.
    fn height_at(&self, x: f64, z: f64) -> Option<f32> {
        let p = [x, 0.0, z];
        let mut weights = [0.0; 3];
        for edge in 0..3 {
            let a = &self.v[(edge + 1) % 3];
            let b = &self.v[(edge + 2) % 3];
            let e = edge_function(a, b, &p);
            if e < 0.0 || (e == 0.0 && !owns_boundary(a, b)) {
                return None;
            }
            weights[edge] = e;
        }
        let y = (weights[0] * self.v[0][1] + weights[1] * self.v[1][1] + weights[2] * self.v[2][1])
            / self.area2;
        let (lo, hi) = self
            .v
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                (lo.min(p[1]), hi.max(p[1]))
            });
        Some(y.clamp(lo, hi) as f32)
    }
}

/// Twice the signed XZ area of (a, b, c); positive when counter-clockwise.
fn orient(a: &[f64; 3], b: &[f64; 3], c: &[f64; 3]) -> f64 {
    (b[0] - a[0]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[0] - a[0])
}

/// The edge function of directed edge a→b at p, evaluated with the endpoints
/// in canonical order so that the reverse edge yields exactly the negation.
fn edge_function(a: &[f64; 3], b: &[f64; 3], p: &[f64; 3]) -> f64 {
    if (a[0], a[2]) <= (b[0], b[2]) {
        orient(a, b, p)
    } else {
        -orient(b, a, p)
    }
}

/// Whether a point exactly on directed edge a→b belongs to the triangle on
/// its left. Exactly one of an edge and its reverse owns its points.
fn owns_boundary(a: &[f64; 3], b: &[f64; 3]) -> bool {
    let dx = b[0] - a[0];
    let dz = b[2] - a[2];
    dz < 0.0 || (dz == 0.0 && dx > 0.0)
}

/// The most vertices a triangle can have after clipping to a rectangle.
const MAX_CLIPPED: usize = 7;

/// A small polygon on the stack.
#[derive(Clone, Copy)]
struct Polygon {
    points: [[f32; 3]; MAX_CLIPPED],
    len: usize,
}

/// The height range of a triangle clipped to a column's square, or `None` if
/// the two do not overlap.
fn clip_to_square(vertices: &[Point3<f32>; 3], column: Column) -> Option<FloorPiece> {
    let (x0, z0) = column.min_corner();
    let (x1, z1) = (x0 + COLUMN_SIZE, z0 + COLUMN_SIZE);
    let inside_square = |p: &Point3<f32>| p.x >= x0 && p.x <= x1 && p.z >= z0 && p.z <= z1;
    if vertices.iter().all(inside_square) {
        let (lo, hi) = vertices
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
                (lo.min(p.y), hi.max(p.y))
            });
        return Some(FloorPiece {
            y_min: lo,
            y_max: hi,
        });
    }
    let mut polygon = Polygon {
        points: [[0.0; 3]; MAX_CLIPPED],
        len: 3,
    };
    for (slot, p) in polygon.points.iter_mut().zip(vertices) {
        *slot = [p.x, p.y, p.z];
    }
    for (axis, bound, keep_above) in [(0, x0, true), (0, x1, false), (2, z0, true), (2, z1, false)]
    {
        polygon = clip_polygon(&polygon, axis, bound, keep_above);
        if polygon.len == 0 {
            return None;
        }
    }
    let (y_min, y_max) = polygon.points[..polygon.len]
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(p[1]), hi.max(p[1]))
        });
    Some(FloorPiece { y_min, y_max })
}

/// One Sutherland–Hodgman pass against an axis-aligned plane.
fn clip_polygon(polygon: &Polygon, axis: usize, bound: f32, keep_above: bool) -> Polygon {
    let inside = |p: &[f32; 3]| {
        if keep_above {
            p[axis] >= bound
        } else {
            p[axis] <= bound
        }
    };
    let mut out = Polygon {
        points: [[0.0; 3]; MAX_CLIPPED],
        len: 0,
    };
    let points = &polygon.points[..polygon.len];
    for (index, current) in points.iter().enumerate() {
        let previous = &points[(index + points.len() - 1) % points.len()];
        let (cur_in, prev_in) = (inside(current), inside(previous));
        if cur_in != prev_in && out.len < MAX_CLIPPED {
            let t = (bound - previous[axis]) / (current[axis] - previous[axis]);
            let mut crossing = [0.0; 3];
            for (c, (p, q)) in crossing.iter_mut().zip(previous.iter().zip(current)) {
                *c = p + (q - p) * t;
            }
            crossing[axis] = bound;
            out.points[out.len] = crossing;
            out.len += 1;
        }
        if cur_in && out.len < MAX_CLIPPED {
            out.points[out.len] = *current;
            out.len += 1;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f32, y: f32, z: f32) -> Point3<f32> {
        Point3::new(x, y, z)
    }

    /// A flat floor quad at `y` over [x0, x1] × [z0, z1], wound to face up,
    /// split along the diagonal that passes through column centres.
    fn floor_quad(x0: f32, z0: f32, x1: f32, z1: f32, y: f32) -> Vec<[Point3<f32>; 3]> {
        vec![
            [p(x0, y, z0), p(x1, y, z1), p(x1, y, z0)],
            [p(x0, y, z0), p(x0, y, z1), p(x1, y, z1)],
        ]
    }

    const TILE: SpanChunkCoord = SpanChunkCoord { x: 0, z: 0 };

    #[test]
    fn a_shared_diagonal_through_centres_counts_each_centre_once() {
        // A 4 m square whose diagonal runs through every diagonal centre.
        let chunk =
            TileCrossings::for_tile(floor_quad(0.25, 0.25, 4.25, 4.25, 1.0).into_iter(), TILE);
        for k in 0..8 {
            for i in 0..8 {
                let c = chunk.column(Column::new(i, k)).crossings.len();
                assert_eq!(c, 1, "column ({i}, {k}) counted {c} times");
            }
        }
    }

    #[test]
    fn floors_face_up_and_ceilings_down() {
        let mut triangles = floor_quad(0.0, 0.0, 1.0, 1.0, 2.0);
        // The same quad wound the other way is a ceiling.
        triangles.push([p(0.0, 5.0, 0.0), p(1.0, 5.0, 0.0), p(1.0, 5.0, 1.0)]);
        triangles.push([p(0.0, 5.0, 0.0), p(1.0, 5.0, 1.0), p(0.0, 5.0, 1.0)]);
        let chunk = TileCrossings::for_tile(triangles.into_iter(), TILE);
        let column = chunk.column(Column::new(0, 0));
        let mut facings: Vec<_> = column
            .crossings
            .iter()
            .map(|c| (c.1.facing, c.1.y))
            .collect();
        facings.sort_by(|a, b| a.1.total_cmp(&b.1));
        assert_eq!(facings, vec![(Facing::Floor, 2.0), (Facing::Ceiling, 5.0)]);
    }

    #[test]
    fn a_floor_piece_spans_the_slope_inside_the_square() {
        // A slope rising 1 m per metre along x, covering column (0, 0).
        let triangles = vec![
            [p(-1.0, -1.0, -1.0), p(2.0, 2.0, 2.0), p(2.0, 2.0, -1.0)],
            [p(-1.0, -1.0, -1.0), p(-1.0, -1.0, 2.0), p(2.0, 2.0, 2.0)],
        ];
        let chunk = TileCrossings::for_tile(triangles.into_iter(), TILE);
        let column = chunk.column(Column::new(0, 0));
        let lo = column
            .pieces
            .iter()
            .map(|p| p.1.y_min)
            .fold(f32::INFINITY, f32::min);
        let hi = column
            .pieces
            .iter()
            .map(|p| p.1.y_max)
            .fold(f32::NEG_INFINITY, f32::max);
        assert!(
            (lo - 0.0).abs() < 1e-5 && (hi - 0.5).abs() < 1e-5,
            "{lo}..{hi}"
        );
        let crossing = column.crossings[0].1;
        assert!((crossing.y - 0.25).abs() < 1e-5);
    }
}
