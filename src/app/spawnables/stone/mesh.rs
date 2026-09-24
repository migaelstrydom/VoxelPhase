//! Drawing a stone block as weathered stone rather than as its collider.
//!
//! ```text
//!   hull ──▶ tessellate each face ──▶ project every point onto ──▶ normal, UV,
//!            (one grid density for      StoneSurface               patina, AO
//!             the whole piece)
//! ```
//!
//! The collider stays the few-faced hull it always was; only the drawing is
//! dense. Faces are tessellated separately — so each keeps its own texture
//! layout — but every point on an edge two faces share is computed from that
//! edge alone, and everything after that is a function of position alone, so
//! the two faces meet without a crack however far the weathering moved them.

use std::collections::HashMap;

use nalgebra::{Vector2, Vector3, Vector4};
use rayon::prelude::*;

use super::patina::patina;
use super::surface::StoneSurface;
use crate::app::spawnables::shared::models::{PieceHull, PiecePlacement, SurfaceUvs};
use crate::collision::convex_hull::{cube_hull, ConvexHull};
use crate::rendering::vertex::Vertex;

/// Spacing of the grid a face is tessellated into, as a fraction of the
/// block's thickness. Fine enough that a rounded arris has several rows of
/// vertices across it; any finer is spent on detail the texture's relief
/// already draws.
const SPACING: f32 = 0.05;

/// Most vertices one piece may be drawn with.
///
/// Every model is re-uploaded each frame, so the drawing's density is paid for
/// every frame by every stone, not once. At this ceiling a thirteen-stone arch
/// is a few megabytes a frame, beside terrain's thirty.
const MAX_VERTICES: usize = 4500;

/// Coarsest grid a face is ever cut into: enough that a small wedge still
/// has its arrises rounded.
const MIN_DIVISIONS: usize = 3;

/// How far a vertex on the crease between a break and old surface is moved
/// into its own face before its normal is read, as a fraction of thickness.
/// Without it the crease vertex averages the two, and the one edge that
/// ought to be sharp is shaded soft.
const CREASE_NUDGE: f32 = 0.008;

/// A weathered stone piece drawn from its hull. Fits
/// [`HullMesh`](crate::app::spawnables::shared::models::HullMesh), so a
/// cleaved block's wedges are drawn by the same code as the block.
pub fn weathered_hull_mesh(piece: &PieceHull, uvs: SurfaceUvs) -> (Vec<Vertex>, Vec<u32>) {
    weathered_mesh(piece.hull, piece.offset, piece.whole, uvs)
}

/// A weathered stone box. Fits
/// [`PieceMesh`](crate::app::spawnables::shared::models::PieceMesh), for a
/// block whose collider is a box.
pub fn weathered_box_mesh(piece: &PiecePlacement, uvs: SurfaceUvs) -> (Vec<Vertex>, Vec<u32>) {
    weathered_mesh(&cube_hull(piece.half_extents), piece.offset, None, uvs)
}

/// One tessellated face: its grid points and the triangles over them.
struct FaceGrid {
    normal: Vector3<f32>,
    centre: Vector3<f32>,
    /// Points on the hull face, in the stone's frame.
    points: Vec<Vector3<f32>>,
    triangles: Vec<[u32; 3]>,
}

/// One grid point once it has been weathered.
struct ShadedPoint {
    /// On the weathered surface, in the stone's frame.
    position: Vector3<f32>,
    normal: Vector3<f32>,
    /// The patina, as a multiplier on the texture.
    colour: Vector4<f32>,
    /// How much of the sky the point sees.
    occlusion: f32,
}

/// `hull` in its own frame, sitting at `offset` in the stone's frame; `whole`
/// is the block it was cut from, in the stone's frame.
fn weathered_mesh(
    hull: &ConvexHull,
    offset: Vector3<f32>,
    whole: Option<&ConvexHull>,
    uvs: SurfaceUvs,
) -> (Vec<Vertex>, Vec<u32>) {
    let framed = hull.translated(offset);
    let surface = StoneSurface::new(&framed, whole);
    let divisions = divisions_for(&framed, surface.thickness());
    let faces: Vec<FaceGrid> = framed
        .faces
        .iter()
        .filter(|face| face.vertex_indices.len() >= 3)
        .map(|face| {
            let corners: Vec<Vector3<f32>> = face
                .vertex_indices
                .iter()
                .map(|&i| framed.vertices[i as usize])
                .collect();
            tessellate(&corners, face.normal, divisions)
        })
        .collect();

    // The expensive part — each point's projection, normal and patina — is
    // independent of every other, so it is done side by side.
    let nudge = CREASE_NUDGE * surface.thickness();
    let shaded: Vec<Vec<ShadedPoint>> = faces
        .par_iter()
        .map(|face| {
            face.points
                .iter()
                .map(|on_hull| {
                    let position = surface.project(on_hull);
                    let wear = surface.wear(&position);
                    let toward_centre = face.centre - on_hull;
                    let read_at = if wear.fresh > 0.02
                        && wear.fresh < 0.98
                        && toward_centre.magnitude() > nudge
                    {
                        surface.project(&(on_hull + toward_centre.normalize() * nudge))
                    } else {
                        position
                    };
                    let normal = surface.normal(&read_at);
                    let (colour, occlusion) =
                        patina(&position, &normal, &wear, surface.thickness());
                    ShadedPoint {
                        position,
                        normal,
                        colour,
                        occlusion,
                    }
                })
                .collect()
        })
        .collect();

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for (face, points) in faces.iter().zip(shaded) {
        let base = vertices.len() as u32;
        let positions: Vec<Vector3<f32>> = points.iter().map(|p| p.position).collect();
        let face_uvs: Vec<Vector2<f32>> = uvs.face_uvs(&positions, face.normal);
        for (point, uv) in points.into_iter().zip(face_uvs) {
            vertices.push(Vertex {
                pos: point.position - offset,
                color: point.colour,
                tex_coords: uv,
                normal: point.normal,
                ao: point.occlusion,
            });
        }
        for triangle in &face.triangles {
            indices.extend(triangle.iter().map(|i| base + i));
        }
    }
    (vertices, indices)
}

/// How many segments every hull edge is cut into.
///
/// One number for the whole piece, because an edge is shared by two faces and
/// both must cut it the same way. Set by the longest edge at [`SPACING`], then
/// brought down if that would spend more than [`MAX_VERTICES`].
fn divisions_for(hull: &ConvexHull, thickness: f32) -> usize {
    let longest = hull
        .faces
        .iter()
        .flat_map(|face| {
            let n = face.vertex_indices.len();
            (0..n).map(move |i| (face.vertex_indices[i], face.vertex_indices[(i + 1) % n]))
        })
        .map(|(a, b)| (hull.vertices[a as usize] - hull.vertices[b as usize]).magnitude())
        .fold(0.0f32, f32::max);
    let wanted = (longest / (SPACING * thickness)).ceil() as usize;

    let fan_triangles: usize = hull
        .faces
        .iter()
        .map(|face| face.vertex_indices.len().saturating_sub(2))
        .sum();
    let points_per_triangle = |n: usize| (n + 1) * (n + 2) / 2;
    let mut divisions = wanted.max(MIN_DIVISIONS);
    while divisions > MIN_DIVISIONS && fan_triangles * points_per_triangle(divisions) > MAX_VERTICES
    {
        divisions -= 1;
    }
    divisions
}

/// A face cut into a fan of triangles from its first corner, each triangle cut
/// into a grid `divisions` to a side.
fn tessellate(corners: &[Vector3<f32>], normal: Vector3<f32>, divisions: usize) -> FaceGrid {
    let n = divisions;
    let centre = corners.iter().sum::<Vector3<f32>>() / corners.len() as f32;
    let mut points: Vec<Vector3<f32>> = Vec::new();
    let mut index_of: HashMap<[u32; 3], u32> = HashMap::new();
    let mut triangles = Vec::new();

    let mut point = |p: Vector3<f32>| -> u32 {
        let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        *index_of.entry(key).or_insert_with(|| {
            points.push(p);
            points.len() as u32 - 1
        })
    };

    for k in 1..corners.len() - 1 {
        let (a, b, c) = (corners[0], corners[k], corners[k + 1]);
        // `grid[i][j]` is the point `i` steps towards `b` and `j` towards `c`.
        let grid: Vec<Vec<u32>> = (0..=n)
            .map(|i| {
                (0..=n - i)
                    .map(|j| {
                        point(if j == 0 {
                            edge_point(a, b, i, n)
                        } else if i == 0 {
                            edge_point(a, c, j, n)
                        } else if i + j == n {
                            edge_point(b, c, j, n)
                        } else {
                            a + (b - a) * (i as f32 / n as f32) + (c - a) * (j as f32 / n as f32)
                        })
                    })
                    .collect()
            })
            .collect();
        for (i, row) in grid.iter().enumerate().take(n) {
            let next = &grid[i + 1];
            for j in 0..n - i {
                triangles.push([row[j], next[j], row[j + 1]]);
                if i + j + 1 < n {
                    triangles.push([next[j], next[j + 1], row[j + 1]]);
                }
            }
        }
    }

    FaceGrid {
        normal,
        centre,
        points,
        triangles,
    }
}

/// The `step`th of `n` points along the edge from `from` to `to`, computed the
/// same way whichever end it is walked from.
///
/// The two faces sharing an edge walk it in opposite directions; interpolating
/// from whichever end each started at would put their points a rounding error
/// apart, and a rounding error is a crack a pixel wide.
fn edge_point(from: Vector3<f32>, to: Vector3<f32>, step: usize, n: usize) -> Vector3<f32> {
    // The ends exactly, not by interpolation: `b + (a - b)` need not be `a`.
    if step == 0 {
        return from;
    }
    if step == n {
        return to;
    }
    let forward = (from.x, from.y, from.z) <= (to.x, to.y, to.z);
    let (start, end, step) = if forward {
        (from, to, step)
    } else {
        (to, from, n - step)
    };
    start + (end - start) * (step as f32 / n as f32)
}

/// Triangles of a drawing that face the opposite way to the surface they sit
/// on: each one is a fold, and a fold shows as a hole.
#[cfg(test)]
pub fn inside_out(vertices: &[Vertex], indices: &[u32]) -> usize {
    indices
        .chunks(3)
        .filter(|t| {
            let [a, b, c] = [0, 1, 2].map(|k| &vertices[t[k] as usize]);
            let facet = (b.pos - a.pos).cross(&(c.pos - a.pos));
            facet.magnitude() > 1e-9 && facet.dot(&(a.normal + b.normal + c.normal)) < 0.0
        })
        .count()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::hull_split::{split_hull, Plane};

    fn slab() -> ConvexHull {
        cube_hull(Vector3::new(0.6, 0.25, 0.3))
    }

    /// Every edge of the drawing must be shared by exactly two triangles, or
    /// the stone has a crack through it that shows the sky.
    fn assert_closed(vertices: &[Vertex], indices: &[u32]) {
        let key = |v: &Vertex| [v.pos.x.to_bits(), v.pos.y.to_bits(), v.pos.z.to_bits()];
        let mut edges: HashMap<([u32; 3], [u32; 3]), i32> = HashMap::new();
        for tri in indices.chunks(3) {
            for e in 0..3 {
                let a = key(&vertices[tri[e] as usize]);
                let b = key(&vertices[tri[(e + 1) % 3] as usize]);
                if a == b {
                    continue;
                }
                let (lo, hi, sign) = if a < b { (a, b, 1) } else { (b, a, -1) };
                *edges.entry((lo, hi)).or_insert(0) += sign;
            }
        }
        let open = edges.values().filter(|&&count| count != 0).count();
        assert_eq!(open, 0, "{open} edges are open or wound inconsistently");
    }

    #[test]
    fn a_weathered_block_is_watertight() {
        let hull = slab();
        let (vertices, indices) =
            weathered_mesh(&hull, Vector3::zeros(), None, SurfaceUvs::PerMetre(1.0));
        assert_closed(&vertices, &indices);
    }

    #[test]
    fn a_weathered_wedge_is_watertight() {
        let whole = slab();
        let split = split_hull(
            &whole,
            Plane {
                normal: Vector3::new(1.0, 0.25, -0.1).normalize(),
                offset: 0.1,
            },
        )
        .expect("splits");
        let centre = split.front.centroid();
        let piece = split.front.translated(-centre);
        let (vertices, indices) =
            weathered_mesh(&piece, centre, Some(&whole), SurfaceUvs::PerMetre(1.0));
        assert_closed(&vertices, &indices);
    }

    #[test]
    fn a_stone_keeps_to_its_vertex_budget() {
        for half in [
            Vector3::new(0.25, 0.75, 0.6),
            Vector3::new(2.5, 1.6, 1.5),
            Vector3::new(0.3, 0.3, 0.3),
        ] {
            let (vertices, _) = weathered_mesh(
                &cube_hull(half),
                Vector3::zeros(),
                None,
                SurfaceUvs::PerMetre(1.0),
            );
            assert!(
                vertices.len() <= MAX_VERTICES,
                "{half:?} drew {} vertices",
                vertices.len()
            );
        }
    }

    /// Normals come from the field, and must face out of the stone. The wall
    /// of a chip may lean a little past square to the way the hull faces —
    /// that is what a steep bite looks like — but only on the odd vertex, and
    /// never far: a normal pointing into the stone is a fold in the drawing.
    #[test]
    fn every_normal_faces_out() {
        let hull = slab();
        // Different places in the world wear differently; try several.
        for step in 0..8 {
            let at = Vector3::new(step as f32 * 3.7, step as f32 * 1.3, step as f32 * -2.9);
            let surface = StoneSurface::new(&hull.translated(at), None);
            let (vertices, _) = weathered_mesh(&hull, at, None, SurfaceUvs::PerMetre(1.0));
            let dots: Vec<f32> = vertices
                .iter()
                .map(|v| v.normal.dot(&-surface.inward(&(v.pos + at))))
                .collect();
            let worst = dots.iter().copied().fold(f32::MAX, f32::min);
            let under = dots.iter().filter(|d| **d < 0.0).count();
            assert!(worst > -0.3, "a normal faces into the stone: {worst}");
            assert!(
                under * 1000 <= dots.len(),
                "{under} of {} normals lean past square",
                dots.len()
            );
        }
    }
}
