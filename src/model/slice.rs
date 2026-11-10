//! Slicing a mesh with a plane, and closing the hole the slice leaves.
//!
//! ```text
//!   mesh ──slice(plane)──▶ kept: what lies behind the plane, its triangles
//!                                clipped where the plane crosses them
//!                          rim:  the edges the clipping left on the plane
//!   rim ──cap(normal)────▶ the rim's edges chained into loops, and a face
//!                          over each
//! ```
//!
//! A mesh that encloses a solid, sliced and capped, encloses what is left of
//! the solid: that is how a piece broken off a carved thing keeps its
//! carving. The mesh need not be convex, and may be several closed surfaces
//! that run through one another; each gets its own cap, whatever the outline
//! of its cross-section. A hollow is capped over like the rest.

use nalgebra::{Vector2, Vector3};
use rustc_hash::FxHashMap;

use super::parts::MeshPrimitive;
use crate::rendering::vertex::Vertex;

/// Distance from the plane within which a vertex counts as on it, m.
const ON_PLANE: f32 = 1.0e-6;

/// How far apart the end of one rim edge and the start of the next may be
/// and still be joined, as a share of the rim's extent. Edges of one surface
/// meet exactly; this joins a loop across a seam whose two sides were not
/// made at quite the same place.
const RIM_GAP: f32 = 1.0e-3;

/// An edge left on the cutting plane, from its start to its end: with the
/// plane's normal towards the eye, a mesh wound outward has its solid on
/// the edge's left.
pub type RimEdge = [Vector3<f32>; 2];

/// What a plane leaves of a mesh.
pub struct Slice {
    /// The mesh behind the plane.
    pub kept: MeshPrimitive,
    /// The edges the cut left on the plane, in no order.
    pub rim: Vec<RimEdge>,
}

/// The part of `mesh` behind the plane `normal · x = offset`: where
/// `normal · x` is no more than `offset`.
pub fn slice(mesh: &MeshPrimitive, normal: Vector3<f32>, offset: f32) -> Slice {
    let ahead: Vec<f32> = mesh
        .vertices
        .iter()
        .map(|vertex| {
            let ahead = normal.dot(&vertex.pos) - offset;
            if ahead.abs() <= ON_PLANE {
                0.0
            } else {
                ahead
            }
        })
        .collect();

    let mut kept = Kept::new(mesh, &ahead);
    let mut rim = Vec::new();
    for triangle in mesh.indices.chunks_exact(3) {
        // The triangle clipped to the plane's back: its corners there, and a
        // new corner wherever one of its edges crosses.
        let mut corners: Vec<(u32, bool)> = Vec::with_capacity(4);
        for k in 0..3 {
            let (from, to) = (triangle[k], triangle[(k + 1) % 3]);
            let (here, there) = (ahead[from as usize], ahead[to as usize]);
            if here <= 0.0 {
                corners.push((kept.corner(from), here == 0.0));
            }
            if here * there < 0.0 {
                corners.push((kept.crossing(from, to), true));
            }
        }
        if corners.len() < 3 {
            continue;
        }
        for k in 1..corners.len() - 1 {
            kept.mesh
                .indices
                .extend([corners[0].0, corners[k].0, corners[k + 1].0]);
        }
        // Two corners on the plane are an edge of the hole; three are a
        // triangle lying in the plane, which bounds nothing. Round the
        // clipped triangle the edge runs one way, and round the hole the
        // other.
        let on_plane: Vec<usize> = (0..corners.len()).filter(|&k| corners[k].1).collect();
        if let [first, second] = on_plane[..] {
            let (start, end) = if second == first + 1 {
                (second, first)
            } else {
                (first, second)
            };
            let at = |k: usize| kept.mesh.vertices[corners[k].0 as usize].pos;
            if at(start) != at(end) {
                rim.push([at(start), at(end)]);
            }
        }
    }
    Slice {
        kept: kept.mesh,
        rim,
    }
}

/// The kept side of a mesh as it is gathered: its vertices, each of the
/// source's copied once, and one new vertex per crossed edge.
struct Kept<'a> {
    source: &'a MeshPrimitive,
    /// How far ahead of the plane each of the source's vertices is.
    ahead: &'a [f32],
    mesh: MeshPrimitive,
    /// Where each of the source's vertices landed, once a triangle kept it.
    placed: Vec<Option<u32>>,
    /// The new vertex on each crossed edge, by the edge's ends.
    crossings: FxHashMap<(u32, u32), u32>,
}

impl<'a> Kept<'a> {
    fn new(source: &'a MeshPrimitive, ahead: &'a [f32]) -> Self {
        Self {
            source,
            ahead,
            mesh: MeshPrimitive {
                vertices: Vec::new(),
                indices: Vec::new(),
                material: source.material,
            },
            placed: vec![None; source.vertices.len()],
            crossings: FxHashMap::default(),
        }
    }

    /// The kept mesh's copy of the source's vertex `index`.
    fn corner(&mut self, index: u32) -> u32 {
        if let Some(placed) = self.placed[index as usize] {
            return placed;
        }
        let placed = self.push(self.source.vertices[index as usize]);
        self.placed[index as usize] = Some(placed);
        placed
    }

    /// The vertex where the source's edge between `a` and `b` crosses the
    /// plane.
    ///
    /// Worked from the edge's ends in an order their positions decide, not
    /// their indices: two triangles that meet along a seam hold the edge by
    /// different vertices at the same places, and must agree to the last bit
    /// about where it crosses, or the rim does not join up.
    fn crossing(&mut self, a: u32, b: u32) -> u32 {
        let key = (a.min(b), a.max(b));
        if let Some(&placed) = self.crossings.get(&key) {
            return placed;
        }
        let (va, vb) = (
            self.source.vertices[a as usize],
            self.source.vertices[b as usize],
        );
        let first = |v: &Vertex| [v.pos.x, v.pos.y, v.pos.z];
        let ((from, to), (ahead_from, ahead_to)) = if first(&va) <= first(&vb) {
            ((va, vb), (self.ahead[a as usize], self.ahead[b as usize]))
        } else {
            ((vb, va), (self.ahead[b as usize], self.ahead[a as usize]))
        };
        let t = ahead_from / (ahead_from - ahead_to);
        let placed = self.push(Vertex {
            pos: from.pos + (to.pos - from.pos) * t,
            color: from.color + (to.color - from.color) * t,
            tex_coords: from.tex_coords + (to.tex_coords - from.tex_coords) * t,
            normal: (from.normal + (to.normal - from.normal) * t)
                .try_normalize(0.0)
                .unwrap_or(from.normal),
            ao: from.ao + (to.ao - from.ao) * t,
        });
        self.crossings.insert(key, placed);
        placed
    }

    fn push(&mut self, vertex: Vertex) -> u32 {
        self.mesh.vertices.push(vertex);
        (self.mesh.vertices.len() - 1) as u32
    }
}

/// Faces over the holes whose edges are `rim`, all in a plane facing
/// `normal`: positions and the triangles between them, wound to face along
/// `normal`.
pub fn cap(rim: &[RimEdge], normal: Vector3<f32>) -> (Vec<Vector3<f32>>, Vec<u32>) {
    let across = if normal.x.abs() < 0.9 {
        normal.cross(&Vector3::x()).normalize()
    } else {
        normal.cross(&Vector3::y()).normalize()
    };
    let along = normal.cross(&across);

    let (mut positions, mut indices) = (Vec::new(), Vec::new());
    for outline in loops(rim) {
        let mut flat: Vec<Vector2<f32>> = outline
            .iter()
            .map(|p| Vector2::new(p.dot(&across), p.dot(&along)))
            .collect();
        let mut order: Vec<u32> = (0..outline.len() as u32).collect();
        // Anticlockwise seen from the normal's side, whichever way the mesh
        // was wound.
        if area(&flat) < 0.0 {
            flat.reverse();
            order.reverse();
        }
        let first = positions.len() as u32;
        positions.extend(&outline);
        for corners in triangles(&flat) {
            indices.extend(corners.map(|corner| first + order[corner]));
        }
    }
    (positions, indices)
}

/// The rim's edges chained end to start into closed outlines, each a run of
/// corners.
fn loops(rim: &[RimEdge]) -> Vec<Vec<Vector3<f32>>> {
    let (mut low, mut high) = (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN));
    for end in rim.iter().flatten() {
        low = low.inf(end);
        high = high.sup(end);
    }
    let gap = RIM_GAP * (high - low).max();

    let mut used = vec![false; rim.len()];
    let mut outlines = Vec::new();
    for first in 0..rim.len() {
        if used[first] {
            continue;
        }
        used[first] = true;
        let mut outline = vec![rim[first][0]];
        let mut end = rim[first][1];
        // Follow the edge that starts nearest where the last one ended,
        // until none starts near enough: the loop has come round, or the
        // surface was open there.
        loop {
            let next = (0..rim.len())
                .filter(|&edge| !used[edge])
                .map(|edge| (edge, (rim[edge][0] - end).norm()))
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .filter(|(_, apart)| *apart <= gap);
            let Some((edge, _)) = next else { break };
            used[edge] = true;
            outline.push(rim[edge][0]);
            end = rim[edge][1];
        }
        if (end - outline[0]).norm() > gap {
            outline.push(end);
        }
        if outline.len() >= 3 {
            outlines.push(outline);
        }
    }
    outlines
}

/// The area an outline encloses: positive wound anticlockwise.
fn area(outline: &[Vector2<f32>]) -> f32 {
    let doubled: f32 = (0..outline.len())
        .map(|k| outline[k].perp(&outline[(k + 1) % outline.len()]))
        .sum();
    0.5 * doubled
}

/// An anticlockwise outline cut into triangles, each three of its corners,
/// by taking off one ear at a time: a corner that turns left with no other
/// corner inside the triangle it makes with its neighbours.
fn triangles(outline: &[Vector2<f32>]) -> Vec<[usize; 3]> {
    let mut ring: Vec<usize> = (0..outline.len()).collect();
    let mut cut = Vec::with_capacity(outline.len().saturating_sub(2));
    while ring.len() > 3 {
        let count = ring.len();
        let corner = |k: usize| {
            [
                ring[(k + count - 1) % count],
                ring[k],
                ring[(k + 1) % count],
            ]
        };
        let is_ear = |k: usize| {
            let [a, b, c] = corner(k).map(|i| outline[i]);
            if (b - a).perp(&(c - b)) < 0.0 {
                return false;
            }
            let within = |p: Vector2<f32>| {
                (b - a).perp(&(p - a)) > 0.0
                    && (c - b).perp(&(p - b)) > 0.0
                    && (a - c).perp(&(p - c)) > 0.0
            };
            !ring
                .iter()
                .filter(|i| !corner(k).contains(i))
                .any(|&i| within(outline[i]))
        };
        // An outline that crosses itself may have no ear left; any corner
        // then, so the cutting always ends.
        let ear = (0..count).find(|&k| is_ear(k)).unwrap_or(0);
        cut.push(corner(ear));
        ring.remove(ear);
    }
    if let [a, b, c] = ring[..] {
        cut.push([a, b, c]);
    }
    cut
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector2, Vector4};

    use super::*;
    use crate::rendering::material::MaterialId;

    /// A box of half-size `half` about `centre`, each face its own four
    /// vertices, as a mesh with seams is.
    fn box_mesh(centre: Vector3<f32>, half: Vector3<f32>) -> MeshPrimitive {
        let (mut vertices, mut indices) = (Vec::new(), Vec::new());
        for axis in 0..3 {
            for sign in [-1.0f32, 1.0] {
                let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                let mut normal = Vector3::zeros();
                normal[axis] = sign;
                let first = vertices.len() as u32;
                for (su, sv) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                    let mut pos = Vector3::zeros();
                    pos[axis] = sign * half[axis];
                    pos[u] = su * sign * half[u];
                    pos[v] = sv * half[v];
                    vertices.push(Vertex {
                        pos: centre + pos,
                        color: Vector4::repeat(1.0),
                        tex_coords: Vector2::new(su, sv),
                        normal,
                        ao: 1.0,
                    });
                }
                indices.extend([first, first + 1, first + 2, first, first + 2, first + 3]);
            }
        }
        MeshPrimitive {
            vertices,
            indices,
            material: MaterialId(0),
        }
    }

    /// The volume a closed surface encloses, from its triangles.
    fn enclosed(surfaces: &[(&[Vector3<f32>], &[u32])]) -> f32 {
        surfaces
            .iter()
            .flat_map(|(positions, indices)| {
                indices.chunks_exact(3).map(|t| {
                    let [a, b, c] = [t[0], t[1], t[2]].map(|i| positions[i as usize]);
                    a.dot(&b.cross(&c)) / 6.0
                })
            })
            .sum()
    }

    fn positions(mesh: &MeshPrimitive) -> Vec<Vector3<f32>> {
        mesh.vertices.iter().map(|vertex| vertex.pos).collect()
    }

    #[test]
    fn a_box_sliced_and_capped_encloses_what_is_left_of_it() {
        let mesh = box_mesh(Vector3::zeros(), Vector3::new(1.0, 0.5, 0.25));
        let normal = Vector3::new(0.8, 0.5, 0.33).normalize();
        let sliced = slice(&mesh, normal, 0.1);
        let (lid, lid_indices) = cap(&sliced.rim, normal);
        let other = slice(&mesh, -normal, -0.1);
        let (floor, floor_indices) = cap(&other.rim, -normal);

        let near = enclosed(&[
            (&positions(&sliced.kept), &sliced.kept.indices),
            (&lid, &lid_indices),
        ]);
        let far = enclosed(&[
            (&positions(&other.kept), &other.kept.indices),
            (&floor, &floor_indices),
        ]);
        assert!(near > 0.1 && far > 0.1, "{near} and {far}");
        assert!((near + far - 1.0).abs() < 1.0e-4, "{near} + {far}");
    }

    #[test]
    fn nothing_kept_lies_ahead_of_the_plane() {
        let mesh = box_mesh(Vector3::zeros(), Vector3::repeat(0.5));
        let normal = Vector3::new(0.2, 1.0, -0.4).normalize();
        let sliced = slice(&mesh, normal, 0.05);
        assert!(!sliced.kept.vertices.is_empty());
        for vertex in &sliced.kept.vertices {
            assert!(normal.dot(&vertex.pos) <= 0.05 + 1.0e-5);
        }
    }

    #[test]
    fn a_plane_that_misses_leaves_the_mesh_whole_and_no_rim() {
        let mesh = box_mesh(Vector3::zeros(), Vector3::repeat(0.5));
        let sliced = slice(&mesh, Vector3::y(), 2.0);
        assert_eq!(sliced.kept.indices.len(), mesh.indices.len());
        assert!(sliced.rim.is_empty());
        let gone = slice(&mesh, Vector3::y(), -2.0);
        assert!(gone.kept.indices.is_empty());
    }

    /// An outline that cannot be seen whole from any point inside it: a
    /// fan would spill over the notch.
    #[test]
    fn a_cap_fills_an_outline_with_a_notch_in_it_and_no_more() {
        // A U: three sides of a square, 0.2 wide.
        let outline = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.8, 1.0),
            (0.8, 0.2),
            (0.2, 0.2),
            (0.2, 1.0),
            (0.0, 1.0),
        ]
        .map(|(x, z): (f32, f32)| Vector3::new(x, 0.0, z));
        // Anticlockwise seen from +y runs the other way round in x and z.
        let rim: Vec<RimEdge> = (0..outline.len())
            .map(|k| [outline[(k + 1) % outline.len()], outline[k]])
            .collect();
        let (lid, indices) = cap(&rim, Vector3::y());
        let mut covered = 0.0;
        for t in indices.chunks_exact(3) {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| lid[i as usize]);
            let facing = 0.5 * (b - a).cross(&(c - a)).dot(&Vector3::y());
            assert!(facing >= 0.0, "a triangle faces away");
            covered += facing;
            let middle = (a + b + c) / 3.0;
            let in_notch = middle.x > 0.2 && middle.x < 0.8 && middle.z > 0.2;
            assert!(!in_notch, "a triangle lies in the notch");
        }
        assert!((covered - 0.52).abs() < 1.0e-5, "{covered} m² covered");
    }

    #[test]
    fn two_solids_cut_by_one_plane_are_capped_apart() {
        let half = Vector3::repeat(0.5);
        let mut mesh = box_mesh(Vector3::new(-2.0, 0.0, 0.0), half);
        let other = box_mesh(Vector3::new(2.0, 0.0, 0.0), half);
        let shift = mesh.vertices.len() as u32;
        mesh.vertices.extend(other.vertices);
        mesh.indices.extend(other.indices.iter().map(|i| i + shift));

        let sliced = slice(&mesh, Vector3::y(), 0.0);
        let (lid, indices) = cap(&sliced.rim, Vector3::y());
        // A lid over each, a square metre apiece, and nothing spanning the
        // gap between them.
        let area: f32 = indices
            .chunks_exact(3)
            .map(|t| {
                let [a, b, c] = [t[0], t[1], t[2]].map(|i| lid[i as usize]);
                0.5 * (b - a).cross(&(c - a)).dot(&Vector3::y())
            })
            .sum();
        assert!((area - 2.0).abs() < 1.0e-4, "{area} m² of lid");
        assert!(lid.iter().all(|p| p.x.abs() >= 1.5 - 1.0e-5));
    }
}
