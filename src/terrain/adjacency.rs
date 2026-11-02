//! Triangle adjacency computation for arbitrary triangle meshes.
//!
//! Determines which triangles share edges in a `MeshOctree`, using quantized
//! vertex positions for robust floating-point matching. The adjacency map is
//! a flat side structure — the octree internals are untouched.
//!
//! # Algorithm
//!
//! Vertex positions are quantized to an integer grid (controlled by a
//! tolerance parameter) so that shared vertices compare exactly. Each
//! triangle's three edges are stored in an `FxHashMap` keyed by canonical
//! `Edge` (smaller vertex first). When exactly two triangles share an edge
//! (manifold), they are linked as neighbors.
//!
//! # Use
//!
//! The terrain builds one map per chunk, fresh from the chunk's triangles
//! whenever it remeshes (`from_triangles`), and replaces the old one whole;
//! `segment_adjacency` links triangles across chunk seams on top.

use std::hash::Hash;
use std::time::Duration;

use nalgebra::Point3;
use rustc_hash::FxHashMap;
use smallvec::SmallVec;

use super::mesh_octree::TriangleRef;

/// Quantized vertex position for floating-point-safe edge matching.
///
/// Positions are snapped to a fixed grid (determined by tolerance) so that
/// vertices produced by different marching-cubes cells or octree leaves
/// are guaranteed to compare equal when they represent the same point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) struct QuantizedPos(i64, i64, i64);

impl QuantizedPos {
    fn from_point(p: Point3<f32>, inv_cell: f64) -> Self {
        Self(
            (p.x as f64 * inv_cell).round() as i64,
            (p.y as f64 * inv_cell).round() as i64,
            (p.z as f64 * inv_cell).round() as i64,
        )
    }

    /// The grid point back as a position, given the cell size.
    fn to_point(self, cell: f64) -> Point3<f32> {
        Point3::new(
            (self.0 as f64 * cell) as f32,
            (self.1 as f64 * cell) as f32,
            (self.2 as f64 * cell) as f32,
        )
    }
}

/// Canonical edge representation: smaller quantized vertex first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) struct Edge(QuantizedPos, QuantizedPos);

impl Edge {
    fn new(a: QuantizedPos, b: QuantizedPos) -> Self {
        if (a.0, a.1, a.2) <= (b.0, b.1, b.2) {
            Edge(a, b)
        } else {
            Edge(b, a)
        }
    }

    /// An edge claimed by `triangles` triangles, as a defect report.
    pub(super) fn defect(&self, inv_cell: f64, triangles: usize) -> DefectiveEdge {
        let cell = if inv_cell == 0.0 { 0.0 } else { 1.0 / inv_cell };
        DefectiveEdge {
            from: self.0.to_point(cell),
            to: self.1.to_point(cell),
            triangles,
        }
    }
}

/// Per-triangle adjacency: the neighbor across each of the three edges.
///
/// `neighbors[0]` = neighbor across edge v0→v1
/// `neighbors[1]` = neighbor across edge v1→v2
/// `neighbors[2]` = neighbor across edge v2→v0
/// Wall-clock breakdown of patching the adjacency map for remeshed triangles.
#[derive(Debug, Clone, Copy, Default)]
pub struct AdjacencyTimings {
    /// Removing the old triangles and cutting the links they held.
    pub unlink: Duration,
    /// Inserting the new triangles and linking every edge that became manifold.
    pub link: Duration,
}

impl AdjacencyTimings {
    pub fn total(&self) -> Duration {
        self.unlink + self.link
    }

    pub fn add(&mut self, other: &Self) {
        self.unlink += other.unlink;
        self.link += other.link;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TriangleNeighbors<R> {
    pub neighbors: [Option<R>; 3],
}

/// Triangle adjacency map over an arbitrary triangle identifier `R`.
///
/// Maps each triangle to its up-to-three edge-sharing neighbors. The edge map
/// persists across updates so that incremental region updates only touch
/// affected edges.
///
/// The identifier is a type parameter because adjacency has to span chunk
/// boundaries: triangles meeting at a chunk seam live in different mesh
/// octrees, so the terrain uses a chunk-qualified reference while the octree's
/// own `TriangleRef` remains sufficient in single-octree tests.
pub struct AdjacencyMap<R = TriangleRef> {
    /// Per-triangle neighbor lookup.
    adjacency: FxHashMap<R, TriangleNeighbors<R>>,

    /// Every edge and the triangles (and their edge slots) that claim it.
    /// SmallVec<[_; 2]> keeps manifold edges (the common case) inline.
    edge_map: FxHashMap<Edge, SmallVec<[(R, u8); 2]>>,

    /// Cached inverse cell size for quantization.
    inv_cell: f64,
}

impl<R: Copy + Eq + Hash> AdjacencyMap<R> {
    /// Create an empty adjacency map.
    pub fn new() -> Self {
        Self {
            adjacency: FxHashMap::default(),
            edge_map: FxHashMap::default(),
            inv_cell: 0.0,
        }
    }

    /// Link `triangles` among themselves, as a fresh map: the first two
    /// triangles to claim an edge are neighbours across it.
    pub fn from_triangles(triangles: &[(R, [Point3<f32>; 3])], tolerance: f32) -> Self {
        let mut map = Self::new();
        map.inv_cell = 1.0 / tolerance as f64;
        for (tri_ref, positions) in triangles {
            let edges = map.triangle_edges(positions);
            for (edge_idx, edge) in edges.iter().enumerate() {
                let entries = map.edge_map.entry(*edge).or_default();
                entries.push((*tri_ref, edge_idx as u8));
                if let [(ref_a, edge_a), (ref_b, edge_b)] = entries[..] {
                    map.set_neighbour(ref_a, edge_a, Some(ref_b));
                    map.set_neighbour(ref_b, edge_b, Some(ref_a));
                }
            }
        }
        map
    }

    /// Edges not shared by exactly two of this map's triangles, with the
    /// triangles (and their edge slots) that do claim them.
    pub(super) fn loose_edges(&self) -> impl Iterator<Item = (&Edge, &[(R, u8)])> {
        self.edge_map
            .iter()
            .filter(|(_, entries)| entries.len() != 2)
            .map(|(edge, entries)| (edge, entries.as_slice()))
    }

    /// Set the neighbour across one edge of `triangle`, for a link made
    /// outside this map. Clearing a slot of a triangle with no entry is a
    /// no-op, as is clearing one already clear.
    pub(super) fn set_neighbour(&mut self, triangle: R, slot: u8, neighbour: Option<R>) {
        match neighbour {
            Some(_) => {
                self.adjacency
                    .entry(triangle)
                    .or_insert(TriangleNeighbors {
                        neighbors: [None; 3],
                    })
                    .neighbors[slot as usize] = neighbour;
            }
            None => {
                if let Some(neighbours) = self.adjacency.get_mut(&triangle) {
                    neighbours.neighbors[slot as usize] = None;
                }
            }
        }
    }

    /// Filled neighbour slots across every triangle. A link inside the map
    /// fills two; a link to a triangle outside it fills one here.
    pub(super) fn linked_slot_count(&self) -> usize {
        self.adjacency
            .values()
            .map(|n| n.neighbors.iter().filter(|x| x.is_some()).count())
            .sum()
    }

    /// Compute the 3 quantized edges for a triangle's vertex positions.
    fn triangle_edges(&self, positions: &[Point3<f32>; 3]) -> [Edge; 3] {
        let q = [
            QuantizedPos::from_point(positions[0], self.inv_cell),
            QuantizedPos::from_point(positions[1], self.inv_cell),
            QuantizedPos::from_point(positions[2], self.inv_cell),
        ];
        [
            Edge::new(q[0], q[1]),
            Edge::new(q[1], q[2]),
            Edge::new(q[2], q[0]),
        ]
    }

    /// Every triangle with at least one neighbour, and its neighbours.
    ///
    /// In hash order, which depends on insertion history; a caller comparing
    /// two maps must not rely on it.
    pub fn links(&self) -> impl Iterator<Item = (&R, &TriangleNeighbors<R>)> {
        self.adjacency.iter()
    }

    /// Look up the neighbors of a triangle.
    #[allow(dead_code)]
    pub fn neighbors(&self, tri: &R) -> Option<&TriangleNeighbors<R>> {
        self.adjacency.get(tri)
    }

    /// Total number of triangles with at least one neighbor.
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.adjacency.len()
    }

    /// Whether the adjacency map is empty.
    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.adjacency.is_empty()
    }

    /// Count the total number of manifold edges (shared by exactly 2 triangles).
    pub fn manifold_edge_count(&self) -> usize {
        // Each manifold edge is counted twice (once from each side).
        self.linked_slot_count() / 2
    }

    /// Count boundary edges (edges with only one triangle).
    pub fn boundary_edge_count(&self, total_triangles: usize) -> usize {
        // Total edges in the mesh = 3 * triangles.
        // Each manifold edge is shared, so: boundary = 3*T - 2*manifold.
        let manifold = self.manifold_edge_count();
        3 * total_triangles - 2 * manifold
    }

    /// Every edge not shared by exactly two triangles, as segment-local
    /// endpoints and the number of triangles that claim it.
    ///
    /// `boundary_edge_count` collapses two distinct defects into one number: a
    /// hole (one triangle) and a seam where the surface passes through itself
    /// (three or more). This reports them apart, and says where they are.
    pub fn defective_edges(&self) -> Vec<DefectiveEdge> {
        self.loose_edges()
            .map(|(edge, entries)| edge.defect(self.inv_cell, entries.len()))
            .collect()
    }
}

/// An edge that is not shared by exactly two triangles.
#[derive(Debug, Clone, Copy)]
pub struct DefectiveEdge {
    pub from: Point3<f32>,
    pub to: Point3<f32>,
    /// Triangles claiming the edge: 1 is a hole, 3+ is a non-manifold seam.
    pub triangles: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::AABB;
    use crate::rendering::vertex::Vertex;
    use crate::terrain::mesh_octree::MeshOctree;

    /// The map of a whole octree, keyed by its own triangle refs.
    fn adjacency_of(octree: &MeshOctree) -> AdjacencyMap {
        let mut triangles = Vec::new();
        octree.collect_all_triangles_into(&mut triangles);
        AdjacencyMap::from_triangles(&triangles, 1e-4)
    }

    fn test_vertex(x: f32, y: f32, z: f32) -> Vertex {
        Vertex {
            pos: nalgebra::Vector3::new(x, y, z),
            color: nalgebra::Vector4::new(1.0, 1.0, 1.0, 1.0),
            tex_coords: nalgebra::Vector2::new(0.0, 0.0),
            normal: nalgebra::Vector3::new(0.0, 1.0, 0.0),
            ao: 1.0,
        }
    }

    #[test]
    fn test_two_triangles_sharing_edge() {
        let bounds = AABB::new(Point3::origin(), Point3::new(10.0, 10.0, 10.0));
        let mut octree = MeshOctree::new(bounds);

        octree.insert_test_triangle(
            &test_vertex(0.0, 0.0, 0.0),
            &test_vertex(1.0, 0.0, 0.0),
            &test_vertex(0.5, 1.0, 0.0),
        );
        octree.insert_test_triangle(
            &test_vertex(0.0, 0.0, 0.0),
            &test_vertex(1.0, 0.0, 0.0),
            &test_vertex(0.5, -1.0, 0.0),
        );

        octree.rebuild_neighbor_refs();
        assert_eq!(octree.triangle_count(), 2);

        let adj = adjacency_of(&octree);
        assert_eq!(adj.manifold_edge_count(), 1);
        assert_eq!(adj.len(), 2);
    }

    #[test]
    fn test_quad_as_two_triangles() {
        let bounds = AABB::new(Point3::origin(), Point3::new(10.0, 10.0, 10.0));
        let mut octree = MeshOctree::new(bounds);

        octree.insert_test_triangle(
            &test_vertex(0.0, 0.0, 0.0),
            &test_vertex(1.0, 0.0, 0.0),
            &test_vertex(1.0, 1.0, 0.0),
        );
        octree.insert_test_triangle(
            &test_vertex(0.0, 0.0, 0.0),
            &test_vertex(1.0, 1.0, 0.0),
            &test_vertex(0.0, 1.0, 0.0),
        );

        octree.rebuild_neighbor_refs();

        let adj = adjacency_of(&octree);
        assert_eq!(adj.manifold_edge_count(), 1);
        assert_eq!(adj.boundary_edge_count(2), 4);
    }

    /// Open-edge counting is what `level_check` reports mesh integrity from, so
    /// it is checked against hand-built meshes whose boundary is countable by
    /// eye: a lone triangle has three open edges, a fan of three triangles
    /// sharing two interior edges has 3*3 - 2*2 = 5, and a closed tetrahedron
    /// has none.
    #[test]
    fn boundary_edges_counted_on_hand_built_meshes() {
        let bounds = AABB::new(
            Point3::new(-10.0, -10.0, -10.0),
            Point3::new(10.0, 10.0, 10.0),
        );

        let count = |triangles: &[[(f32, f32, f32); 3]]| {
            let mut octree = MeshOctree::new(bounds);
            for [a, b, c] in triangles {
                octree.insert_test_triangle(
                    &test_vertex(a.0, a.1, a.2),
                    &test_vertex(b.0, b.1, b.2),
                    &test_vertex(c.0, c.1, c.2),
                );
            }
            octree.rebuild_neighbor_refs();
            let adj = adjacency_of(&octree);
            adj.boundary_edge_count(triangles.len())
        };

        let lone = [[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]];
        assert_eq!(count(&lone), 3, "lone triangle");

        // Three triangles fanning from the origin, each sharing one edge with
        // the next: two interior edges, five on the boundary.
        let fan = [
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)],
            [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)],
            [(0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)],
        ];
        assert_eq!(count(&fan), 5, "triangle fan");

        // A closed tetrahedron: every edge is shared by exactly two faces.
        let (p, q, r, t) = (
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (1.0, 2.0, 0.0),
            (1.0, 0.7, 2.0),
        );
        let tetra = [[p, q, r], [p, q, t], [q, r, t], [r, p, t]];
        assert_eq!(count(&tetra), 0, "closed tetrahedron");
    }

    #[test]
    fn test_isolated_triangle_no_adjacency() {
        let bounds = AABB::new(Point3::origin(), Point3::new(10.0, 10.0, 10.0));
        let mut octree = MeshOctree::new(bounds);

        octree.insert_test_triangle(
            &test_vertex(1.0, 1.0, 1.0),
            &test_vertex(2.0, 1.0, 1.0),
            &test_vertex(1.5, 2.0, 1.0),
        );

        octree.rebuild_neighbor_refs();

        let adj = adjacency_of(&octree);
        assert_eq!(adj.manifold_edge_count(), 0);
        assert!(adj.is_empty());
    }
}
