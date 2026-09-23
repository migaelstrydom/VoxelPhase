//! Triangle adjacency across a segment, stored chunk by chunk.
//!
//! ```text
//!   ChunkRebuild (parallel)                 SegmentAdjacency (sequential commit)
//!   ─────────────────────────               ────────────────────────────────────
//!   AdjacencyMap::from_triangles  ──────▶   chunks: coord → AdjacencyMap
//!     links inside the chunk                  swapped whole on replace_chunk
//!     loose edges (not shared by                     │
//!     exactly two of its triangles)  ────▶   seams: edge → [(triangle, slot)]
//!                                              linked across chunks, patching
//!                                              both chunks' maps
//! ```
//!
//! A remeshed chunk used to unlink and relink every one of its triangles in a
//! segment-wide edge map: thousands of cold hash lookups for a crater that
//! changed a few dozen triangles. Here a chunk's own links are built with its
//! mesh, off the main thread, and replacing a chunk swaps its map whole. Only
//! the edges a chunk cannot pair on its own — in practice those on its faces,
//! shared with a neighbouring chunk — go through the segment-wide seam map.
//!
//! The linking rule is the flat map's: a triangle pair is linked when an
//! edge gains its second claimant, and the survivor is unlinked when an edge
//! with two claimants loses one. An edge a chunk pairs by itself is assumed not
//! to be claimed by any other chunk too; marching cubes gives each face segment
//! of a cell one triangle, so a pair on a chunk face never happens within one
//! chunk.

use std::time::Instant;

use rustc_hash::FxHashMap;
use smallvec::SmallVec;

use super::adjacency::{AdjacencyMap, AdjacencyTimings, DefectiveEdge, Edge, TriangleNeighbors};
use super::chunk::{ChunkCoord, ChunkTriangleRef};

/// Fraction of a voxel used as the vertex-matching tolerance when linking
/// triangles.
const TOLERANCE_FACTOR: f32 = 0.01;

/// The vertex-matching tolerance for a grid of `voxel_size` voxels. Every
/// chunk of a segment must be linked with the same one, or its seams will not
/// match.
pub fn adjacency_tolerance(voxel_size: f32) -> f32 {
    voxel_size * TOLERANCE_FACTOR
}

/// One chunk's adjacency, built from that chunk's triangles alone.
pub type ChunkAdjacency = AdjacencyMap<ChunkTriangleRef>;

/// Triangle adjacency across every chunk of a segment.
pub struct SegmentAdjacency {
    /// Each chunk's links, including those to triangles in other chunks,
    /// which the seam map writes in.
    chunks: FxHashMap<ChunkCoord, ChunkAdjacency>,
    /// Every chunk's loose edges, with the triangles claiming them. An edge
    /// with two claimants from different chunks is a linked seam; any other
    /// count is a defect.
    seams: FxHashMap<Edge, SmallVec<[(ChunkTriangleRef, u8); 2]>>,
    /// Inverse of the matching tolerance, for reporting edge positions.
    inv_cell: f64,
}

impl SegmentAdjacency {
    pub fn new(voxel_size: f32) -> Self {
        Self {
            chunks: FxHashMap::default(),
            seams: FxHashMap::default(),
            inv_cell: 1.0 / adjacency_tolerance(voxel_size) as f64,
        }
    }

    /// Install `adjacency` as chunk `coord`'s, dropping whatever it replaces
    /// and re-linking the chunk's seams with its neighbours.
    pub fn replace_chunk(
        &mut self,
        coord: ChunkCoord,
        adjacency: ChunkAdjacency,
    ) -> AdjacencyTimings {
        let t_unlink = Instant::now();
        if let Some(old) = self.chunks.remove(&coord) {
            self.unlink_seams(coord, &old);
        }
        let unlink = t_unlink.elapsed();

        let t_link = Instant::now();
        let loose: Vec<(Edge, ChunkTriangleRef, u8)> = adjacency
            .loose_edges()
            .flat_map(|(edge, entries)| {
                entries
                    .iter()
                    .map(|&(triangle, slot)| (*edge, triangle, slot))
            })
            .collect();
        if !adjacency.is_empty() || !loose.is_empty() {
            self.chunks.insert(coord, adjacency);
        }
        for (edge, triangle, slot) in loose {
            self.link_seam(edge, triangle, slot);
        }

        AdjacencyTimings {
            unlink,
            link: t_link.elapsed(),
        }
    }

    /// Take a departing chunk's triangles off every seam edge they claimed,
    /// clearing the link a neighbour in another chunk held to them.
    fn unlink_seams(&mut self, coord: ChunkCoord, old: &ChunkAdjacency) {
        for (edge, entries) in old.loose_edges() {
            let Some(claimants) = self.seams.get_mut(edge) else {
                continue;
            };
            for (triangle, _) in entries {
                if claimants.len() == 2 {
                    let survivor = if claimants[0].0 == *triangle {
                        claimants[1]
                    } else {
                        claimants[0]
                    };
                    if survivor.0.chunk != coord {
                        if let Some(map) = self.chunks.get_mut(&survivor.0.chunk) {
                            map.set_neighbour(survivor.0, survivor.1, None);
                        }
                    }
                }
                claimants.retain(|(r, _)| r != triangle);
            }
            if claimants.is_empty() {
                self.seams.remove(edge);
            }
        }
    }

    /// Add one claimant to a seam edge, linking it to the other when it is the
    /// second.
    fn link_seam(&mut self, edge: Edge, triangle: ChunkTriangleRef, slot: u8) {
        let claimants = self.seams.entry(edge).or_default();
        claimants.push((triangle, slot));
        if claimants.len() != 2 {
            return;
        }
        let [(a, slot_a), (b, slot_b)] = [claimants[0], claimants[1]];
        self.set_neighbour(a, slot_a, b);
        self.set_neighbour(b, slot_b, a);
    }

    fn set_neighbour(&mut self, triangle: ChunkTriangleRef, slot: u8, neighbour: ChunkTriangleRef) {
        self.chunks
            .entry(triangle.chunk)
            .or_insert_with(AdjacencyMap::new)
            .set_neighbour(triangle, slot, Some(neighbour));
    }

    /// The neighbours of a triangle, if it has any.
    pub fn neighbors(
        &self,
        triangle: &ChunkTriangleRef,
    ) -> Option<&TriangleNeighbors<ChunkTriangleRef>> {
        self.chunks.get(&triangle.chunk)?.neighbors(triangle)
    }

    /// Every triangle with at least one neighbour, and its neighbours, in no
    /// particular order.
    pub fn links(
        &self,
    ) -> impl Iterator<Item = (&ChunkTriangleRef, &TriangleNeighbors<ChunkTriangleRef>)> {
        self.chunks.values().flat_map(|chunk| chunk.links())
    }

    /// Edges belonging to exactly one of `total_triangles` triangles. A closed
    /// surface has none.
    pub fn boundary_edge_count(&self, total_triangles: usize) -> usize {
        // Each manifold edge fills a neighbour slot on both of its triangles,
        // whether they share a chunk or not.
        let linked: usize = self.chunks.values().map(|c| c.linked_slot_count()).sum();
        3 * total_triangles - linked
    }

    /// Every edge not shared by exactly two triangles, in segment-local space.
    pub fn defective_edges(&self) -> Vec<DefectiveEdge> {
        self.seams
            .iter()
            .filter(|(_, claimants)| claimants.len() != 2)
            .map(|(edge, claimants)| edge.defect(self.inv_cell, claimants.len()))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::terrain::mesh_octree::TriangleRef;

    fn triangle(chunk: i32, index: u32) -> ChunkTriangleRef {
        ChunkTriangleRef {
            chunk: ChunkCoord {
                x: chunk,
                y: 0,
                z: 0,
            },
            triangle: TriangleRef::for_test(index),
        }
    }

    fn p(x: f32, y: f32) -> Point3<f32> {
        Point3::new(x, y, 0.0)
    }

    /// A unit square split into two triangles along x = 1, one each side, so
    /// that chunk 0 owns x ≤ 1 and chunk 1 owns x ≥ 1.
    fn left() -> ChunkAdjacency {
        AdjacencyMap::from_triangles(
            &[(triangle(0, 0), [p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0)])],
            1e-3,
        )
    }

    fn right() -> ChunkAdjacency {
        AdjacencyMap::from_triangles(
            &[(triangle(1, 0), [p(1.0, 1.0), p(1.0, 0.0), p(2.0, 0.0)])],
            1e-3,
        )
    }

    const VOXEL: f32 = 0.1;

    #[test]
    fn triangles_in_neighbouring_chunks_link_across_the_seam() {
        let mut adjacency = SegmentAdjacency::new(VOXEL);
        adjacency.replace_chunk(triangle(0, 0).chunk, left());
        adjacency.replace_chunk(triangle(1, 0).chunk, right());

        let left_links = adjacency
            .neighbors(&triangle(0, 0))
            .expect("left has a neighbour");
        assert!(left_links.neighbors.contains(&Some(triangle(1, 0))));
        let right_links = adjacency
            .neighbors(&triangle(1, 0))
            .expect("right has a neighbour");
        assert!(right_links.neighbors.contains(&Some(triangle(0, 0))));
        assert_eq!(adjacency.boundary_edge_count(2), 4);
    }

    #[test]
    fn replacing_a_chunk_unlinks_its_neighbour_and_relinks_the_new_mesh() {
        let mut adjacency = SegmentAdjacency::new(VOXEL);
        adjacency.replace_chunk(triangle(0, 0).chunk, left());
        adjacency.replace_chunk(triangle(1, 0).chunk, right());

        adjacency.replace_chunk(triangle(1, 0).chunk, AdjacencyMap::new());
        let left_links = adjacency
            .neighbors(&triangle(0, 0))
            .expect("entry survives");
        assert_eq!(left_links.neighbors, [None; 3]);
        assert_eq!(adjacency.defective_edges().len(), 3);

        adjacency.replace_chunk(triangle(1, 0).chunk, right());
        let left_links = adjacency.neighbors(&triangle(0, 0)).expect("relinked");
        assert!(left_links.neighbors.contains(&Some(triangle(1, 0))));
        assert_eq!(adjacency.boundary_edge_count(2), 4);
    }

    #[test]
    fn links_inside_a_chunk_never_reach_the_seam_map() {
        let square = AdjacencyMap::from_triangles(
            &[
                (triangle(0, 0), [p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0)]),
                (triangle(0, 1), [p(1.0, 1.0), p(0.0, 1.0), p(0.0, 0.0)]),
            ],
            1e-3,
        );
        let mut adjacency = SegmentAdjacency::new(VOXEL);
        adjacency.replace_chunk(triangle(0, 0).chunk, square);

        assert!(adjacency.neighbors(&triangle(0, 0)).is_some());
        assert_eq!(
            adjacency.seams.len(),
            4,
            "only the square's outline is loose"
        );
        assert_eq!(adjacency.boundary_edge_count(2), 4);
    }
}
