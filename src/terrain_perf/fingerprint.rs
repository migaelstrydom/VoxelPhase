use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use crate::terrain::TerrainWorld;

/// A hash of everything a rebuild produces: the render buffers bit for bit,
/// and every segment's adjacency links.
///
/// A change meant only to make rebuilding faster must leave this alone. The
/// render buffers are hashed in order, since draw order is part of the output.
/// Adjacency is a hash map whose iteration order depends on insertion history,
/// which a change of scheduling may legitimately alter, so its links are hashed
/// one by one and combined with an order-independent sum.
pub fn terrain_fingerprint(terrain: &TerrainWorld) -> u64 {
    let mut hasher = DefaultHasher::new();
    for vertex in terrain.render_vertices() {
        let values = vertex
            .pos
            .iter()
            .chain(vertex.color.iter())
            .chain(vertex.tex_coords.iter())
            .chain(vertex.normal.iter())
            .chain(std::iter::once(&vertex.ao));
        for value in values {
            hasher.write_u32(value.to_bits());
        }
    }
    for &index in terrain.render_indices() {
        hasher.write_u32(index);
    }

    for segment in terrain.segments() {
        let links = segment
            .adjacency()
            .links()
            .map(|(triangle, neighbours)| {
                let mut link = DefaultHasher::new();
                triangle.hash(&mut link);
                neighbours.neighbors.hash(&mut link);
                link.finish()
            })
            .fold(0u64, u64::wrapping_add);
        segment.name().hash(&mut hasher);
        hasher.write_u64(links);
    }
    hasher.finish()
}
