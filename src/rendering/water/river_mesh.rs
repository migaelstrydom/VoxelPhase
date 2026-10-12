//! River surfaces as the renderer holds them: vertices that carry their own
//! surface height, depth and flow, and one draw per surface. A surface's
//! falling sheets are indexed apart from it, because they are drawn after
//! every surface, blended over them.

use crate::water::ids::SurfaceId;

use super::field_sources::SurfaceSources;
use super::vertex::RiverVertex;

/// One surface's draw.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RiverDraw {
    pub id: SurfaceId,
    /// Its surface's triangles, in `RiverMesh::indices`.
    pub first_index: u32,
    pub index_count: u32,
    /// Its falling sheets' triangles, in `RiverMesh::sheet_indices`.
    pub first_sheet_index: u32,
    pub sheet_index_count: u32,
}

/// River surfaces, in one vertex and index buffer.
#[derive(Debug, Clone, Default)]
pub struct RiverMesh {
    pub vertices: Vec<RiverVertex>,
    pub indices: Vec<u32>,
    /// The falling sheets' triangles, into the same vertices.
    pub sheet_indices: Vec<u32>,
    pub draws: Vec<RiverDraw>,
    /// Where each vertex reads its water, for a surface that can be read
    /// again without being built; empty for one that cannot, and in a mesh
    /// joined from several.
    pub sources: SurfaceSources,
}

impl RiverMesh {
    /// Add another mesh's surfaces after this one's.
    pub fn append(&mut self, other: &RiverMesh) {
        let base = self.vertices.len() as u32;
        let first = self.indices.len() as u32;
        let first_sheet = self.sheet_indices.len() as u32;
        self.vertices.extend_from_slice(&other.vertices);
        self.indices.extend(other.indices.iter().map(|i| i + base));
        self.sheet_indices
            .extend(other.sheet_indices.iter().map(|i| i + base));
        self.draws.extend(other.draws.iter().map(|d| RiverDraw {
            first_index: d.first_index + first,
            first_sheet_index: d.first_sheet_index + first_sheet,
            ..*d
        }));
    }
}
