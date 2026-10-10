//! River surfaces as the renderer holds them: vertices that carry their own
//! surface height, depth and flow, and one draw per surface.

use crate::water::ids::SurfaceId;

use super::field_sources::SurfaceSources;
use super::vertex::RiverVertex;

/// Depth at which a river surface is cut, m: the shoreline's contour.
/// Thinner water would lie on its floor and fight it for depth.
pub const SHORE_DEPTH: f32 = 0.005;

/// One surface's draw.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RiverDraw {
    pub id: SurfaceId,
    pub first_index: u32,
    pub index_count: u32,
}

/// River surfaces, in one vertex and index buffer.
#[derive(Debug, Clone, Default)]
pub struct RiverMesh {
    pub vertices: Vec<RiverVertex>,
    pub indices: Vec<u32>,
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
        self.vertices.extend_from_slice(&other.vertices);
        self.indices.extend(other.indices.iter().map(|i| i + base));
        self.draws.extend(other.draws.iter().map(|d| RiverDraw {
            first_index: d.first_index + first,
            ..*d
        }));
    }
}
