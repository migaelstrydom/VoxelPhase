//! A still body of water for a bench shot to stand things in.
//!
//! Water divides the scene's blended surfaces: what lies beyond its surface is
//! drawn before it and seen through it, and what lies this side is drawn over
//! it. A shot with a pool is the only way to see that division hold.
//!
//! The pool is drawn by the game's own water renderer, from a mesh built
//! directly rather than from a hydrology network: a shot has to frame the same
//! water every run.

use nalgebra::Vector2;

use crate::rendering::water::{BasinVertex, MeshKey, WaterDraw, WaterMesh, WaterScene};
use crate::water::geometry::{SpanChunkCoord, COLUMN_SIZE};
use crate::water::ids::StoreId;

/// The pool's body id, as the renderer sees it.
const POOL_BODY: StoreId = StoreId(0);

/// A rectangular pool at a fixed level, centred on the origin in x and z.
#[derive(Clone, Copy, Debug)]
pub struct ScenePool {
    /// World height of the water surface.
    pub level: f32,

    /// World height of the floor the water rests on. The difference from
    /// `level` is the depth the water shader shades by, so a pool that is too
    /// shallow reads as clear glass rather than as water.
    pub floor: f32,

    /// Half-width of the pool in x and z.
    pub half_extent: f32,
}

impl ScenePool {
    pub fn new(level: f32, floor: f32, half_extent: f32) -> Self {
        Self {
            level,
            floor,
            half_extent,
        }
    }

    /// Columns along each side of the pool.
    fn columns(&self) -> i32 {
        (self.half_extent * 2.0 / COLUMN_SIZE).ceil() as i32
    }
}

impl WaterScene for ScenePool {
    fn mesh_key(&self) -> MeshKey {
        vec![(POOL_BODY, 0)]
    }

    /// One quad per column over the pool's square, on a lattice starting at
    /// its corner.
    fn build_mesh(&self) -> WaterMesh {
        let n = self.columns();
        let mut mesh = WaterMesh::default();
        for k in 0..=n {
            for i in 0..=n {
                mesh.vertices.push(BasinVertex {
                    xz: Vector2::new(
                        -self.half_extent + i as f32 * COLUMN_SIZE,
                        -self.half_extent + k as f32 * COLUMN_SIZE,
                    ),
                    floor: self.floor,
                    swell_share: 1.0,
                });
            }
        }
        let at = |i: i32, k: i32| (k * (n + 1) + i) as u32;
        for k in 0..n {
            for i in 0..n {
                let (a, b, c, d) = (at(i, k), at(i + 1, k), at(i + 1, k + 1), at(i, k + 1));
                mesh.indices.extend_from_slice(&[a, d, c, a, c, b]);
            }
        }
        mesh.draws.push(WaterDraw {
            body: POOL_BODY,
            tile: SpanChunkCoord { x: 0, z: 0 },
            first_index: 0,
            index_count: mesh.indices.len() as u32,
        });
        mesh
    }

    fn level(&self, body: StoreId) -> Option<f32> {
        (body == POOL_BODY).then_some(self.level)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The mesh has to cover the square asked for, or every shot that stands
    /// something half in the water is framed wrong.
    #[test]
    fn the_pool_covers_its_square() {
        let pool = ScenePool::new(0.4, -1.0, 3.0);
        let mesh = pool.build_mesh();
        let xs: Vec<f32> = mesh.vertices.iter().map(|v| v.xz.x).collect();
        let lo = xs.iter().copied().fold(f32::INFINITY, f32::min);
        let hi = xs.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        assert_eq!((lo, hi), (-3.0, 3.0));
        assert_eq!(pool.level(POOL_BODY), Some(0.4));
    }
}
