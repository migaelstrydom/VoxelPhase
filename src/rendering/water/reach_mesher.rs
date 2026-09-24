//! Static meshes for reaches: built on route or re-route, never per frame.
//!
//! ```text
//!   centreline ──▶ a cross-section of quads every point, out to the design
//!                  top width; each vertex carries the section's bed, the
//!                  design depth there, its distance down the reach, and the
//!                  flow direction
//! ```
//!
//! Each draw pushes the reach's depth scale `(Q/Q_design)^0.6` and its wetted
//! range `[x_t, x_f]`; the shader discards outside the range, and the depth
//! test trims the width at lower flow, where the banks stand above the water.

use nalgebra::Vector2;

use crate::water::ids::StoreId;
use crate::water::network::Reach;

use super::vertex::RiverVertex;

/// Spacing of vertices across a section, m.
const ACROSS: f32 = 0.5;

/// Width beyond the design top width each side, m, so the bank cuts the
/// water rather than the mesh edge.
const BANK_MARGIN: f32 = 0.5;

/// One reach's draw.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RiverDraw {
    pub reach: StoreId,
    pub first_index: u32,
    pub index_count: u32,
}

/// Every reach's surface, in one vertex and index buffer.
#[derive(Debug, Clone, Default)]
pub struct RiverMesh {
    pub vertices: Vec<RiverVertex>,
    pub indices: Vec<u32>,
    pub draws: Vec<RiverDraw>,
}

/// A reach's state as a draw needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RiverState {
    /// Depth now over depth at the design discharge.
    pub depth_scale: f32,
    /// Speed now over speed at the design discharge.
    pub speed_scale: f32,
    /// The wetted range, m from the top of the reach.
    pub tail: f32,
    pub front: f32,
}

impl RiverState {
    pub fn of(reach: &Reach) -> Self {
        let design = reach.rating.at(reach.rating.design());
        let running = reach.running();
        Self {
            depth_scale: if design.depth > 0.0 {
                running.depth / design.depth
            } else {
                0.0
            },
            speed_scale: if design.velocity > 0.0 {
                running.velocity / design.velocity
            } else {
                0.0
            },
            tail: reach.tail,
            front: reach.front,
        }
    }
}

/// Build the surface of every reach.
pub fn build<'a>(reaches: impl Iterator<Item = (StoreId, &'a Reach)>) -> RiverMesh {
    let mut mesh = RiverMesh::default();
    for (id, reach) in reaches {
        append_reach(&mut mesh, id, reach);
    }
    mesh
}

fn append_reach(mesh: &mut RiverMesh, id: StoreId, reach: &Reach) {
    let design = reach.rating.at(reach.rating.design());
    let half = design.top_width * 0.5 + BANK_MARGIN;
    let steps = (half / ACROSS).ceil() as i32;
    let row = (2 * steps + 1) as u32;
    let first_index = mesh.indices.len() as u32;
    let base = mesh.vertices.len() as u32;
    let line = &reach.centreline;
    for (i, point) in line.points.iter().enumerate() {
        let tangent = line.tangents[i];
        let across = Vector2::new(-tangent.y, tangent.x);
        for s in -steps..=steps {
            let offset = across * (s as f32 * ACROSS);
            mesh.vertices.push(RiverVertex {
                xz: Vector2::new(point.x + offset.x, point.z + offset.y),
                bed: point.y,
                depth: design.depth,
                along: line.distance[i],
                flow: tangent * design.velocity,
            });
        }
    }
    for i in 0..line.points.len().saturating_sub(1) as u32 {
        for s in 0..row - 1 {
            let a = base + i * row + s;
            let b = a + 1;
            let c = a + row + 1;
            let d = a + row;
            mesh.indices.extend_from_slice(&[a, d, c, a, c, b]);
        }
    }
    mesh.draws.push(RiverDraw {
        reach: id,
        first_index,
        index_count: mesh.indices.len() as u32 - first_index,
    });
}
