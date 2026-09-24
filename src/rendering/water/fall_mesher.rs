//! Static meshes for falls: built when a fall is traced, never per frame.
//!
//! ```text
//!   FallPath ──▶ two vertices at each point of the arc, one each side, with
//!                the horizontal direction across the sheet and the seconds
//!                the water has fallen there
//! ```
//!
//! Each draw pushes the sheet's half width and strength from the link's
//! discharge now, so a change in flow needs no new mesh.

use nalgebra::Vector3;

use crate::water::ids::LinkId;
use crate::water::network::FallPath;

use super::vertex::FallVertex;

/// Half width of a sheet per √(m³/s), m: 1 m³/s falls as a 2 m sheet.
const HALF_WIDTH_PER_ROOT_Q: f32 = 1.0;
const MIN_HALF_WIDTH: f32 = 0.1;
const MAX_HALF_WIDTH: f32 = 4.0;

/// Discharge at which a sheet is solid, m³/s; a trickle below it thins.
const SOLID_DISCHARGE: f64 = 0.05;

/// How much wider a sheet is at its foot than at its lip, as a share.
pub const SPREAD: f32 = 0.3;

/// Changes whenever [`build`] would build something different.
pub type FallKey = Vec<(LinkId, u32)>;

/// One fall's draw.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FallDraw {
    pub link: LinkId,
    pub first_index: u32,
    pub index_count: u32,
}

/// Every fall's sheet, in one vertex and index buffer.
#[derive(Debug, Clone, Default)]
pub struct FallMesh {
    pub vertices: Vec<FallVertex>,
    pub indices: Vec<u32>,
    pub draws: Vec<FallDraw>,
}

/// A fall's state as a draw needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FallState {
    /// At the lip, m.
    pub half_width: f32,
    /// 0 to 1: how solid the sheet is.
    pub strength: f32,
}

impl FallState {
    /// A sheet carrying `discharge` m³/s. `None` when nothing falls.
    pub fn carrying(discharge: f64) -> Option<Self> {
        if discharge <= 0.0 {
            return None;
        }
        Some(Self {
            half_width: (HALF_WIDTH_PER_ROOT_Q * (discharge as f32).sqrt())
                .clamp(MIN_HALF_WIDTH, MAX_HALF_WIDTH),
            strength: (discharge / SOLID_DISCHARGE).min(1.0) as f32,
        })
    }
}

/// The key of a set of falls.
pub fn fall_key<'a>(falls: impl Iterator<Item = (LinkId, &'a FallPath)>) -> FallKey {
    falls.map(|(id, f)| (id, f.points.len() as u32)).collect()
}

/// Build the sheet of every fall.
pub fn build<'a>(falls: impl Iterator<Item = (LinkId, &'a FallPath)>) -> FallMesh {
    let mut mesh = FallMesh::default();
    for (link, fall) in falls {
        append_fall(&mut mesh, link, fall);
    }
    mesh
}

fn append_fall(mesh: &mut FallMesh, link: LinkId, fall: &FallPath) {
    let n = fall.points.len();
    if n < 2 {
        return;
    }
    let (first, last) = (fall.points[0], fall.points[n - 1]);
    // Across the sheet: square to its horizontal run, or along x for water
    // dropping straight down.
    let run = Vector3::new(last.x - first.x, 0.0, last.z - first.z);
    let side = run
        .try_normalize(0.05)
        .map_or_else(Vector3::x, |d| Vector3::new(-d.z, 0.0, d.x));
    let total = fall.times[n - 1].max(1e-3);
    let base = mesh.vertices.len() as u32;
    let first_index = mesh.indices.len() as u32;
    for (point, time) in fall.points.iter().zip(&fall.times) {
        for across in [-1.0f32, 1.0] {
            mesh.vertices.push(FallVertex {
                centre: point.coords,
                side,
                across,
                time: *time,
                along: time / total,
            });
        }
    }
    for i in 0..(n as u32 - 1) {
        let a = base + 2 * i;
        mesh.indices
            .extend_from_slice(&[a, a + 1, a + 3, a, a + 3, a + 2]);
    }
    mesh.draws.push(FallDraw {
        link,
        first_index,
        index_count: mesh.indices.len() as u32 - first_index,
    });
}
