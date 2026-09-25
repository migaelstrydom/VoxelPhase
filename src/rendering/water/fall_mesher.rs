//! Static meshes for falls: built when a fall is traced, never per frame.
//!
//! ```text
//!   FallPath ──▶ two vertices at each point of the arc, one each side, with
//!                the horizontal direction across the sheet and the seconds
//!                the water has fallen there
//! ```
//!
//! Each draw pushes the sheet's half width and strength from the link's
//! discharge now, and from the heights where its stores meet (§7.9): how
//! far to lift the arc so it leaves the surface it leaves from, where to cut
//! it off at the water it enters, and whether it falls free and aerated or
//! drops clear into water standing over its lip. A change in flow or level
//! needs no new mesh. An arc is traced to the ground, so the cut, not the
//! mesh, ends the sheet.

use nalgebra::Vector3;

use crate::water::ids::LinkId;
use crate::water::network::{FallPath, Interface, STEP_EPSILON};

use super::vertex::FallVertex;

/// Half width of a sheet per √(m³/s), m: 1 m³/s falls as a 2 m sheet.
const HALF_WIDTH_PER_ROOT_Q: f32 = 1.0;
const MIN_HALF_WIDTH: f32 = 0.1;
const MAX_HALF_WIDTH: f32 = 4.0;

/// Discharge at which a sheet is solid, m³/s; a trickle below it thins.
const SOLID_DISCHARGE: f64 = 0.05;

/// How much wider a sheet is at its foot than at its lip, as a share.
pub const SPREAD: f32 = 0.3;

/// Height over which a jet turns from clear to aerated as the water below
/// drops away from its lip, m.
const AERATION_BAND: f32 = 0.05;

/// Changes whenever [`build`] would build something different.
pub type FallKey = Vec<(LinkId, bool, u32)>;

/// One fall's draw.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FallDraw {
    pub link: LinkId,
    /// The arc water running back over a reversible link falls along.
    pub back: bool,
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
    /// Added to the arc's height, so it leaves the surface it leaves from.
    pub lift: f32,
    /// The top of the sheet, after the lift.
    pub top: f32,
    /// Where it is cut off: the surface of the water it enters.
    pub cut: f32,
    /// 0 to 1: clear where the water below stands over the lip, aerated
    /// where the jet falls free.
    pub aeration: f32,
}

impl FallState {
    /// The sheet along `arc` carrying `discharge` m³/s across `heights`.
    /// `None` when nothing falls, or there is no step to fall down.
    pub fn of(discharge: f64, heights: Interface, arc: &FallPath) -> Option<Self> {
        let start = arc.points.first()?.y;
        if discharge <= 0.0 || heights.step() <= STEP_EPSILON {
            return None;
        }
        Some(Self {
            half_width: (HALF_WIDTH_PER_ROOT_Q * (discharge as f32).sqrt())
                .clamp(MIN_HALF_WIDTH, MAX_HALF_WIDTH),
            strength: (discharge / SOLID_DISCHARGE).min(1.0) as f32,
            lift: heights.upper - start,
            top: heights.upper,
            cut: heights.lower,
            aeration: ((heights.lip - heights.lower) / AERATION_BAND).clamp(0.0, 1.0),
        })
    }
}

/// The key of a set of falls.
pub fn fall_key<'a>(falls: impl Iterator<Item = (LinkId, bool, &'a FallPath)>) -> FallKey {
    falls
        .map(|(id, back, f)| (id, back, f.points.len() as u32))
        .collect()
}

/// Build the sheet of every fall.
pub fn build<'a>(falls: impl Iterator<Item = (LinkId, bool, &'a FallPath)>) -> FallMesh {
    let mut mesh = FallMesh::default();
    for (link, back, fall) in falls {
        append_fall(&mut mesh, link, back, fall);
    }
    mesh
}

fn append_fall(mesh: &mut FallMesh, link: LinkId, back: bool, fall: &FallPath) {
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
    let base = mesh.vertices.len() as u32;
    let first_index = mesh.indices.len() as u32;
    for (point, time) in fall.points.iter().zip(&fall.times) {
        for across in [-1.0f32, 1.0] {
            mesh.vertices.push(FallVertex {
                centre: point.coords,
                side,
                across,
                time: *time,
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
        back,
        first_index,
        index_count: mesh.indices.len() as u32 - first_index,
    });
}
