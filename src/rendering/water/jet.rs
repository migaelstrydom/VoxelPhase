//! A source's jet: a tube of water round its path (`FallPath::jet`) from
//! where its water comes out down to the water it lands in, the section the
//! physics sees: carrying the discharge at the speed the water has there,
//! spreading across its flight and flattening as it falls.
//!
//! ```text
//!   origin ●──╮   row 0          each row: a ring of JET_RING vertices round
//!              ╲                 the arc, an ellipse square to the flight;
//!               ╲  row k         the sheet shader sees (drop, fall, 0, along)
//!                ╲               as on any sheet, `along` across the flight
//!   landing ~~~~~~●~~  row JET_ROWS, at the landing cell's surface
//! ```

use std::f32::consts::TAU;

use nalgebra::{Vector2, Vector3, Vector4};

use crate::water::falling::FallPath;
use crate::water::field::Source;

use super::nappe::{PLUNGE_LIFT, PLUNGE_START};
use super::vertex::RiverVertex;

/// Rows of vertices along a jet's arc, after the first. Fixed, so a jet
/// keeps its shape as the water it lands in rises and falls.
pub(super) const JET_ROWS: u32 = 20;
/// Vertices round each row.
pub(super) const JET_RING: u32 = 10;

/// Corners round the foam where a jet lands, and how far out past its
/// edge it reaches, m.
pub(super) const PLUNGE_CORNERS: u32 = 8;
const PLUNGE_MARGIN: f32 = 0.6;

/// Deepest a jet is drawn, m. A sheet's depth is how much of a curtain it
/// is; a jet's is its own thickness, so a small jet is as solid as a big
/// one.
const MAX_DEPTH: f32 = 0.3;

/// One source's jet down to a landing surface.
pub(super) struct Jet {
    path: FallPath,
    /// The way it flies, level.
    face: Vector2<f32>,
    /// Its flow, level, as every vertex carries it.
    flow: Vector2<f32>,
}

impl Jet {
    /// The jet of `source` landing in water at `level`; none if it does not
    /// come out above it.
    pub fn new(source: &Source, level: f32) -> Option<Self> {
        let path = FallPath::jet(source.origin, source.velocity, source.discharge, level)?;
        let flow = Vector2::new(source.velocity.x, source.velocity.z);
        Some(Self {
            path,
            face: flow.try_normalize(1.0e-4).unwrap_or_else(Vector2::x),
            flow,
        })
    }

    /// The vertex `around` of `JET_RING` round row `row`, from the top of
    /// the jet.
    pub fn vertex(&self, row: u32, around: u32) -> RiverVertex {
        let slab = self
            .path
            .at(self.path.duration() * row as f32 / JET_ROWS as f32);
        let angle = TAU * around as f32 / JET_RING as f32 + 0.25 * TAU;
        let (sin, cos) = angle.sin_cos();
        let offset =
            slab.across * (cos * slab.half_length) + slab.over * (sin * slab.half_thickness);
        let normal = (slab.across * (cos / slab.half_length)
            + slab.over * (sin / slab.half_thickness))
            .normalize();
        let point = slab.centre + offset;
        let origin = self.path.origin();
        let edge = Vector2::new(slab.across.x, slab.across.z);
        let mouth = Vector2::new(origin.x, origin.z).dot(&edge);
        RiverVertex {
            xz: Vector2::new(point.x, point.z),
            surface: point.y,
            depth: (2.0 * slab.half_thickness).min(MAX_DEPTH),
            flow: self.flow,
            face: self.face,
            sheet: Vector4::new(
                origin.y - slab.centre.y,
                self.path.fall(),
                0.0,
                mouth + cos * slab.half_length,
            ),
            normal,
            fray: 0.0,
        }
    }

    /// A vertex of the foam where the jet lands, lying on the water: its
    /// middle for `corner` 0, else corner `corner` of `PLUNGE_CORNERS` round
    /// it, as far out as the jet is wide and `PLUNGE_MARGIN` more. Its drop
    /// is its fall plus how far out it is as a share of that, past the foot.
    pub fn plunge(&self, corner: u32) -> RiverVertex {
        let slab = self.path.at(self.path.duration());
        let landed = slab.centre + slab.over * slab.half_thickness;
        let (share, offset) = if corner == 0 {
            (PLUNGE_START, Vector2::zeros())
        } else {
            let angle = TAU * (corner - 1) as f32 / PLUNGE_CORNERS as f32;
            let (sin, cos) = angle.sin_cos();
            let reach = slab.half_length + PLUNGE_MARGIN;
            let edge = Vector2::new(slab.across.x, slab.across.z);
            (1.0, (edge * cos + self.face * sin) * reach)
        };
        let fall = self.path.fall();
        RiverVertex {
            xz: Vector2::new(landed.x, landed.z) + offset,
            surface: landed.y + PLUNGE_LIFT,
            depth: (2.0 * slab.half_thickness).min(MAX_DEPTH),
            flow: self.flow,
            face: self.face,
            sheet: Vector4::new(fall + share, fall, 0.0, 0.0),
            normal: Vector3::y(),
            fray: 0.0,
        }
    }
}
