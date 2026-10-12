//! A source's jet, drawn as a falling sheet: a body of water along its
//! ballistic arc from where its water comes out down to the water it lands
//! in, its faces square to the way it flies (`nappe::layered`), widening and
//! thinning as it falls.
//!
//! ```text
//!   origin ●──╮   row 0          each row of each face: two vertices, a side apart
//!              ╲                 across the flight; the sheet shader sees
//!               ╲  row k         (drop, fall, side, along) as on any sheet
//!                ╲
//!   landing ~~~~~~●~~  row JET_ROWS, at the landing cell's surface
//! ```

use nalgebra::{Vector2, Vector3, Vector4};

use crate::water::field::Source;

use super::nappe::{layered, Face};
use super::vertex::RiverVertex;

/// Rows of vertices along a jet's arc, after the first. Fixed, so a jet
/// keeps its shape as the water it lands in rises and falls.
pub(super) const JET_ROWS: u32 = 20;

/// Gravity on a jet, m/s².
const GRAVITY: f32 = 9.81;
/// A jet's width where it comes out, per square root of its discharge in
/// m³/s, m; and the least and most it is.
const WIDTH: f32 = 0.35;
const MIN_WIDTH: f32 = 0.15;
const MAX_WIDTH: f32 = 1.0;
/// How much wider a jet is where it lands than where it comes out, as a
/// share of its width there: it spreads as it falls.
const SPREAD: f32 = 0.8;
/// How deep a jet is drawn per m³/s, and at most, m: a sheet's depth is how
/// much of a curtain it is.
const DEPTH_PER_DISCHARGE: f32 = 0.1;
const MAX_DEPTH: f32 = 0.3;

/// One source's jet down to a landing surface.
pub(super) struct Jet<'a> {
    source: &'a Source,
    /// How far it falls, m.
    fall: f32,
    /// How long it flies, s.
    duration: f32,
    /// The way it flies, level, and across it.
    face: Vector2<f32>,
    edge: Vector2<f32>,
    width: f32,
    depth: f32,
}

impl<'a> Jet<'a> {
    /// The jet of `source` landing in water at `level`; none if it does not
    /// come out above it.
    pub fn new(source: &'a Source, level: f32) -> Option<Self> {
        let (origin, velocity) = (source.origin, source.velocity);
        let fall = origin.y - level;
        if fall <= 0.0 {
            return None;
        }
        let duration = (velocity.y + (velocity.y.powi(2) + 2.0 * GRAVITY * fall).sqrt()) / GRAVITY;
        let flight = Vector2::new(velocity.x, velocity.z);
        let face = if flight.norm() > 1.0e-4 {
            flight.normalize()
        } else {
            Vector2::new(1.0, 0.0)
        };
        Some(Self {
            source,
            fall,
            duration,
            face,
            edge: Vector2::new(-face.y, face.x),
            width: (WIDTH * source.discharge.sqrt()).clamp(MIN_WIDTH, MAX_WIDTH),
            depth: (source.discharge * DEPTH_PER_DISCHARGE).min(MAX_DEPTH),
        })
    }

    /// The vertex of `face` on `side` (-1 or 1 across the flight) of row
    /// `row`.
    pub fn vertex(&self, row: u32, side: f32, face: Face) -> RiverVertex {
        let (origin, velocity) = (self.source.origin, self.source.velocity);
        let share = row as f32 / JET_ROWS as f32;
        let t = self.duration * share;
        let at = origin.coords + velocity * t - Vector3::y() * (0.5 * GRAVITY * t * t);
        let half = 0.5 * self.width * (1.0 + SPREAD * share);
        let mouth = Vector2::new(origin.x, origin.z).dot(&self.edge);
        let moving = velocity - Vector3::y() * (GRAVITY * t);
        // Its discharge through the width it has spread to, at its speed.
        let thickness = self.source.discharge / (2.0 * half * moving.norm().max(1.0e-3));
        let top = RiverVertex {
            xz: Vector2::new(at.x, at.z) + self.edge * (side * half),
            surface: at.y,
            depth: self.depth,
            flow: Vector2::new(velocity.x, velocity.z),
            face: self.face,
            sheet: Vector4::new(origin.y - at.y, self.fall, side, side * half + mouth),
            normal: Vector3::y(),
            fray: 1.0,
        };
        layered(top, moving, thickness, face)
    }
}
