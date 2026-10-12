//! A source's jet: a tube of water along its ballistic arc from where its
//! water comes out down to the water it lands in. Its cross-section carries
//! the discharge at the speed the water has there, so it narrows as it
//! speeds up; it also spreads across its flight and flattens as it falls.
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

use crate::water::field::Source;

use super::vertex::RiverVertex;

/// Rows of vertices along a jet's arc, after the first. Fixed, so a jet
/// keeps its shape as the water it lands in rises and falls.
pub(super) const JET_ROWS: u32 = 20;
/// Vertices round each row.
pub(super) const JET_RING: u32 = 10;

/// Gravity on a jet, m/s².
const GRAVITY: f32 = 9.81;
/// Least and most half-width a jet is drawn, m.
const MIN_RADIUS: f32 = 0.04;
const MAX_RADIUS: f32 = 0.6;
/// Least half-thickness a jet is drawn, m.
const MIN_HALF_THICKNESS: f32 = 0.02;
/// How much wider a jet is where it lands than a round jet of the same
/// section would be, as a share: it spreads across its flight as it falls,
/// and flattens to keep its section.
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
            depth: (source.discharge * DEPTH_PER_DISCHARGE).min(MAX_DEPTH),
        })
    }

    /// The vertex `around` of `JET_RING` round row `row`, from the top of
    /// the jet.
    pub fn vertex(&self, row: u32, around: u32) -> RiverVertex {
        let (origin, velocity) = (self.source.origin, self.source.velocity);
        let share = row as f32 / JET_ROWS as f32;
        let t = self.duration * share;
        let at = origin.coords + velocity * t - Vector3::y() * (0.5 * GRAVITY * t * t);
        let moving = velocity - Vector3::y() * (GRAVITY * t);
        // Its section carries the discharge at the speed it has here.
        let section = self.source.discharge / moving.norm().max(1.0e-3);
        let round = (section / std::f32::consts::PI).sqrt();
        let half_width = (round * (1.0 + SPREAD * share)).clamp(MIN_RADIUS, MAX_RADIUS);
        let half_thickness = (section / (std::f32::consts::PI * half_width))
            .clamp(MIN_HALF_THICKNESS, half_width.max(MIN_HALF_THICKNESS));
        // Square to the flight: across it, level, and over it, upwards.
        let across = Vector3::new(self.edge.x, 0.0, self.edge.y);
        let tangent = moving.try_normalize(1.0e-6).unwrap_or_else(Vector3::x);
        let over = tangent.cross(&across);
        let over = if over.y < 0.0 { -over } else { over };
        let angle = TAU * around as f32 / JET_RING as f32 + 0.25 * TAU;
        let (sin, cos) = angle.sin_cos();
        let offset = across * (cos * half_width) + over * (sin * half_thickness);
        let normal = (across * (cos / half_width) + over * (sin / half_thickness)).normalize();
        let point = at + offset;
        let mouth = Vector2::new(origin.x, origin.z).dot(&self.edge);
        RiverVertex {
            xz: Vector2::new(point.x, point.z),
            surface: point.y,
            depth: self.depth,
            flow: Vector2::new(velocity.x, velocity.z),
            face: self.face,
            sheet: Vector4::new(origin.y - at.y, self.fall, 0.0, mouth + cos * half_width),
            normal,
            fray: 0.0,
        }
    }
}
