//! The falling water in the field this frame, and what it strikes.
//!
//! ```text
//!   the field's falls ──────┐  each a sheet a lip long, from its lip along
//!   springs and pumps ──────┴─▶ its arc to where it comes down; a spring
//!                               or pump: its jet
//!                                  │
//!   strikes(centre, radius) ◀──────┘ each stream crossing the sphere: its
//!                                    velocity there and the area it strikes
//! ```
//!
//! Each sheet is one fall (`field::Fall`): one face's worth of lip, carrying
//! the face's own discharge along the arc the field found for it, so what
//! strikes a body is the water that is drawn and that lands.

use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashSet;
use smallvec::SmallVec;

use crate::water::field::WaterField;
use crate::water::geometry::{SpanIndex, COLUMN_SIZE};

use super::path::FallPath;

/// How much of the momentum it carries falling water gives up to what it
/// strikes: it stops along its path and runs off sideways, so nearly all.
const STRIKE_DRAG: f32 = 1.0;

/// Samples along a stream's path its bounds are taken over.
const BOUND_SAMPLES: u32 = 4;

/// Falling water crossing a body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Strike {
    /// How fast the water moves there, m/s.
    pub velocity: Vector3<f32>,
    /// The area of the body's section it strikes, m².
    pub area: f32,
    /// The middle of where it strikes, in the body's section through the
    /// sphere's centre: off the centre, it turns the body.
    pub at: Point3<f32>,
}

impl Strike {
    /// The drag it puts on a body moving at `velocity` through water of
    /// `density`, per unit velocity relative to the water, N·s/m: the
    /// momentum flux through the struck area, ρ C A |u − v| (u − v).
    pub fn drag_coeff(&self, velocity: Vector3<f32>, density: f32) -> f32 {
        density * STRIKE_DRAG * self.area * (self.velocity - velocity).norm()
    }
}

/// Where a stream lands in the water below.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Landing {
    /// The middle of where it meets the water.
    pub at: Point3<f32>,
    /// How fast its water moves as it lands, m/s.
    pub velocity: Vector3<f32>,
    /// Across it, level, and how far either side of `at` it reaches, m.
    pub across: Vector3<f32>,
    pub half_length: f32,
    /// How much water lands, m³/s.
    pub discharge: f32,
    /// Whether the water it lands in falls on again, as on a step down a
    /// cliff: it lands on a film, not in a pool.
    pub onward: bool,
}

/// One stream and the box its water stays in.
#[derive(Debug, Clone, Copy)]
struct Stream {
    path: FallPath,
    /// The cell it falls from, and the one it lands in: none for a jet's
    /// mouth, or off the map.
    from: Option<SpanIndex>,
    into: Option<SpanIndex>,
    /// When its water meets the world, s after it leaves: the end of its
    /// path, or sooner where a wall stands in its way.
    lands: f32,
    /// The stretch of its path that holds water, as s after leaving: all
    /// of it, but for a fall just begun, whose water has not yet got far,
    /// or one spent, whose last water has left the lip behind.
    flying: (f32, f32),
    min: Point3<f32>,
    max: Point3<f32>,
}

impl Stream {
    fn new(path: FallPath, from: Option<SpanIndex>, into: Option<SpanIndex>) -> Self {
        let (mut min, mut max) = (path.origin(), path.origin());
        for k in 0..=BOUND_SAMPLES {
            let slab = path.at(path.duration() * k as f32 / BOUND_SAMPLES as f32);
            let reach = Vector3::repeat(slab.half_length + 2.0 * slab.half_thickness);
            min = min.inf(&(slab.centre - reach));
            max = max.sup(&(slab.centre + reach));
        }
        Self {
            path,
            from,
            into,
            lands: path.duration(),
            flying: (0.0, f32::INFINITY),
            min,
            max,
        }
    }

    /// The same stream, holding water only over `flying` (s after leaving).
    fn flying(self, flying: (f32, f32)) -> Self {
        Self { flying, ..self }
    }

    /// Whether its water is meeting the world now.
    fn landing(&self) -> bool {
        const NEAR: f32 = 1.0e-3;
        self.flying.0 <= self.lands + NEAR && self.lands <= self.flying.1 + NEAR
    }

    /// The same stream, meeting the world as it passes height `y`.
    fn landing_at(self, y: f32) -> Self {
        Self {
            lands: self.path.time_at(y),
            ..self
        }
    }

    /// The area of a sphere's section this stream strikes, if it crosses
    /// it. The sphere stands in for a square of its own area square to the
    /// flight, overlapped with the stream's section where the stream is at
    /// the sphere's height.
    fn strike(&self, centre: Point3<f32>, radius: f32) -> Option<Strike> {
        let reach = Vector3::repeat(radius);
        let (low, high) = (centre - reach, centre + reach);
        if (0..3).any(|axis| low[axis] > self.max[axis] || high[axis] < self.min[axis]) {
            return None;
        }
        let passing = self.path.time_at(centre.y);
        if passing < self.flying.0 || passing > self.flying.1 {
            return None;
        }
        let slab = self.path.at(passing);
        let half_side = 0.5 * std::f32::consts::PI.sqrt() * radius;
        let offset = centre - slab.centre;
        let tangent = slab.velocity.try_normalize(1.0e-6)?;
        if offset.dot(&tangent).abs() > half_side {
            return None;
        }
        // The span of the square and the stream's section that overlap,
        // along each axis of the section, from the sphere's centre.
        let overlap = |at: f32, half: f32| {
            let (low, high) = ((at - half_side).max(-half), (at + half_side).min(half));
            ((high - low).max(0.0), 0.5 * (low + high) - at)
        };
        let (along, along_middle) = overlap(offset.dot(&slab.across), slab.half_length);
        let (over, over_middle) = overlap(offset.dot(&slab.over), slab.half_thickness);
        let area = along * over;
        (area > 0.0).then_some(Strike {
            velocity: slab.velocity,
            area,
            at: centre + slab.across * along_middle + slab.over * over_middle,
        })
    }
}

/// The field's falling water as it stands after a step.
#[derive(Debug, Clone, Default)]
pub struct FallingWater {
    streams: Vec<Stream>,
    /// For each stream, whether the water it lands in falls on again.
    onward: Vec<bool>,
}

impl FallingWater {
    /// Every stream of falling water in `field`.
    pub fn build(field: &WaterField) -> Self {
        let mut streams = Vec::new();
        for fall in field.falls().iter() {
            let path = FallPath::sheet(
                fall.lip,
                fall.face(),
                fall.throw(),
                fall.height(),
                0.5 * COLUMN_SIZE,
                fall.pour,
            );
            let stream = Stream::new(path, Some(fall.from), fall.into)
                .landing_at(fall.landing.y)
                .flying(fall.flying());
            streams.push(stream);
        }
        let cells = field.cells();
        for jet in field.jets() {
            let level = cells.surface(jet.landing) as f32;
            if let Some(path) = FallPath::jet(jet.origin, jet.velocity, jet.discharge, level) {
                streams.push(Stream::new(path, None, Some(jet.landing)));
            }
        }
        let falling_from: FxHashSet<SpanIndex> = streams.iter().filter_map(|s| s.from).collect();
        let onward = streams
            .iter()
            .map(|s| s.into.is_some_and(|c| falling_from.contains(&c)))
            .collect();
        Self { streams, onward }
    }

    /// How many streams of falling water there are.
    pub fn len(&self) -> usize {
        self.streams.len()
    }

    pub fn is_empty(&self) -> bool {
        self.streams.is_empty()
    }

    /// Where each stream whose water is coming down now lands.
    pub fn landings(&self) -> impl Iterator<Item = Landing> + '_ {
        self.streams
            .iter()
            .zip(&self.onward)
            .filter(|(stream, _)| stream.landing())
            .map(|(stream, &onward)| {
                let path = &stream.path;
                let slab = path.at(stream.lands);
                Landing {
                    at: slab.centre + slab.over * slab.half_thickness,
                    velocity: slab.velocity,
                    across: slab.across,
                    half_length: slab.half_length,
                    discharge: path.discharge(),
                    onward,
                }
            })
    }

    /// The falling water striking a sphere at `centre` of `radius`.
    pub fn strikes(&self, centre: Point3<f32>, radius: f32) -> SmallVec<[Strike; 4]> {
        self.streams
            .iter()
            .filter_map(|stream| stream.strike(centre, radius))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector2;

    use super::*;

    fn curtain() -> Stream {
        // A sheet off a lip at x = 0, 10 m up, falling 4 m, carrying
        // 1.5 m²/s; its foot lands 1 m out.
        let path = FallPath::sheet(
            Point3::new(0.0, 10.0, 0.0),
            Vector2::x(),
            Vector2::new(1.0, 0.0),
            4.0,
            0.25,
            1.5,
        );
        Stream::new(path, None, None)
    }

    #[test]
    fn a_body_in_the_curtain_is_struck_downwards() {
        let stream = curtain();
        // 3 m down the water is 0.61 s out, 0.61 × 1.09 m/s ahead.
        let at = stream.path.at(stream.path.time_at(7.0)).centre;
        let strike = stream.strike(at, 0.3).expect("in the curtain");
        assert!(strike.velocity.y < -7.0);
        assert!(strike.area > 0.0 && strike.area < 0.5 * 0.6);
    }

    #[test]
    fn a_body_wider_than_the_stream_is_struck() {
        // A narrow jet through the middle of a crate twice its width.
        let path = FallPath::jet(
            Point3::new(0.0, 4.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            0.3,
            0.0,
        )
        .unwrap();
        let stream = Stream::new(path, None, None);
        let through = path.at(path.time_at(1.5)).centre;
        assert!(stream.strike(through, 0.52).is_some());
    }

    #[test]
    fn a_body_reaching_past_its_end_is_struck_on_the_side_it_reaches_in() {
        let stream = curtain();
        let middle = stream.path.at(stream.path.time_at(7.0)).centre;
        // Half a sphere past the curtain's end, along the lip (z).
        let centre = middle + Vector3::z() * 0.3;
        let strike = stream
            .strike(centre, 0.3)
            .expect("its near side is in the curtain");
        assert!(strike.at.z < centre.z - 0.05, "struck at {}", strike.at);
    }

    #[test]
    fn a_body_beside_or_behind_it_is_not() {
        let stream = curtain();
        let at = stream.path.at(stream.path.time_at(7.0)).centre;
        assert!(
            stream.strike(at + Vector3::z(), 0.3).is_none(),
            "past its end"
        );
        assert!(
            stream.strike(at - Vector3::x() * 0.6, 0.3).is_none(),
            "behind"
        );
        assert!(
            stream.strike(Point3::new(1.5, 10.0, 0.0), 0.3).is_none(),
            "ahead of the lip"
        );
        assert!(
            stream.strike(Point3::new(1.0, 4.0, 0.0), 0.3).is_none(),
            "under the pool"
        );
    }

    #[test]
    fn its_drag_is_the_momentum_flux_through_the_struck_area() {
        let strike = Strike {
            velocity: Vector3::new(0.0, -8.0, 0.0),
            area: 0.1,
            at: Point3::origin(),
        };
        let drag = strike.drag_coeff(Vector3::zeros(), 1000.0);
        assert!((drag * 8.0 - 1000.0 * 0.1 * 64.0).abs() < 1.0e-3);
    }
}
