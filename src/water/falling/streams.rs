//! The falling water in the field this frame, and what it strikes.
//!
//! ```text
//!   faces of awake chunks ──┐  a face pouring from water that `falls` to
//!   boundary faces ─────────┼─▶ the water past it, or off the map: a sheet
//!   sources ────────────────┘   a lip long; a spring: its jet
//!                                  │
//!   strikes(centre, radius) ◀──────┘ each stream crossing the sphere: its
//!                                    velocity there and the area it strikes
//! ```
//!
//! Each sheet is one face's worth of lip, carrying the face's own discharge,
//! so a fall is as many streams as faces it pours over, and a fall down a
//! stepped cliff a stream for every step.

use nalgebra::{Point3, Vector2, Vector3};
use rustc_hash::FxHashSet;
use smallvec::SmallVec;

use crate::water::field::{Axis, WaterField};
use crate::water::geometry::{SpanIndex, SpanRef, Step, COLUMN_SIZE};

use super::lip::{falls, ledge_reach, off_map, thrown};
use super::path::FallPath;

/// Least flow over an edge, per metre of it, that is a falling stream,
/// m²/s.
const MIN_POUR: f32 = 0.005;

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
            min,
            max,
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
        let slab = self.path.at(self.path.time_at(centre.y));
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
        let cells = field.cells();
        for (slot, chunk) in cells.chunks.iter().enumerate() {
            if !chunk.awake {
                continue;
            }
            for face in &chunk.faces {
                if face.q.abs() < MIN_POUR {
                    continue;
                }
                let own = SpanIndex {
                    slot: slot as u32,
                    flat: face.a,
                };
                let (upper, lower, sign) = if face.q > 0.0 {
                    (own, face.b, 1.0)
                } else {
                    (face.b, own, -1.0)
                };
                if let Some(path) = sheet(field, upper, lower, face.axis, sign, face.q.abs()) {
                    streams.push(Stream::new(path, Some(upper), Some(lower)));
                }
            }
            for edge in &chunk.boundary {
                if edge.q < MIN_POUR {
                    continue;
                }
                let upper = SpanIndex {
                    slot: slot as u32,
                    flat: edge.cell,
                };
                let level = cells.surface(upper);
                let below = off_map(level, edge.outside.map(f64::from));
                if !falls(level, cells.bed(upper) as f64, below) {
                    continue;
                }
                let face = axis_vector(edge.axis) * edge.outward;
                let lip = lip_point(field, upper, face, 0.0, level as f32);
                let fall = (level - below) as f32;
                let throw = thrown(face, field.velocity(upper), fall);
                let half_length = 0.5 * COLUMN_SIZE;
                let path = FallPath::sheet(lip, face, throw, fall, half_length, edge.q);
                streams.push(Stream::new(path, Some(upper), None));
            }
        }
        for source in field.sources() {
            let Some(&landing) = source.landing.first() else {
                continue;
            };
            let level = cells.surface(landing) as f32;
            if let Some(path) =
                FallPath::jet(source.origin, source.velocity, source.discharge, level)
            {
                streams.push(Stream::new(path, None, Some(landing)));
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

    /// Where each stream lands.
    pub fn landings(&self) -> impl Iterator<Item = Landing> + '_ {
        self.streams
            .iter()
            .zip(&self.onward)
            .map(|(stream, &onward)| {
                let path = &stream.path;
                let slab = path.at(path.duration());
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

/// The sheet pouring from cell `upper` to `lower` over the face between
/// them on `axis`, facing `sign` along it, carrying `per_metre` m²/s, if
/// the water falls there.
fn sheet(
    field: &WaterField,
    upper: SpanIndex,
    lower: SpanIndex,
    axis: Axis,
    sign: f32,
    per_metre: f32,
) -> Option<FallPath> {
    let cells = field.cells();
    let (level, below) = (cells.surface(upper), cells.surface(lower));
    if !falls(level, cells.bed(upper) as f64, below) {
        return None;
    }
    let face = axis_vector(axis) * sign;
    let step = Step {
        di: face.x as i32,
        dk: face.y as i32,
    };
    // Where the lower column's own floor carries the ledge on, the water
    // leaves where the ledge really ends: the mean over the edge's two ends.
    let lower_span = field.graph().span(span_ref(field, lower));
    let ledge = 0.5 * (cells.bed(upper) + lower_span.bed);
    let reach = edge_end_sides(step)
        .iter()
        .map(|&sides| ledge_reach(step, sides, &lower_span.quarters, ledge))
        .sum::<f32>()
        * 0.5;
    let lip = lip_point(field, upper, face, reach, level as f32);
    let fall = (level - below) as f32;
    let throw = thrown(face, field.velocity(upper), fall);
    Some(FallPath::sheet(
        lip,
        face,
        throw,
        fall,
        0.5 * COLUMN_SIZE,
        per_metre,
    ))
}

/// The middle of the edge of cell `at`'s column facing `face`, moved out
/// `reach` columns, at height `level`.
fn lip_point(
    field: &WaterField,
    at: SpanIndex,
    face: Vector2<f32>,
    reach: f32,
    level: f32,
) -> Point3<f32> {
    let column = span_ref(field, at).column;
    let (cx, cz) = column.centre();
    let middle = Vector2::new(cx, cz) + face * ((0.5 + reach) * COLUMN_SIZE);
    Point3::new(middle.x, level, middle.y)
}

/// The span a cell is.
fn span_ref(field: &WaterField, at: SpanIndex) -> SpanRef {
    let chunk = &field.cells().chunks[at.slot as usize];
    let (local, ordinal) = chunk.column_of(at.flat as usize);
    SpanRef::key(chunk.coord.column(local), ordinal)
}

/// The unit vector along `axis`, x and z.
fn axis_vector(axis: Axis) -> Vector2<f32> {
    match axis {
        Axis::X => Vector2::x(),
        Axis::Z => Vector2::y(),
    }
}

/// The two ends of a column's edge across `step`, as the sides of the
/// neighbouring column they lie on.
fn edge_end_sides(step: Step) -> [(i32, i32); 2] {
    match (step.di, step.dk) {
        (1, _) => [(-1, -1), (-1, 1)],
        (-1, _) => [(1, -1), (1, 1)],
        (_, 1) => [(-1, -1), (1, -1)],
        _ => [(-1, 1), (1, 1)],
    }
}

#[cfg(test)]
mod tests {
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
