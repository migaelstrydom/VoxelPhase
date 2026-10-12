//! The path of falling water: from a lip or a spring's mouth, ballistic,
//! down to the water it lands in, and the body of water along it.
//!
//! ```text
//!   origin ●━━━━╮            a sheet: a slab a lip long, from the lip's
//!                ╲           surface down, as thick as its flow per metre
//!                 ╲          over its speed
//!                  ╲         a jet: an ellipse round its arc, carrying its
//!   landing ~~~~~~~~●~~      discharge at its speed, spreading as it falls
//! ```

use nalgebra::{Point3, Vector2, Vector3};

use super::lip::GRAVITY;

/// Thinnest and thickest a falling sheet is taken to be, m.
const MIN_SHEET_THICKNESS: f32 = 0.01;
const MAX_SHEET_THICKNESS: f32 = 1.0;
/// Least and most half-width a jet is taken to be, m.
const MIN_JET_RADIUS: f32 = 0.04;
const MAX_JET_RADIUS: f32 = 0.6;
/// Least half-thickness a jet is taken to be, m.
const MIN_JET_HALF_THICKNESS: f32 = 0.02;
/// How much wider a jet is where it lands than a round jet of the same
/// section would be, as a share: it spreads across its flight as it falls,
/// and flattens to keep its section.
const JET_SPREAD: f32 = 0.8;

/// How the water along a path is shaped across it.
#[derive(Debug, Clone, Copy)]
enum Section {
    /// A sheet off a straight lip `half_length` either side of the path,
    /// carrying `per_metre` m²/s.
    Sheet { half_length: f32, per_metre: f32 },
    /// A spring's jet carrying `discharge` m³/s.
    Jet { discharge: f32 },
}

/// The water along a path at one moment of its fall: a slab square to its
/// flight, `half_length` along `across` and `half_thickness` along `over`.
#[derive(Debug, Clone, Copy)]
pub struct Slab {
    /// The middle of the water's section.
    pub centre: Point3<f32>,
    /// How fast the water moves there.
    pub velocity: Vector3<f32>,
    /// Across the flight, level.
    pub across: Vector3<f32>,
    /// Square to the flight and to `across`, upwards.
    pub over: Vector3<f32>,
    pub half_length: f32,
    pub half_thickness: f32,
}

/// One stream of falling water.
#[derive(Debug, Clone, Copy)]
pub struct FallPath {
    /// Where it leaves: a sheet's lip, at the middle of its surface; a jet's
    /// mouth, at the middle of its section.
    origin: Point3<f32>,
    /// Its velocity there.
    velocity: Vector3<f32>,
    /// Across its flight, level.
    across: Vector3<f32>,
    /// How far below its origin it lands, m.
    fall: f32,
    /// How long its water falls, s.
    duration: f32,
    section: Section,
}

impl FallPath {
    /// The sheet off a lip whose middle is at `lip`, facing `face`, thrown
    /// out `throw` (`lip::thrown`) as it falls `fall` metres, `half_length`
    /// either side, carrying `per_metre` m²/s.
    pub fn sheet(
        lip: Point3<f32>,
        face: Vector2<f32>,
        throw: Vector2<f32>,
        fall: f32,
        half_length: f32,
        per_metre: f32,
    ) -> Self {
        let fall = fall.max(0.0);
        let duration = (2.0 * fall / GRAVITY).sqrt();
        let level = if duration > 0.0 {
            throw / duration
        } else {
            Vector2::zeros()
        };
        Self {
            origin: lip,
            velocity: Vector3::new(level.x, 0.0, level.y),
            across: Vector3::new(-face.y, 0.0, face.x),
            fall,
            duration,
            section: Section::Sheet {
                half_length,
                per_metre,
            },
        }
    }

    /// The jet out of `origin` at `velocity` carrying `discharge` m³/s,
    /// landing in water at `level`; none if it does not come out above it.
    pub fn jet(
        origin: Point3<f32>,
        velocity: Vector3<f32>,
        discharge: f32,
        level: f32,
    ) -> Option<Self> {
        let fall = origin.y - level;
        if fall <= 0.0 {
            return None;
        }
        let duration = (velocity.y + (velocity.y.powi(2) + 2.0 * GRAVITY * fall).sqrt()) / GRAVITY;
        let flight = Vector2::new(velocity.x, velocity.z)
            .try_normalize(1.0e-4)
            .unwrap_or_else(Vector2::x);
        Some(Self {
            origin,
            velocity,
            across: Vector3::new(-flight.y, 0.0, flight.x),
            fall,
            duration,
            section: Section::Jet { discharge },
        })
    }

    pub fn origin(&self) -> Point3<f32> {
        self.origin
    }

    pub fn fall(&self) -> f32 {
        self.fall
    }

    /// How much water it carries, m³/s.
    pub fn discharge(&self) -> f32 {
        match self.section {
            Section::Sheet {
                half_length,
                per_metre,
            } => 2.0 * half_length * per_metre,
            Section::Jet { discharge } => discharge,
        }
    }

    /// How long its water falls, s.
    pub fn duration(&self) -> f32 {
        self.duration
    }

    /// The water `t` seconds after it leaves, clamped to its flight.
    pub fn at(&self, t: f32) -> Slab {
        let t = t.clamp(0.0, self.duration);
        let share = if self.duration > 0.0 {
            t / self.duration
        } else {
            0.0
        };
        let point = self.origin + self.velocity * t - Vector3::y() * (0.5 * GRAVITY * t * t);
        let velocity = self.velocity - Vector3::y() * (GRAVITY * t);
        let tangent = velocity.try_normalize(1.0e-6).unwrap_or_else(Vector3::x);
        let over = tangent.cross(&self.across);
        let over = if over.y < 0.0 { -over } else { over };
        let speed = velocity.norm().max(1.0e-3);
        let (centre, half_length, half_thickness) = match self.section {
            Section::Sheet {
                half_length,
                per_metre,
            } => {
                // Its flow per metre keeps its rate as it speeds up: it
                // thins. The origin is its surface; its body lies under it.
                let thickness = (per_metre / speed).clamp(MIN_SHEET_THICKNESS, MAX_SHEET_THICKNESS);
                (
                    point - over * (0.5 * thickness),
                    half_length,
                    0.5 * thickness,
                )
            }
            Section::Jet { discharge } => {
                let section = discharge / speed;
                let round = (section / std::f32::consts::PI).sqrt();
                let half_width =
                    (round * (1.0 + JET_SPREAD * share)).clamp(MIN_JET_RADIUS, MAX_JET_RADIUS);
                let half_thickness = (section / (std::f32::consts::PI * half_width)).clamp(
                    MIN_JET_HALF_THICKNESS,
                    half_width.max(MIN_JET_HALF_THICKNESS),
                );
                (point, half_width, half_thickness)
            }
        };
        Slab {
            centre,
            velocity,
            across: self.across,
            over,
            half_length,
            half_thickness,
        }
    }

    /// When the water's origin line passes height `y` on its way down,
    /// clamped to its flight.
    pub fn time_at(&self, y: f32) -> f32 {
        let vy = self.velocity.y;
        let reach = vy * vy + 2.0 * GRAVITY * (self.origin.y - y);
        if reach < 0.0 {
            return vy / GRAVITY;
        }
        ((vy + reach.sqrt()) / GRAVITY).clamp(0.0, self.duration)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sheet_thins_as_it_falls_and_lands_thrown_out() {
        let path = FallPath::sheet(
            Point3::new(0.0, 10.0, 0.0),
            Vector2::x(),
            Vector2::new(1.0, 0.0),
            4.0,
            0.25,
            1.5,
        );
        let (top, foot) = (path.at(0.0), path.at(path.duration()));
        assert!(foot.half_thickness < top.half_thickness);
        let landed = foot.centre + foot.over * foot.half_thickness;
        assert!((landed - Point3::new(1.0, 6.0, 0.0)).norm() < 1.0e-4);
        assert!((path.time_at(6.0) - path.duration()).abs() < 1.0e-5);
    }

    #[test]
    fn a_jet_carries_its_discharge_through_its_section() {
        let path = FallPath::jet(
            Point3::new(0.0, 10.0, 0.0),
            Vector3::new(7.0, 0.0, 0.0),
            3.0,
            4.0,
        )
        .unwrap();
        for t in [0.0, 0.5 * path.duration(), path.duration()] {
            let slab = path.at(t);
            let area = std::f32::consts::PI * slab.half_length * slab.half_thickness;
            assert!((area * slab.velocity.norm() - 3.0).abs() < 1.0e-3);
        }
    }
}
