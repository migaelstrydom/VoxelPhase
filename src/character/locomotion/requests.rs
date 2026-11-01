use nalgebra::{Point3, Vector3};

/// One of a biped's feet, as a request names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Foot {
    Left,
    Right,
}

/// A takeoff a plan asks for: a hop, or a roll-off at a lip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannedTakeoff {
    /// Planar velocity to leave with, in m/s; it caps the air speed.
    pub planar: Vector3<f32>,
    /// Upward speed to leave with, in m/s. `None` for a roll-off, which
    /// leaves on the climb the body already has.
    pub climb: Option<f32>,
    /// Where the flight comes down. `None` only where the plan's flight runs
    /// past what it planned.
    pub landing: Option<PlannedLanding>,
}

/// Where a Hop comes down: the foot that lands it, where, and when.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannedLanding {
    /// The foot the flight lands on.
    pub foot: Foot,
    /// Where that foot lands, in the world.
    pub at: Point3<f32>,
    /// How long from now it lands, in seconds; negative once that is past.
    pub in_seconds: f32,
    /// The plan's name for that footfall, opaque here: the legs hand it back
    /// when the foot lands on it. `None` for a landing no plan names.
    pub footfall: Option<u32>,
    /// How far, across the ground, the landing may lie from where the body's
    /// centre comes down and still be stood on, in metres: the leg's reach
    /// from the hip, and the hip's offset from the centre.
    pub reach: f32,
}

impl PlannedLanding {
    /// The same landing `dt` seconds on.
    pub fn later(self, dt: f32) -> Self {
        Self {
            in_seconds: self.in_seconds - dt,
            ..self
        }
    }
}
