use std::mem::discriminant;

use nalgebra::Point3;

use super::placer::FootSide;
use super::planned::StepId;

/// What the legs do: the one thing that tells them whether the body has left
/// the ground. The caller maps its own state to it (a character's locomotion
/// state, a creature's grounding); the legs never decide it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LegMode {
    /// The gait: clock, swings, capture point, a plan's footfalls if it has
    /// any.
    Step,
    /// A flight the plan took off on: the landing foot swings onto
    /// `landing`, the other trails. With no landing the feet are held.
    Fly(Option<FlightLanding>),
    /// Nothing under the body the legs answer to: the pose draws the legs.
    Off,
}

impl LegMode {
    /// Whether `other` is the same mode, whatever it carries.
    pub fn same_as(&self, other: &Self) -> bool {
        discriminant(self) == discriminant(other)
    }
}

/// Where a flight comes down: the foot that lands it, where, and when.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightLanding {
    pub side: FootSide,
    /// Where that foot lands, in the world: the sole's contact point.
    pub at: Point3<f32>,
    /// Seconds from the start of the frame until it lands.
    pub in_seconds: f32,
    /// The plan's name for the footfall, handed back when the foot lands on
    /// it.
    pub footfall: Option<StepId>,
}
