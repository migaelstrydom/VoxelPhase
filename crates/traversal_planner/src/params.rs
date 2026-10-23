//! What the body and its legs can do: the limits every plan keeps within.

use serde::{Deserialize, Serialize};

/// The capsule that is the body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyDims {
    /// Capsule radius, in metres.
    pub radius: f32,
    /// Capsule height, end to end, in metres.
    pub height: f32,
    /// Steepest body floor, in degrees from level, the capsule can be driven
    /// over on its contacts. Steeper, as on a step's rounded lip or out of a
    /// wide gap, it has to fly.
    pub max_ground_slope_degrees: f32,
}

impl BodyDims {
    /// Height of the top sphere's centre above the bottom sphere's.
    pub fn spine(&self) -> f32 {
        self.height - 2.0 * self.radius
    }
}

impl Default for BodyDims {
    /// The player's capsule: 0.5 m across and 1 m tall.
    fn default() -> Self {
        Self {
            radius: 0.25,
            height: 1.0,
            max_ground_slope_degrees: 40.0,
        }
    }
}

/// A foot, and how far it may land from where it naturally falls.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FootDims {
    /// Radius of the foothold casts, in metres: a sole's size.
    pub radius: f32,
    /// Offset across the path where each foot lands on open ground, in metres.
    pub stance_half_width: f32,
    /// How far past its natural offset a foot may land outward, in metres.
    pub reach_out: f32,
    /// How far inward of its natural offset a foot may land, in metres; past
    /// `stance_half_width` it crosses over the other foot's side.
    pub reach_in: f32,
    /// Steepest ground, in degrees from level, a foot can stand on.
    pub max_slope_degrees: f32,
}

impl Default for FootDims {
    fn default() -> Self {
        Self {
            radius: 0.05,
            stance_half_width: 0.1,
            reach_out: 0.15,
            reach_in: 0.15,
            max_slope_degrees: 50.0,
        }
    }
}

/// How far one footfall may be from the last.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StrideLimits {
    /// Shortest step along the path, in metres.
    pub min_length: f32,
    /// Longest step along the path, in metres: a running stride.
    pub max_length: f32,
    /// Highest a footfall may be above the last, in metres.
    pub max_rise: f32,
    /// Lowest a footfall may be below the last, in metres.
    pub max_drop: f32,
}

impl Default for StrideLimits {
    fn default() -> Self {
        Self {
            min_length: 0.2,
            max_length: 1.8,
            max_rise: 0.45,
            max_drop: 1.0,
        }
    }
}

/// What a flight between footfalls may ask of the body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlightLimits {
    /// Least gap, in metres, between the capsule and the ground it flies over.
    pub clearance: f32,
    /// Fastest upward speed at takeoff, in m/s: about a 0.45 m hop.
    pub max_takeoff_climb: f32,
}

impl Default for FlightLimits {
    fn default() -> Self {
        Self {
            clearance: 0.03,
            max_takeoff_climb: 3.0,
        }
    }
}

/// Standard gravity, in m/s²: the default until the game passes its own.
pub const STANDARD_GRAVITY: f32 = 9.81;

/// The limits every plan keeps within. The game fills them in from the
/// capsule and the rig's legs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GaitParams {
    pub body: BodyDims,
    pub foot: FootDims,
    pub stride: StrideLimits,
    pub flight: FlightLimits,
    /// Downward acceleration, in m/s².
    pub gravity: f32,
}

impl Default for GaitParams {
    fn default() -> Self {
        Self {
            body: BodyDims::default(),
            foot: FootDims::default(),
            stride: StrideLimits::default(),
            flight: FlightLimits::default(),
            gravity: STANDARD_GRAVITY,
        }
    }
}
