use serde::{Deserialize, Serialize};

use super::surface::Surface;

/// Where a section samples: its lanes, spacing and reach.
///
/// Shared by the game's sampler and the authored scenes' sampler, so both
/// build sections by the same rules.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StripLayout {
    /// Offsets `d` of the foothold lanes across the path, in increasing order.
    pub lanes: Vec<f32>,
    /// Sample spacing along `s` of the body floor and the ceiling, in metres.
    pub body_spacing: f32,
    /// Sample spacing along `s` of the foothold lanes, in metres: fine enough
    /// to find a plank's edge to within a foot's margin.
    pub foot_spacing: f32,
    /// Distance sampled behind the body, in metres, so the planted foot's
    /// ground is in the section.
    pub behind: f32,
    /// Distance sampled ahead of the body, in metres: at least how far a plan
    /// reaches at the body's speed (`PlannerSettings::reach`).
    pub ahead: f32,
}

impl Default for StripLayout {
    /// Five lanes over ±0.3 m, the body floor every 5 cm and footholds every
    /// 2 cm, from 0.5 m behind to 10 m ahead: a sprint's plan.
    fn default() -> Self {
        Self {
            lanes: vec![-0.3, -0.15, 0.0, 0.15, 0.3],
            body_spacing: 0.05,
            foot_spacing: 0.02,
            behind: 0.5,
            ahead: 10.0,
        }
    }
}

/// The surfaces along one line of the strip, in increasing `s`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Profile {
    /// Surfaces in increasing `s`; they do not overlap in `s`.
    pub surfaces: Vec<Surface>,
}

/// The footholds along one lane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FootholdLane {
    /// The lane's offset across the path.
    pub d: f32,
    /// What a foot-sized sphere cast down along the lane rests on.
    pub profile: Profile,
}

/// The ground ahead of the body, sampled into a strip.
///
/// ```text
///   side view                      top view (s →, d ↑)
///   ceiling     ‾‾‾‾‾‾‾‾‾‾‾‾‾‾      lane  0.3  ─ ─ ─ ─ ─ ─ ─
///   body floor  ⌒⌒⌒⌒‿⌒⌒⌒⌒          lane  0.15 ─ ─ ─ ─ ─ ─ ─
///   footholds   ▔▔▔▔ ▔▔▔▔ ▔▔        lane  0    ─ ─ ─ ─ ─ ─ ─ ◄ body floor, ceiling
///                                   lane -0.15 ─ ─ ─ ─ ─ ─ ─
///                                   lane -0.3  ─ ─ ─ ─ ─ ─ ─
/// ```
///
/// The body floor and ceiling are cast with the capsule's radius along the
/// path itself, which covers the capsule's width; footholds are cast with a
/// foot's radius along every lane. The wide cast bridges a gap narrower than
/// the capsule, so the body floor dips over it where the footholds break.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Section {
    /// Where the strip sampled.
    pub layout: StripLayout,
    /// The body's `s` when sampled: the stations of each line are
    /// `origin - layout.behind + k * spacing`.
    pub origin: f32,
    /// Radius of the body floor and ceiling casts: the capsule's.
    pub body_radius: f32,
    /// Radius of the foothold casts: a foot's.
    pub foot_radius: f32,
    /// What the capsule's bottom sphere, cast down along the path, rests on.
    pub body_floor: Profile,
    /// What the capsule's top sphere, cast up along the path, meets.
    pub ceiling: Profile,
    /// One per lane of the layout, in the same order.
    pub foothold_lanes: Vec<FootholdLane>,
}
