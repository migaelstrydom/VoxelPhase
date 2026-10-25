use crate::params::GaitParams;

use super::settings::PlannerSettings;

/// Slowest speed the body is planned at, in m/s: below it a stretch would
/// take unboundedly long.
pub const MIN_PLANNING_SPEED: f32 = 0.3;

/// How far ahead of a takeoff the foot pushing off may be, in metres: a foot
/// is placed by where a sphere touches, so one standing at the takeoff can
/// read a fraction of a millimetre past it. More lets the body plan
/// takeoffs its feet then cannot make.
pub const PUSH_OFF_TOLERANCE: f32 = 0.001;
/// How far a foot may be behind a takeoff or past a landing, in metres.
pub const FOOT_SLACK: f32 = 0.15;

/// How much of a flight `span` metres long is a jump's, not a step's: what
/// is beyond the longest flight a stride spans with a foot's slack at
/// either end. A flight off an edge may carry that far past a stride.
pub fn jumped(span: f32, params: &GaitParams) -> f32 {
    (span - (params.stride.max_length - 2.0 * FOOT_SLACK)).max(0.0)
}

/// How the body's speed changes while it is on the ground.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pace {
    /// Toward the speed the player asks for, as fast as the drive allows.
    Press,
    /// Down, as fast as the drive allows: to meet stepping stones or a hop at
    /// a speed the asked-for one would overshoot.
    Brake,
}

impl Pace {
    /// The speed after riding `distance` from `speed`, asked for `target`.
    pub fn speed_after(self, speed: f32, distance: f32, target: f32, params: &GaitParams) -> f32 {
        let (slowest, fastest) = speed_reach(speed, distance, params);
        let next = match self {
            Pace::Press => target.clamp(slowest, fastest),
            Pace::Brake => slowest,
        };
        next.max(MIN_PLANNING_SPEED)
    }
}

/// The slowest and fastest the drive can bring `speed` to over `distance`,
/// at its acceleration held throughout.
pub fn speed_reach(speed: f32, distance: f32, params: &GaitParams) -> (f32, f32) {
    let change = 2.0 * params.body.max_acceleration * distance;
    (
        (speed * speed - change).max(0.0).sqrt(),
        (speed * speed + change).sqrt(),
    )
}

/// What running below the asked-for speed for `distance` costs, charged by
/// the natural step so a stride's worth costs `slowdown`.
pub fn slowdown_cost(
    speed: f32,
    target: f32,
    distance: f32,
    natural_step: f32,
    settings: &PlannerSettings,
) -> f32 {
    let target = target.max(MIN_PLANNING_SPEED);
    let short = ((target - speed) / target).max(0.0);
    settings.costs.slowdown * short * short * distance / natural_step
}
