use serde::{Deserialize, Serialize};

use crate::params::GaitParams;

use super::state::PlanState;

/// What the planner weighs. Every cost is unitless, so the weights say how
/// much one kind of awkwardness is worth against another.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CostWeights {
    /// A step's length off the natural length for its speed, per squared
    /// fraction of the natural length.
    pub step_length: f32,
    /// A foot off its natural offset across the path, per squared fraction of
    /// its reach.
    pub sideways: f32,
    /// Each flight.
    pub flight: f32,
    /// Upward speed at takeoff, per (m/s)².
    pub takeoff_climb: f32,
    /// Height gained and lost riding the floor, per metre: the dips and bumps
    /// felt as snags.
    pub ride_travel: f32,
    /// A footfall on loose footing as heavy as `loose_reference_mass`; a
    /// heavier body costs less, a lighter one more.
    pub loose: f32,
    /// In kg.
    pub loose_reference_mass: f32,
    /// A footfall closer to an edge than `comfortable_edge_margin`, per
    /// squared fraction of that margin.
    pub edge: f32,
    /// In metres.
    pub comfortable_edge_margin: f32,
    /// Landing below the speed the player asks for, per squared fraction of
    /// that speed.
    pub slowdown: f32,
    /// A body landing on ground neither foot is on, as on a stepping stone
    /// between the two it steps on.
    pub unheld_body: f32,
    /// A tentative footfall moved from where the last plan had it, per metre.
    pub churn: f32,
}

impl Default for CostWeights {
    fn default() -> Self {
        Self {
            step_length: 1.0,
            sideways: 0.5,
            flight: 0.3,
            takeoff_climb: 0.1,
            ride_travel: 4.0,
            loose: 0.5,
            loose_reference_mass: 100.0,
            edge: 1.0,
            comfortable_edge_margin: 0.1,
            slowdown: 1.0,
            unheld_body: 1.0,
            churn: 2.0,
        }
    }
}

/// How the planner searches: its resolution and reach, and its costs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlannerSettings {
    /// Spacing of candidate footfalls along the path, as a fraction of the
    /// natural step: fine for a crouch's short steps, coarser for a sprint's.
    pub footfall_spacing_per_step: f32,
    /// Bounds on that spacing, in metres.
    pub footfall_spacing_range: (f32, f32),
    /// Fewest footfalls a finished plan holds: the legs need the one the
    /// swinging foot aims at and the one after, and commitment needs a
    /// tentative tail chosen knowing what lies past it.
    pub min_footfalls: usize,
    /// A finished plan carries the body as far as the asked-for speed would
    /// in this long, in seconds: at speed, time to see an obstacle, set up a
    /// hop or stop. A plan that only slows to a stop before an obstacle
    /// covers less, so it is a refusal, not a plan.
    pub min_duration: f32,
    /// How far behind a landing foot the body lands, as fractions of the step:
    /// `0.5` halfway between the feet, `0` over the landing foot.
    pub landing_leads: Vec<f32>,
    /// Takeoff points tried for a flight, in seconds at the body's speed
    /// before the furthest point it can ride to: further back flies higher
    /// over a lip, and the faster the body, the further back that is.
    pub takeoff_lead_times: Vec<f32>,
    /// Shortest flight, in metres along the path.
    pub min_flight: f32,
    /// Natural steps per second: the natural step length is speed over this.
    pub step_rate: f32,
    /// How far, in metres, the last plan's committed footfall may be from a
    /// candidate and still be kept.
    pub commit_tolerance: f32,
    pub costs: CostWeights,
}

impl Default for PlannerSettings {
    fn default() -> Self {
        Self {
            footfall_spacing_per_step: 0.035,
            footfall_spacing_range: (0.04, 0.1),
            min_footfalls: 3,
            min_duration: 0.6,
            landing_leads: vec![0.5, 0.25, 0.0],
            takeoff_lead_times: vec![0.0, 0.05, 0.1, 0.17, 0.27],
            min_flight: 0.1,
            step_rate: 3.0,
            commit_tolerance: 0.03,
            costs: CostWeights::default(),
        }
    }
}

impl PlannerSettings {
    /// Whether a plan from `start` that has reached `state`, asked for
    /// `target_speed`, is long enough to finish.
    pub fn is_finished(&self, start: f32, state: &PlanState, target_speed: f32) -> bool {
        state.footfalls >= self.min_footfalls
            && state.body.s - start >= target_speed * self.min_duration
    }

    /// The natural step at `speed`, from footfall to footfall, in metres.
    pub fn natural_step(&self, speed: f32, params: &GaitParams) -> f32 {
        (speed / self.step_rate).clamp(params.stride.min_length, params.stride.max_length)
    }

    /// Spacing of candidate footfalls along the path at `speed`, in metres.
    pub fn footfall_spacing(&self, speed: f32, params: &GaitParams) -> f32 {
        let (least, most) = self.footfall_spacing_range;
        (self.footfall_spacing_per_step * self.natural_step(speed, params)).clamp(least, most)
    }

    /// How far ahead of the body a plan at `speed` reaches, in metres: the
    /// least the strip must sample for the plan to finish inside it.
    pub fn reach(&self, speed: f32, params: &GaitParams) -> f32 {
        let footfalls = self.min_footfalls as f32 * self.natural_step(speed, params);
        footfalls.max(speed * self.min_duration) + params.stride.max_length
    }
}
