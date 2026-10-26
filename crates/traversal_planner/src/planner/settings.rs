use serde::{Deserialize, Serialize};

use crate::params::GaitParams;

/// What the planner weighs. Every cost is unitless, so the weights say how
/// much one kind of awkwardness is worth against another.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
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
    /// Speed along the path a landing loses to the floor's face, per
    /// (m/s)²: the jolt of meeting a rise rather than landing on it.
    pub landing_loss: f32,
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
            landing_loss: 0.3,
            ride_travel: 4.0,
            loose: 0.5,
            loose_reference_mass: 100.0,
            edge: 1.0,
            comfortable_edge_margin: 0.1,
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
    /// A finished plan carries the body this many natural steps and then
    /// as far as the longest hop, and a step past its last landing: the
    /// legs need the footfall the swinging foot aims at and the one after,
    /// and commitment needs a tentative tail chosen knowing every flight
    /// that could take off past it, so no takeoff it rules out goes unseen.
    pub min_footfalls: usize,
    /// Spacing of the stations the body's path is searched over, in metres.
    pub body_step: f32,
    /// Spacing of the takeoffs and landings tried for flights, in metres.
    pub flight_step: f32,
    /// Height the body would gain and lose riding the next step that makes a
    /// flight worth trying instead, in metres; where the ride breaks, one
    /// always is.
    pub rough_ride: f32,
    /// How far behind a landing foot the body lands, as fractions of the step:
    /// `0.5` halfway between the feet, `0` over the landing foot.
    pub landing_leads: Vec<f32>,
    /// How many flights the feet could not follow the body's path may be
    /// searched again without, before the planner refuses.
    pub max_vetoes: usize,
    /// Shortest flight, in metres along the path.
    pub min_flight: f32,
    /// Upward speed a hop off a brow into what lies out of sight leaves
    /// with, in m/s: a runner jumps off an edge without seeing the landing.
    pub leap_climb: f32,
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
            body_step: 0.05,
            flight_step: 0.1,
            rough_ride: 0.02,
            landing_leads: vec![0.5, 0.25, 0.0],
            max_vetoes: 4,
            min_flight: 0.1,
            leap_climb: 1.5,
            commit_tolerance: 0.03,
            costs: CostWeights::default(),
        }
    }
}

impl PlannerSettings {
    /// How far a finished plan carries the body at `speed`, in metres.
    pub fn body_goal(&self, speed: f32, params: &GaitParams) -> f32 {
        self.min_footfalls as f32 * self.natural_step(speed, params) + params.longest_hop(speed)
    }

    /// The natural step at `speed`, from footfall to footfall, in metres:
    /// the legs' gait's, within the stride limits.
    pub fn natural_step(&self, speed: f32, params: &GaitParams) -> f32 {
        params
            .stride
            .natural
            .at(speed)
            .clamp(params.stride.min_length, params.stride.max_length)
    }

    /// Spacing of candidate footfalls along the path at `speed`, in metres.
    pub fn footfall_spacing(&self, speed: f32, params: &GaitParams) -> f32 {
        let (least, most) = self.footfall_spacing_range;
        (self.footfall_spacing_per_step * self.natural_step(speed, params)).clamp(least, most)
    }

    /// How far ahead of the body a plan at `speed` reaches, in metres: the
    /// least the strip must sample for the plan to finish inside it.
    pub fn reach(&self, speed: f32, params: &GaitParams) -> f32 {
        self.body_goal(speed, params) + params.stride.max_length
    }
}
