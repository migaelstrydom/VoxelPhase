use serde::{Deserialize, Serialize};

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
            unheld_body: 1.0,
            churn: 2.0,
        }
    }
}

/// How the planner searches: its resolution and reach, and its costs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlannerSettings {
    /// Spacing of candidate footfalls along the path, in metres.
    pub footfall_spacing: f32,
    /// How far along the path from the body a plan must carry it, in metres.
    pub horizon: f32,
    /// How far behind a landing foot the body lands, as fractions of the step:
    /// `0.5` halfway between the feet, `0` over the landing foot.
    pub landing_leads: Vec<f32>,
    /// Takeoff points tried for a flight, in metres back from the furthest
    /// point the body can ride to: further back flies higher over a lip.
    pub takeoff_setbacks: Vec<f32>,
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
            footfall_spacing: 0.06,
            horizon: 1.6,
            landing_leads: vec![0.5, 0.25, 0.0],
            takeoff_setbacks: vec![0.0, 0.15, 0.3, 0.5, 0.8],
            min_flight: 0.1,
            step_rate: 2.5,
            commit_tolerance: 0.03,
            costs: CostWeights::default(),
        }
    }
}
