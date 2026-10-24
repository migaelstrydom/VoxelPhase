use smallvec::SmallVec;

use crate::params::GaitParams;
use crate::plan::Phase;
use crate::section::SectionQuery;

use super::settings::PlannerSettings;
use super::state::PlanState;

/// One way on from a plan state: the phases to the next footfall, and that
/// footfall's state.
#[derive(Clone, Debug, PartialEq)]
pub struct Candidate {
    /// In time order, starting at the state expanded.
    pub phases: SmallVec<[Phase; 2]>,
    pub landing: PlanState,
    /// The model's own cost for the move; the planner adds the costs common
    /// to every footfall.
    pub cost: f32,
}

/// What a model reads while expanding a state.
pub struct ExpandContext<'a> {
    pub query: &'a dyn SectionQuery,
    pub params: &'a GaitParams,
    pub settings: &'a PlannerSettings,
    /// The speed along the path the player asks for, in m/s.
    pub target_speed: f32,
}

/// Proposes the ways on from a plan state. The planner validates every
/// proposal by the same rules (the body's path clears the floor and ceiling,
/// footfalls are on footing that will hold), so a model only proposes.
pub trait TransitionModel {
    fn expand(&self, from: &PlanState, context: &ExpandContext, out: &mut Vec<Candidate>);
}
